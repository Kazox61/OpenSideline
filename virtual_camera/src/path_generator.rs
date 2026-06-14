use crate::{
    detector::FrameTarget,
    smooth_damp::SmoothDamp,
    virtual_camera_path::VirtualCameraSample,
};

#[derive(Debug, Clone)]
pub enum ZoomMode {
    Fixed,
    Adaptive,
}

#[derive(Debug, Clone)]
pub struct PathConfig {
    pub smooth_sec: f32,
    pub max_speed_frac: Option<f32>,
    pub lookahead_sec: f32,
    pub zoom: ZoomMode,
    pub base_zoom: f32,
    pub zoom_gain: f32,
    pub keyframe_rate: u32,
    pub aspect: [u32; 2],
}

impl Default for PathConfig {
    fn default() -> Self {
        Self {
            smooth_sec: 1.5,
            max_speed_frac: None,
            lookahead_sec: 0.0,
            zoom: ZoomMode::Fixed,
            base_zoom: 0.4,
            zoom_gain: 1.6,
            keyframe_rate: 4,
            aspect: [16, 9],
        }
    }
}

/// Linearly interpolate NaN gaps in a signal. Returns `None` if no valid values exist.
fn fill_gaps(values: &[Option<f32>]) -> Option<Vec<f32>> {
    let good: Vec<(usize, f32)> = values
        .iter()
        .enumerate()
        .filter_map(|(i, v)| v.map(|x| (i, x)))
        .collect();
    if good.is_empty() {
        return None;
    }
    let n = values.len();
    let mut out = vec![0.0f32; n];
    for i in 0..n {
        let j = good.partition_point(|&(gi, _)| gi <= i);
        if j == 0 {
            out[i] = good[0].1;
        } else if j == good.len() {
            out[i] = good[good.len() - 1].1;
        } else {
            let (i0, v0) = good[j - 1];
            let (i1, v1) = good[j];
            let t = (i - i0) as f32 / (i1 - i0) as f32;
            out[i] = v0 + (v1 - v0) * t;
        }
    }
    Some(out)
}

/// Compute a smooth virtual camera path from per-frame detection targets.
///
/// `targets` and `frame_indices` are parallel slices — one entry per *processed* frame
/// (i.e. accounting for detection stride). Returns keyframe samples ready for `VirtualCameraPath`.
pub fn compute_virtual_camera_path(
    targets: &[Option<FrameTarget>],
    frame_indices: &[u32],
    pano_size: [u32; 2],
    fps: f64,
    total_frames: u32,
    config: &PathConfig,
) -> Vec<VirtualCameraSample> {
    let [pw, ph] = pano_size;
    let ar = config.aspect[0] as f32 / config.aspect[1] as f32;
    let n = targets.len();

    if n == 0 {
        eprintln!("[compute_virtual_camera_path] WARNING: no targets — returning centered default path");
        let step = (fps as f32 / config.keyframe_rate as f32).round().max(1.0) as u32;
        let mut kf_indices: Vec<u32> = (0..total_frames).step_by(step as usize).collect();
        if !kf_indices.is_empty() && kf_indices.last() != Some(&(total_frames - 1)) {
            kf_indices.push(total_frames - 1);
        }
        let cx = pw as f32 / 2.0;
        let cy = ph as f32 / 2.0;
        let win_w = (config.base_zoom * pw as f32).clamp(64.0, pw as f32);
        let win_h = (win_w / ar).clamp(36.0, ph as f32);
        let win_w = win_w.min(win_h * ar);
        return kf_indices
            .into_iter()
            .map(|i| VirtualCameraSample { i, cx, cy, w: win_w, h: win_h })
            .collect();
    }

    let tx_raw: Vec<Option<f32>> = targets.iter().map(|t| t.as_ref().map(|t| t.tx)).collect();
    let ty_raw: Vec<Option<f32>> = targets.iter().map(|t| t.as_ref().map(|t| t.ty)).collect();
    let sp_raw: Vec<Option<f32>> = targets.iter().map(|t| t.as_ref().map(|t| t.spread)).collect();

    let tx = match fill_gaps(&tx_raw) {
        Some(v) => v,
        None => vec![pw as f32 / 2.0; n],
    };
    let ty = match fill_gaps(&ty_raw) {
        Some(v) => v,
        None => vec![ph as f32 / 2.0; n],
    };
    let sp = fill_gaps(&sp_raw).unwrap_or_else(|| vec![0.0; n]);

    // time step between processed frames (accounts for detection stride)
    let dt = if frame_indices.len() > 1 {
        let median_gap = {
            let mut gaps: Vec<f32> = frame_indices.windows(2).map(|w| (w[1] - w[0]) as f32).collect();
            gaps.sort_by(|a, b| a.partial_cmp(b).unwrap());
            gaps[gaps.len() / 2]
        };
        median_gap / fps as f32
    } else {
        1.0 / fps as f32
    };

    let max_speed = config.max_speed_frac.map(|f| f * pw as f32);

    // look-ahead: shift the aim point forward using a finite-difference velocity estimate
    let (tx_aim, ty_aim): (Vec<f32>, Vec<f32>) = if config.lookahead_sec > 0.0 {
        let vx = finite_diff_velocity(&tx, dt);
        let vy = finite_diff_velocity(&ty, dt);
        let la = config.lookahead_sec;
        (
            tx.iter().zip(vx.iter()).map(|(x, v)| x + v * la).collect(),
            ty.iter().zip(vy.iter()).map(|(y, v)| y + v * la).collect(),
        )
    } else {
        (tx.clone(), ty.clone())
    };

    // run SmoothDamp followers
    let mut sd_cx = SmoothDamp::new(tx_aim[0], config.smooth_sec);
    let mut sd_cy = SmoothDamp::new(ty_aim[0], config.smooth_sec);

    let base_w = config.base_zoom * pw as f32;
    let mut sd_w = SmoothDamp::new(base_w, (config.smooth_sec * 1.5).max(2.0));

    let mut cx_series = Vec::with_capacity(n);
    let mut cy_series = Vec::with_capacity(n);
    let mut w_series = Vec::with_capacity(n);
    let mut h_series = Vec::with_capacity(n);

    for k in 0..n {
        let cx = sd_cx.update(tx_aim[k], dt, max_speed);
        let cy = sd_cy.update(ty_aim[k], dt, max_speed);

        let target_w = match config.zoom {
            ZoomMode::Fixed => base_w,
            ZoomMode::Adaptive => {
                let raw = (sp[k] * config.zoom_gain).max(base_w);
                raw.clamp(0.25 * pw as f32, 0.9 * pw as f32)
            }
        };
        let win_w = sd_w.update(target_w, dt, None);
        let win_w = win_w.clamp(64.0, pw as f32);
        let win_h = (win_w / ar).clamp(36.0, ph as f32);
        let win_w = win_w.min(win_h * ar);

        // clamp center so window stays inside panorama
        let cx = cx.clamp(win_w / 2.0, pw as f32 - win_w / 2.0);
        let cy = cy.clamp(win_h / 2.0, ph as f32 - win_h / 2.0);

        cx_series.push(cx);
        cy_series.push(cy);
        w_series.push(win_w);
        h_series.push(win_h);
    }

    // downsample to keyframe_rate via linear interpolation over the full frame range
    let step = (fps as f32 / config.keyframe_rate as f32).round().max(1.0) as u32;
    let mut kf_indices: Vec<u32> = (0..total_frames).step_by(step as usize).collect();
    if kf_indices.last() != Some(&(total_frames - 1)) {
        kf_indices.push(total_frames - 1);
    }

    let fi: Vec<f32> = frame_indices.iter().map(|&i| i as f32).collect();

    kf_indices
        .into_iter()
        .map(|kf| {
            let kff = kf as f32;
            VirtualCameraSample {
                i: kf,
                cx: interp(&fi, &cx_series, kff),
                cy: interp(&fi, &cy_series, kff),
                w: interp(&fi, &w_series, kff),
                h: interp(&fi, &h_series, kff),
            }
        })
        .collect()
}

fn interp(xs: &[f32], ys: &[f32], x: f32) -> f32 {
    if xs.is_empty() {
        return 0.0;
    }
    if x <= xs[0] {
        return ys[0];
    }
    if x >= xs[xs.len() - 1] {
        return ys[ys.len() - 1];
    }
    let j = xs.partition_point(|&xi| xi <= x);
    let (x0, x1) = (xs[j - 1], xs[j]);
    let t = (x - x0) / (x1 - x0);
    ys[j - 1] + (ys[j] - ys[j - 1]) * t
}

fn finite_diff_velocity(signal: &[f32], dt: f32) -> Vec<f32> {
    let n = signal.len();
    let mut v = vec![0.0f32; n];
    if n < 2 {
        return v;
    }
    v[0] = (signal[1] - signal[0]) / dt;
    for i in 1..n - 1 {
        v[i] = (signal[i + 1] - signal[i - 1]) / (2.0 * dt);
    }
    v[n - 1] = (signal[n - 1] - signal[n - 2]) / dt;
    v
}
