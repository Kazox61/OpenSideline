use crate::config::{DESC_HIST_BIN_NUM, DESC_HIST_WIDTH, DESC_LEN, config};
use crate::feature::dog::ScaleSpace;
use crate::feature::{Descriptor, SSPoint};
use crate::utils::between;
use std::f32::consts::{PI, SQRT_2};

pub struct Sift<'a> {
    ss: &'a ScaleSpace,
    points: &'a [SSPoint],
}

impl<'a> Sift<'a> {
    pub fn new(ss: &'a ScaleSpace, points: &'a [SSPoint]) -> Self {
        Sift { ss, points }
    }

    pub fn get_descriptor(&self) -> Vec<Descriptor> {
        self.points
            .iter()
            .map(|p| self.calc_descriptor(p))
            .collect()
    }

    fn calc_descriptor(&self, p: &SSPoint) -> Descriptor {
        let cfg = config();
        let pi2 = 2.0 * PI;
        let nbin_per_rad = DESC_HIST_BIN_NUM as f32 / pi2;

        let pyramid = &self.ss.pyramids[p.pyr_id];
        let w = pyramid.w;
        let h = pyramid.h;
        let mag_img = pyramid.get_mag(p.scale_id);
        let ort_img = pyramid.get_ort(p.scale_id);

        let coor = p.coor;
        let ort = p.dir;
        let hist_w = p.scale_factor * cfg.desc_hist_scale_factor as f32;
        let exp_denom = 2.0 * (DESC_HIST_WIDTH as f32) * (DESC_HIST_WIDTH as f32);
        let radius = (SQRT_2 * hist_w * (DESC_HIST_WIDTH as f32 + 1.0)).round() as i32;

        let mut hist = [[0.0f32; DESC_HIST_BIN_NUM]; DESC_HIST_WIDTH * DESC_HIST_WIDTH];
        let cosort = ort.cos();
        let sinort = ort.sin();

        for xx in -radius..=radius {
            let nowx = coor.x + xx;
            if !between(nowx, 1, w as i32 - 1) {
                continue;
            }
            for yy in -radius..=radius {
                let nowy = coor.y + yy;
                if !between(nowy, 1, h as i32 - 1) {
                    continue;
                }
                if (xx * xx + yy * yy) as f32 > (radius * radius) as f32 {
                    continue;
                }

                let y_rot = (-xx as f32 * sinort + yy as f32 * cosort) / hist_w;
                let x_rot = (xx as f32 * cosort + yy as f32 * sinort) / hist_w;
                let ybin = y_rot + DESC_HIST_WIDTH as f32 / 2.0 - 0.5;
                let xbin = x_rot + DESC_HIST_WIDTH as f32 / 2.0 - 0.5;

                if !between(ybin, -1.0, DESC_HIST_WIDTH as f32)
                    || !between(xbin, -1.0, DESC_HIST_WIDTH as f32)
                {
                    continue;
                }

                let now_mag = *mag_img.at2(nowy as usize, nowx as usize);
                let now_ort = ort_img.at2(nowy as usize, nowx as usize);
                let weight = (-(x_rot * x_rot + y_rot * y_rot) / exp_denom).exp() * now_mag;

                let mut now_ort = *now_ort - ort;
                if now_ort < 0.0 {
                    now_ort += pi2;
                }
                if now_ort > pi2 {
                    now_ort -= pi2;
                }
                let hist_bin = now_ort * nbin_per_rad;

                trilinear_interpolate(xbin, ybin, hist_bin, weight, &mut hist);
            }
        }

        let mut desp = Descriptor {
            coor: p.real_coor,
            descriptor: vec![0.0; DESC_LEN],
        };

        // Flatten hist into descriptor
        for i in 0..DESC_HIST_WIDTH * DESC_HIST_WIDTH {
            for j in 0..DESC_HIST_BIN_NUM {
                desp.descriptor[i * DESC_HIST_BIN_NUM + j] = hist[i][j];
            }
        }

        // RootSIFT normalization
        let sum: f32 = desp.descriptor.iter().sum();
        if sum > 1e-10 {
            for v in &mut desp.descriptor {
                *v /= sum;
            }
        }
        let factor = cfg.desc_int_factor as f32;
        for v in &mut desp.descriptor {
            *v = v.sqrt() * factor;
        }

        desp
    }
}

fn trilinear_interpolate(
    xbin: f32,
    ybin: f32,
    hbin: f32,
    weight: f32,
    hist: &mut [[f32; DESC_HIST_BIN_NUM]; DESC_HIST_WIDTH * DESC_HIST_WIDTH],
) {
    let ybinf = ybin.floor() as i32;
    let xbinf = xbin.floor() as i32;
    let hbinf = hbin.floor() as i32;
    let ybind = ybin - ybinf as f32;
    let xbind = xbin - xbinf as f32;
    let hbind = hbin - hbinf as f32;

    for dy in 0i32..=1 {
        if !between(ybinf + dy, 0, DESC_HIST_WIDTH as i32) {
            continue;
        }
        let w_y = weight * if dy == 1 { ybind } else { 1.0 - ybind };
        for dx in 0i32..=1 {
            if !between(xbinf + dx, 0, DESC_HIST_WIDTH as i32) {
                continue;
            }
            let w_x = w_y * if dx == 1 { xbind } else { 1.0 - xbind };
            let bin_2d_idx = ((ybinf + dy) * DESC_HIST_WIDTH as i32 + (xbinf + dx)) as usize;
            let h0 = (hbinf % DESC_HIST_BIN_NUM as i32) as usize;
            let h1 = ((hbinf + 1) % DESC_HIST_BIN_NUM as i32) as usize;
            hist[bin_2d_idx][h0] += w_x * (1.0 - hbind);
            hist[bin_2d_idx][h1] += w_x * hbind;
        }
    }
}
