use crate::{
    detector::detect_players, path_generator::PathConfig,
    path_generator::compute_virtual_camera_path,
};
use std::path::Path;
use yolo_ort::yolo::yolo_session::YoloSession;

use serde::{Deserialize, Serialize};

pub struct GenerateProgress {
    pub percentage: f64,
    pub step: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VirtualCameraSample {
    pub i: u32,
    pub cx: f32,
    pub cy: f32,
    pub w: f32,
    pub h: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VirtualCameraPath {
    pub version: u32,
    pub source: String,
    pub panorama_size: [u32; 2],
    pub fps: f64,
    pub frame_count: u32,
    pub aspect: [u32; 2],
    pub samples: Vec<VirtualCameraSample>,
}

impl VirtualCameraPath {
    pub fn new(
        source: String,
        panorama_size: [u32; 2],
        fps: f64,
        frame_count: u32,
        aspect: [u32; 2],
        mut samples: Vec<VirtualCameraSample>,
    ) -> Self {
        samples.sort_by_key(|s| s.i);
        Self {
            version: 1,
            source,
            panorama_size,
            fps,
            frame_count,
            aspect,
            samples,
        }
    }

    pub fn save(&self, path: &Path) -> Result<(), Box<dyn std::error::Error>> {
        let file = std::fs::File::create(path)?;
        serde_json::to_writer(file, self)?;
        Ok(())
    }

    pub fn generate(
        video_path: &Path,
        model_path: &Path,
        on_progress: impl Fn(GenerateProgress) + Send + 'static,
    ) -> VirtualCameraPath {
        on_progress(GenerateProgress {
            percentage: 0.0,
            step: "Loading model...".into(),
        });

        let mut yolo = YoloSession::new(
            model_path,
            (640, 640),
            true,
            "yolov10".into(),
        )
        .unwrap();

        on_progress(GenerateProgress {
            percentage: 20.0,
            step: "Detecting players...".into(),
        });

        let (targets, indices, fps, pano_size, total, class_stats, player_dets) =
            detect_players(video_path, &mut yolo, 2, 2, 0.3, None, |frame, total| {
                let pct = 20.0 + (frame as f64 / total.max(1) as f64) * 50.0;
                on_progress(GenerateProgress {
                    percentage: pct,
                    step: format!("Detecting players… {frame}/{total}"),
                });
            })
            .unwrap();

        // Emit class detection summary so it appears in the editor log.
        {
            let mut msg = format!("Detected {player_dets} player detections (class 2).");
            if !class_stats.is_empty() {
                msg.push_str(" All classes: ");
                let parts: Vec<String> = class_stats
                    .iter()
                    .take(6)
                    .map(|(cls, cnt)| {
                        let label = match cls {
                            0 => "ball",
                            1 => "goalkeeper",
                            2 => "player",
                            3 => "referee",
                            _ => "other",
                        };
                        format!("{label}({cnt})")
                    })
                    .collect();
                msg.push_str(&parts.join(", "));
            }
            on_progress(GenerateProgress { percentage: 72.0, step: msg });
        }

        on_progress(GenerateProgress {
            percentage: 75.0,
            step: "Computing camera path...".into(),
        });

        let samples = compute_virtual_camera_path(
            &targets,
            &indices,
            pano_size,
            fps,
            total,
            &PathConfig::default(),
        );

        on_progress(GenerateProgress {
            percentage: 100.0,
            step: "Done!".into(),
        });

        VirtualCameraPath::new(
            video_path.to_str().unwrap().into(),
            pano_size,
            fps,
            total,
            [16, 9],
            samples,
        )
    }

    pub fn load(path: &Path) -> Result<Self, Box<dyn std::error::Error>> {
        let file = std::fs::File::open(path)?;
        let mut cam: Self = serde_json::from_reader(file)?;
        if cam.version != 1 {
            return Err(format!("Unsupported vcam version: {}", cam.version).into());
        }
        cam.samples.sort_by_key(|s| s.i);
        Ok(cam)
    }

    /// Returns (cx, cy, w, h) at any frame using a uniform Catmull-Rom spline.
    pub fn rect_at(&self, frame_index: u32) -> (f32, f32, f32, f32) {
        let s = &self.samples;
        if s.is_empty() {
            return (0.0, 0.0, 0.0, 0.0);
        }
        if s.len() == 1 || frame_index <= s[0].i {
            return (s[0].cx, s[0].cy, s[0].w, s[0].h);
        }
        if frame_index >= s[s.len() - 1].i {
            let last = &s[s.len() - 1];
            return (last.cx, last.cy, last.w, last.h);
        }

        let j = s.partition_point(|smp| smp.i <= frame_index);
        let i1 = j - 1;
        let i2 = j;
        let span = s[i2].i - s[i1].i;
        let t = if span == 0 {
            0.0
        } else {
            (frame_index - s[i1].i) as f32 / span as f32
        };

        let i0 = i1.saturating_sub(1);
        let i3 = (i2 + 1).min(s.len() - 1);
        let (p0, p1, p2, p3) = (&s[i0], &s[i1], &s[i2], &s[i3]);

        let cr = |a: f32, b: f32, c: f32, d: f32| {
            let t2 = t * t;
            let t3 = t2 * t;
            0.5 * (2.0 * b
                + (-a + c) * t
                + (2.0 * a - 5.0 * b + 4.0 * c - d) * t2
                + (-a + 3.0 * b - 3.0 * c + d) * t3)
        };

        (
            cr(p0.cx, p1.cx, p2.cx, p3.cx),
            cr(p0.cy, p1.cy, p2.cy, p3.cy),
            cr(p0.w, p1.w, p2.w, p3.w),
            cr(p0.h, p1.h, p2.h, p3.h),
        )
    }

    /// Returns the integer pixel crop box `(x0, y0, x1, y1)`, clamped to the panorama.
    pub fn bbox_at(&self, frame_index: u32) -> (u32, u32, u32, u32) {
        let (cx, cy, w, h) = self.rect_at(frame_index);
        let [pw, ph] = self.panorama_size;
        let x0 = ((cx - w / 2.0).round() as i64).clamp(0, pw as i64) as u32;
        let y0 = ((cy - h / 2.0).round() as i64).clamp(0, ph as i64) as u32;
        let x1 = ((cx + w / 2.0).round() as i64).clamp(0, pw as i64) as u32;
        let y1 = ((cy + h / 2.0).round() as i64).clamp(0, ph as i64) as u32;
        (x0, y0, x1, y1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_path(samples: Vec<(u32, f32)>) -> VirtualCameraPath {
        VirtualCameraPath::new(
            String::new(),
            [1920, 1080],
            30.0,
            300,
            [16, 9],
            samples
                .into_iter()
                .map(|(i, cx)| VirtualCameraSample {
                    i,
                    cx,
                    cy: 0.0,
                    w: 100.0,
                    h: 56.0,
                })
                .collect(),
        )
    }

    #[test]
    fn rect_at_clamps_before_first() {
        let p = make_path(vec![(10, 500.0), (20, 600.0)]);
        let (cx, _, _, _) = p.rect_at(0);
        assert_eq!(cx, 500.0);
    }

    #[test]
    fn rect_at_clamps_after_last() {
        let p = make_path(vec![(10, 500.0), (20, 600.0)]);
        let (cx, _, _, _) = p.rect_at(100);
        assert_eq!(cx, 600.0);
    }

    #[test]
    fn rect_at_midpoint_between_endpoints() {
        let p = make_path(vec![(0, 0.0), (10, 100.0), (20, 200.0)]);
        let (cx, _, _, _) = p.rect_at(10);
        assert_eq!(cx, 100.0);
        // midpoint between frame 0 and 20 should be near 100
        let (mid, _, _, _) = p.rect_at(10);
        assert!((mid - 100.0).abs() < 1.0);
    }
}
