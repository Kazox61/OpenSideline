use crate::warp_map::apply_warp;
use open_pano::mat::Mat32f;
use open_pano::stitch::stitch_transform::{PrecomputedWarp, StitchTransform};
use open_pano::stitch::stitcher::Stitcher;

pub struct StitcherState {
    transform: Option<StitchTransform>,
    warp: Option<PrecomputedWarp>,
    keyframe_interval: usize,
}

impl StitcherState {
    pub fn new(keyframe_interval: usize) -> Self {
        StitcherState {
            transform: None,
            warp: None,
            keyframe_interval,
        }
    }

    pub fn should_recompute(&self, frame_idx: usize) -> bool {
        frame_idx % self.keyframe_interval == 0
    }

    /// Run SIFT + matching + camera estimation and cache the transform + warp map.
    /// Returns true on success; on failure the previous transform/warp are kept.
    pub fn compute(&mut self, frames: Vec<Mat32f>) -> bool {
        eprintln!(
            "[video_stitch] computing transform from {} frames …",
            frames.len()
        );
        let t0 = std::time::Instant::now();
        let mut stitcher = Stitcher::from_mats(frames);
        match stitcher.compute_transform() {
            Some(t) => {
                eprintln!(
                    "[video_stitch] transform computed in {:.1}s — building warp map …",
                    t0.elapsed().as_secs_f32()
                );
                let tw = std::time::Instant::now();
                let warp = t.precompute_warp();
                eprintln!(
                    "[video_stitch] warp map {}×{} built in {:.2}s",
                    warp.out_w, warp.out_h,
                    tw.elapsed().as_secs_f32()
                );
                self.transform = Some(t);
                self.warp = Some(warp);
                true
            }
            None => {
                eprintln!(
                    "[video_stitch] WARNING: matching failed after {:.1}s — keeping previous transform",
                    t0.elapsed().as_secs_f32()
                );
                false
            }
        }
    }

    /// Warp + blend using the precomputed warp map (fast path: no trig, parallel).
    pub fn apply(&self, frames: Vec<Mat32f>) -> Mat32f {
        match &self.warp {
            Some(warp) => apply_warp(warp, &frames),
            None => panic!("StitcherState::compute() must be called before apply()"),
        }
    }

    pub fn has_transform(&self) -> bool {
        self.transform.is_some()
    }
}
