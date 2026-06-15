use open_pano::mat::Mat32f;
use open_pano::stitch::stitch_transform::StitchTransform;
use open_pano::stitch::stitcher::Stitcher;

pub struct StitcherState {
    transform: Option<StitchTransform>,
    keyframe_interval: usize,
}

impl StitcherState {
    pub fn new(keyframe_interval: usize) -> Self {
        StitcherState {
            transform: None,
            keyframe_interval,
        }
    }

    pub fn should_recompute(&self, frame_idx: usize) -> bool {
        frame_idx % self.keyframe_interval == 0
    }

    /// Run SIFT + matching + camera estimation and cache the result.
    /// Returns true if a new transform was computed, false if matching failed
    /// (in which case the previous transform is kept).
    pub fn compute(&mut self, frames: Vec<Mat32f>) -> bool {
        eprintln!(
            "[video_stitch] computing transform from {} frames …",
            frames.len()
        );
        let t0 = std::time::Instant::now();
        let mut stitcher = Stitcher::from_mats(frames);
        match stitcher.compute_transform() {
            Some(t) => {
                self.transform = Some(t);
                eprintln!(
                    "[video_stitch] transform computed in {:.1}s",
                    t0.elapsed().as_secs_f32()
                );
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

    /// Warp + blend using the cached transform. Panics if `compute` has not been called.
    pub fn apply(&self, frames: Vec<Mat32f>) -> Mat32f {
        self.transform
            .as_ref()
            .expect("StitcherState::compute() must be called before apply()")
            .apply(frames)
    }

    pub fn has_transform(&self) -> bool {
        self.transform.is_some()
    }
}
