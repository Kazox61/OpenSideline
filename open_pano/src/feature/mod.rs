pub mod brief;
pub mod dist;
pub mod dog;
pub mod extrema;
pub mod gaussian;
pub mod matcher;
pub mod orientation;
pub mod sift;

use crate::config::config;
use crate::geometry::{Coor, Vec2D};
use crate::imgproc::resize;
use crate::mat::Mat32f;

/// Feature descriptor with 2D coordinate and descriptor vector
#[derive(Clone, Debug)]
pub struct Descriptor {
    pub coor: Vec2D,
    pub descriptor: Vec<f32>,
}

impl Descriptor {
    pub fn euclidean_sqr(&self, r: &Descriptor, now_thres: f32) -> f32 {
        dist::euclidean_sqr(&self.descriptor, &r.descriptor, now_thres)
    }

    pub fn hamming(&self, r: &Descriptor) -> i32 {
        dist::hamming(&self.descriptor, &r.descriptor)
    }
}

/// A keypoint in the scale-space pyramid
#[derive(Clone, Debug)]
pub struct SSPoint {
    pub coor: Coor,
    pub real_coor: Vec2D,
    pub pyr_id: usize,
    pub scale_id: usize,
    pub dir: f32,
    pub scale_factor: f32,
}

/// Base feature detector trait
pub trait FeatureDetector: Send + Sync {
    fn do_detect_feature(&self, img: &Mat32f) -> Vec<Descriptor>;

    /// Detect features and convert coordinates to half-shifted image coordinates [-w/2, w/2]
    fn detect_feature(&self, img: &Mat32f) -> Vec<Descriptor> {
        let mut ret = self.do_detect_feature(img);
        let w = img.width() as f64;
        let h = img.height() as f64;
        for d in &mut ret {
            d.coor.x = (d.coor.x - 0.5) * w;
            d.coor.y = (d.coor.y - 0.5) * h;
        }
        ret
    }
}

/// SIFT feature detector
pub struct SiftDetector;

impl FeatureDetector for SiftDetector {
    fn do_detect_feature(&self, mat: &Mat32f) -> Vec<Descriptor> {
        let cfg = config();
        let ratio = cfg.sift_working_size as f32 * 2.0 / (mat.width() + mat.height()) as f32;
        let new_rows = (mat.rows() as f32 * ratio).round() as usize;
        let new_cols = (mat.cols() as f32 * ratio).round() as usize;
        let mut resized = Mat32f::new(new_rows, new_cols, 3);
        resize(mat, &mut resized);

        let ss = dog::ScaleSpace::new(&resized, cfg.num_octave as usize, cfg.num_scale as usize);
        let sp = dog::DogSpace::new(&ss);
        let ex = extrema::ExtremaDetector::new(&sp);
        let keyp = ex.get_extrema();
        let ort = orientation::OrientationAssign::new(&ss, &keyp);
        let keyp = ort.work();
        let sift = sift::Sift::new(&ss, &keyp);
        sift.get_descriptor()
    }
}

/// BRIEF feature detector
pub struct BriefDetector {
    pattern: brief::BriefPattern,
}

impl BriefDetector {
    pub fn new() -> Self {
        let pattern = brief::Brief::gen_brief_pattern(
            crate::config::BRIEF_PATH_SIZE as usize,
            crate::config::BRIEF_NR_PAIR as usize,
        );
        BriefDetector { pattern }
    }
}

impl FeatureDetector for BriefDetector {
    fn do_detect_feature(&self, mat: &Mat32f) -> Vec<Descriptor> {
        let cfg = config();
        let ss = dog::ScaleSpace::new(mat, cfg.num_octave as usize, cfg.num_scale as usize);
        let sp = dog::DogSpace::new(&ss);
        let ex = extrema::ExtremaDetector::new(&sp);
        let keyp = ex.get_extrema();
        let b = brief::Brief::new(mat, &keyp, &self.pattern);
        b.get_descriptor()
    }
}
