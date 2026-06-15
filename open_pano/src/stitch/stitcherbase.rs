use crate::feature::{Descriptor, FeatureDetector, SiftDetector};
use crate::geometry::Vec2D;
use crate::stitch::imageref::ImageRef;

pub struct StitcherBase {
    pub imgs: Vec<ImageRef>,
    pub feats: Vec<Vec<Descriptor>>,
    pub keypoints: Vec<Vec<Vec2D>>,
    detector: Box<dyn FeatureDetector>,
}

impl StitcherBase {
    pub fn new(filenames: &[String]) -> Self {
        let imgs = filenames.iter().map(|f| ImageRef::new(f)).collect();
        StitcherBase {
            imgs,
            feats: Vec::new(),
            keypoints: Vec::new(),
            detector: Box::new(SiftDetector),
        }
    }

    pub fn from_mats(mats: Vec<crate::mat::Mat32f>) -> Self {
        let imgs = mats.into_iter().map(ImageRef::from_mat).collect();
        StitcherBase {
            imgs,
            feats: Vec::new(),
            keypoints: Vec::new(),
            detector: Box::new(SiftDetector),
        }
    }

    pub fn calc_feature(&mut self) {
        let n = self.imgs.len();
        self.feats.resize(n, Vec::new());
        self.keypoints.resize(n, Vec::new());

        for k in 0..n {
            self.imgs[k].load();
            let descs = self.detector.detect_feature(self.imgs[k].mat());
            if descs.is_empty() {
                panic!("Cannot find features in image {}!", k);
            }
            self.keypoints[k] = descs.iter().map(|d| d.coor).collect();
            self.feats[k] = descs;
        }
    }

    pub fn free_feature(&mut self) {
        self.feats.clear();
        self.feats.shrink_to_fit();
        self.keypoints.clear();
        self.keypoints.shrink_to_fit();
    }
}
