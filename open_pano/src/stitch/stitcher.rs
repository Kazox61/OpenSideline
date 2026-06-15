use crate::config::config;
use crate::feature::matcher::{MatchData, PairWiseMatcher};
use crate::geometry::Vec2D;
use crate::mat::Mat32f;
use crate::stitch::camera::Camera;
use crate::stitch::camera_estimator::CameraEstimator;
use crate::stitch::homography::Homography;
use crate::stitch::match_info::{MatchInfo, Shape2D};
use crate::stitch::stitcher_image::{ConnectedImages, ImageComponent, ProjectionMethod};
use crate::stitch::stitcherbase::StitcherBase;
use crate::stitch::transform_estimate::TransformEstimation;

pub struct Stitcher {
    base: StitcherBase,
    bundle: ConnectedImages,
    pairwise_matches: Vec<Vec<MatchInfo>>,
}

impl Stitcher {
    pub fn new(filenames: &[String]) -> Self {
        let base = StitcherBase::new(filenames);
        let n = base.imgs.len();
        let mut bundle = ConnectedImages::new();
        bundle.component = base
            .imgs
            .iter()
            .enumerate()
            .map(|(_, _)| {
                // We need a mutable pointer; base.imgs is owned by self.base
                // We'll fix the pointers after constructing.
                ImageComponent {
                    homo: Homography::identity(),
                    homo_inv: Homography::identity(),
                    imgptr: std::ptr::null_mut(),
                    range: crate::stitch::stitcher_image::ComponentRange {
                        min: Vec2D::new(0.0, 0.0),
                        max: Vec2D::new(0.0, 0.0),
                    },
                }
            })
            .collect();

        let mut s = Stitcher {
            base,
            bundle,
            pairwise_matches: Vec::new(),
        };
        // Fix up imgptr to point into self.base.imgs
        for i in 0..n {
            s.bundle.component[i].imgptr = &mut s.base.imgs[i] as *mut _;
        }
        s
    }

    pub fn build(&mut self) -> Mat32f {
        self.base.calc_feature();

        let n = self.base.imgs.len();
        self.pairwise_matches = vec![(0..n).map(|_| MatchInfo::new()).collect::<Vec<_>>(); n];

        let cfg = config();
        if cfg.ordered_input {
            self.linear_pairwise_match();
        } else {
            self.pairwise_match();
        }
        self.base.free_feature();

        self.assign_center();

        if cfg.estimate_camera {
            self.estimate_camera();
            self.bundle.proj_method = ProjectionMethod::Spherical;
        } else {
            self.build_linear_simple();
            self.bundle.proj_method = ProjectionMethod::Flat;
        }
        self.pairwise_matches.clear();
        self.bundle.update_proj_range();
        self.bundle.blend()
    }

    fn process_match(&mut self, match_data: MatchData, i: usize, j: usize) -> bool {
        let shape_i = Shape2D::new(self.base.imgs[i].width, self.base.imgs[i].height);
        let shape_j = Shape2D::new(self.base.imgs[j].width, self.base.imgs[j].height);
        let te = TransformEstimation::new(
            &match_data,
            &self.base.keypoints[i],
            &self.base.keypoints[j],
            shape_i,
            shape_j,
        );
        match te.get_transform() {
            None => false,
            Some(info) => {
                let mut succ = false;
                let inv = info.homo.inverse(Some(&mut succ));
                if !succ {
                    return false;
                }
                let mut inv = inv;
                let scale = 1.0 / inv.data[8];
                inv.mult_scalar(scale);

                self.pairwise_matches[i][j] = info;
                let mut rev_info = self.pairwise_matches[i][j].clone_meta();
                rev_info.homo = inv;
                rev_info.reverse();
                self.pairwise_matches[j][i] = rev_info;
                true
            }
        }
    }

    fn pairwise_match(&mut self) {
        let n = self.base.imgs.len();
        let tasks: Vec<(usize, usize)> = (0..n)
            .flat_map(|i| (i + 1..n).map(move |j| (i, j)))
            .collect();
        // Pre-compute all match data before mutating pairwise_matches
        let all_matches: Vec<_> = {
            let pwmatcher = PairWiseMatcher::new(&self.base.feats);
            tasks
                .iter()
                .map(|&(i, j)| (i, j, pwmatcher.match_pair(i, j)))
                .collect()
        };
        for (i, j, match_data) in all_matches {
            self.process_match(match_data, i, j);
        }
    }

    fn linear_pairwise_match(&mut self) {
        let n = self.base.imgs.len();
        let all_matches: Vec<_> = {
            let pwmatcher = PairWiseMatcher::new(&self.base.feats);
            (0..n)
                .map(|i| {
                    let next = (i + 1) % n;
                    (i, next, pwmatcher.match_pair(i, next))
                })
                .collect()
        };
        for (i, next, match_data) in all_matches {
            let ok = self.process_match(match_data, i, next);
            if !ok && i != n - 1 {
                panic!("Image {} and {} don't match", i, next);
            }
        }
    }

    fn assign_center(&mut self) {
        self.bundle.identity_idx = self.base.imgs.len() / 2;
    }

    fn estimate_camera(&mut self) {
        let shapes: Vec<Shape2D> = self
            .base
            .imgs
            .iter()
            .map(|img| Shape2D::new(img.width, img.height))
            .collect();
        let cameras = CameraEstimator::new(&mut self.pairwise_matches, &shapes).estimate();

        let n = self.base.imgs.len();
        for i in 0..n {
            self.bundle.component[i].homo_inv = cameras[i].k() * cameras[i].r;
            self.bundle.component[i].homo = cameras[i].r_inv() * cameras[i].k_inv();
        }
    }

    fn build_linear_simple(&mut self) {
        let n = self.base.imgs.len();
        let mid = self.bundle.identity_idx;
        self.bundle.component[mid].homo = Homography::identity();

        if mid + 1 < n {
            self.bundle.component[mid + 1].homo = self.pairwise_matches[mid][mid + 1].homo;
            for k in mid + 2..n {
                let h = self.bundle.component[k - 1].homo * self.pairwise_matches[k - 1][k].homo;
                self.bundle.component[k].homo = h;
            }
        }
        if mid > 0 {
            self.bundle.component[mid - 1].homo = self.pairwise_matches[mid][mid - 1].homo;
            for k in (0..mid - 1).rev() {
                let h = self.bundle.component[k + 1].homo * self.pairwise_matches[k + 1][k].homo;
                self.bundle.component[k].homo = h;
            }
        }

        let cfg = config();
        let mut f = -1.0f64;
        if !cfg.trans {
            f = Camera::estimate_focal(&self.pairwise_matches);
        }
        if f <= 0.0 {
            let img = &self.base.imgs[mid];
            f = 0.5 * (img.width + img.height) as f64;
        }
        let fi = 1.0 / f;
        let m = Homography::from_array([fi, 0.0, 0.0, 0.0, fi, 0.0, 0.0, 0.0, 1.0]);
        for comp in &mut self.bundle.component {
            comp.homo = m * comp.homo;
        }
        self.bundle.calc_inverse_homo();
    }
}

// MatchInfo needs a clone_meta helper for reverse ops
impl MatchInfo {
    fn clone_meta(&self) -> MatchInfo {
        MatchInfo {
            match_pairs: self.match_pairs.clone(),
            confidence: self.confidence,
            homo: self.homo,
        }
    }
}
