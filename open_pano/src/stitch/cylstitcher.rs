use crate::config::config;
use crate::feature::matcher::PairWiseMatcher;
use crate::geometry::{Coor, Vec2D};
use crate::imgproc::get_perspective_transform;
use crate::mat::Mat32f;
use crate::stitch::blender::LinearBlender;
use crate::stitch::homography::Homography;
use crate::stitch::imageref::ImageRef;
use crate::stitch::match_info::Shape2D;
use crate::stitch::stitcher_image::{
    ComponentRange, ConnectedImages, ImageComponent, ProjectionMethod,
};
use crate::stitch::stitcherbase::StitcherBase;
use crate::stitch::transform_estimate::TransformEstimation;
use crate::stitch::warp::CylinderWarper;
use crate::utils::update_min;

pub struct CylinderStitcher {
    base: StitcherBase,
    bundle: ConnectedImages,
}

impl CylinderStitcher {
    pub fn new(filenames: &[String]) -> Self {
        let base = StitcherBase::new(filenames);
        let n = base.imgs.len();
        let mut bundle = ConnectedImages::new();
        bundle.component = (0..n)
            .map(|_| ImageComponent {
                homo: Homography::identity(),
                homo_inv: Homography::identity(),
                imgptr: std::ptr::null_mut(),
                range: ComponentRange {
                    min: Vec2D::new(0.0, 0.0),
                    max: Vec2D::new(0.0, 0.0),
                },
            })
            .collect();

        let mut s = CylinderStitcher { base, bundle };
        for i in 0..n {
            s.bundle.component[i].imgptr = &mut s.base.imgs[i] as *mut _;
        }
        s
    }

    pub fn build(&mut self) -> Mat32f {
        self.base.calc_feature();
        self.bundle.identity_idx = self.base.imgs.len() / 2;
        self.build_warp();
        self.base.free_feature();
        self.bundle.proj_method = ProjectionMethod::Flat;
        self.bundle.update_proj_range();
        let ret = self.bundle.blend();
        self.perspective_correction(ret)
    }

    fn build_warp(&mut self) {
        let n = self.base.imgs.len();
        let mid = self.bundle.identity_idx;
        for comp in &mut self.bundle.component {
            comp.homo = Homography::identity();
        }

        let feats_ref: &Vec<Vec<_>> = &self.base.feats;
        let pwmatcher = PairWiseMatcher::new(feats_ref);
        let mut matches: Vec<crate::feature::matcher::MatchData> = (0..n - 1)
            .map(|k| pwmatcher.match_pair(k, (k + 1) % n))
            .collect();

        // Search for best h_factor
        let mut min_slope = f32::MAX;
        let mut best_factor = 1.0f32;
        let mut best_mat: Vec<Homography> = Vec::new();

        if n - mid > 1 {
            let mut new_factor = 1.0f32;
            let slope = self.update_h_factor(
                new_factor,
                &mut min_slope,
                &mut best_factor,
                &mut best_mat,
                &matches,
            );
            if best_mat.is_empty() {
                panic!("Failed to find hfactor");
            }
            let center2 = best_mat[0].trans_xy(0.0, 0.0);
            let order = if center2.x > 0.0 { 1.0f32 } else { -1.0 };
            let mut slope = slope;
            let cfg = config();
            for k in 0..3 {
                if slope.abs() < cfg.slope_plain {
                    break;
                }
                new_factor +=
                    (if slope < 0.0 { order } else { -order }) / (5.0 * 2.0f32.powi(k as i32));
                slope = self.update_h_factor(
                    new_factor,
                    &mut min_slope,
                    &mut best_factor,
                    &mut best_mat,
                    &matches,
                );
            }
        }

        let warper = CylinderWarper::new(best_factor as f64);
        for k in 0..n {
            self.base.imgs[k].load();
            warper.warp_img(
                self.base.imgs[k].img.as_mut().unwrap(),
                &mut self.base.keypoints[k],
            );
        }

        // Accumulate transforms right of mid
        for k in mid + 1..n {
            self.bundle.component[k].homo = best_mat[k - mid - 1];
        }

        // Left of mid: reverse matches
        for i in (0..mid).rev() {
            matches[i].reverse();
            let shape_ip1 = Shape2D::new(self.base.imgs[i + 1].width, self.base.imgs[i + 1].height);
            let shape_i = Shape2D::new(self.base.imgs[i].width, self.base.imgs[i].height);
            let te = TransformEstimation::new(
                &matches[i],
                &self.base.keypoints[i + 1],
                &self.base.keypoints[i],
                shape_ip1,
                shape_i,
            );
            match te.get_transform() {
                None => panic!("Failed to match between image {} and {}", i, i + 1),
                Some(info) => {
                    self.bundle.component[i].homo = info.homo;
                }
            }
        }

        // Chain transforms left of mid
        for i in (0..mid.saturating_sub(1)).rev() {
            let h = self.bundle.component[i + 1].homo * self.bundle.component[i].homo;
            self.bundle.component[i].homo = h;
        }

        self.bundle.calc_inverse_homo();
    }

    fn update_h_factor(
        &self,
        factor: f32,
        min_slope: &mut f32,
        best_factor: &mut f32,
        best_mat: &mut Vec<Homography>,
        matches: &[crate::feature::matcher::MatchData],
    ) -> f32 {
        let n = self.base.imgs.len();
        let mid = self.bundle.identity_idx;
        let start = mid;
        let end = n;
        let len = end - start;

        let mut now_shapes: Vec<Shape2D> = (start..end)
            .map(|k| Shape2D::new(self.base.imgs[k].width, self.base.imgs[k].height))
            .collect();
        let mut now_kpts: Vec<Vec<Vec2D>> = (start..end)
            .map(|k| self.base.keypoints[k].clone())
            .collect();

        let warper = CylinderWarper::new(factor as f64);
        for k in 0..len {
            warper.warp_shape(&mut now_shapes[k], &mut now_kpts[k]);
        }

        let mut now_mat: Vec<Option<Homography>> = vec![None; len - 1];
        let mut failed = false;
        for k in 1..len {
            let te = TransformEstimation::new(
                &matches[k - 1 + mid],
                &now_kpts[k - 1],
                &now_kpts[k],
                now_shapes[k - 1],
                now_shapes[k],
            );
            match te.get_transform() {
                None => {
                    failed = true;
                }
                Some(info) => {
                    now_mat[k - 1] = Some(info.homo);
                }
            }
        }
        if failed {
            return 0.0;
        }

        // Accumulate transforms
        let mut mats: Vec<Homography> = now_mat.into_iter().map(|m| m.unwrap()).collect();
        for k in 1..len - 1 {
            let h = mats[k - 1] * mats[k];
            mats[k] = h;
        }

        let center2 = mats.last().unwrap().trans_xy(0.0, 0.0);
        let slope = if center2.x.abs() < 1e-9 {
            0.0
        } else {
            (center2.y / center2.x) as f32
        };

        if update_min(min_slope, slope.abs()) {
            *best_factor = factor;
            *best_mat = mats;
        }
        slope
    }

    fn perspective_correction(&self, img: Mat32f) -> Mat32f {
        let w = img.width() as i32;
        let h = img.height() as i32;
        let mid = self.bundle.identity_idx;
        let ref_w = self.base.imgs[mid].width as f64;
        let ref_h = self.base.imgs[mid].height as f64;
        let homo2proj = self.bundle.get_homo2proj();
        let proj_min = self.bundle.proj_range.min;

        let mut corners: Vec<Vec2D> = Vec::new();
        let to_ref_coor = |comp: &ImageComponent, v: Vec2D| -> Vec2D {
            let vw = Vec2D::new(
                v.x * comp.imgref().width as f64,
                v.y * comp.imgref().height as f64,
            );
            let homo = comp
                .homo
                .trans_vec(crate::geometry::Vec3::new(vw.x, vw.y, 1.0));
            let hn = crate::geometry::Vec3::new(homo.x / ref_w, homo.y / ref_h, homo.z);
            let tc = homo2proj(hn);
            Vec2D::new(tc.x * ref_w - proj_min.x, tc.y * ref_h - proj_min.y)
        };

        let front = &self.bundle.component[0];
        corners.push(to_ref_coor(front, Vec2D::new(-0.5, -0.5)));
        corners.push(to_ref_coor(front, Vec2D::new(-0.5, 0.5)));
        let back = self.bundle.component.last().unwrap();
        corners.push(to_ref_coor(back, Vec2D::new(0.5, -0.5)));
        corners.push(to_ref_coor(back, Vec2D::new(0.5, 0.5)));

        let std_corners = vec![
            Vec2D::new(0.0, 0.0),
            Vec2D::new(0.0, h as f64),
            Vec2D::new(w as f64, 0.0),
            Vec2D::new(w as f64, h as f64),
        ];
        let m = get_perspective_transform(&corners, &std_corners);
        let inv = Homography::from_matrix(&m);

        let mut blender = LinearBlender::new();
        // We need a temporary ImageRef for the input image.
        // Create a temporary one with a clone of the image.
        let img_clone = img.clone_data();
        let mut tmp = ImageRef::new("(correction)");
        tmp.img = Some(img_clone);
        tmp.width = w;
        tmp.height = h;

        blender.add_image_entry(
            Coor::new(0, 0),
            Coor::new(w, h),
            &mut tmp,
            Box::new(move |c: Coor| inv.trans2d(Vec2D::new(c.x as f64, c.y as f64))),
        );
        blender.run(true)
    }
}
