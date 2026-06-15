use crate::config::config;
use crate::geometry::{Vec2D, Vec3};
use crate::mat::Mat32f;
use crate::stitch::homography::Homography;
use crate::stitch::imageref::ImageRef;
use crate::stitch::projection::{cylindrical, flat, spherical, Homo2Proj, Proj2Homo};
use crate::stitch::stitcher_image::{
    ComponentRange, ConnectedImages, ImageComponent, ProjRange, ProjectionMethod,
};
use std::f64::consts::PI;

/// Plain-data snapshot of a computed stitching transform — no raw pointers.
/// Cheap to clone and safe to store between frames.
pub struct StitchTransform {
    pub proj_method: ProjectionMethod,
    pub proj_range: ProjRange,
    pub identity_idx: usize,
    pub components: Vec<TransformComponent>,
}

pub struct TransformComponent {
    pub homo: Homography,
    pub homo_inv: Homography,
    pub range: ComponentRange,
    pub img_width: i32,
    pub img_height: i32,
}

/// Per-pixel sample entry in a warp lookup table.
/// src_x == f32::NAN means this output pixel is not covered by this camera.
#[derive(Clone, Copy)]
pub struct WarpEntry {
    pub src_x: f32,
    pub src_y: f32,
    pub weight: f32,
}

impl WarpEntry {
    const INVALID: WarpEntry = WarpEntry { src_x: f32::NAN, src_y: 0.0, weight: 0.0 };

    #[inline]
    pub fn is_valid(&self) -> bool {
        !self.src_x.is_nan()
    }
}

/// Precomputed warp lookup table — built once per keyframe, applied every frame.
/// cam_maps[camera_idx] is flat [out_h * out_w]; index by row * out_w + col.
pub struct PrecomputedWarp {
    pub out_w: usize,
    pub out_h: usize,
    pub cam_maps: Vec<Vec<WarpEntry>>,
}

impl StitchTransform {
    /// Extract the reusable transform data from a fully-built ConnectedImages.
    pub fn from_bundle(bundle: &ConnectedImages) -> Self {
        let components = bundle
            .component
            .iter()
            .map(|c| TransformComponent {
                homo: c.homo,
                homo_inv: c.homo_inv,
                range: c.range,
                img_width: c.imgref().width,
                img_height: c.imgref().height,
            })
            .collect();
        StitchTransform {
            proj_method: bundle.proj_method,
            proj_range: bundle.proj_range,
            identity_idx: bundle.identity_idx,
            components,
        }
    }

    /// Apply the stored transform to a fresh set of frames (one per camera).
    /// Does NOT re-run SIFT or camera estimation — only warps and blends.
    /// Uses multiband blending from config. For video, use precompute_warp() instead.
    pub fn apply(&self, images: Vec<Mat32f>) -> Mat32f {
        assert_eq!(
            images.len(),
            self.components.len(),
            "number of frames must match number of cameras in transform"
        );

        let mut imgs: Vec<ImageRef> = images.into_iter().map(ImageRef::from_mat).collect();

        let mut bundle = ConnectedImages::new();
        bundle.proj_method = self.proj_method;
        bundle.proj_range = self.proj_range;
        bundle.identity_idx = self.identity_idx;
        bundle.component = self
            .components
            .iter()
            .zip(imgs.iter_mut())
            .map(|(c, img)| ImageComponent {
                homo: c.homo,
                homo_inv: c.homo_inv,
                imgptr: img as *mut ImageRef,
                range: c.range,
            })
            .collect();

        bundle.blend()
    }

    fn get_homo2proj(&self) -> Homo2Proj {
        match self.proj_method {
            ProjectionMethod::Flat => flat::homo2proj,
            ProjectionMethod::Cylindrical => cylindrical::homo2proj,
            ProjectionMethod::Spherical => spherical::homo2proj,
        }
    }

    fn get_proj2homo(&self) -> Proj2Homo {
        match self.proj_method {
            ProjectionMethod::Flat => flat::proj2homo,
            ProjectionMethod::Cylindrical => cylindrical::proj2homo,
            ProjectionMethod::Spherical => spherical::proj2homo,
        }
    }

    /// Mirrors ConnectedImages::get_final_resolution().
    fn resolution(&self) -> Vec2D {
        let cfg = config();
        let ref_comp = &self.components[self.identity_idx];
        let refw = ref_comp.img_width as f64;
        let refh = ref_comp.img_height as f64;
        let homo2proj = self.get_homo2proj();
        let h = &ref_comp.homo;

        let c2 = h.trans_vec(Vec3::new(refw / 2.0, refh / 2.0, 1.0));
        let c1 = h.trans_vec(Vec3::new(-refw / 2.0, -refh / 2.0, 1.0));
        let mut id_range = Vec2D::new(
            homo2proj(c2).x - homo2proj(c1).x,
            homo2proj(c2).y - homo2proj(c1).y,
        );

        if self.proj_method != ProjectionMethod::Flat {
            if id_range.x < 0.0 {
                id_range.x = 2.0 * PI + id_range.x;
            }
            if id_range.y < 0.0 {
                id_range.y = PI + id_range.y;
            }
        }

        let mut resolution = Vec2D::new(id_range.x.abs() / refw, id_range.y.abs() / refh);
        let ps = self.proj_range.size();
        let target_size = Vec2D::new(ps.x / resolution.x, ps.y / resolution.y);
        let max_edge = target_size.x.max(target_size.y);

        if max_edge > cfg.max_output_size as f64 {
            let ratio = max_edge / cfg.max_output_size as f64;
            resolution.x *= ratio;
            resolution.y *= ratio;
        }
        resolution
    }

    /// Build a warp lookup table once per keyframe.
    /// For each output pixel × camera: source (x, y) + linear blend weight.
    /// Per-frame blending then needs only table lookups + bilinear interpolation.
    pub fn precompute_warp(&self) -> PrecomputedWarp {
        let cfg = config();
        let resolution = self.resolution();
        let prmin = self.proj_range.min;
        let ps = self.proj_range.size();
        let out_w = (ps.x / resolution.x).round() as usize;
        let out_h = (ps.y / resolution.y).round() as usize;
        let n_pixels = out_h * out_w;

        let proj2homo = self.get_proj2homo();

        let cam_maps: Vec<Vec<WarpEntry>> = self
            .components
            .iter()
            .map(|comp| {
                let h_inv = comp.homo_inv;
                let iw = comp.img_width as f64;
                let ih = comp.img_height as f64;
                let cw = iw / 2.0;
                let ch = ih / 2.0;

                let tl_x = ((comp.range.min.x - prmin.x) / resolution.x).floor() as i32;
                let tl_y = ((comp.range.min.y - prmin.y) / resolution.y).floor() as i32;
                let br_x = ((comp.range.max.x - prmin.x) / resolution.x).ceil() as i32;
                let br_y = ((comp.range.max.y - prmin.y) / resolution.y).ceil() as i32;

                let col0 = tl_x.max(0) as usize;
                let row0 = tl_y.max(0) as usize;
                let col1 = (br_x.max(0) as usize).min(out_w);
                let row1 = (br_y.max(0) as usize).min(out_h);

                let mut map = vec![WarpEntry::INVALID; n_pixels];

                for row in row0..row1 {
                    for col in col0..col1 {
                        let c = Vec2D::new(
                            col as f64 * resolution.x + prmin.x,
                            row as f64 * resolution.y + prmin.y,
                        );
                        let homo = proj2homo(c);
                        let ret = h_inv.trans_vec(homo);
                        if ret.z < 0.0 {
                            continue;
                        }
                        let d = 1.0 / ret.z;
                        let src_x = (ret.x * d + cw) as f32;
                        let src_y = (ret.y * d + ch) as f32;
                        // Keep 1px margin for bilinear interpolation
                        if src_x < 0.0
                            || src_x >= (iw - 1.0) as f32
                            || src_y < 0.0
                            || src_y >= (ih - 1.0) as f32
                        {
                            continue;
                        }
                        // Same weight formula as LinearBlender::run()
                        let wx = (0.5 - (src_x as f64 / iw - 0.5).abs()) as f32;
                        let weight = if cfg.ordered_input {
                            wx
                        } else {
                            let wy = (0.5 - (src_y as f64 / ih - 0.5).abs()) as f32;
                            wx * wy
                        };
                        map[row * out_w + col] = WarpEntry { src_x, src_y, weight };
                    }
                }
                map
            })
            .collect();

        PrecomputedWarp { out_w, out_h, cam_maps }
    }
}
