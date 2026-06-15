use crate::config::config;
use crate::geometry::{Coor, Vec2D, Vec3};
use crate::mat::Mat32f;
use crate::stitch::blender::LinearBlender;
use crate::stitch::homography::Homography;
use crate::stitch::imageref::ImageRef;
use crate::stitch::multiband::MultiBandBlender;
use crate::stitch::projection::{Homo2Proj, Proj2Homo};
use crate::stitch::projection::{cylindrical, flat, spherical};
use std::f64::consts::PI;

#[derive(Clone, Copy, Debug)]
pub struct ProjRange {
    pub min: Vec2D,
    pub max: Vec2D,
}

impl ProjRange {
    pub fn size(&self) -> Vec2D {
        Vec2D::new(self.max.x - self.min.x, self.max.y - self.min.y)
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum ProjectionMethod {
    Flat,
    Cylindrical,
    Spherical,
}

#[derive(Clone, Copy, Debug)]
pub struct ComponentRange {
    pub min: Vec2D,
    pub max: Vec2D,
}

pub struct ImageComponent {
    /// From image plane (2D) to homogeneous 3D space.
    pub homo: Homography,
    /// Inverse: from 3D space to image plane (K * R * P).
    pub homo_inv: Homography,
    /// Pointer to the original ImageRef.
    pub imgptr: *mut ImageRef,
    pub range: ComponentRange,
}

impl ImageComponent {
    pub fn new(img: &mut ImageRef) -> Self {
        ImageComponent {
            homo: Homography::identity(),
            homo_inv: Homography::identity(),
            imgptr: img as *mut ImageRef,
            range: ComponentRange {
                min: Vec2D::new(0.0, 0.0),
                max: Vec2D::new(0.0, 0.0),
            },
        }
    }

    pub fn imgref(&self) -> &ImageRef {
        unsafe { &*self.imgptr }
    }
    pub fn imgref_mut(&mut self) -> &mut ImageRef {
        unsafe { &mut *self.imgptr }
    }
}

pub struct ConnectedImages {
    pub proj_method: ProjectionMethod,
    pub proj_range: ProjRange,
    pub identity_idx: usize,
    pub component: Vec<ImageComponent>,
}

impl ConnectedImages {
    pub fn new() -> Self {
        ConnectedImages {
            proj_method: ProjectionMethod::Flat,
            proj_range: ProjRange {
                min: Vec2D::new(0.0, 0.0),
                max: Vec2D::new(0.0, 0.0),
            },
            identity_idx: 0,
            component: Vec::new(),
        }
    }

    pub fn get_homo2proj(&self) -> Homo2Proj {
        match self.proj_method {
            ProjectionMethod::Flat => flat::homo2proj,
            ProjectionMethod::Cylindrical => cylindrical::homo2proj,
            ProjectionMethod::Spherical => spherical::homo2proj,
        }
    }

    pub fn get_proj2homo(&self) -> Proj2Homo {
        match self.proj_method {
            ProjectionMethod::Flat => flat::proj2homo,
            ProjectionMethod::Cylindrical => cylindrical::proj2homo,
            ProjectionMethod::Spherical => spherical::proj2homo,
        }
    }

    pub fn shift_all_homo(&mut self) {
        let mid = self.identity_idx;
        let mid_w = self.component[mid].imgref().width as f64;
        let mid_h = self.component[mid].imgref().height as f64;
        let t2 = Homography::get_translation(mid_w * 0.5, mid_h * 0.5);

        for i in 0..self.component.len() {
            if i == mid {
                continue;
            }
            let (iw, ih) = {
                let c = &self.component[i];
                (c.imgref().width as f64, c.imgref().height as f64)
            };
            let t1 = Homography::get_translation(iw * 0.5, ih * 0.5);
            let h = self.component[i].homo;
            self.component[i].homo = t2 * h * t1.inverse(None);
        }
    }

    pub fn calc_inverse_homo(&mut self) {
        for c in &mut self.component {
            c.homo_inv = c.homo.inverse(None);
        }
    }

    pub fn update_proj_range(&mut self) {
        const CORNER_SAMPLE: usize = 100;
        let mut corners: Vec<Vec2D> = Vec::with_capacity(4 * CORNER_SAMPLE);
        for i in 0..CORNER_SAMPLE {
            let xi = i as f64 / CORNER_SAMPLE as f64 - 0.5;
            corners.push(Vec2D::new(xi, -0.5));
            corners.push(Vec2D::new(xi, 0.5));
        }
        for j in 0..CORNER_SAMPLE {
            let yj = j as f64 / CORNER_SAMPLE as f64 - 0.5;
            corners.push(Vec2D::new(-0.5, yj));
            corners.push(Vec2D::new(0.5, yj));
        }

        let homo2proj = self.get_homo2proj();
        let (mut proj_min_x, mut proj_min_y) = (f64::MAX, f64::MAX);
        let (mut proj_max_x, mut proj_max_y) = (f64::MIN, f64::MIN);

        for comp in &mut self.component {
            let iw = comp.imgref().width as f64;
            let ih = comp.imgref().height as f64;
            let (mut nx, mut ny) = (f64::MAX, f64::MAX);
            let (mut xx, mut xy) = (f64::MIN, f64::MIN);
            for &v in &corners {
                let homo = comp.homo.trans_vec(Vec3::new(v.x * iw, v.y * ih, 1.0));
                let tc = homo2proj(homo);
                nx = nx.min(tc.x);
                ny = ny.min(tc.y);
                xx = xx.max(tc.x);
                xy = xy.max(tc.y);
            }
            comp.range = ComponentRange {
                min: Vec2D::new(nx, ny),
                max: Vec2D::new(xx, xy),
            };
            proj_min_x = proj_min_x.min(nx);
            proj_min_y = proj_min_y.min(ny);
            proj_max_x = proj_max_x.max(xx);
            proj_max_y = proj_max_y.max(xy);
        }
        self.proj_range = ProjRange {
            min: Vec2D::new(proj_min_x, proj_min_y),
            max: Vec2D::new(proj_max_x, proj_max_y),
        };
    }

    pub fn get_final_resolution(&self) -> Vec2D {
        let cfg = config();
        let ref_comp = &self.component[self.identity_idx];
        let refw = ref_comp.imgref().width as f64;
        let refh = ref_comp.imgref().height as f64;
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

    pub fn blend(&self) -> Mat32f {
        let cfg = config();
        let proj2homo = self.get_proj2homo();
        let resolution = self.get_final_resolution();

        let prmin = self.proj_range.min;

        if cfg.multiband > 0 {
            let mut blender = MultiBandBlender::new(cfg.multiband as usize);
            for comp in &self.component {
                let tl = self.scale_to_img_coor(comp.range.min, prmin, resolution);
                let br = self.scale_to_img_coor(comp.range.max, prmin, resolution);
                let imgref = unsafe { &mut *comp.imgptr };
                let h_inv = comp.homo_inv;
                let cw = comp.imgref().shape().halfw();
                let ch = comp.imgref().shape().halfh();
                blender.add_image(
                    tl,
                    br,
                    imgref,
                    Box::new(move |t: Coor| {
                        let c = Vec2D::new(
                            t.x as f64 * resolution.x + prmin.x,
                            t.y as f64 * resolution.y + prmin.y,
                        );
                        let homo = proj2homo(c);
                        let ret = h_inv.trans_vec(homo);
                        if ret.z < 0.0 {
                            return Vec2D::new(-10.0, -10.0);
                        }
                        let d = 1.0 / ret.z;
                        Vec2D::new(ret.x * d + cw, ret.y * d + ch)
                    }),
                );
            }
            blender.run()
        } else {
            let mut blender = LinearBlender::new();
            for comp in &self.component {
                let tl = self.scale_to_img_coor(comp.range.min, prmin, resolution);
                let br = self.scale_to_img_coor(comp.range.max, prmin, resolution);
                let imgref = unsafe { &mut *comp.imgptr };
                let h_inv = comp.homo_inv;
                let cw = comp.imgref().shape().halfw();
                let ch = comp.imgref().shape().halfh();
                blender.add_image_entry(
                    tl,
                    br,
                    imgref,
                    Box::new(move |t: Coor| {
                        let c = Vec2D::new(
                            t.x as f64 * resolution.x + prmin.x,
                            t.y as f64 * resolution.y + prmin.y,
                        );
                        let homo = proj2homo(c);
                        let ret = h_inv.trans_vec(homo);
                        if ret.z < 0.0 {
                            return Vec2D::new(-10.0, -10.0);
                        }
                        let d = 1.0 / ret.z;
                        Vec2D::new(ret.x * d + cw, ret.y * d + ch)
                    }),
                );
            }
            blender.run(cfg.ordered_input)
        }
    }

    fn scale_to_img_coor(&self, v: Vec2D, prmin: Vec2D, resolution: Vec2D) -> Coor {
        Coor::new(
            ((v.x - prmin.x) / resolution.x) as i32,
            ((v.y - prmin.y) / resolution.y) as i32,
        )
    }
}
