use crate::color::Color;
use crate::geometry::{Vec2D, Vec3};
use crate::imgproc::{fill_color, interpolate};
use crate::mat::Mat32f;
use crate::stitch::match_info::Shape2D;

pub struct CylinderProject {
    pub center: Vec3,
    pub r: i32,
    pub sizefactor: i32,
}

impl CylinderProject {
    pub fn new(r: i32, center: Vec3, sizefactor: i32) -> Self {
        CylinderProject {
            center,
            r,
            sizefactor,
        }
    }

    fn proj_vec(&self, p: &Vec3) -> Vec2D {
        let dx = p.x - self.center.x;
        let x = (dx / self.r as f64).atan();
        let y = (p.y - self.center.y) / dx.hypot(self.r as f64);
        Vec2D::new(x, y)
    }

    fn proj(&self, p: Vec2D) -> Vec2D {
        self.proj_vec(&Vec3::new(p.x, p.y, 0.0))
    }

    fn proj_r(&self, p: Vec2D) -> Vec2D {
        let x = self.r as f64 * p.x.tan() + self.center.x;
        let y = p.y * self.r as f64 / p.x.cos() + self.center.y;
        Vec2D::new(x, y)
    }

    pub fn project_img(&self, img: &Mat32f, pts: &mut Vec<Vec2D>) -> Mat32f {
        let mut shape = Shape2D::new(img.width() as i32, img.height() as i32);
        let offset = self.project_shape(&mut shape, pts);

        let sfactor_inv = 1.0 / self.sizefactor as f64;
        let mut mat = Mat32f::new(shape.h as usize, shape.w as usize, 3);
        fill_color(&mut mat, Color::NO);

        for i in 0..shape.h as usize {
            for j in 0..shape.w as usize {
                let ori = self.proj_r((Vec2D::new(j as f64, i as f64) - offset) * sfactor_inv);
                if ori.x >= 0.0
                    && ori.x < img.width() as f64
                    && ori.y >= 0.0
                    && ori.y < img.height() as f64
                {
                    let c = interpolate(img, ori.y as f32, ori.x as f32);
                    let p = mat.pixel_mut(i, j);
                    p[0] = c.x;
                    p[1] = c.y;
                    p[2] = c.z;
                }
            }
        }
        mat
    }

    pub fn project_shape(&self, shape: &mut Shape2D, pts: &mut Vec<Vec2D>) -> Vec2D {
        let sf = self.sizefactor as f64;
        let (mut min_x, mut min_y) = (f64::MAX, f64::MAX);
        let (mut max_x, mut max_y) = (f64::MIN, f64::MIN);

        for i in 0..shape.h {
            for j in 0..shape.w {
                let c = self.proj(Vec2D::new(j as f64, i as f64));
                min_x = min_x.min(c.x);
                min_y = min_y.min(c.y);
                max_x = max_x.max(c.x);
                max_y = max_y.max(c.y);
            }
        }

        let (min_x, min_y) = (min_x * sf, min_y * sf);
        let (max_x, max_y) = (max_x * sf, max_y * sf);
        let offset = Vec2D::new(-min_x, -min_y);
        let real_w = (max_x - min_x) as i32;
        let real_h = (max_y - min_y) as i32;

        let hw = shape.w as f64 / 2.0;
        let hh = shape.h as f64 / 2.0;
        for f in pts.iter_mut() {
            let coor = Vec2D::new(f.x + hw, f.y + hh);
            let proj = self.proj(coor) * sf + offset;
            f.x = proj.x - real_w as f64 / 2.0;
            f.y = proj.y - real_h as f64 / 2.0;
        }

        shape.w = real_w;
        shape.h = real_h;
        offset
    }
}

pub struct CylinderWarper {
    h_factor: f64,
}

impl CylinderWarper {
    pub fn new(h_factor: f64) -> Self {
        CylinderWarper { h_factor }
    }

    pub fn get_projector(&self, w: i32, h: i32) -> CylinderProject {
        let cfg = crate::config::config();
        let r = ((w as f64).hypot(h as f64) * cfg.focal_length as f64 / 43.266) as i32;
        let center = Vec3::new(w as f64 / 2.0, h as f64 / 2.0 * self.h_factor, r as f64);
        CylinderProject::new(r, center, r)
    }

    pub fn warp_img(&self, mat: &mut Mat32f, kpts: &mut Vec<Vec2D>) {
        let proj = self.get_projector(mat.width() as i32, mat.height() as i32);
        *mat = proj.project_img(mat, kpts);
    }

    pub fn warp_shape(&self, shape: &mut Shape2D, kpts: &mut Vec<Vec2D>) {
        let proj = self.get_projector(shape.w, shape.h);
        proj.project_shape(shape, kpts);
    }

    pub fn warp_img_only(&self, mat: &mut Mat32f) {
        let mut empty = Vec::new();
        self.warp_img(mat, &mut empty);
    }
}
