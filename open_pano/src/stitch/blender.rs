use crate::geometry::{Coor, Vec2D};
use crate::imgproc::interpolate;
use crate::mat::Mat32f;
use crate::stitch::imageref::ImageRef;

pub struct Range {
    pub min: Coor,
    pub max: Coor,
}

impl Range {
    pub fn contains(&self, r: i32, c: i32) -> bool {
        r >= self.min.y && r <= self.max.y && c >= self.min.x && c <= self.max.x
    }
    pub fn width(&self) -> i32 {
        self.max.x - self.min.x + 1
    }
    pub fn height(&self) -> i32 {
        self.max.y - self.min.y + 1
    }
}

pub trait Blender {
    fn add_image(
        &mut self,
        upper_left: Coor,
        bottom_right: Coor,
        img: &mut ImageRef,
        coor_func: Box<dyn Fn(Coor) -> Vec2D>,
    );
    fn run(&self) -> Mat32f;
}

pub struct LinearBlender {
    images: Vec<(Range, *mut ImageRef, Box<dyn Fn(Coor) -> Vec2D>)>,
    target_w: i32,
    target_h: i32,
}

impl LinearBlender {
    pub fn new() -> Self {
        LinearBlender {
            images: Vec::new(),
            target_w: 0,
            target_h: 0,
        }
    }

    pub fn add_image_entry(
        &mut self,
        upper_left: Coor,
        bottom_right: Coor,
        img: &mut ImageRef,
        coor_func: Box<dyn Fn(Coor) -> Vec2D>,
    ) {
        self.target_w = self.target_w.max(bottom_right.x);
        self.target_h = self.target_h.max(bottom_right.y);
        self.images.push((
            Range {
                min: upper_left,
                max: bottom_right,
            },
            img as *mut ImageRef,
            coor_func,
        ));
    }

    pub fn run(&self, ordered_input: bool) -> Mat32f {
        let tw = self.target_w as usize;
        let th = self.target_h as usize;
        let mut target = Mat32f::new(th, tw, 3);
        // Do NOT pre-fill with NO — accumulation starts from 0; NO is set below for zero-weight pixels.

        let mut weight = vec![0.0f32; th * tw];

        for &(ref range, imgref_ptr, ref coor_func) in &self.images {
            let imgref = unsafe { &mut *imgref_ptr };
            imgref.load();
            let iw = imgref.width as f64;
            let ih = imgref.height as f64;
            let img_mat = imgref.mat();

            for i in range.min.y..range.max.y {
                for j in range.min.x..range.max.x {
                    let img_coor = (coor_func)(Coor::new(j, i));
                    if img_coor.x < 0.0 || img_coor.x >= iw || img_coor.y < 0.0 || img_coor.y >= ih
                    {
                        continue;
                    }
                    let color = interpolate(img_mat, img_coor.y as f32, img_coor.x as f32);
                    if color.x < 0.0 {
                        continue;
                    }

                    let mut w = (0.5 - (img_coor.x / iw - 0.5).abs()) as f32;
                    if !ordered_input {
                        w *= (0.5 - (img_coor.y / ih - 0.5).abs()) as f32;
                    }

                    let idx = i as usize * tw + j as usize;
                    let p = target.pixel_mut(i as usize, j as usize);
                    p[0] += color.x * w;
                    p[1] += color.y * w;
                    p[2] += color.z * w;
                    weight[idx] += w;
                }
            }
        }

        for i in 0..th {
            for j in 0..tw {
                let w = weight[i * tw + j];
                let p = target.pixel_mut(i, j);
                if w > 0.0 {
                    p[0] /= w;
                    p[1] /= w;
                    p[2] /= w;
                } else {
                    p[0] = -1.0;
                    p[1] = -1.0;
                    p[2] = -1.0;
                }
            }
        }
        target
    }
}
