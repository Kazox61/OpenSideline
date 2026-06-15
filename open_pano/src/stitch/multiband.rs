use crate::color::Color;
use crate::feature::gaussian::GaussianBlur;
use crate::geometry::{Coor, Vec2D};
use crate::imgproc::{fill_color, interpolate};
use crate::mat::{Mat, Mat32f};
use crate::stitch::blender::Range;
use crate::stitch::imageref::ImageRef;
use crate::utils::EPS;

#[derive(Clone, Copy, Default)]
struct WeightedPixel {
    c: Color,
    w: f32,
}

impl std::ops::AddAssign for WeightedPixel {
    fn add_assign(&mut self, o: WeightedPixel) {
        self.w += o.w;
        self.c = self.c + o.c;
    }
}

impl std::ops::Add for WeightedPixel {
    type Output = WeightedPixel;
    fn add(self, o: WeightedPixel) -> WeightedPixel {
        WeightedPixel {
            c: self.c + o.c,
            w: self.w + o.w,
        }
    }
}

impl std::ops::Mul<f32> for WeightedPixel {
    type Output = WeightedPixel;
    fn mul(self, v: f32) -> WeightedPixel {
        WeightedPixel {
            c: self.c * v,
            w: self.w * v,
        }
    }
}

struct Mask2D {
    row_stride: usize,
    data: Vec<bool>,
}

impl Mask2D {
    fn new(h: usize, w: usize) -> Self {
        let row_stride = (w + 7) / 8 * 8;
        Mask2D {
            row_stride,
            data: vec![false; h * row_stride],
        }
    }
    fn get(&self, i: usize, j: usize) -> bool {
        self.data[i * self.row_stride + j]
    }
    fn set(&mut self, i: usize, j: usize) {
        self.data[i * self.row_stride + j] = true;
    }
}

struct MetaImage {
    range: Range,
    mask: Mask2D,
}

struct ImageToBlend {
    img: Mat<WeightedPixel>,
    meta_idx: usize,
}

struct PendingImage {
    range: Range,
    imgref_ptr: *mut ImageRef,
    coor_func: Box<dyn Fn(Coor) -> Vec2D>,
}

pub struct MultiBandBlender {
    pending: Vec<PendingImage>,
    meta_images: Vec<MetaImage>,
    images: Vec<ImageToBlend>,
    next_lvl_images: Vec<ImageToBlend>,
    target_w: i32,
    target_h: i32,
    band_level: usize,
}

impl MultiBandBlender {
    pub fn new(band_level: usize) -> Self {
        MultiBandBlender {
            pending: Vec::new(),
            meta_images: Vec::new(),
            images: Vec::new(),
            next_lvl_images: Vec::new(),
            target_w: 0,
            target_h: 0,
            band_level,
        }
    }

    pub fn add_image(
        &mut self,
        upper_left: Coor,
        bottom_right: Coor,
        img: &mut ImageRef,
        coor_func: Box<dyn Fn(Coor) -> Vec2D>,
    ) {
        self.target_w = self.target_w.max(bottom_right.x);
        self.target_h = self.target_h.max(bottom_right.y);
        self.pending.push(PendingImage {
            range: Range {
                min: upper_left,
                max: bottom_right,
            },
            imgref_ptr: img as *mut ImageRef,
            coor_func,
        });
    }

    fn create_first_level(&mut self) {
        for item in self.pending.drain(..) {
            let imgref = unsafe { &mut *item.imgref_ptr };
            imgref.load();
            let iw = imgref.width as f64;
            let ih = imgref.height as f64;
            let rh = item.range.height() as usize;
            let rw = item.range.width() as usize;
            let mut wimg: Mat<WeightedPixel> = Mat::new(rh, rw, 1);
            let mut mask = Mask2D::new(rh, rw);

            for i in 0..rh {
                for j in 0..rw {
                    let tc = Coor::new(j as i32 + item.range.min.x, i as i32 + item.range.min.y);
                    let oc = (item.coor_func)(tc);
                    let color = interpolate(imgref.mat(), oc.y as f32, oc.x as f32);
                    let wp = wimg.at2_mut(i, j);
                    if color.x < 0.0 {
                        wp.w = 0.0;
                        wp.c = Color::BLACK;
                        mask.set(i, j);
                    } else {
                        wp.c = color;
                        let nx = oc.x / iw - 0.5;
                        let ny = oc.y / ih - 0.5;
                        wp.w = (0.0f32).max((0.5 - nx.abs()) as f32 * (0.5 - ny.abs()) as f32)
                            + EPS as f32;
                    }
                }
            }

            imgref.release();
            let meta_idx = self.meta_images.len();
            self.meta_images.push(MetaImage {
                range: item.range,
                mask,
            });
            self.images.push(ImageToBlend {
                img: wimg,
                meta_idx,
            });
        }
    }

    fn update_weight_map(&mut self) {
        let th = self.target_h as usize;
        let tw = self.target_w as usize;
        for i in 0..th {
            for j in 0..tw {
                let mut max_w = 0.0f32;
                let mut max_imgid = None;
                for (id, img) in self.images.iter_mut().enumerate() {
                    let meta = &self.meta_images[img.meta_idx];
                    if !meta.range.contains(i as i32, j as i32) {
                        continue;
                    }
                    let ri = i - meta.range.min.y as usize;
                    let rj = j - meta.range.min.x as usize;
                    let w = img.img.at2(ri, rj).w;
                    if w > max_w {
                        max_w = w;
                        max_imgid = Some((id, ri, rj));
                    }
                    img.img.at2_mut(ri, rj).w = 0.0;
                }
                if let Some((id, ri, rj)) = max_imgid {
                    self.images[id].img.at2_mut(ri, rj).w = 1.0;
                }
            }
        }
    }

    fn create_next_level(&mut self, level: usize) {
        let sigma = ((level * 2 + 1) as f64).sqrt() * 4.0;
        let blurer = GaussianBlur::new(sigma as f32);
        self.next_lvl_images.clear();
        for img in &self.images {
            let blurred = blurer.blur(&img.img);
            self.next_lvl_images.push(ImageToBlend {
                img: blurred,
                meta_idx: img.meta_idx,
            });
        }
    }

    pub fn run(&mut self) -> Mat32f {
        self.create_first_level();
        self.update_weight_map();

        let tw = self.target_w as usize;
        let th = self.target_h as usize;
        let mut target = Mat32f::new(th, tw, 3);
        fill_color(&mut target, Color::NO);
        let mut target_mask = Mask2D::new(th, tw);

        for img in &self.images {
            self.next_lvl_images.push(ImageToBlend {
                img: img.img.clone_data(),
                meta_idx: img.meta_idx,
            });
        }

        for level in 0..self.band_level {
            let is_last = level == self.band_level - 1;
            if !is_last {
                self.create_next_level(level);
            }

            for i in 0..th {
                for j in 0..tw {
                    let mut isum = Color::BLACK;
                    let mut wsum = 0.0f32;
                    for (imgid, img_cur) in self.images.iter().enumerate() {
                        let meta = &self.meta_images[img_cur.meta_idx];
                        if !meta.range.contains(i as i32, j as i32) {
                            continue;
                        }
                        let ri = i - meta.range.min.y as usize;
                        let rj = j - meta.range.min.x as usize;
                        if meta.mask.get(ri, rj) {
                            continue;
                        }
                        let wp = *img_cur.img.at2(ri, rj);
                        if wp.w <= 0.0 {
                            continue;
                        }

                        let c = if !is_last {
                            let img_next = &self.next_lvl_images[imgid];
                            let cn = img_next.img.at2(ri, rj).c;
                            wp.c - cn
                        } else {
                            wp.c
                        };
                        isum = isum + c * wp.w;
                        wsum += wp.w;
                    }
                    if wsum < EPS as f32 {
                        continue;
                    }
                    isum = isum * (1.0 / wsum);
                    let p = target.pixel_mut(i, j);
                    if !target_mask.get(i, j) {
                        p[0] = isum.x;
                        p[1] = isum.y;
                        p[2] = isum.z;
                        target_mask.set(i, j);
                    } else {
                        p[0] += isum.x;
                        p[1] += isum.y;
                        p[2] += isum.z;
                    }
                }
            }
            std::mem::swap(&mut self.next_lvl_images, &mut self.images);
        }

        self.images.clear();
        self.next_lvl_images.clear();

        for i in 0..th {
            for j in 0..tw {
                if target_mask.get(i, j) {
                    let p = target.pixel_mut(i, j);
                    p[0] = p[0].clamp(0.0, 1.0);
                    p[1] = p[1].clamp(0.0, 1.0);
                    p[2] = p[2].clamp(0.0, 1.0);
                }
            }
        }
        target
    }
}
