use crate::config::config;
use crate::feature::gaussian::MultiScaleGaussianBlur;
use crate::imgproc::{resize, rgb2grey};
use crate::mat::Mat32f;
use std::f32::consts::PI;

/// Fast atan2 approximation (same as C++ fast_atan)
fn fast_atan(y: f32, x: f32) -> f32 {
    let absx = x.abs();
    let absy = y.abs();
    let m = absx.max(absy);
    if m < 1e-6_f32 {
        return -PI;
    }
    let a = absx.min(absy) / m;
    let s = a * a;
    let mut r = ((-0.0464964749 * s + 0.15931422) * s - 0.327622764) * s * a + a;
    if absy > absx {
        r = std::f32::consts::FRAC_PI_2 - r;
    }
    if x < 0.0 {
        r = PI - r;
    }
    if y < 0.0 {
        r = -r;
    }
    r
}

/// One octave: nscale blurred images + their magnitude/orientation maps
pub struct GaussianPyramid {
    nscale: usize,
    pub data: Vec<Mat32f>,
    pub mag: Vec<Mat32f>,
    pub ort: Vec<Mat32f>,
    pub w: usize,
    pub h: usize,
}

impl GaussianPyramid {
    pub fn new(m: &Mat32f, num_scale: usize) -> Self {
        let cfg = config();
        let mut data = Vec::with_capacity(num_scale);
        let mag = vec![Mat32f::new(0, 0, 1); num_scale];
        let ort = vec![Mat32f::new(0, 0, 1); num_scale];
        let w = m.width();
        let h = m.height();

        let grey = if m.channels() == 3 {
            rgb2grey(m)
        } else {
            m.clone_data()
        };
        data.push(grey);

        let blurer = MultiScaleGaussianBlur::new(num_scale, cfg.gauss_sigma, cfg.scale_factor);
        for i in 1..num_scale {
            data.push(blurer.blur(&data[0], i));
        }

        let mut pyr = GaussianPyramid {
            nscale: num_scale,
            data,
            mag,
            ort,
            w,
            h,
        };
        for i in 1..num_scale {
            pyr.cal_mag_ort(i);
        }
        pyr
    }

    fn cal_mag_ort(&mut self, i: usize) {
        let orig = &self.data[i];
        let w = orig.width();
        let h = orig.height();
        let mut mag = Mat32f::new(h, w, 1);
        let mut ort = Mat32f::new(h, w, 1);
        let orig_data = orig.data().to_vec();

        for y in 0..h {
            let row_mag = mag.row_mut(y);
            let row_ort = ort.row_mut(y);

            row_mag[0] = 0.0;
            row_ort[0] = PI;

            for x in 1..w - 1 {
                if y >= 1 && y < h - 1 {
                    let dy = orig_data[(y + 1) * w + x] - orig_data[(y - 1) * w + x];
                    let dx = orig_data[y * w + x + 1] - orig_data[y * w + x - 1];
                    row_mag[x] = dx.hypot(dy);
                    row_ort[x] = fast_atan(dy, dx) + PI;
                } else {
                    row_mag[x] = 0.0;
                    row_ort[x] = PI;
                }
            }
            row_mag[w - 1] = 0.0;
            row_ort[w - 1] = PI;
        }
        self.mag[i] = mag;
        self.ort[i] = ort;
    }

    pub fn get(&self, i: usize) -> &Mat32f {
        &self.data[i]
    }
    pub fn get_mag(&self, i: usize) -> &Mat32f {
        &self.mag[i]
    }
    pub fn get_ort(&self, i: usize) -> &Mat32f {
        &self.ort[i]
    }
    pub fn get_len(&self) -> usize {
        self.nscale
    }
}

pub struct ScaleSpace {
    pub noctave: usize,
    pub nscale: usize,
    pub origw: usize,
    pub origh: usize,
    pub pyramids: Vec<GaussianPyramid>,
}

impl ScaleSpace {
    pub fn new(mat: &Mat32f, num_octave: usize, num_scale: usize) -> Self {
        let cfg = config();
        let origw = mat.width();
        let origh = mat.height();
        let mut pyramids = Vec::with_capacity(num_octave);

        for i in 0..num_octave {
            if i == 0 {
                pyramids.push(GaussianPyramid::new(mat, num_scale));
            } else {
                let factor = (cfg.scale_factor as f64).powi(-(i as i32)) as f32;
                let neww = (origw as f32 * factor).ceil() as usize;
                let newh = (origh as f32 * factor).ceil() as usize;
                assert!(neww > 5 && newh > 5);
                let mut resized = Mat32f::new(newh, neww, 3);
                resize(mat, &mut resized);
                pyramids.push(GaussianPyramid::new(&resized, num_scale));
            }
        }

        ScaleSpace {
            noctave: num_octave,
            nscale: num_scale,
            origw,
            origh,
            pyramids,
        }
    }
}

pub type Dog = Vec<Mat32f>; // difference of gaussians for one octave

pub struct DogSpace {
    pub noctave: usize,
    pub nscale: usize,
    pub origw: usize,
    pub origh: usize,
    pub dogs: Vec<Dog>,
}

impl DogSpace {
    pub fn new(ss: &ScaleSpace) -> Self {
        let noctave = ss.noctave;
        let nscale = ss.nscale;
        let mut dogs = vec![Vec::new(); noctave];

        for i in 0..noctave {
            let o = &ss.pyramids[i];
            let ns = o.get_len();
            for j in 0..ns - 1 {
                dogs[i].push(Self::diff(o.get(j), o.get(j + 1)));
            }
        }

        DogSpace {
            noctave,
            nscale,
            origw: ss.origw,
            origh: ss.origh,
            dogs,
        }
    }

    fn diff(img1: &Mat32f, img2: &Mat32f) -> Mat32f {
        let w = img1.width();
        let h = img1.height();
        assert_eq!(w, img2.width());
        assert_eq!(h, img2.height());
        let mut ret = Mat32f::new(h, w, 1);
        let d1 = img1.data();
        let d2 = img2.data();
        let dr = ret.data_mut();
        for k in 0..d1.len() {
            dr[k] = (d1[k] - d2[k]).abs();
        }
        ret
    }
}
