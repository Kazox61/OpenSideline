use crate::config::{ORI_HIST_BIN_NUM, ORI_HIST_PEAK_RATIO, ORI_WINDOW_FACTOR, config};
use crate::feature::SSPoint;
use crate::feature::dog::ScaleSpace;
use crate::utils::{between, sqr_f32, update_max};
use std::f32::consts::PI;

pub struct OrientationAssign<'a> {
    ss: &'a ScaleSpace,
    points: &'a [SSPoint],
}

impl<'a> OrientationAssign<'a> {
    pub fn new(ss: &'a ScaleSpace, points: &'a [SSPoint]) -> Self {
        OrientationAssign { ss, points }
    }

    pub fn work(&self) -> Vec<SSPoint> {
        let mut ret = Vec::new();
        for p in self.points {
            let dirs = self.calc_dir(p);
            for o in dirs {
                let mut sp = p.clone();
                sp.dir = o;
                ret.push(sp);
            }
        }
        ret
    }

    fn calc_dir(&self, p: &SSPoint) -> Vec<f32> {
        let cfg = config();
        let halfipi = 0.5 / PI;
        let pyramid = &self.ss.pyramids[p.pyr_id];
        let orient_img = pyramid.get_ort(p.scale_id);
        let mag_img = pyramid.get_mag(p.scale_id);

        let gauss_weight_sigma = p.scale_factor * ORI_WINDOW_FACTOR;
        let rad = (p.scale_factor * cfg.ori_radius).round() as i32;
        let exp_denom = 2.0 * sqr_f32(gauss_weight_sigma);
        let mut hist = [0.0f32; ORI_HIST_BIN_NUM];

        for xx in -rad..rad {
            let newx = p.coor.x + xx;
            if !between(newx, 1, pyramid.w as i32 - 1) {
                continue;
            }
            for yy in -rad..rad {
                let newy = p.coor.y + yy;
                if !between(newy, 1, pyramid.h as i32 - 1) {
                    continue;
                }
                if sqr_f32(xx as f32) + sqr_f32(yy as f32) > sqr_f32(rad as f32) {
                    continue;
                }
                let orient = *orient_img.at2(newy as usize, newx as usize);
                let mut bin = (ORI_HIST_BIN_NUM as f32 * halfipi * orient).round() as i32;
                if bin == ORI_HIST_BIN_NUM as i32 {
                    bin = 0;
                }
                let bin = bin as usize;
                let weight = (-(sqr_f32(xx as f32) + sqr_f32(yy as f32)) / exp_denom).exp();
                hist[bin] += weight * mag_img.at2(newy as usize, newx as usize);
            }
        }

        // smooth histogram
        for _ in 0..cfg.ori_hist_smooth_count {
            let prev_hist = hist;
            for i in 0..ORI_HIST_BIN_NUM {
                let prev = prev_hist[if i == 0 { ORI_HIST_BIN_NUM - 1 } else { i - 1 }];
                let next = prev_hist[if i == ORI_HIST_BIN_NUM - 1 { 0 } else { i + 1 }];
                hist[i] = hist[i] * 0.5 + (prev + next) * 0.25;
            }
        }

        let mut maxbin = 0.0f32;
        for &v in &hist {
            update_max(&mut maxbin, v);
        }
        let thres = maxbin * ORI_HIST_PEAK_RATIO;

        let mut ret = Vec::new();
        for i in 0..ORI_HIST_BIN_NUM {
            let prev = hist[if i == 0 { ORI_HIST_BIN_NUM - 1 } else { i - 1 }];
            let next = hist[if i == ORI_HIST_BIN_NUM - 1 { 0 } else { i + 1 }];
            if hist[i] > thres && hist[i] > prev.max(next) {
                let mut newbin = i as f32 - 0.5 + (hist[i] - prev) / (prev + next - 2.0 * hist[i]);
                if newbin < 0.0 {
                    newbin += ORI_HIST_BIN_NUM as f32;
                } else if newbin >= ORI_HIST_BIN_NUM as f32 {
                    newbin -= ORI_HIST_BIN_NUM as f32;
                }
                let ort = newbin / ORI_HIST_BIN_NUM as f32 * 2.0 * PI;
                ret.push(ort);
            }
        }
        ret
    }
}
