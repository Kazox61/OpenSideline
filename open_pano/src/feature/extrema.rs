use crate::config::config;
use crate::feature::SSPoint;
use crate::feature::dog::{Dog, DogSpace};
use crate::geometry::{Coor, Vec2D, Vec3};
use crate::mat::Mat32f;
use crate::matrix::Matrix;
use crate::utils::between;

pub struct ExtremaDetector<'a> {
    dog: &'a DogSpace,
}

impl<'a> ExtremaDetector<'a> {
    pub fn new(dog: &'a DogSpace) -> Self {
        ExtremaDetector { dog }
    }

    pub fn get_raw_extrema(&self) -> Vec<Coor> {
        let mut ret = Vec::new();
        let npyramid = self.dog.noctave;
        let nscale = self.dog.nscale;
        for i in 0..npyramid {
            for j in 1..nscale - 2 {
                let now = &self.dog.dogs[i][j];
                let w = now.width();
                let h = now.height();
                let v = self.get_local_raw_extrema(i, j);
                for c in v {
                    ret.push(Coor::new(
                        (c.x as f32 / w as f32 * self.dog.origw as f32) as i32,
                        (c.y as f32 / h as f32 * self.dog.origh as f32) as i32,
                    ));
                }
            }
        }
        ret
    }

    pub fn get_extrema(&self) -> Vec<SSPoint> {
        let npyramid = self.dog.noctave;
        let nscale = self.dog.nscale;
        let mut ret = Vec::new();

        for i in 0..npyramid {
            for j in 1..nscale - 2 {
                let v = self.get_local_raw_extrema(i, j);
                for c in v {
                    let mut sp = SSPoint {
                        coor: c,
                        pyr_id: i,
                        scale_id: j,
                        real_coor: Vec2D::zero(),
                        dir: 0.0,
                        scale_factor: 0.0,
                    };
                    if !self.calc_kp_offset(&mut sp) {
                        continue;
                    }
                    let img = &self.dog.dogs[i][sp.scale_id];
                    if self.is_edge_response(sp.coor, img) {
                        continue;
                    }
                    ret.push(sp);
                }
            }
        }
        ret
    }

    fn get_local_raw_extrema(&self, pyr_id: usize, scale_id: usize) -> Vec<Coor> {
        let cfg = config();
        let now = &self.dog.dogs[pyr_id][scale_id];
        let w = now.width();
        let h = now.height();
        let mut ret = Vec::new();

        let is_extrema = |r: usize, c: usize| -> bool {
            let center = *now.at2(r, c);
            if center < cfg.pre_color_thres {
                return false;
            }

            let cmp1 = center - cfg.judge_extrema_diff_thres;
            let cmp2 = center + cfg.judge_extrema_diff_thres;
            let mut max = true;
            let mut min = true;

            // same scale neighbors
            for di in -1i32..=1 {
                for dj in -1i32..=1 {
                    if di == 0 && dj == 0 {
                        continue;
                    }
                    let nr = (r as i32 + di) as usize;
                    let nc = (c as i32 + dj) as usize;
                    let newval = *now.at2(nr, nc);
                    if newval >= cmp1 {
                        max = false;
                    }
                    if newval <= cmp2 {
                        min = false;
                    }
                    if !max && !min {
                        return false;
                    }
                }
            }

            // adjacent scales
            for ds in [-1i32, 1] {
                let nl = (scale_id as i32 + ds) as usize;
                let mat = &self.dog.dogs[pyr_id][nl];
                for di in -1i32..=1 {
                    let row_ptr = mat.row((r as i32 + di) as usize);
                    let base = (c as isize - 1) as usize;
                    for k in 0..3 {
                        let newval = row_ptr[base + k];
                        if newval >= cmp1 {
                            max = false;
                        }
                        if newval <= cmp2 {
                            min = false;
                        }
                        if !max && !min {
                            return false;
                        }
                    }
                }
            }
            true
        };

        for i in 1..h - 1 {
            for j in 1..w - 1 {
                if is_extrema(i, j) {
                    ret.push(Coor::new(j as i32, i as i32));
                }
            }
        }
        ret
    }

    fn calc_kp_offset(&self, sp: &mut SSPoint) -> bool {
        let cfg = config();
        let now_pyramid = &self.dog.dogs[sp.pyr_id];
        let now_img = &now_pyramid[sp.scale_id];
        let w = now_img.width();
        let h = now_img.height();
        let nscale = self.dog.nscale;

        let mut nowx = sp.coor.x as isize;
        let mut nowy = sp.coor.y as isize;
        let mut nows = sp.scale_id as isize;

        let mut offset = Vec3::zero();
        let mut delta = Vec3::zero();

        for _iter in 0..cfg.calc_offset_depth {
            if !between(nowx, 1, w as isize - 1)
                || !between(nowy, 1, h as isize - 1)
                || !between(nows, 1, nscale as isize - 2)
            {
                return false;
            }

            let (off, del) =
                self.calc_kp_offset_iter(now_pyramid, nowx as usize, nowy as usize, nows as usize);
            offset = off;
            delta = del;

            if offset.get_abs_max() < cfg.offset_thres as f64 {
                break;
            }
            nowx += offset.x.round() as isize;
            nowy += offset.y.round() as isize;
            nows += offset.z.round() as isize;
        }

        // Final bounds check — the loop may have updated coords in its last iteration
        // without ever reaching the guard at the top of the next one.
        if !between(nowx, 1, w as isize - 1)
            || !between(nowy, 1, h as isize - 1)
            || !between(nows, 1, nscale as isize - 2)
        {
            return false;
        }

        let dextr = offset.dot(&delta);
        let dextr =
            *now_pyramid[nows as usize].at2(nowy as usize, nowx as usize) as f64 + dextr / 2.0;
        if dextr < cfg.contrast_thres as f64 {
            return false;
        }

        sp.coor = Coor::new(nowx as i32, nowy as i32);
        sp.scale_id = nows as usize;
        sp.scale_factor = cfg.gauss_sigma
            * (cfg.scale_factor as f64).powf((nows as f64 + offset.z) / nscale as f64) as f32;
        sp.real_coor = Vec2D::new(
            (nowx as f64 + offset.x) / w as f64,
            (nowy as f64 + offset.y) / h as f64,
        );
        true
    }

    fn calc_kp_offset_iter(&self, now_pyramid: &Dog, x: usize, y: usize, s: usize) -> (Vec3, Vec3) {
        let now_scale = &now_pyramid[s];
        let d = |xi: usize, yi: usize, si: usize| *now_pyramid[si].at2(yi, xi) as f64;
        let ds = |xi: usize, yi: usize| *now_scale.at2(yi, xi) as f64;

        let val = ds(x, y);
        let delta = Vec3::new(
            (ds(x + 1, y) - ds(x - 1, y)) / 2.0,
            (ds(x, y + 1) - ds(x, y - 1)) / 2.0,
            (d(x, y, s + 1) - d(x, y, s - 1)) / 2.0,
        );

        let dxx = ds(x + 1, y) + ds(x - 1, y) - val - val;
        let dyy = ds(x, y + 1) + ds(x, y - 1) - val - val;
        let dss = d(x, y, s + 1) + d(x, y, s - 1) - val - val;
        let dxy = (ds(x + 1, y + 1) - ds(x + 1, y - 1) - ds(x - 1, y + 1) + ds(x - 1, y - 1)) / 4.0;
        let dys = (d(x, y + 1, s + 1) - d(x, y - 1, s + 1) - d(x, y + 1, s - 1)
            + d(x, y - 1, s - 1))
            / 4.0;
        let dsx = (d(x + 1, y, s + 1) - d(x - 1, y, s + 1) - d(x + 1, y, s - 1)
            + d(x - 1, y, s - 1))
            / 4.0;

        let mut m = Matrix::new(3, 3);
        m.set(0, 0, dxx);
        m.set(1, 1, dyy);
        m.set(2, 2, dss);
        m.set(0, 1, dxy);
        m.set(1, 0, dxy);
        m.set(0, 2, dsx);
        m.set(2, 0, dsx);
        m.set(1, 2, dys);
        m.set(2, 1, dys);

        let pdpx_data = [delta.x, delta.y, delta.z];
        let pdpx = Matrix::from_slice(3, 1, &pdpx_data);

        let inv = m.inverse().unwrap_or_else(|| m.pseudo_inverse());
        let prod = inv.prod(&pdpx);
        let p = prod.ptr();
        let offset = Vec3::new(p[0], p[1], p[2]);
        (offset, delta)
    }

    fn is_edge_response(&self, coor: Coor, img: &Mat32f) -> bool {
        let cfg = config();
        let x = coor.x as usize;
        let y = coor.y as usize;
        let val = *img.at2(y, x);
        let dxx = img.at2(y, x + 1) + img.at2(y, x - 1) - val - val;
        let dyy = img.at2(y + 1, x) + img.at2(y - 1, x) - val - val;
        let dxy = (img.at2(y + 1, x + 1) + img.at2(y - 1, x - 1)
            - img.at2(y + 1, x - 1)
            - img.at2(y - 1, x + 1))
            / 4.0;
        let det = dxx * dyy - dxy * dxy;
        if det <= 0.0 {
            return true;
        }
        let tr2 = (dxx + dyy) * (dxx + dyy);
        let ratio = cfg.edge_ratio;
        tr2 / det >= (ratio + 1.0) * (ratio + 1.0) / ratio
    }
}
