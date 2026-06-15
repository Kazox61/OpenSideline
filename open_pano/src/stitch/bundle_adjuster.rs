use nalgebra::{DMatrix, DVector};
use std::collections::BTreeSet;

use crate::config::config;
use crate::geometry::{Vec2D, Vec3};
use crate::stitch::camera::Camera;
use crate::stitch::homography::Homography;
use crate::stitch::match_info::MatchInfo;
use crate::utils::GEO_EPS_SQR;

const NR_PARAM: usize = 6;
const NR_TERM: usize = 2;
const LM_MAX_ITER: usize = 100;

fn camera_to_params(c: &Camera) -> [f64; 6] {
    let (rx, ry, rz) = Camera::rotation_to_angle(&c.r);
    [c.focal, c.ppx, c.ppy, rx, ry, rz]
}

fn params_to_camera(p: &[f64]) -> Camera {
    let mut c = Camera::new();
    c.focal = p[0];
    c.ppx = p[1];
    c.ppy = p[2];
    c.aspect = 1.0;
    c.r = Camera::angle_to_rotation(p[3], p[4], p[5]);
    c
}

fn cross_product_matrix(x: f64, y: f64, z: f64) -> Homography {
    Homography::from_array([0.0, -z, y, z, 0.0, -x, -y, x, 0.0])
}

fn dr_dvi(r: &Homography) -> [Homography; 3] {
    let (v0, v1, v2) = Camera::rotation_to_angle(r);
    let vsqr = v0 * v0 + v1 * v1 + v2 * v2;
    if vsqr < GEO_EPS_SQR {
        return [
            cross_product_matrix(1.0, 0.0, 0.0),
            cross_product_matrix(0.0, 1.0, 0.0),
            cross_product_matrix(0.0, 0.0, 1.0),
        ];
    }
    let cr = cross_product_matrix(v0, v1, v2);
    let v = [v0, v1, v2];
    let mut ret = [cr, cr, cr];
    for i in 0..3 {
        ret[i].mult_scalar(v[i]);
    }

    let r_data = &r.data;
    let vvec = Vec3::new(v0, v1, v2);
    let irm = [
        Vec3::new(1.0 - r_data[0], -r_data[3], -r_data[6]),
        Vec3::new(-r_data[1], 1.0 - r_data[4], -r_data[7]),
        Vec3::new(-r_data[2], -r_data[5], 1.0 - r_data[8]),
    ];
    for i in 0..3 {
        let e = vvec.cross(&irm[i]);
        ret[i] += cross_product_matrix(e.x, e.y, e.z);
    }
    for i in 0..3 {
        ret[i].mult_scalar(1.0 / vsqr);
        ret[i] = ret[i] * *r;
    }
    ret
}

// Owns its match data to avoid lifetime entanglement with cameras borrow.
struct MatchPair {
    from: usize,
    to: usize,
    points: Vec<(Vec2D, Vec2D)>,
}

struct ParamState {
    cameras: Vec<Camera>,
    params: Vec<f64>,
}

impl ParamState {
    fn from_cameras(cams: &[Camera]) -> Self {
        let params: Vec<f64> = cams
            .iter()
            .flat_map(|c| camera_to_params(c).to_vec())
            .collect();
        ParamState {
            cameras: Vec::new(),
            params,
        }
    }

    fn get_cameras(&mut self) -> &Vec<Camera> {
        if self.cameras.is_empty() {
            self.cameras = self.params.chunks(NR_PARAM).map(params_to_camera).collect();
        }
        &self.cameras
    }

    fn get_params(&mut self) -> &Vec<f64> {
        if self.params.is_empty() {
            self.params = self
                .cameras
                .iter()
                .flat_map(|c| camera_to_params(c).to_vec())
                .collect();
        }
        &self.params
    }
}

pub struct ErrorStats {
    pub residuals: Vec<f64>,
    pub max: f64,
    pub avg: f64,
}

impl ErrorStats {
    fn new(size: usize) -> Self {
        ErrorStats {
            residuals: vec![0.0; size],
            max: 0.0,
            avg: 0.0,
        }
    }

    fn update_stats(&mut self) {
        self.avg = 0.0;
        self.max = 0.0;
        if self.residuals.is_empty() {
            return;
        }
        for &e in &self.residuals {
            self.avg += e * e;
            let ae = e.abs();
            if ae > self.max {
                self.max = ae;
            }
        }
        self.avg = (self.avg / self.residuals.len() as f64).sqrt();
    }
}

pub struct IncrementalBundleAdjuster<'a> {
    result_cameras: &'a mut Vec<Camera>,
    nr_pointwise_match: usize,
    match_pairs: Vec<MatchPair>,
    match_cnt_prefix: Vec<usize>,
    identity_idx: usize,
    idx_added: BTreeSet<usize>,
    index_map: Vec<usize>,
}

impl<'a> IncrementalBundleAdjuster<'a> {
    pub fn new(cameras: &'a mut Vec<Camera>) -> Self {
        let n = cameras.len();
        IncrementalBundleAdjuster {
            result_cameras: cameras,
            nr_pointwise_match: 0,
            match_pairs: Vec::new(),
            match_cnt_prefix: Vec::new(),
            identity_idx: 0,
            idx_added: BTreeSet::new(),
            index_map: vec![0; n],
        }
    }

    pub fn set_identity_idx(&mut self, idx: usize) {
        self.identity_idx = idx;
    }

    pub fn has_matches(&self) -> bool {
        !self.idx_added.is_empty()
    }

    // Clones match point pairs so the IBA owns its data independently of
    // the MatchInfo lifetime (avoids conflict with the cameras mutable borrow).
    pub fn add_match(&mut self, i: usize, j: usize, m: &MatchInfo) {
        self.match_cnt_prefix.push(self.nr_pointwise_match);
        self.nr_pointwise_match += m.match_pairs.len();
        self.idx_added.insert(i);
        self.idx_added.insert(j);
        self.match_pairs.push(MatchPair {
            from: i,
            to: j,
            points: m.match_pairs.clone(),
        });
    }

    fn update_index_map(&mut self) {
        for (cnt, &i) in self.idx_added.iter().enumerate() {
            self.index_map[i] = cnt;
        }
    }

    pub fn optimize(&mut self) {
        if !self.has_matches() {
            return;
        }
        let cfg = config();
        self.update_index_map();
        let nr_img = self.idx_added.len();

        let mut state = {
            let cams: Vec<Camera> = self
                .idx_added
                .iter()
                .map(|&i| self.result_cameras[i].clone())
                .collect();
            ParamState::from_cameras(&cams)
        };

        let mut err_stat = self.calc_error(&mut state);
        let mut best_err = err_stat.avg;

        let mut itr = 0;
        let mut nr_non_decrease = 0;
        let idt = self.index_map[self.identity_idx];

        while itr < LM_MAX_ITER {
            let update = self.get_param_update(
                &mut state,
                &err_stat.residuals,
                cfg.lm_lambda as f32,
                nr_img,
            );
            let old_params = state.get_params().clone();

            let mut new_params = old_params.clone();
            for i in 0..new_params.len() {
                // do not update R (params 3..6) of identity image
                if i < idt * NR_PARAM + 3 || i >= idt * NR_PARAM + 6 {
                    new_params[i] -= update[i];
                }
            }
            let mut new_state = ParamState {
                cameras: Vec::new(),
                params: new_params,
            };
            err_stat = self.calc_error(&mut new_state);

            if err_stat.avg >= best_err - 1e-3 {
                nr_non_decrease += 1;
            } else {
                nr_non_decrease = 0;
                best_err = err_stat.avg;
                state = new_state;
            }
            if nr_non_decrease > 5 {
                break;
            }
            itr += 1;
        }
        let results = state.get_cameras().clone();
        for (_, (&i, cam)) in self.idx_added.iter().zip(results.iter()).enumerate() {
            self.result_cameras[i] = cam.clone();
        }
    }

    fn calc_error(&self, state: &mut ParamState) -> ErrorStats {
        let mut ret = ErrorStats::new(self.nr_pointwise_match * NR_TERM);
        let cameras = state.get_cameras().clone();

        let mut idx = 0;
        for pair in &self.match_pairs {
            let from = self.index_map[pair.from];
            let to = self.index_map[pair.to];
            let c_from = &cameras[from];
            let c_to = &cameras[to];
            let h = (c_from.k() * c_from.r) * (c_to.r_inv() * c_to.k_inv());

            for &(to_pt, from_pt) in &pair.points {
                let transformed = h.trans2d(to_pt);
                ret.residuals[idx] = from_pt.x - transformed.x;
                ret.residuals[idx + 1] = from_pt.y - transformed.y;
                idx += 2;
            }
        }
        ret.update_stats();
        ret
    }

    fn get_param_update(
        &mut self,
        state: &mut ParamState,
        residuals: &[f64],
        lambda: f32,
        nr_img: usize,
    ) -> Vec<f64> {
        let nrows = NR_TERM * self.nr_pointwise_match;
        let ncols = NR_PARAM * nr_img;

        let mut j_mat = DMatrix::<f64>::zeros(nrows, ncols);
        let mut jtj = DMatrix::<f64>::zeros(ncols, ncols);

        self.calc_jacobian_symbolic(state, &mut j_mat, &mut jtj, nr_img);

        for i in 0..nr_img * NR_PARAM {
            let add = if i % NR_PARAM >= 3 {
                lambda as f64
            } else {
                lambda as f64 / 10.0
            };
            jtj[(i, i)] += add;
        }

        let err_vec = DVector::from_column_slice(residuals);
        let b = j_mat.transpose() * err_vec;
        let solution = jtj
            .clone()
            .col_piv_qr()
            .solve(&b)
            .unwrap_or_else(|| DVector::zeros(ncols));
        solution.as_slice().to_vec()
    }

    fn calc_jacobian_symbolic(
        &self,
        state: &mut ParamState,
        j: &mut DMatrix<f64>,
        jtj: &mut DMatrix<f64>,
        _nr_img: usize,
    ) {
        j.fill(0.0);
        jtj.fill(0.0);
        let cameras = state.get_cameras().clone();

        let all_dr: Vec<[Homography; 3]> = cameras.iter().map(|c| dr_dvi(&c.r)).collect();

        let dk_focal = Homography::from_array([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0]);
        let dk_ppx = Homography::from_array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        let dk_ppy = Homography::from_array([0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);

        for (pair_idx, pair) in self.match_pairs.iter().enumerate() {
            let mut idx = self.match_cnt_prefix[pair_idx] * 2;
            let from = self.index_map[pair.from];
            let to = self.index_map[pair.to];
            let pf = from * NR_PARAM;
            let pt = to * NR_PARAM;
            let c_from = &cameras[from];
            let c_to = &cameras[to];
            let fk = c_from.k();
            let tk_inv = c_to.k_inv();
            let tr_inv = c_to.r_inv();
            let dr_from = &all_dr[from];
            let mut dr_to_t: [Homography; 3] = all_dr[to];
            for m in &mut dr_to_t {
                *m = m.transpose();
            }

            let h = (fk * c_from.r) * (tr_inv * tk_inv);

            for &(to_pt, _from_pt) in &pair.points {
                let homo = h.trans_vec(Vec3::new(to_pt.x, to_pt.y, 1.0));
                let hz_sqr_inv = 1.0 / (homo.z * homo.z);
                let hz_inv = 1.0 / homo.z;

                let drdv = |dh: Vec3| -> Vec2D {
                    Vec2D::new(
                        -dh.x * hz_inv + dh.z * homo.x * hz_sqr_inv,
                        -dh.y * hz_inv + dh.z * homo.y * hz_sqr_inv,
                    )
                };

                // d/d(from params)
                let m_fr = c_from.r * tr_inv * tk_inv;
                let u2 = m_fr.trans_vec(Vec3::new(to_pt.x, to_pt.y, 1.0));
                let mut dfrom = [Vec2D::new(0.0, 0.0); 6];
                dfrom[0] = drdv(dk_focal.trans_vec(u2));
                dfrom[1] = drdv(dk_ppx.trans_vec(u2));
                dfrom[2] = drdv(dk_ppy.trans_vec(u2));
                let u2_rot = (tr_inv * tk_inv).trans_vec(Vec3::new(to_pt.x, to_pt.y, 1.0));
                dfrom[3] = drdv((fk * dr_from[0]).trans_vec(u2_rot));
                dfrom[4] = drdv((fk * dr_from[1]).trans_vec(u2_rot));
                dfrom[5] = drdv((fk * dr_from[2]).trans_vec(u2_rot));

                // d/d(to params) via d(Kinv)/dv = -Kinv * dK/dv * Kinv
                let m_to = fk * c_from.r * tr_inv * tk_inv;
                let u2n = tk_inv.trans_vec(Vec3::new(to_pt.x, to_pt.y, 1.0));
                let u2n_neg = Vec3::new(-u2n.x, -u2n.y, -u2n.z);
                let mut dto = [Vec2D::new(0.0, 0.0); 6];
                dto[0] = drdv((m_to * dk_focal).trans_vec(u2n_neg));
                dto[1] = drdv((m_to * dk_ppx).trans_vec(u2n_neg));
                dto[2] = drdv((m_to * dk_ppy).trans_vec(u2n_neg));
                let m_to2 = fk * c_from.r;
                let u2k = tk_inv.trans_vec(Vec3::new(to_pt.x, to_pt.y, 1.0));
                dto[3] = drdv((m_to2 * dr_to_t[0]).trans_vec(u2k));
                dto[4] = drdv((m_to2 * dr_to_t[1]).trans_vec(u2k));
                dto[5] = drdv((m_to2 * dr_to_t[2]).trans_vec(u2k));

                for i in 0..6 {
                    j[(idx, pf + i)] = dfrom[i].x;
                    j[(idx, pt + i)] = dto[i].x;
                    j[(idx + 1, pf + i)] = dfrom[i].y;
                    j[(idx + 1, pt + i)] = dto[i].y;
                }

                for i in 0..6 {
                    for jj in 0..6 {
                        let val = dfrom[i].x * dto[jj].x + dfrom[i].y * dto[jj].y;
                        jtj[(pf + i, pt + jj)] += val;
                        jtj[(pt + jj, pf + i)] += val;
                    }
                    for jj in i..6 {
                        let vf = dfrom[i].x * dfrom[jj].x + dfrom[i].y * dfrom[jj].y;
                        jtj[(pf + i, pf + jj)] += vf;
                        if i != jj {
                            jtj[(pf + jj, pf + i)] += vf;
                        }

                        let vt = dto[i].x * dto[jj].x + dto[i].y * dto[jj].y;
                        jtj[(pt + i, pt + jj)] += vt;
                        if i != jj {
                            jtj[(pt + jj, pt + i)] += vt;
                        }
                    }
                }
                idx += 2;
            }
        }
    }
}
