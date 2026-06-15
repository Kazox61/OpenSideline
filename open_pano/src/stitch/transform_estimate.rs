use crate::config::config;
use crate::feature::matcher::MatchData;
use crate::geometry::Vec2D;
use crate::imgproc::{get_affine_transform, get_perspective_transform};
use crate::polygon::{PointInPolygon, polygon_area};
use crate::stitch::homography::{Homography, overlap_region};
use crate::stitch::match_info::{MatchInfo, Shape2D};
use crate::utils::sqr;

const ESTIMATE_MIN_NR_MATCH: usize = 8;

enum TransformType {
    Affine,
    Homo,
}

pub struct TransformEstimation<'a> {
    match_data: &'a MatchData,
    kp1: &'a [Vec2D],
    kp2: &'a [Vec2D],
    shape1: Shape2D,
    shape2: Shape2D,
    /// Pre-built n×3 matrix of homogeneous kp2 coords, row-major
    f2_homo_coor: Vec<[f64; 3]>,
    ransac_inlier_thres: f32,
    transform_type: TransformType,
}

impl<'a> TransformEstimation<'a> {
    pub fn new(
        match_data: &'a MatchData,
        kp1: &'a [Vec2D],
        kp2: &'a [Vec2D],
        shape1: Shape2D,
        shape2: Shape2D,
    ) -> Self {
        let cfg = config();
        let transform_type = if cfg.cylinder || cfg.trans {
            TransformType::Affine
        } else {
            TransformType::Homo
        };
        let n = match_data.size();
        let mut f2_homo_coor = Vec::with_capacity(n);
        for pair in &match_data.data {
            let p = kp2[pair.1];
            f2_homo_coor.push([p.x, p.y, 1.0]);
        }
        let ransac_inlier_thres =
            (shape1.w + shape1.h) as f32 * 0.5 / 800.0 * cfg.ransac_inlier_thres as f32;
        TransformEstimation {
            match_data,
            kp1,
            kp2,
            shape1,
            shape2,
            f2_homo_coor,
            ransac_inlier_thres,
            transform_type,
        }
    }

    pub fn get_transform(&self) -> Option<MatchInfo> {
        let nr_match_used = match self.transform_type {
            TransformType::Affine => 7,
            TransformType::Homo => 8,
        };
        let nr_match = self.match_data.size();
        if nr_match < nr_match_used {
            return None;
        }

        let mut best_inlier_cnt = -1i32;
        let mut best_transform = Homography::identity();

        for _ in 0..config().ransac_iterations {
            // random sample without replacement
            let samples = random_sample(nr_match, nr_match_used);
            let transform = self.calc_transform(&samples);
            if !transform.health() {
                continue;
            }
            let n_inlier = self.get_inliers(&transform).len() as i32;
            if n_inlier > best_inlier_cnt {
                best_inlier_cnt = n_inlier;
                best_transform = transform;
            }
        }

        let inliers = self.get_inliers(&best_transform);
        self.fill_inliers_to_matchinfo(&inliers)
    }

    fn calc_transform(&self, matches: &[usize]) -> Homography {
        let mut p1: Vec<Vec2D> = matches
            .iter()
            .map(|&i| self.kp1[self.match_data.data[i].0])
            .collect();
        let mut p2: Vec<Vec2D> = matches
            .iter()
            .map(|&i| self.kp2[self.match_data.data[i].1])
            .collect();

        let param1 = normalize_pts(&mut p1);
        let param2 = normalize_pts(&mut p2);

        let homo_mat = match self.transform_type {
            TransformType::Affine => get_affine_transform(&p1, &p2),
            TransformType::Homo => get_perspective_transform(&p1, &p2),
        };

        let t1 = Homography::from_array([
            param1.1,
            0.0,
            -param1.1 * param1.0.x,
            0.0,
            param1.1,
            -param1.1 * param1.0.y,
            0.0,
            0.0,
            1.0,
        ]);
        let t2 = Homography::from_array([
            param2.1,
            0.0,
            -param2.1 * param2.0.x,
            0.0,
            param2.1,
            -param2.1 * param2.0.y,
            0.0,
            0.0,
            1.0,
        ]);
        t1.inverse(None) * Homography::from_matrix(&homo_mat) * t2
    }

    fn get_inliers(&self, trans: &Homography) -> Vec<usize> {
        let inlier_dist_sq = sqr(self.ransac_inlier_thres as f64);
        let trans_t = trans.transpose().to_matrix();
        let n = self.match_data.size();

        // multiply f2_homo_coor (nx3) by trans_t (3x3) -> nx3
        let mut ret = Vec::new();
        for i in 0..n {
            let row = &self.f2_homo_coor[i];
            let mut out = [0.0f64; 3];
            for c in 0..3 {
                out[c] = row[0] * trans_t.at(0, c)
                    + row[1] * trans_t.at(1, c)
                    + row[2] * trans_t.at(2, c);
            }
            let idenom = 1.0 / out[2];
            let proj = Vec2D::new(out[0] * idenom, out[1] * idenom);
            let fcoor = self.kp1[self.match_data.data[i].0];
            let dist = (proj.x - fcoor.x).powi(2) + (proj.y - fcoor.y).powi(2);
            if dist < inlier_dist_sq {
                ret.push(i);
            }
        }
        ret
    }

    fn fill_inliers_to_matchinfo(&self, inliers: &[usize]) -> Option<MatchInfo> {
        if inliers.len() < ESTIMATE_MIN_NR_MATCH {
            return None;
        }

        let get_match_cnt = |poly: &[Vec2D], first: bool| -> usize {
            if poly.len() < 3 {
                return 0;
            }
            let pip = PointInPolygon::new(poly);
            self.match_data
                .data
                .iter()
                .filter(|p| {
                    let pt = if first { self.kp1[p.0] } else { self.kp2[p.1] };
                    pip.in_polygon(pt)
                })
                .count()
        };

        let get_keypoint_cnt = |poly: &[Vec2D], first: bool| -> usize {
            let pip = PointInPolygon::new(poly);
            let pts: &[Vec2D] = if first { self.kp1 } else { self.kp2 };
            pts.iter().filter(|&&p| pip.in_polygon(p)).count()
        };

        let cfg = config();
        let homo = self.calc_transform(inliers);
        let homo_mat = homo.to_matrix();
        let mut succ = false;
        let inv = homo.inverse(Some(&mut succ));
        if !succ {
            return None;
        }

        let overlap1 = overlap_region(&self.shape1, &self.shape2, &homo_mat, &inv);
        let mc1 = get_match_cnt(&overlap1, true);
        if mc1 == 0 {
            return None;
        }
        let r1m = inliers.len() as f32 / mc1 as f32;
        if r1m < cfg.inlier_in_match_ratio {
            return None;
        }
        let kp1_cnt = get_keypoint_cnt(&overlap1, true);
        if kp1_cnt == 0 {
            return None;
        }
        let r1p = inliers.len() as f32 / kp1_cnt as f32;
        if r1p < 0.01 || r1p > 1.0 {
            return None;
        }

        let inv_mat = inv.to_matrix();
        let overlap2 = overlap_region(&self.shape2, &self.shape1, &inv_mat, &homo);
        let mc2 = get_match_cnt(&overlap2, false);
        if mc2 == 0 {
            return None;
        }
        let r2m = inliers.len() as f32 / mc2 as f32;
        if r2m < cfg.inlier_in_match_ratio {
            return None;
        }
        let kp2_cnt = get_keypoint_cnt(&overlap2, false);
        if kp2_cnt == 0 {
            return None;
        }
        let r2p = inliers.len() as f32 / kp2_cnt as f32;
        if r2p < 0.01 || r2p > 1.0 {
            return None;
        }

        let confidence = (r1p + r2p) * 0.5;
        if confidence < cfg.inlier_in_points_ratio {
            return None;
        }

        let area = polygon_area(&overlap1);
        let max_area = (self.shape1.w * self.shape1.h).max(self.shape2.w * self.shape2.h) as f64;
        if area / max_area < 0.15 {
            return None;
        }

        let mut info = MatchInfo::new();
        info.confidence = confidence;
        info.homo = homo;
        info.match_pairs = inliers
            .iter()
            .map(|&idx| {
                let p = &self.match_data.data[idx];
                (self.kp1[p.0], self.kp2[p.1])
            })
            .collect();
        Some(info)
    }
}

fn normalize_pts(pts: &mut Vec<Vec2D>) -> (Vec2D, f64) {
    let n_inv = 1.0 / pts.len() as f64;
    let sqrsum: f64 = pts.iter().map(|p| (p.x * p.x + p.y * p.y) * n_inv).sum();
    let div_inv = (2.0 / sqrsum).sqrt();
    for p in pts.iter_mut() {
        p.x *= div_inv;
        p.y *= div_inv;
    }
    (Vec2D::new(0.0, 0.0), div_inv)
}

fn random_sample(n: usize, k: usize) -> Vec<usize> {
    use std::collections::HashSet;
    let mut rng = simple_rng();
    let mut selected = HashSet::new();
    let mut out = Vec::with_capacity(k);
    while out.len() < k {
        let r = (next_rand(&mut rng) as usize) % n;
        if selected.insert(r) {
            out.push(r);
        }
    }
    out
}

static RNG_STATE: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(12345);

fn simple_rng() -> u64 {
    RNG_STATE.load(std::sync::atomic::Ordering::Relaxed)
}

fn next_rand(state: &mut u64) -> u64 {
    *state ^= *state << 13;
    *state ^= *state >> 7;
    *state ^= *state << 17;
    RNG_STATE.store(*state, std::sync::atomic::Ordering::Relaxed);
    *state
}
