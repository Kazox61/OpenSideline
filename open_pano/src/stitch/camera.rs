use crate::stitch::homography::Homography;
use crate::stitch::match_info::MatchInfo;
use crate::utils::{EPS, GEO_EPS, GEO_EPS_SQR};
use nalgebra::{Matrix3, Vector3};

pub struct Camera {
    pub focal: f64,
    pub aspect: f64,
    pub ppx: f64,
    pub ppy: f64,
    pub r: Homography,
}

impl Default for Camera {
    fn default() -> Self {
        Camera {
            focal: 1.0,
            aspect: 1.0,
            ppx: 0.0,
            ppy: 0.0,
            r: Homography::identity(),
        }
    }
}

impl Clone for Camera {
    fn clone(&self) -> Self {
        Camera {
            focal: self.focal,
            aspect: self.aspect,
            ppx: self.ppx,
            ppy: self.ppy,
            r: self.r,
        }
    }
}

impl Camera {
    pub fn new() -> Self {
        Camera::default()
    }

    pub fn k(&self) -> Homography {
        let mut ret = Homography::identity();
        ret[0] = self.focal;
        ret[2] = self.ppx;
        ret[4] = self.focal * self.aspect;
        ret[5] = self.ppy;
        ret
    }

    pub fn k_inv(&self) -> Homography {
        self.k().inverse(None)
    }

    pub fn r_inv(&self) -> Homography {
        self.r.transpose()
    }

    pub fn estimate_focal(matches: &[Vec<MatchInfo>]) -> f64 {
        let n = matches.len();
        let mut estimates: Vec<f64> = Vec::new();
        for i in 0..n {
            for j in i + 1..n {
                let m = &matches[i][j];
                if m.confidence < EPS as f32 {
                    continue;
                }
                let f = get_focal_from_matrix(&m.homo);
                estimates.push(f);
            }
        }
        let ne = estimates.len();
        if ne < (n - 1).min(3) {
            return -1.0;
        }
        estimates.sort_by(|a, b| a.partial_cmp(b).unwrap());
        if ne % 2 == 1 {
            estimates[ne / 2]
        } else {
            (estimates[ne / 2] + estimates[ne / 2 - 1]) * 0.5
        }
    }

    pub fn rotation_to_angle(r: &Homography) -> (f64, f64, f64) {
        let m = Matrix3::<f64>::from_fn(|row, col| r.data[row * 3 + col]);
        let svd = m.svd(true, true);
        let u = svd.u.unwrap();
        let vt = svd.v_t.unwrap();
        let mut rnew = u * vt;
        if rnew.determinant() < 0.0 {
            rnew *= -1.0;
        }

        let rx = rnew[(2, 1)] - rnew[(1, 2)];
        let ry = rnew[(0, 2)] - rnew[(2, 0)];
        let rz = rnew[(1, 0)] - rnew[(0, 1)];
        let s = (rx * rx + ry * ry + rz * rz).sqrt();

        if s < GEO_EPS {
            return (0.0, 0.0, 0.0);
        }

        let cos = ((rnew[(0, 0)] + rnew[(1, 1)] + rnew[(2, 2)] - 1.0) * 0.5).clamp(-1.0, 1.0);
        let theta = cos.acos();
        let mul = theta / s;
        (rx * mul, ry * mul, rz * mul)
    }

    pub fn angle_to_rotation(rx: f64, ry: f64, rz: f64) -> Homography {
        let theta_sq = rx * rx + ry * ry + rz * rz;
        if theta_sq < GEO_EPS_SQR {
            return Homography::from_array([1.0, -rz, ry, rz, 1.0, -rx, -ry, rx, 1.0]);
        }
        let theta = theta_sq.sqrt();
        let itheta = 1.0 / theta;
        let (rx, ry, rz) = (rx * itheta, ry * itheta, rz * itheta);

        let u_outp = [
            rx * rx,
            rx * ry,
            rx * rz,
            rx * ry,
            ry * ry,
            ry * rz,
            rx * rz,
            ry * rz,
            rz * rz,
        ];
        let u_crossp = [0.0, -rz, ry, rz, 0.0, -rx, -ry, rx, 0.0];

        let c = theta.cos();
        let s = theta.sin();
        let c1 = 1.0 - c;

        let mut r = Homography::identity();
        r.mult_scalar(c);
        for k in 0..9 {
            r.data[k] += c1 * u_outp[k] + s * u_crossp[k];
        }
        r
    }

    pub fn straighten(cameras: &mut Vec<Camera>) {
        let mut cov = Matrix3::<f64>::zeros();
        for cam in cameras.iter() {
            let v = Vector3::new(cam.r[0], cam.r[1], cam.r[2]);
            cov += v * v.transpose();
        }
        let svd = cov.svd(true, true);
        let v = svd.v_t.unwrap().transpose();
        let norm_y = v.column(2).into_owned();

        let mut vz = Vector3::<f64>::zeros();
        for cam in cameras.iter() {
            vz[0] += cam.r[6];
            vz[1] += cam.r[7];
            vz[2] += cam.r[8];
        }
        let mut norm_x = norm_y.cross(&vz);
        norm_x.normalize_mut();
        let norm_z = norm_x.cross(&norm_y);

        let s: f64 = cameras
            .iter()
            .map(|c| {
                let v = Vector3::new(c.r[0], c.r[1], c.r[2]);
                norm_x.dot(&v)
            })
            .sum();
        let (norm_x, norm_y) = if s < 0.0 {
            (-norm_x, -norm_y)
        } else {
            (norm_x, norm_y)
        };

        let mut r = Homography::new();
        for i in 0..3 {
            r.data[i * 3] = norm_x[i];
            r.data[i * 3 + 1] = norm_y[i];
            r.data[i * 3 + 2] = norm_z[i];
        }
        for cam in cameras.iter_mut() {
            cam.r = cam.r * r;
        }
    }
}

fn get_focal_from_matrix(h: &Homography) -> f64 {
    let d1 = h[6] * h[7];
    let d2 = (h[7] - h[6]) * (h[7] + h[6]);
    let v1 = -(h[0] * h[1] + h[3] * h[4]) / d1;
    let v2 = (h[0] * h[0] + h[3] * h[3] - h[1] * h[1] - h[4] * h[4]) / d2;
    let (v1, v2) = if v1 < v2 { (v2, v1) } else { (v1, v2) };
    let f1;
    if v1 > 0.0 && v2 > 0.0 {
        f1 = (if d1.abs() > d2.abs() { v1 } else { v2 }).sqrt();
    } else if v1 > 0.0 {
        f1 = v1.sqrt();
    } else {
        return 0.0;
    }

    let d1 = h[0] * h[3] + h[1] * h[4];
    let d2 = h[0] * h[0] + h[1] * h[1] - h[3] * h[3] - h[4] * h[4];
    let v1 = -h[2] * h[5] / d1;
    let v2 = (h[5] * h[5] - h[2] * h[2]) / d2;
    let (v1, v2) = if v1 < v2 { (v2, v1) } else { (v1, v2) };
    let f0;
    if v1 > 0.0 && v2 > 0.0 {
        f0 = (if d1.abs() > d2.abs() { v1 } else { v2 }).sqrt();
    } else if v1 > 0.0 {
        f0 = v1.sqrt();
    } else {
        return 0.0;
    }

    if f1.is_infinite() || f0.is_infinite() {
        return 0.0;
    }
    (f1 * f0).sqrt()
}
