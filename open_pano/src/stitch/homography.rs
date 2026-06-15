use crate::geometry::{Vec2D, Vec3};
use crate::matrix::Matrix;
use crate::polygon::convex_hull;
use crate::stitch::match_info::Shape2D;
use nalgebra::Matrix3;

#[derive(Clone, Debug, Copy)]
pub struct Homography {
    pub data: [f64; 9],
}

impl Homography {
    pub fn new() -> Self {
        Homography { data: [0.0; 9] }
    }

    pub fn from_array(arr: [f64; 9]) -> Self {
        Homography { data: arr }
    }

    pub fn from_matrix(m: &Matrix) -> Self {
        assert_eq!(m.rows(), 3);
        assert_eq!(m.cols(), 3);
        let mut data = [0.0; 9];
        data.copy_from_slice(m.ptr());
        Homography { data }
    }

    pub fn identity() -> Self {
        let mut h = Homography::new();
        h.data[0] = 1.0;
        h.data[4] = 1.0;
        h.data[8] = 1.0;
        h
    }

    pub fn get_translation(dx: f64, dy: f64) -> Self {
        let mut h = Self::identity();
        h.data[2] = dx;
        h.data[5] = dy;
        h
    }

    pub fn transpose(&self) -> Self {
        let d = &self.data;
        Homography::from_array([d[0], d[3], d[6], d[1], d[4], d[7], d[2], d[5], d[8]])
    }

    pub fn mult_scalar(&mut self, r: f64) {
        for v in &mut self.data {
            *v *= r;
        }
    }

    /// Transform a homogeneous vector.
    pub fn trans_vec(&self, m: Vec3) -> Vec3 {
        let d = &self.data;
        Vec3::new(
            d[0] * m.x + d[1] * m.y + d[2] * m.z,
            d[3] * m.x + d[4] * m.y + d[5] * m.z,
            d[6] * m.x + d[7] * m.y + d[8] * m.z,
        )
    }

    pub fn trans_normalize(&self, m: Vec3) -> Vec2D {
        let ret = self.trans_vec(m);
        let inv = 1.0 / ret.z;
        Vec2D::new(ret.x * inv, ret.y * inv)
    }

    pub fn trans2d(&self, m: Vec2D) -> Vec2D {
        self.trans_normalize(Vec3::new(m.x, m.y, 1.0))
    }

    pub fn trans_xy(&self, x: f64, y: f64) -> Vec2D {
        self.trans2d(Vec2D::new(x, y))
    }

    pub fn zero(&mut self) {
        self.data = [0.0; 9];
    }

    pub fn inverse(&self, succ: Option<&mut bool>) -> Homography {
        let m = Matrix3::<f64>::from_row_slice(&self.data);
        match m.try_inverse() {
            Some(inv) => {
                if let Some(s) = succ {
                    *s = true;
                }
                // nalgebra stores column-major; convert to row-major
                let mut data = [0.0f64; 9];
                for r in 0..3 {
                    for c in 0..3 {
                        data[r * 3 + c] = inv[(r, c)];
                    }
                }
                Homography { data }
            }
            None => {
                if let Some(s) = succ {
                    *s = false;
                }
                Homography::new()
            }
        }
    }

    pub fn normalize(&mut self) {
        let fac: f64 = self.data.iter().map(|&v| v * v).sum::<f64>().sqrt();
        let fac = 9.0 / fac;
        for v in &mut self.data {
            *v *= fac;
        }
    }

    pub fn health(&self) -> bool {
        Self::health_arr(&self.data)
    }

    pub fn health_arr(mat: &[f64; 9]) -> bool {
        const HOMO_MAX_PERSPECTIVE: f64 = 2e-3;
        if mat[6].abs() > HOMO_MAX_PERSPECTIVE {
            return false;
        }
        if mat[7].abs() > HOMO_MAX_PERSPECTIVE {
            return false;
        }
        let x0 = Vec3::new(mat[2], mat[5], mat[8]);
        let x1 = Vec3::new(mat[1] + mat[2], mat[4] + mat[5], mat[7] + mat[8]);
        if x1.y <= x0.y {
            return false;
        }
        let x2 = Vec3::new(
            mat[0] + mat[1] + mat[2],
            mat[3] + mat[4] + mat[5],
            mat[6] + mat[7] + mat[8],
        );
        if x2.x <= x1.x {
            return false;
        }
        true
    }

    pub fn to_matrix(&self) -> Matrix {
        Matrix::from_slice(3, 3, &self.data)
    }
}

impl std::ops::Mul for Homography {
    type Output = Homography;
    fn mul(self, r: Homography) -> Homography {
        let a = Matrix3::<f64>::from_fn(|row, col| self.data[row * 3 + col]);
        let b = Matrix3::<f64>::from_fn(|row, col| r.data[row * 3 + col]);
        let c = a * b;
        let mut data = [0.0f64; 9];
        for row in 0..3 {
            for col in 0..3 {
                data[row * 3 + col] = c[(row, col)];
            }
        }
        Homography { data }
    }
}

impl std::ops::AddAssign for Homography {
    fn add_assign(&mut self, r: Homography) {
        for i in 0..9 {
            self.data[i] += r.data[i];
        }
    }
}

impl std::ops::Index<usize> for Homography {
    type Output = f64;
    fn index(&self, i: usize) -> &f64 {
        &self.data[i]
    }
}

impl std::ops::IndexMut<usize> for Homography {
    fn index_mut(&mut self, i: usize) -> &mut f64 {
        &mut self.data[i]
    }
}

impl std::fmt::Display for Homography {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let d = &self.data;
        write!(
            f,
            "[{} {} {}; {} {} {}; {} {} {}]",
            d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7], d[8]
        )
    }
}

/// Compute the overlap region polygon (in image1's half-shifted coords) given
/// homo mapping from image2 to image1, and inv mapping from image1 to image2.
pub fn overlap_region(
    shape1: &Shape2D,
    shape2: &Shape2D,
    homo: &Matrix,
    inv: &Homography,
) -> Vec<Vec2D> {
    const NR_POINT_ON_EDGE: usize = 100;
    let stepw = shape2.w as f64 / NR_POINT_ON_EDGE as f64;
    let steph = shape2.h as f64 / NR_POINT_ON_EDGE as f64;

    let mut edge_points: Vec<Vec3> = Vec::with_capacity(4 * NR_POINT_ON_EDGE);
    for i in 0..NR_POINT_ON_EDGE {
        let fi = i as f64;
        edge_points.push(Vec3::new(
            -shape2.halfw() + fi * stepw,
            -shape2.halfh(),
            1.0,
        ));
        edge_points.push(Vec3::new(-shape2.halfw() + fi * stepw, shape2.halfh(), 1.0));
        edge_points.push(Vec3::new(
            -shape2.halfw(),
            -shape2.halfh() + fi * steph,
            1.0,
        ));
        edge_points.push(Vec3::new(shape2.halfw(), -shape2.halfh() + fi * steph, 1.0));
    }

    // Build a 3×(4n) matrix and multiply by homo
    let n = edge_points.len();
    let mut ep_mat = Matrix::new(3, n);
    for (i, v) in edge_points.iter().enumerate() {
        ep_mat.set(0, i, v.x);
        ep_mat.set(1, i, v.y);
        ep_mat.set(2, i, v.z);
    }
    let transformed = homo.prod(&ep_mat);

    let mut pts2in1: Vec<Vec2D> = Vec::new();
    for i in 0..n {
        let denom = 1.0 / transformed.at(2, i);
        let pin1 = Vec2D::new(transformed.at(0, i) * denom, transformed.at(1, i) * denom);
        if shape1.shifted_in(pin1) {
            pts2in1.push(pin1);
        }
    }

    for c in shape1.shifted_corners() {
        let cin2 = inv.trans2d(c);
        if shape2.shifted_in(cin2) {
            pts2in1.push(c);
        }
    }

    convex_hull(&mut pts2in1)
}
