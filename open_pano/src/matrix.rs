use crate::geometry::Vec3;
use crate::utils::{EPS, sqr};
use nalgebra::{DMatrix, DVector};

/// Dense matrix of f64, stored row-major.
/// Mirrors the C++ Matrix class which wraps Mat<double> with channels=1.
#[derive(Clone, Debug)]
pub struct Matrix {
    rows: usize,
    cols: usize,
    data: Vec<f64>,
}

impl Matrix {
    pub fn new(rows: usize, cols: usize) -> Self {
        Matrix {
            rows,
            cols,
            data: vec![0.0; rows * cols],
        }
    }

    pub fn rows(&self) -> usize {
        self.rows
    }
    pub fn cols(&self) -> usize {
        self.cols
    }
    pub fn pixels(&self) -> usize {
        self.rows * self.cols
    }

    #[inline]
    pub fn at(&self, r: usize, c: usize) -> f64 {
        self.data[r * self.cols + c]
    }

    #[inline]
    pub fn at_mut(&mut self, r: usize, c: usize) -> &mut f64 {
        &mut self.data[r * self.cols + c]
    }

    #[inline]
    pub fn set(&mut self, r: usize, c: usize, v: f64) {
        self.data[r * self.cols + c] = v;
    }

    pub fn ptr(&self) -> &[f64] {
        &self.data
    }
    pub fn ptr_mut(&mut self) -> &mut [f64] {
        &mut self.data
    }

    pub fn row_ptr(&self, r: usize) -> &[f64] {
        &self.data[r * self.cols..(r + 1) * self.cols]
    }

    pub fn zero(&mut self) {
        self.data.iter_mut().for_each(|v| *v = 0.0);
    }

    pub fn identity(k: usize) -> Self {
        let mut m = Matrix::new(k, k);
        for i in 0..k {
            m.set(i, i, 1.0);
        }
        m
    }

    pub fn transpose(&self) -> Matrix {
        let mut ret = Matrix::new(self.cols, self.rows);
        for i in 0..self.rows {
            for j in 0..self.cols {
                ret.set(j, i, self.at(i, j));
            }
        }
        ret
    }

    pub fn prod(&self, r: &Matrix) -> Matrix {
        assert_eq!(self.cols, r.rows);
        let m1 = self.to_nalgebra();
        let m2 = r.to_nalgebra();
        let res = m1 * m2;
        Matrix::from_nalgebra(&res)
    }

    pub fn elem_prod(&self, r: &Matrix) -> Matrix {
        assert_eq!(self.rows, r.rows);
        assert_eq!(self.cols, r.cols);
        let mut ret = Matrix::new(self.rows, self.cols);
        for i in 0..self.data.len() {
            ret.data[i] = self.data[i] * r.data[i];
        }
        ret
    }

    pub fn mult(&mut self, m: f64) {
        self.data.iter_mut().for_each(|v| *v *= m);
    }

    pub fn inverse(&self) -> Option<Matrix> {
        let m = self.to_nalgebra();
        m.try_inverse().map(|inv| Matrix::from_nalgebra(&inv))
    }

    pub fn pseudo_inverse(&self) -> Matrix {
        assert!(self.rows >= self.cols);
        let m = self.to_nalgebra();
        let svd = m.svd(true, true);
        let u = svd.u.unwrap();
        let v_t = svd.v_t.unwrap();
        let mut s_inv = svd.singular_values.clone();
        for v in s_inv.iter_mut() {
            *v = if *v > EPS { 1.0 / *v } else { 0.0 };
        }
        // pseudo_inverse = V * S_inv * U^T
        let result = v_t.transpose() * DMatrix::from_diagonal(&s_inv) * u.transpose();
        Matrix::from_nalgebra(&result)
    }

    pub fn normrot(&mut self) {
        assert_eq!(self.cols, 3);
        let mut p = Vec3::new(self.at(0, 0), self.at(1, 0), self.at(2, 0));
        let mut q = Vec3::new(self.at(0, 1), self.at(1, 1), self.at(2, 1));
        let mut r = Vec3::new(self.at(0, 2), self.at(1, 2), self.at(2, 2));
        p.normalize();
        q.normalize();
        r.normalize();
        let vtmp = p.cross(&q);
        if (vtmp - r).modulus() > 1e-6 {
            r = vtmp;
        }
        self.set(0, 0, p.x);
        self.set(1, 0, p.y);
        self.set(2, 0, p.z);
        self.set(0, 1, q.x);
        self.set(1, 1, q.y);
        self.set(2, 1, q.z);
        self.set(0, 2, r.x);
        self.set(1, 2, r.y);
        self.set(2, 2, r.z);
    }

    pub fn sqrsum(&self) -> f64 {
        assert_eq!(self.cols, 1);
        self.data.iter().map(|&v| sqr(v)).sum()
    }

    pub fn col(&self, c: usize) -> Matrix {
        assert!(c < self.cols);
        let mut ret = Matrix::new(self.rows, 1);
        for j in 0..self.rows {
            ret.set(j, 0, self.at(j, c));
        }
        ret
    }

    pub fn from_slice(rows: usize, cols: usize, data: &[f64]) -> Self {
        assert_eq!(data.len(), rows * cols);
        Matrix {
            rows,
            cols,
            data: data.to_vec(),
        }
    }

    fn to_nalgebra(&self) -> DMatrix<f64> {
        DMatrix::from_row_slice(self.rows, self.cols, &self.data)
    }

    fn from_nalgebra(m: &DMatrix<f64>) -> Self {
        let rows = m.nrows();
        let cols = m.ncols();
        let mut data = vec![0.0; rows * cols];
        for i in 0..rows {
            for j in 0..cols {
                data[i * cols + j] = m[(i, j)];
            }
        }
        Matrix { rows, cols, data }
    }
}

impl std::ops::Add for Matrix {
    type Output = Matrix;
    fn add(self, r: Matrix) -> Matrix {
        assert_eq!(self.rows, r.rows);
        assert_eq!(self.cols, r.cols);
        let data: Vec<f64> = self
            .data
            .iter()
            .zip(r.data.iter())
            .map(|(a, b)| a + b)
            .collect();
        Matrix {
            rows: self.rows,
            cols: self.cols,
            data,
        }
    }
}

impl std::ops::Sub for Matrix {
    type Output = Matrix;
    fn sub(self, r: Matrix) -> Matrix {
        assert_eq!(self.rows, r.rows);
        assert_eq!(self.cols, r.cols);
        let data: Vec<f64> = self
            .data
            .iter()
            .zip(r.data.iter())
            .map(|(a, b)| a - b)
            .collect();
        Matrix {
            rows: self.rows,
            cols: self.cols,
            data,
        }
    }
}

impl std::ops::Mul for &Matrix {
    type Output = Matrix;
    fn mul(self, r: &Matrix) -> Matrix {
        self.prod(r)
    }
}

impl std::fmt::Display for Matrix {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "[{} {}]:", self.rows, self.cols)?;
        for i in 0..self.rows {
            writeln!(f)?;
            for j in 0..self.cols {
                write!(f, "{}", self.at(i, j))?;
                if j < self.cols - 1 {
                    write!(f, ", ")?;
                }
            }
        }
        Ok(())
    }
}

// Solve the least-squares problem min ||Ax - b||
// Uses Jacobi SVD (thin U, thin V)
pub fn solve_least_squares(a: &Matrix, b: &[f64]) -> Vec<f64> {
    let m = a.to_nalgebra();
    let bv = DVector::from_column_slice(b);
    let sol = m.svd(true, true).solve(&bv, EPS).unwrap();
    sol.iter().copied().collect()
}
