use crate::utils::{EPS, update_max, update_min};
use std::fmt;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Geometry {
    pub w: i32,
    pub h: i32,
}

impl Geometry {
    pub fn new(w: i32, h: i32) -> Self {
        Geometry { w, h }
    }
    pub fn area(&self) -> i32 {
        self.w * self.h
    }
    pub fn ratio(&self) -> f64 {
        self.w.max(self.h) as f64 / self.w.min(self.h) as f64
    }
    pub fn contains(&self, x: i32, y: i32) -> bool {
        x >= 0 && x < self.w && y >= 0 && y < self.h
    }
}

// 3D vector (called "Vec" in C++ codebase, renamed to avoid conflict)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Vec3 { x, y, z }
    }
    pub fn zero() -> Self {
        Vec3::new(0.0, 0.0, 0.0)
    }

    pub fn from_slice(p: &[f64]) -> Self {
        Vec3::new(p[0], p[1], p[2])
    }

    pub fn sqr(&self) -> f64 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }
    pub fn modulus(&self) -> f64 {
        self.sqr().sqrt()
    }

    pub fn dot(&self, v: &Vec3) -> f64 {
        self.x * v.x + self.y * v.y + self.z * v.z
    }

    pub fn cross(&self, v: &Vec3) -> Vec3 {
        Vec3::new(
            self.y * v.z - self.z * v.y,
            self.z * v.x - self.x * v.z,
            self.x * v.y - self.y * v.x,
        )
    }

    pub fn normalize(&mut self) {
        let m = 1.0 / self.modulus();
        self.x *= m;
        self.y *= m;
        self.z *= m;
    }

    pub fn get_normalized(&self) -> Vec3 {
        let mut v = *self;
        v.normalize();
        v
    }

    pub fn is_zero(&self, threshold: f64) -> bool {
        self.x.abs() < threshold && self.y.abs() < threshold && self.z.abs() < threshold
    }

    pub fn update_min(&mut self, v: &Vec3) {
        update_min(&mut self.x, v.x);
        update_min(&mut self.y, v.y);
        update_min(&mut self.z, v.z);
    }

    pub fn update_max(&mut self, v: &Vec3) {
        update_max(&mut self.x, v.x);
        update_max(&mut self.y, v.y);
        update_max(&mut self.z, v.z);
    }

    pub fn get_max(&self) -> f64 {
        self.x.max(self.y).max(self.z)
    }
    pub fn get_min(&self) -> f64 {
        self.x.min(self.y).min(self.z)
    }
    pub fn get_abs_max(&self) -> f64 {
        self.x.abs().max(self.y.abs()).max(self.z.abs())
    }

    pub fn write_to(&self, p: &mut [f64]) {
        p[0] = self.x;
        p[1] = self.y;
        p[2] = self.z;
    }

    pub fn min_comp_abs(&self) -> f64 {
        let a = self.x.abs();
        let b = self.y.abs();
        let c = self.z.abs();
        a.min(b).min(c)
    }

    pub fn max_value() -> Vec3 {
        Vec3::new(f64::MAX, f64::MAX, f64::MAX)
    }
    pub fn infinity() -> Vec3 {
        Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY)
    }
}

impl std::ops::Add for Vec3 {
    type Output = Vec3;
    fn add(self, v: Vec3) -> Vec3 {
        Vec3::new(self.x + v.x, self.y + v.y, self.z + v.z)
    }
}
impl std::ops::AddAssign for Vec3 {
    fn add_assign(&mut self, v: Vec3) {
        self.x += v.x;
        self.y += v.y;
        self.z += v.z;
    }
}
impl std::ops::Sub for Vec3 {
    type Output = Vec3;
    fn sub(self, v: Vec3) -> Vec3 {
        Vec3::new(self.x - v.x, self.y - v.y, self.z - v.z)
    }
}
impl std::ops::SubAssign for Vec3 {
    fn sub_assign(&mut self, v: Vec3) {
        self.x -= v.x;
        self.y -= v.y;
        self.z -= v.z;
    }
}
impl std::ops::Neg for Vec3 {
    type Output = Vec3;
    fn neg(self) -> Vec3 {
        Vec3::new(-self.x, -self.y, -self.z)
    }
}
impl std::ops::Mul<f64> for Vec3 {
    type Output = Vec3;
    fn mul(self, p: f64) -> Vec3 {
        Vec3::new(self.x * p, self.y * p, self.z * p)
    }
}
impl std::ops::MulAssign<f64> for Vec3 {
    fn mul_assign(&mut self, p: f64) {
        self.x *= p;
        self.y *= p;
        self.z *= p;
    }
}
impl std::ops::Div<f64> for Vec3 {
    type Output = Vec3;
    fn div(self, p: f64) -> Vec3 {
        self * (1.0 / p)
    }
}
impl std::ops::DivAssign<f64> for Vec3 {
    fn div_assign(&mut self, p: f64) {
        *self *= 1.0 / p;
    }
}
impl fmt::Display for Vec3 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} {} {}", self.x, self.y, self.z)
    }
}

// 2D vector generic type
#[derive(Debug, Clone, Copy)]
pub struct Vec2<T> {
    pub x: T,
    pub y: T,
}

impl<T: Copy + Default> Vec2<T> {
    pub fn new(x: T, y: T) -> Self {
        Vec2 { x, y }
    }
}

impl<T: Copy + Default> Default for Vec2<T> {
    fn default() -> Self {
        Vec2 {
            x: T::default(),
            y: T::default(),
        }
    }
}

impl<T: Copy + PartialEq + std::fmt::Debug> PartialEq for Vec2<T> {
    fn eq(&self, other: &Self) -> bool {
        self.x == other.x && self.y == other.y
    }
}

// Integer 2D coordinate
pub type Coor = Vec2<i32>;

impl Coor {
    pub fn zero() -> Self {
        Vec2::new(0, 0)
    }
}

impl std::ops::Add for Coor {
    type Output = Coor;
    fn add(self, v: Coor) -> Coor {
        Coor::new(self.x + v.x, self.y + v.y)
    }
}
impl std::ops::AddAssign for Coor {
    fn add_assign(&mut self, v: Coor) {
        self.x += v.x;
        self.y += v.y;
    }
}

impl fmt::Display for Coor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} {}", self.x, self.y)
    }
}

// Double-precision 2D vector
pub type Vec2D = Vec2<f64>;

impl Vec2D {
    pub fn zero() -> Self {
        Vec2::new(0.0, 0.0)
    }

    pub fn nan() -> Self {
        Vec2::new(f64::NAN, f64::NAN)
    }
    pub fn is_nan(&self) -> bool {
        self.x.is_nan()
    }

    pub fn max_value() -> Self {
        Vec2::new(f64::MAX, f64::MAX)
    }

    pub fn dot(&self, v: &Vec2D) -> f64 {
        self.x * v.x + self.y * v.y
    }
    pub fn cross(&self, v: &Vec2D) -> f64 {
        self.x * v.y - self.y * v.x
    }

    pub fn sqr(&self) -> f64 {
        self.x * self.x + self.y * self.y
    }
    pub fn modulus(&self) -> f64 {
        self.x.hypot(self.y)
    }

    pub fn get_normalized(&self) -> Vec2D {
        let m = self.modulus();
        Vec2D::new(self.x / m, self.y / m)
    }

    pub fn normalize(&mut self) {
        let m = 1.0 / self.modulus();
        self.x *= m;
        self.y *= m;
    }

    pub fn is_zero(&self) -> bool {
        self.x.abs() < EPS && self.y.abs() < EPS
    }

    pub fn update_min(&mut self, v: &Vec2D) {
        update_min(&mut self.x, v.x);
        update_min(&mut self.y, v.y);
    }

    pub fn update_max(&mut self, v: &Vec2D) {
        update_max(&mut self.x, v.x);
        update_max(&mut self.y, v.y);
    }

    // operator! from C++: negate y component
    pub fn negate_y(self) -> Vec2D {
        Vec2D::new(self.x, -self.y)
    }
    // operator~ from C++: swap components
    pub fn swap(self) -> Vec2D {
        Vec2D::new(self.y, self.x)
    }
}

impl std::ops::Add for Vec2D {
    type Output = Vec2D;
    fn add(self, v: Vec2D) -> Vec2D {
        Vec2D::new(self.x + v.x, self.y + v.y)
    }
}
impl std::ops::AddAssign for Vec2D {
    fn add_assign(&mut self, v: Vec2D) {
        self.x += v.x;
        self.y += v.y;
    }
}
impl std::ops::Sub for Vec2D {
    type Output = Vec2D;
    fn sub(self, v: Vec2D) -> Vec2D {
        Vec2D::new(self.x - v.x, self.y - v.y)
    }
}
impl std::ops::SubAssign for Vec2D {
    fn sub_assign(&mut self, v: Vec2D) {
        self.x -= v.x;
        self.y -= v.y;
    }
}
impl std::ops::Neg for Vec2D {
    type Output = Vec2D;
    fn neg(self) -> Vec2D {
        Vec2D::new(-self.x, -self.y)
    }
}
impl std::ops::Mul<f64> for Vec2D {
    type Output = Vec2D;
    fn mul(self, f: f64) -> Vec2D {
        Vec2D::new(self.x * f, self.y * f)
    }
}
impl std::ops::MulAssign<f64> for Vec2D {
    fn mul_assign(&mut self, p: f64) {
        self.x *= p;
        self.y *= p;
    }
}
impl std::ops::Mul<Vec2D> for Vec2D {
    type Output = Vec2D;
    fn mul(self, v: Vec2D) -> Vec2D {
        Vec2D::new(self.x * v.x, self.y * v.y)
    }
}
impl std::ops::Div<f64> for Vec2D {
    type Output = Vec2D;
    fn div(self, f: f64) -> Vec2D {
        self * (1.0 / f)
    }
}
impl std::ops::Div<Vec2D> for Vec2D {
    type Output = Vec2D;
    fn div(self, v: Vec2D) -> Vec2D {
        Vec2D::new(self.x / v.x, self.y / v.y)
    }
}
impl fmt::Display for Vec2D {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} {}", self.x, self.y)
    }
}
