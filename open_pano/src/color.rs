use std::fmt;
use std::ops::{Add, AddAssign, Div, DivAssign, Mul, Sub, SubAssign};

/// RGB color with float components in [0,1].
/// Color::NO (-1,-1,-1) is the sentinel for "no color" / transparent.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Color {
    pub x: f32, // red
    pub y: f32, // green
    pub z: f32, // blue
}

impl Color {
    pub const fn new(r: f32, g: f32, b: f32) -> Self {
        Color { x: r, y: g, z: b }
    }

    pub const WHITE: Color = Color::new(1.0, 1.0, 1.0);
    pub const BLACK: Color = Color::new(0.0, 0.0, 0.0);
    pub const RED: Color = Color::new(1.0, 0.0, 0.0);
    pub const BLUE: Color = Color::new(0.0, 0.0, 1.0);
    /// Sentinel value for "no color" / transparent pixel
    pub const NO: Color = Color::new(-1.0, -1.0, -1.0);

    pub fn from_slice(p: &[f32]) -> Self {
        Color::new(p[0], p[1], p[2])
    }

    pub fn write_to(&self, p: &mut [f32]) {
        p[0] = self.x;
        p[1] = self.y;
        p[2] = self.z;
    }

    pub fn black(&self) -> bool {
        self.x.abs() < 1e-4 && self.y.abs() < 1e-4 && self.z.abs() < 1e-4
    }

    pub fn is_no(&self) -> bool {
        self.x < 0.0
    }

    pub fn normalize(&mut self) {
        let max = self.x.max(self.y).max(self.z);
        if max > 1.0 {
            self.x /= max;
            self.y /= max;
            self.z /= max;
        }
        self.x = self.x.clamp(0.0, 1.0);
        self.y = self.y.clamp(0.0, 1.0);
        self.z = self.z.clamp(0.0, 1.0);
    }

    pub fn get_min(&self) -> f32 {
        self.x.min(self.y).min(self.z)
    }
    pub fn get_max(&self) -> f32 {
        self.x.max(self.y).max(self.z)
    }
}

impl Default for Color {
    fn default() -> Self {
        Color::BLACK
    }
}

impl Add for Color {
    type Output = Color;
    fn add(self, v: Color) -> Color {
        Color::new(self.x + v.x, self.y + v.y, self.z + v.z)
    }
}
impl AddAssign for Color {
    fn add_assign(&mut self, v: Color) {
        self.x += v.x;
        self.y += v.y;
        self.z += v.z;
    }
}
impl Sub for Color {
    type Output = Color;
    fn sub(self, v: Color) -> Color {
        Color::new(self.x - v.x, self.y - v.y, self.z - v.z)
    }
}
impl SubAssign for Color {
    fn sub_assign(&mut self, v: Color) {
        self.x -= v.x;
        self.y -= v.y;
        self.z -= v.z;
    }
}
impl Mul<f32> for Color {
    type Output = Color;
    fn mul(self, p: f32) -> Color {
        Color::new(self.x * p, self.y * p, self.z * p)
    }
}
impl Mul<Color> for Color {
    type Output = Color;
    fn mul(self, c: Color) -> Color {
        Color::new(self.x * c.x, self.y * c.y, self.z * c.z)
    }
}
impl Div<f32> for Color {
    type Output = Color;
    fn div(self, p: f32) -> Color {
        Color::new(self.x / p, self.y / p, self.z / p)
    }
}
impl DivAssign<f32> for Color {
    fn div_assign(&mut self, p: f32) {
        self.x /= p;
        self.y /= p;
        self.z /= p;
    }
}

impl fmt::Display for Color {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({}, {}, {})", self.x, self.y, self.z)
    }
}
