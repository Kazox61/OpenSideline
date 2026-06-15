pub const EPS: f64 = 1e-6;
pub const GEO_EPS_SQR: f64 = 1e-14;
pub const GEO_EPS: f64 = 1e-7;

#[inline]
pub fn sqr_f32(x: f32) -> f32 {
    x * x
}

#[inline]
pub fn sqr(x: f64) -> f64 {
    x * x
}

#[inline]
pub fn update_min<T: PartialOrd>(dest: &mut T, val: T) -> bool {
    if val < *dest {
        *dest = val;
        true
    } else {
        false
    }
}

#[inline]
pub fn update_max<T: PartialOrd>(dest: &mut T, val: T) -> bool {
    if *dest < val {
        *dest = val;
        true
    } else {
        false
    }
}

// between(a, b, c) means a >= b && a <= c-1, i.e. b <= a < c
#[inline]
pub fn between<T: PartialOrd>(a: T, b: T, c: T) -> bool {
    a >= b && a < c
}

pub fn exists_file(path: &str) -> bool {
    std::path::Path::new(path).exists()
}

pub fn endswith(s: &str, suffix: &str) -> bool {
    s.ends_with(suffix)
}
