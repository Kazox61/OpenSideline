/// Squared Euclidean distance with early stopping.
/// Returns f32::MAX if intermediate sum exceeds now_thres.
pub fn euclidean_sqr(x: &[f32], y: &[f32], now_thres: f32) -> f32 {
    debug_assert_eq!(x.len(), y.len());
    let mut ans = 0.0f32;
    for (xi, yi) in x.iter().zip(y.iter()) {
        let d = xi - yi;
        ans += d * d;
        if ans > now_thres {
            return f32::MAX;
        }
    }
    ans
}

/// Hamming distance via bit-reinterpretation of f32 values
pub fn hamming(x: &[f32], y: &[f32]) -> i32 {
    debug_assert_eq!(x.len(), y.len());
    let mut sum = 0i32;
    for (xi, yi) in x.iter().zip(y.iter()) {
        let p1 = xi.to_bits();
        let p2 = yi.to_bits();
        sum += (p1 ^ p2).count_ones() as i32;
    }
    sum
}
