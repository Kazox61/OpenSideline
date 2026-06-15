use crate::color::Color;
use crate::geometry::Vec2D;
use crate::mat::{Mat32f, Matuc};
use crate::matrix::Matrix;
use crate::utils::exists_file;

pub fn read_img(fname: &str) -> Mat32f {
    if !exists_file(fname) {
        panic!("File \"{}\" does not exist!", fname);
    }
    let img = image::open(fname)
        .unwrap_or_else(|e| panic!("Failed to open image {}: {}", fname, e))
        .to_rgb32f();
    let (w, h) = (img.width() as usize, img.height() as usize);
    let raw = img.into_raw(); // Vec<f32>, interleaved RGB, row-major
    Mat32f::from_data(h, w, 3, raw)
}

pub fn read_img_uc(fname: &str) -> Matuc {
    cvt_f2uc(&read_img(fname))
}

pub fn write_rgb(fname: &str, mat: &Mat32f) {
    assert_eq!(mat.channels(), 3);
    let (h, w) = (mat.height(), mat.width());
    let mut buf = vec![0u8; h * w * 3];
    let src = mat.data();
    for i in 0..src.len() {
        let v = src[i];
        buf[i] = ((if v < 0.0 { 0.0 } else { v }) * 255.0).clamp(0.0, 255.0) as u8;
    }
    let img: image::ImageBuffer<image::Rgb<u8>, Vec<u8>> =
        image::ImageBuffer::from_raw(w as u32, h as u32, buf)
            .expect("Failed to create image buffer");
    img.save(fname)
        .unwrap_or_else(|e| panic!("Failed to write image {}: {}", fname, e));
}

pub fn hconcat(mats: &[Mat32f]) -> Mat32f {
    let wsum: usize = mats.iter().map(|m| m.width()).sum();
    let hmax = mats.iter().map(|m| m.height()).max().unwrap_or(0);
    let ch = mats[0].channels();
    let mut ret = Mat32f::new(hmax, wsum, ch);
    fill_color(&mut ret, Color::BLACK);
    let mut woff = 0;
    for m in mats {
        assert_eq!(m.channels(), ch);
        for i in 0..m.height() {
            let src = m.row(i);
            let dst_start = i * wsum * ch + woff * ch;
            let dst = ret.data_mut();
            dst[dst_start..dst_start + m.width() * ch].copy_from_slice(src);
        }
        woff += m.width();
    }
    ret
}

pub fn vconcat(mats: &[Mat32f]) -> Mat32f {
    let hsum: usize = mats.iter().map(|m| m.height()).sum();
    let wmax: usize = mats.iter().map(|m| m.width()).max().unwrap_or(0);
    let ch = mats[0].channels();
    let mut ret = Mat32f::new(hsum, wmax, ch);
    fill_color(&mut ret, Color::BLACK);
    let mut hoff = 0;
    for m in mats {
        assert_eq!(m.channels(), ch);
        for i in 0..m.height() {
            let src = m.row(i);
            let dst_start = (hoff + i) * wmax * ch;
            let dst = ret.data_mut();
            dst[dst_start..dst_start + m.width() * ch].copy_from_slice(src);
        }
        hoff += m.height();
    }
    ret
}

/// Bilinear interpolation. Returns Color::NO if any neighbor is Color::NO or out of bounds.
pub fn interpolate(mat: &Mat32f, r: f32, c: f32) -> Color {
    assert_eq!(mat.channels(), 3);
    let fr = r.floor() as isize;
    let fc = c.floor() as isize;
    if fr < 0 || fc < 0 || fc + 1 >= mat.cols() as isize || fr + 1 >= mat.rows() as isize {
        return Color::NO;
    }
    let fr = fr as usize;
    let fc = fc as usize;
    let dr = r - r.floor();
    let dc = c - c.floor();

    let p00 = mat.pixel(fr, fc);
    if p00[0] < 0.0 {
        return Color::NO;
    }
    let mut ret = Color::from_slice(p00) * ((1.0 - dr) * (1.0 - dc));

    let p10 = mat.pixel(fr + 1, fc);
    if p10[0] < 0.0 {
        return Color::NO;
    }
    ret += Color::from_slice(p10) * (dr * (1.0 - dc));

    let p11 = mat.pixel(fr + 1, fc + 1);
    if p11[0] < 0.0 {
        return Color::NO;
    }
    ret += Color::from_slice(p11) * (dr * dc);

    let p01 = mat.pixel(fr, fc + 1);
    if p01[0] < 0.0 {
        return Color::NO;
    }
    ret += Color::from_slice(p01) * ((1.0 - dr) * dc);

    ret
}

/// Bilinear interpolation for u8 mat (always succeeds if in bounds).
pub fn interpolate_uc(mat: &Matuc, r: f32, c: f32) -> Color {
    assert_eq!(mat.channels(), 3);
    let fr = r.floor() as isize;
    let fc = c.floor() as isize;
    if fr < 0 || fc < 0 || fc + 1 >= mat.cols() as isize || fr + 1 >= mat.rows() as isize {
        return Color::NO;
    }
    let fr = fr as usize;
    let fc = fc as usize;
    let dr = r - r.floor();
    let dc = c - c.floor();

    let to_f = |p: &[u8]| Color::new(p[0] as f32, p[1] as f32, p[2] as f32);
    let mut ret = to_f(mat.pixel(fr, fc)) * ((1.0 - dr) * (1.0 - dc));
    ret += to_f(mat.pixel(fr + 1, fc)) * (dr * (1.0 - dc));
    ret += to_f(mat.pixel(fr + 1, fc + 1)) * (dr * dc);
    ret += to_f(mat.pixel(fr, fc + 1)) * ((1.0 - dr) * dc);
    ret / 255.0
}

pub fn fill_color(mat: &mut Mat32f, c: Color) {
    let data = mat.data_mut();
    let n = data.len() / 3;
    for i in 0..n {
        data[i * 3] = c.x;
        data[i * 3 + 1] = c.y;
        data[i * 3 + 2] = c.z;
    }
}

pub fn fill_scalar(mat: &mut Mat32f, v: f32) {
    let data = mat.data_mut();
    data.iter_mut().for_each(|x| *x = v);
}

/// Crop the largest valid (non-NO) rectangle from the image.
pub fn crop(mat: &Mat32f) -> Mat32f {
    let w = mat.width();
    let h = mat.height();
    let mut height = vec![0i32; w];
    let mut left = vec![0usize; w];
    let mut right = vec![0usize; w];
    let mut max_area: i64 = 0;
    let (mut ll, mut rr, mut hh, mut nl) = (0usize, 0usize, 0i32, 0usize);

    for line in 0..h {
        for k in 0..w {
            let p = mat.pixel(line, k);
            let m = p[0].max(p[1]).max(p[2]);
            height[k] = if m < 0.0 { 0 } else { height[k] + 1 };
        }
        for k in 0..w {
            left[k] = k;
            while left[k] > 0 && height[k] <= height[left[k] - 1] {
                left[k] = left[left[k] - 1];
            }
        }
        for k in (0..w).rev() {
            right[k] = k;
            while right[k] < w - 1 && height[k] <= height[right[k] + 1] {
                right[k] = right[right[k] + 1];
            }
        }
        for k in 0..w {
            let area = (right[k] - left[k] + 1) as i64 * height[k] as i64;
            if area > max_area {
                max_area = area;
                ll = left[k];
                rr = right[k];
                hh = height[k];
                nl = line;
            }
        }
    }

    let out_w = rr - ll + 1;
    let out_h = hh as usize;
    let off_x = ll;
    let off_y = nl + 1 - out_h;
    let mut ret = Mat32f::new(out_h, out_w, 3);
    for i in 0..out_h {
        let src = mat.row(i + off_y);
        let src_start = off_x * 3;
        let dst = ret.row_mut(i);
        dst.copy_from_slice(&src[src_start..src_start + out_w * 3]);
    }
    ret
}

pub fn rgb2grey(mat: &Mat32f) -> Mat32f {
    assert_eq!(mat.channels(), 3);
    let mut ret = Mat32f::new(mat.height(), mat.width(), 1);
    let src = mat.data();
    let dst = ret.data_mut();
    for i in 0..mat.pixels() {
        dst[i] = (src[i * 3] + src[i * 3 + 1] + src[i * 3 + 2]) / 3.0;
    }
    ret
}

/// Bilinear resize
pub fn resize(src: &Mat32f, dst: &mut Mat32f) {
    assert!(src.rows() > 1 && src.cols() > 1);
    assert!(dst.rows() > 1 && dst.cols() > 1);
    assert_eq!(src.channels(), dst.channels());
    let ch = src.channels();
    let (dh, dw) = (dst.rows(), dst.cols());
    let (sh, sw) = (src.rows(), src.cols());

    let mut tabsx = vec![0usize; dh];
    let mut tabsy = vec![0usize; dw];
    let mut tabrx = vec![0.0f32; dh];
    let mut tabry = vec![0.0f32; dw];

    let ifx = sh as f32 / dh as f32;
    let ify = sw as f32 / dw as f32;

    for dx in 0..dh {
        let mut rx = (dx as f32 + 0.5) * ifx - 0.5;
        let mut sx = rx.floor() as isize;
        rx -= sx as f32;
        if sx < 0 {
            sx = 0;
            rx = 0.0;
        } else if sx + 1 >= sh as isize {
            sx = (sh - 2) as isize;
            rx = 1.0;
        }
        tabsx[dx] = sx as usize;
        tabrx[dx] = rx;
    }
    for dy in 0..dw {
        let mut ry = (dy as f32 + 0.5) * ify - 0.5;
        let mut sy = ry.floor() as isize;
        ry -= sy as f32;
        if sy < 0 {
            sy = 0;
            ry = 0.0;
        } else if sy + 1 >= sw as isize {
            sy = (sw - 2) as isize;
            ry = 1.0;
        }
        tabsy[dy] = sy as usize;
        tabry[dy] = ry;
    }

    let src_data = src.data();
    let dst_data = dst.data_mut();
    for dx in 0..dh {
        let rx = tabrx[dx];
        let irx = 1.0 - rx;
        let r0 = tabsx[dx];
        let r1 = r0 + 1;
        for dy in 0..dw {
            let ry = tabry[dy];
            let iry = 1.0 - ry;
            let c0 = tabsy[dy];
            let c1 = c0 + 1;
            for c in 0..ch {
                let v00 = src_data[(r0 * sw + c0) * ch + c];
                let v01 = src_data[(r0 * sw + c1) * ch + c];
                let v10 = src_data[(r1 * sw + c0) * ch + c];
                let v11 = src_data[(r1 * sw + c1) * ch + c];
                dst_data[(dx * dw + dy) * ch + c] =
                    rx * (v11 * ry + v10 * iry) + irx * (v01 * ry + v00 * iry);
            }
        }
    }
}

pub fn cvt_f2uc(mat: &Mat32f) -> Matuc {
    assert_eq!(mat.channels(), 3);
    let src = mat.data();
    let dst_data: Vec<u8> = src
        .iter()
        .map(|&v| (v * 255.0).clamp(0.0, 255.0) as u8)
        .collect();
    Matuc::from_data(mat.rows(), mat.cols(), 3, dst_data)
}

/// Compute perspective homography from p2 to p1.
pub fn get_perspective_transform(p1: &[Vec2D], p2: &[Vec2D]) -> Matrix {
    let n = p1.len();
    assert_eq!(n, p2.len());
    assert!(n >= 4);

    // Build linear system: min ||Ax - b||
    // h[8] = 1 constraint (inhomogeneous)
    let mut a_data = vec![0.0f64; n * 2 * 8];
    let mut b_data = vec![0.0f64; n * 2];

    for i in 0..n {
        let m0 = &p1[i];
        let m1 = &p2[i];
        let row = i * 8;
        a_data[row] = m1.x;
        a_data[row + 1] = m1.y;
        a_data[row + 2] = 1.0;
        a_data[row + 3] = 0.0;
        a_data[row + 4] = 0.0;
        a_data[row + 5] = 0.0;
        a_data[row + 6] = -m1.x * m0.x;
        a_data[row + 7] = -m1.y * m0.x;
        b_data[i] = m0.x;

        let row = (n + i) * 8;
        a_data[row] = 0.0;
        a_data[row + 1] = 0.0;
        a_data[row + 2] = 0.0;
        a_data[row + 3] = m1.x;
        a_data[row + 4] = m1.y;
        a_data[row + 5] = 1.0;
        a_data[row + 6] = -m1.x * m0.y;
        a_data[row + 7] = -m1.y * m0.y;
        b_data[n + i] = m0.y;
    }

    let a_mat = Matrix::from_slice(n * 2, 8, &a_data);
    let ans = crate::matrix::solve_least_squares(&a_mat, &b_data);

    let mut ret = Matrix::new(3, 3);
    for i in 0..8 {
        ret.set(i / 3, i % 3, ans[i]);
    }
    ret.set(2, 2, 1.0);
    ret
}

/// Compute affine transform from p2 to p1.
pub fn get_affine_transform(p1: &[Vec2D], p2: &[Vec2D]) -> Matrix {
    let n = p1.len();
    assert_eq!(n, p2.len());
    assert!(n >= 3);

    let mut a_data = vec![0.0f64; n * 2 * 6];
    let mut b_data = vec![0.0f64; n * 2];

    for i in 0..n {
        let m0 = &p1[i];
        let m1 = &p2[i];
        let row0 = (i * 2) * 6;
        a_data[row0] = m1.x;
        a_data[row0 + 1] = m1.y;
        a_data[row0 + 2] = 1.0;
        b_data[i * 2] = m0.x;

        let row1 = (i * 2 + 1) * 6;
        a_data[row1 + 3] = m1.x;
        a_data[row1 + 4] = m1.y;
        a_data[row1 + 5] = 1.0;
        b_data[i * 2 + 1] = m0.y;
    }

    let a_mat = Matrix::from_slice(n * 2, 6, &a_data);
    let ans = crate::matrix::solve_least_squares(&a_mat, &b_data);

    let mut ret = Matrix::new(3, 3);
    for i in 0..6 {
        ret.set(i / 3, i % 3, ans[i]);
    }
    ret.set(2, 2, 1.0);
    ret
}

// Allow Matrix::from_slice constructor (add to matrix.rs separately)
