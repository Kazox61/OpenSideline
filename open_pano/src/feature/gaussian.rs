use crate::config::config;
use crate::mat::Mat;
use std::ops::{Add, AddAssign, Mul};

pub struct GaussCache {
    pub kernel: Vec<f32>,
    pub kw: usize, // full kernel width
}

impl GaussCache {
    pub fn new(sigma: f32) -> Self {
        let cfg = config();
        // Mirror C++ formula: kw = ceil(0.3*(sigma/2-1)+0.8) * window_factor; if even, kw++
        let mut kw =
            (0.3 * (sigma / 2.0 - 1.0) + 0.8).ceil() as usize * cfg.gauss_window_factor as usize;
        if kw % 2 == 0 {
            kw += 1;
        }
        let center = (kw / 2) as isize;
        let exp_coeff = -1.0 / (sigma * sigma * 2.0);
        let mut kernel = vec![0.0f32; kw];
        kernel[center as usize] = 1.0;
        let mut wsum = 1.0f32;
        for i in 1..=center as usize {
            let v = (i as f32 * i as f32 * exp_coeff).exp();
            kernel[center as usize + i] = v;
            kernel[center as usize - i] = v;
            wsum += v * 2.0;
        }
        let fac = 1.0 / wsum;
        kernel.iter_mut().for_each(|v| *v *= fac);
        GaussCache { kernel, kw }
    }

    pub fn center(&self) -> usize {
        self.kw / 2
    }
}

/// 2D separable Gaussian blur.
/// T must support: Default, Copy, Add, AddAssign, Mul<f32>
pub struct GaussianBlur {
    pub sigma: f32,
    pub gcache: GaussCache,
}

impl GaussianBlur {
    pub fn new(sigma: f32) -> Self {
        GaussianBlur {
            sigma,
            gcache: GaussCache::new(sigma),
        }
    }

    pub fn blur<T>(&self, img: &Mat<T>) -> Mat<T>
    where
        T: Clone + Default + Copy + Add<Output = T> + AddAssign + Mul<f32, Output = T>,
    {
        assert_eq!(img.channels(), 1);
        let w = img.width();
        let h = img.height();
        let kw = self.gcache.kw;
        let center = self.gcache.center() as isize;
        let kernel = &self.gcache.kernel;

        let mut ret = Mat::<T>::new(h, w, 1);
        let mut tmp = vec![T::default(); center as usize * 2 + w.max(h)];

        // Apply to columns first
        let src_data = img.data();
        let ret_data = ret.data_mut();

        for j in 0..w {
            // copy column j into tmp
            for i in 0..h {
                tmp[i] = src_data[i * w + j];
            }

            // pad borders
            let v0 = tmp[0];
            let tmp_len = tmp.len();
            for i in 1..=center as usize {
                tmp[tmp_len - i] = v0;
            } // negative side - actually pad left
            // We need to pad left side: tmp[-1...-center] = v0
            // Use a shifted buffer approach instead
            let pad = center as usize;
            let mut col = vec![T::default(); h + pad * 2];
            let bv = tmp[0];
            for i in 0..pad {
                col[i] = bv;
            }
            col[pad..pad + h].copy_from_slice(&tmp[..h]);
            let bv = tmp[h - 1];
            for i in 0..pad {
                col[pad + h + i] = bv;
            }

            // convolve
            for i in 0..h {
                let mut v = T::default();
                for k in 0..kw {
                    v += col[i + k] * kernel[k];
                }
                ret_data[i * w + j] = v;
            }
        }

        // Apply to rows
        let ret_data_snap: Vec<T> = ret_data.to_vec();
        let ret_data = ret.data_mut();

        for i in 0..h {
            let pad = center as usize;
            let mut row = vec![T::default(); w + pad * 2];
            let bv = ret_data_snap[i * w];
            for k in 0..pad {
                row[k] = bv;
            }
            row[pad..pad + w].copy_from_slice(&ret_data_snap[i * w..i * w + w]);
            let bv = ret_data_snap[i * w + w - 1];
            for k in 0..pad {
                row[pad + w + k] = bv;
            }

            for j in 0..w {
                let mut v = T::default();
                for k in 0..kw {
                    v += row[j + k] * kernel[k];
                }
                ret_data[i * w + j] = v;
            }
        }

        ret
    }
}

pub struct MultiScaleGaussianBlur {
    gauss: Vec<GaussianBlur>,
}

impl MultiScaleGaussianBlur {
    pub fn new(nscale: usize, mut gauss_sigma: f32, scale_factor: f32) -> Self {
        let mut gauss = Vec::new();
        for _ in 0..nscale - 1 {
            gauss.push(GaussianBlur::new(gauss_sigma));
            gauss_sigma *= scale_factor;
        }
        MultiScaleGaussianBlur { gauss }
    }

    /// n is 1-indexed (blur level 1..nscale-1)
    pub fn blur(&self, img: &crate::mat::Mat32f, n: usize) -> crate::mat::Mat32f {
        self.gauss[n - 1].blur(img)
    }
}
