use open_pano::imgproc::interpolate;
use open_pano::mat::Mat32f;
use open_pano::stitch::stitch_transform::PrecomputedWarp;
use rayon::prelude::*;

/// Fast per-frame blend using a precomputed warp map.
/// Parallelises across output rows. No trig — only table lookups + bilinear interpolation.
pub fn apply_warp(warp: &PrecomputedWarp, images: &[Mat32f]) -> Mat32f {
    assert_eq!(images.len(), warp.cam_maps.len());

    let out_w = warp.out_w;
    let out_h = warp.out_h;
    let n_cams = warp.cam_maps.len();

    // Flat pixel buffer: -1.0 sentinel for uncovered pixels.
    let mut pixels = vec![-1.0f32; out_h * out_w * 3];

    pixels
        .par_chunks_mut(out_w * 3)
        .enumerate()
        .for_each(|(row, row_slice)| {
            let row_base = row * out_w;
            for col in 0..out_w {
                let pixel_idx = row_base + col;
                let mut r = 0.0f32;
                let mut g = 0.0f32;
                let mut b = 0.0f32;
                let mut wsum = 0.0f32;

                for cam in 0..n_cams {
                    let entry = &warp.cam_maps[cam][pixel_idx];
                    if !entry.is_valid() || entry.weight <= 0.0 {
                        continue;
                    }
                    let color =
                        interpolate(&images[cam], entry.src_y, entry.src_x);
                    if color.x < 0.0 {
                        continue;
                    }
                    r += color.x * entry.weight;
                    g += color.y * entry.weight;
                    b += color.z * entry.weight;
                    wsum += entry.weight;
                }

                if wsum > 0.0 {
                    let inv = 1.0 / wsum;
                    let base3 = col * 3;
                    row_slice[base3] = r * inv;
                    row_slice[base3 + 1] = g * inv;
                    row_slice[base3 + 2] = b * inv;
                }
            }
        });

    Mat32f::from_data(out_h, out_w, 3, pixels)
}
