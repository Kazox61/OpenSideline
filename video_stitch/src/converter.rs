use ffmpeg_next as ffmpeg;
use ffmpeg::format::Pixel;
use ffmpeg::software::scaling::{context::Context as SwsCtx, flag::Flags};
use open_pano::mat::Mat32f;

/// Decode a YUV420P (or any pixel format) ffmpeg frame into a Mat32f RGB [0,1].
pub fn frame_to_mat32f(frame: &ffmpeg::frame::Video) -> Result<Mat32f, ffmpeg::Error> {
    let w = frame.width();
    let h = frame.height();

    // Convert to packed RGB24 via ffmpeg scaler.
    let mut rgb_frame = ffmpeg::frame::Video::new(Pixel::RGB24, w, h);
    let mut sws = SwsCtx::get(frame.format(), w, h, Pixel::RGB24, w, h, Flags::BILINEAR)?;
    sws.run(frame, &mut rgb_frame)?;

    let stride = rgb_frame.stride(0);
    let data = rgb_frame.data(0);
    let mut pixels = vec![0.0f32; (h * w * 3) as usize];
    for row in 0..h as usize {
        for col in 0..w as usize {
            let src = row * stride + col * 3;
            let dst = (row * w as usize + col) * 3;
            pixels[dst]     = data[src]     as f32 / 255.0;
            pixels[dst + 1] = data[src + 1] as f32 / 255.0;
            pixels[dst + 2] = data[src + 2] as f32 / 255.0;
        }
    }
    Ok(Mat32f::from_data(h as usize, w as usize, 3, pixels))
}

/// Write a Mat32f RGB [0,1] panorama into a YUV420P ffmpeg frame (already allocated).
pub fn mat32f_to_frame(
    mat: &Mat32f,
    out: &mut ffmpeg::frame::Video,
) -> Result<(), ffmpeg::Error> {
    let w = mat.width() as u32;
    let h = mat.height() as u32;
    let src = mat.data();

    // Build packed RGB24 buffer.
    let mut rgb_buf: Vec<u8> = vec![0u8; (w * h * 3) as usize];
    for i in 0..src.len() {
        rgb_buf[i] = (src[i].clamp(0.0, 1.0) * 255.0) as u8;
    }

    let mut rgb_frame = ffmpeg::frame::Video::new(Pixel::RGB24, w, h);
    {
        let stride = rgb_frame.stride(0);
        let data = rgb_frame.data_mut(0);
        for row in 0..h as usize {
            let src_start = row * w as usize * 3;
            let dst_start = row * stride;
            data[dst_start..dst_start + w as usize * 3]
                .copy_from_slice(&rgb_buf[src_start..src_start + w as usize * 3]);
        }
    }

    let out_w = out.width();
    let out_h = out.height();
    let mut sws = SwsCtx::get(Pixel::RGB24, w, h, out.format(), out_w, out_h, Flags::BILINEAR)?;
    sws.run(&rgb_frame, out)?;
    Ok(())
}
