use std::path::Path;

use ffmpeg_next as ffmpeg;
use ffmpeg::format::Pixel;
use ffmpeg::software::scaling::{context::Context as SwsCtx, flag::Flags};

use crate::virtual_camera_path::VirtualCameraPath;

/// Export the virtual-camera crop of `input_path` to `output_path` (MP4/H264, 1920×1080).
/// `on_progress(frames_done, total_frames)` is called after every encoded frame.
pub fn export_video(
    vcam: &VirtualCameraPath,
    input_path: &Path,
    output_path: &Path,
    on_progress: impl Fn(u32, u32),
) -> Result<(), Box<dyn std::error::Error>> {
    ffmpeg::init()?;

    // ── Decoder ──────────────────────────────────────────────────────────────
    let mut ictx = ffmpeg::format::input(input_path)?;
    let video_stream_idx = ictx
        .streams()
        .best(ffmpeg::media::Type::Video)
        .ok_or("no video stream")?
        .index();

    let in_stream = ictx.stream(video_stream_idx).unwrap();
    let fps = in_stream.avg_frame_rate();
    let total_frames = vcam.frame_count;

    let dec_ctx =
        ffmpeg::codec::context::Context::from_parameters(in_stream.parameters())?;
    let mut decoder = dec_ctx.decoder().video()?;
    let src_fmt = decoder.format();
    let src_w = decoder.width();
    let src_h = decoder.height();

    drop(in_stream);

    // ── Encoder ──────────────────────────────────────────────────────────────
    const OUT_W: u32 = 1920;
    const OUT_H: u32 = 1080;
    const OUT_FMT: Pixel = Pixel::YUV420P;

    let enc_codec = ["h264_videotoolbox", "libx264"]
        .iter()
        .find_map(|n| ffmpeg::encoder::find_by_name(n))
        .or_else(|| ffmpeg::encoder::find(ffmpeg::codec::Id::H264))
        .ok_or("no H264 encoder found — install libx264 or use macOS")?;

    let mut octx = ffmpeg::format::output(output_path)?;
    let needs_global_header = octx
        .format()
        .flags()
        .contains(ffmpeg::format::Flags::GLOBAL_HEADER);

    // Add a video stream (placeholder — parameters overwritten after encoder opens)
    let out_time_base = ffmpeg::Rational::new(fps.1, fps.0);
    {
        octx.add_stream(enc_codec)?;
    }

    // Build & open the video encoder
    let mut video_enc = {
        let mut builder = ffmpeg::codec::context::Context::new_with_codec(enc_codec)
            .encoder()
            .video()?;
        builder.set_width(OUT_W);
        builder.set_height(OUT_H);
        builder.set_format(OUT_FMT);
        builder.set_time_base(out_time_base);
        builder.set_frame_rate(Some(fps));
        if needs_global_header {
            builder.set_flags(ffmpeg::codec::Flags::GLOBAL_HEADER);
        }
        builder.open_as(enc_codec)?
    };

    octx.stream_mut(0)
        .ok_or("no output stream")?
        .set_parameters(&video_enc);

    octx.write_header()?;
    let out_tb = octx.stream(0).unwrap().time_base();

    // ── Scalers (pixel-format conversion + crop/scale) ────────────────────────
    // Optional: convert decoded frame to YUV420P first if needed.
    let mut fmt_converter: Option<SwsCtx> = if src_fmt != Pixel::YUV420P {
        Some(SwsCtx::get(
            src_fmt, src_w, src_h,
            Pixel::YUV420P, src_w, src_h,
            Flags::BILINEAR,
        )?)
    } else {
        None
    };
    let mut yuv_tmp = ffmpeg::frame::Video::new(Pixel::YUV420P, src_w, src_h);

    // Crop-to-output scaler (re-created if crop size changes)
    let mut last_crop_size = (0u32, 0u32);
    let mut crop_scaler: Option<SwsCtx> = None;

    let mut crop_buf = ffmpeg::frame::Video::new(Pixel::YUV420P, 0, 0);
    let mut out_frame = ffmpeg::frame::Video::new(OUT_FMT, OUT_W, OUT_H);

    // ── Main loop ─────────────────────────────────────────────────────────────
    let mut decoded = ffmpeg::frame::Video::empty();
    let mut frame_idx: u32 = 0;
    let mut pts: i64 = 0;
    let mut pkt = ffmpeg::Packet::empty();

    for (stream, packet) in ictx.packets() {
        if stream.index() != video_stream_idx {
            continue;
        }
        decoder.send_packet(&packet)?;

        while decoder.receive_frame(&mut decoded).is_ok() {
            // Get crop rect for this frame
            let (x0, y0, x1, y1) = vcam.bbox_at(frame_idx);
            let crop_w = ((x1 - x0) & !1).max(2); // must be even for YUV420P
            let crop_h = ((y1 - y0) & !1).max(2);

            // Convert pixel format to YUV420P if necessary
            let yuv_ref = if let Some(ref mut conv) = fmt_converter {
                conv.run(&decoded, &mut yuv_tmp)?;
                &yuv_tmp
            } else {
                &decoded
            };

            // Rebuild crop buffer and scaler when crop size changes
            if last_crop_size != (crop_w, crop_h) {
                crop_buf = ffmpeg::frame::Video::new(Pixel::YUV420P, crop_w, crop_h);
                crop_scaler = Some(SwsCtx::get(
                    Pixel::YUV420P, crop_w, crop_h,
                    OUT_FMT, OUT_W, OUT_H,
                    Flags::BILINEAR,
                )?);
                last_crop_size = (crop_w, crop_h);
            }

            // Copy crop region from yuv_ref into crop_buf
            copy_yuv_crop(yuv_ref, &mut crop_buf, x0, y0, crop_w, crop_h);

            // Scale crop to output resolution
            if let Some(ref mut scaler) = crop_scaler {
                scaler.run(&crop_buf, &mut out_frame)?;
            }

            out_frame.set_pts(Some(pts));
            pts += 1;

            // Encode
            video_enc.send_frame(&out_frame)?;
            while video_enc.receive_packet(&mut pkt).is_ok() {
                pkt.rescale_ts(out_time_base, out_tb);
                pkt.set_stream(0);
                pkt.write_interleaved(&mut octx)?;
            }

            frame_idx += 1;
            on_progress(frame_idx, total_frames);
        }
    }

    // Flush encoder
    video_enc.send_eof()?;
    while video_enc.receive_packet(&mut pkt).is_ok() {
        pkt.rescale_ts(out_time_base, out_tb);
        pkt.set_stream(0);
        pkt.write_interleaved(&mut octx)?;
    }

    octx.write_trailer()?;
    Ok(())
}

/// Copy the YUV420P crop region `(x0, y0, crop_w, crop_h)` from `src` into `dst`.
/// `dst` must already be allocated at `(crop_w, crop_h)`.
fn copy_yuv_crop(
    src: &ffmpeg::frame::Video,
    dst: &mut ffmpeg::frame::Video,
    x0: u32,
    y0: u32,
    crop_w: u32,
    crop_h: u32,
) {
    // Y plane
    {
        let ss = src.stride(0);
        let ds = dst.stride(0);
        let sd = src.data(0);
        let dd = dst.data_mut(0);
        for row in 0..crop_h as usize {
            let s = (y0 as usize + row) * ss + x0 as usize;
            let d = row * ds;
            dd[d..d + crop_w as usize].copy_from_slice(&sd[s..s + crop_w as usize]);
        }
    }
    // U plane (half resolution)
    {
        let hx = (x0 / 2) as usize;
        let hy = (y0 / 2) as usize;
        let hw = (crop_w / 2) as usize;
        let hh = (crop_h / 2) as usize;
        let ss = src.stride(1);
        let ds = dst.stride(1);
        let sd = src.data(1);
        let dd = dst.data_mut(1);
        for row in 0..hh {
            let s = (hy + row) * ss + hx;
            let d = row * ds;
            dd[d..d + hw].copy_from_slice(&sd[s..s + hw]);
        }
    }
    // V plane (half resolution)
    {
        let hx = (x0 / 2) as usize;
        let hy = (y0 / 2) as usize;
        let hw = (crop_w / 2) as usize;
        let hh = (crop_h / 2) as usize;
        let ss = src.stride(2);
        let ds = dst.stride(2);
        let sd = src.data(2);
        let dd = dst.data_mut(2);
        for row in 0..hh {
            let s = (hy + row) * ss + hx;
            let d = row * ds;
            dd[d..d + hw].copy_from_slice(&sd[s..s + hw]);
        }
    }
}
