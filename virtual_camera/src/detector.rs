use std::path::Path;

use ffmpeg_next as ffmpeg;
use image::RgbImage;
use yolo_ort::{
    utils::image_util::{load_frame_u8, normalize_image_f32},
    yolo::{yolo_session::YoloSession, yolo_utils::nms},
};

#[derive(Debug, Clone)]
pub struct FrameTarget {
    pub tx: f32,
    pub ty: f32,
    pub spread: f32,
}

/// Trimmed-mean centroid of player foot positions.
/// Returns `None` when `foot_points` is empty.
pub fn frame_target(foot_points: &[(f32, f32)], confidences: &[f32], trim: f32) -> Option<FrameTarget> {
    if foot_points.is_empty() {
        return None;
    }

    let mut pts: Vec<(f32, f32, f32)> = foot_points
        .iter()
        .zip(confidences.iter())
        .map(|(&(x, y), &c)| (x, y, c))
        .collect();

    // trim outliers by x when we have enough points
    if pts.len() >= 5 && trim > 0.0 {
        let mut xs: Vec<f32> = pts.iter().map(|p| p.0).collect();
        xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let n = xs.len();
        let lo = xs[(trim * n as f32) as usize];
        let hi = xs[(((1.0 - trim) * n as f32) as usize).min(n - 1)];
        let filtered: Vec<_> = pts.iter().filter(|p| p.0 >= lo && p.0 <= hi).cloned().collect();
        if filtered.len() >= 3 {
            pts = filtered;
        }
    }

    let total_conf: f32 = pts.iter().map(|p| p.2).sum();
    let (tx, ty) = if total_conf > 0.0 {
        (
            pts.iter().map(|p| p.0 * p.2).sum::<f32>() / total_conf,
            pts.iter().map(|p| p.1 * p.2).sum::<f32>() / total_conf,
        )
    } else {
        let n = pts.len() as f32;
        (pts.iter().map(|p| p.0).sum::<f32>() / n, pts.iter().map(|p| p.1).sum::<f32>() / n)
    };

    let xs: Vec<f32> = pts.iter().map(|p| p.0).collect();
    let spread = if xs.len() > 1 {
        xs.iter().cloned().fold(f32::NEG_INFINITY, f32::max)
            - xs.iter().cloned().fold(f32::INFINITY, f32::min)
    } else {
        0.0
    };

    Some(FrameTarget { tx, ty, spread })
}

/// Decode `video_path`, run YOLO every `stride` frames, return per-processed-frame targets.
///
/// Returns `(targets, frame_indices, fps, panorama_size, total_frames)`.
pub fn detect_players(
    video_path: &Path,
    yolo: &mut YoloSession,
    stride: u32,
    player_class: usize,
    conf_threshold: f32,
    roi: Option<[u32; 4]>,
    on_frame: impl Fn(u32, u32),
) -> Result<(Vec<Option<FrameTarget>>, Vec<u32>, f64, [u32; 2], u32), Box<dyn std::error::Error>> {
    ffmpeg::init()?;

    let mut ictx = ffmpeg::format::input(video_path)?;
    let video_stream_idx = ictx
        .streams()
        .best(ffmpeg::media::Type::Video)
        .ok_or("no video stream")?
        .index();

    let stream = ictx.stream(video_stream_idx).unwrap();
    let _time_base = stream.time_base();
    let fps = f64::from(stream.avg_frame_rate());
    let total_frames = stream.frames() as u32;

    let codec_ctx = ffmpeg::codec::context::Context::from_parameters(stream.parameters())?;
    let mut decoder = codec_ctx.decoder().video()?;

    let panorama_size = [decoder.width(), decoder.height()];

    // Pre-scale to 640-wide in ffmpeg (SIMD-optimised). letterbox_rgb then only adds padding,
    // avoiding an expensive full-res resize in the Rust image crate.
    let pre_w: u32 = 640;
    let pre_h: u32 = ((panorama_size[1] as f32 * 640.0 / panorama_size[0] as f32).round() as u32).max(1);
    let pre_scale = panorama_size[0] as f32 / 640.0;

    let mut scaler = ffmpeg::software::scaling::context::Context::get(
        decoder.format(),
        decoder.width(),
        decoder.height(),
        ffmpeg::format::Pixel::RGB24,
        pre_w,
        pre_h,
        ffmpeg::software::scaling::flag::Flags::BILINEAR,
    )?;

    let estimated_processed = (total_frames / stride.max(1)).max(1);

    let mut targets: Vec<Option<FrameTarget>> = Vec::new();
    let mut frame_indices: Vec<u32> = Vec::new();
    let mut frame_idx: u32 = 0;

    let mut class_counts: std::collections::HashMap<usize, usize> = std::collections::HashMap::new();
    let mut total_player_dets: usize = 0;
    let mut sample_feet: Vec<(f32, f32)> = Vec::new();

    let mut decoded_video = ffmpeg::frame::Video::empty();

    for (stream, packet) in ictx.packets() {
        if stream.index() != video_stream_idx {
            continue;
        }
        decoder.send_packet(&packet)?;

        while decoder.receive_frame(&mut decoded_video).is_ok() {
            if frame_idx % stride == 0 {
                let mut rgb_frame = ffmpeg::frame::Video::empty();
                scaler.run(&decoded_video, &mut rgb_frame)?;

                let data = rgb_frame.data(0);
                let stride_bytes = rgb_frame.stride(0);
                let pw = pre_w as usize;
                let ph = pre_h as usize;

                // copy with stride alignment into a packed buffer (640×pre_h, not full panorama)
                let mut packed = vec![0u8; pw * ph * 3];
                for row in 0..ph {
                    let src_start = row * stride_bytes;
                    let dst_start = row * pw * 3;
                    packed[dst_start..dst_start + pw * 3]
                        .copy_from_slice(&data[src_start..src_start + pw * 3]);
                }

                let rgb_image = RgbImage::from_raw(pre_w, pre_h, packed)
                    .ok_or("failed to create RgbImage from frame")?;

                let loaded = load_frame_u8(&rgb_image, (640, 640));
                let normalized = normalize_image_f32(&loaded, None, None);
                let mut boxes = yolo.run_inference(normalized.image_array);
                boxes = nms(boxes, 0.45);

                let mut foot_points: Vec<(f32, f32)> = Vec::new();
                let mut confs: Vec<f32> = Vec::new();

                for bbox in &boxes {
                    *class_counts.entry(bbox.class_id).or_insert(0) += 1;

                    if bbox.class_id != player_class || bbox.probability < conf_threshold {
                        continue;
                    }
                    total_player_dets += 1;
                    // foot = bottom-center in 640×640 space → pre-scaled (640×pre_h) → panorama
                    let info = &loaded.letterbox_info;
                    let fx = ((bbox.x1 + bbox.x2) / 2.0 - info.pad_left as f32) / info.scale * pre_scale;
                    let fy = (bbox.y2 - info.pad_top as f32) / info.scale * pre_scale;

                    if sample_feet.len() < 5 {
                        sample_feet.push((fx, fy));
                    }

                    if let Some([rx0, ry0, rx1, ry1]) = roi {
                        if fx < rx0 as f32 || fx > rx1 as f32 || fy < ry0 as f32 || fy > ry1 as f32 {
                            continue;
                        }
                    }
                    foot_points.push((fx, fy));
                    confs.push(bbox.probability);
                }

                targets.push(frame_target(&foot_points, &confs, 0.1));
                frame_indices.push(frame_idx);
                on_frame(frame_indices.len() as u32, estimated_processed);
            }
            frame_idx += 1;
        }
    }
    eprintln!();

    // --- diagnostics ---
    let total_dets: usize = class_counts.values().sum();
    eprintln!("[detect_players] processed {} frames, {} total detections across {} class(es)",
        frame_indices.len(), total_dets, class_counts.len());
    let mut class_vec: Vec<(usize, usize)> = class_counts.into_iter().collect();
    class_vec.sort_by(|a, b| b.1.cmp(&a.1));
    for (cls, cnt) in class_vec.iter().take(8) {
        eprintln!("  class {:3}: {:5} detections{}",
            cls, cnt,
            if *cls == player_class { "  <-- player_class" } else { "" });
    }
    if total_player_dets > 0 {
        eprintln!("[detect_players] player_class={player_class}: {total_player_dets} detections kept");
        eprintln!("[detect_players] sample foot positions (panorama px): {sample_feet:?}");
    } else {
        eprintln!("[detect_players] WARNING: zero player detections (player_class={player_class}, conf_threshold={conf_threshold})");
        eprintln!("  → Check that model_name matches the ONNX file (yolov8 vs yolov10) and that player_class is correct");
    }

    Ok((targets, frame_indices, fps, panorama_size, total_frames))
}
