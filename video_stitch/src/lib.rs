mod converter;
mod stitcher_state;
mod video_reader;
mod video_writer;
mod warp_map;

use converter::{frame_to_mat32f, mat32f_to_frame};
use ffmpeg_next as ffmpeg;
use open_pano::config::init_config_default;
use stitcher_state::StitcherState;
use video_reader::VideoReader;
use video_writer::VideoWriter;

pub struct StitchProgress {
    pub frame: usize,
    pub total: usize,
}

/// Stitch multiple input videos into a single panorama video.
///
/// `progress` is called after each encoded frame with `(done, total)`.
pub fn stitch_videos(
    input_paths: &[&str],
    output_path: &str,
    keyframe_interval: usize,
    progress: impl Fn(StitchProgress),
) -> Result<(), Box<dyn std::error::Error>> {
    ffmpeg::init()?;
    init_config_default();

    let mut readers: Vec<VideoReader> = input_paths
        .iter()
        .map(|p| VideoReader::open(p).unwrap_or_else(|e| panic!("Cannot open {}: {}", p, e)))
        .collect();

    let fps = readers[0].fps;
    let fps_f = if fps.1 == 0 { 30.0 } else { fps.0 as f64 / fps.1 as f64 };
    let total_frames = readers[0].total_frames as usize;

    let _ = fps_f;

    let mut state = StitcherState::new(keyframe_interval);
    let mut writer: Option<VideoWriter> = None;
    let mut frame_idx: usize = 0;

    loop {
        let mut yuv_frames = Vec::with_capacity(readers.len());
        for reader in &mut readers {
            match reader.next_frame()? {
                Some(f) => yuv_frames.push(f),
                None => {
                    if let Some(ref mut w) = writer {
                        w.finish()?;
                    }
                    return Ok(());
                }
            }
        }

        let mats: Vec<_> = yuv_frames
            .iter()
            .map(|f| frame_to_mat32f(f).expect("frame conversion failed"))
            .collect();

        if state.should_recompute(frame_idx) {
            let ok = state.compute(mats.clone());
            if !ok && !state.has_transform() {
                return Err("Could not compute initial stitch transform — are the videos overlapping?".into());
            }
        }

        let panorama = state.apply(mats);

        if writer.is_none() {
            let pw = (panorama.width() as u32).max(2);
            let ph = (panorama.height() as u32).max(2);
            writer = Some(VideoWriter::new(output_path, pw, ph, fps)?);
        }

        let w = writer.as_mut().unwrap();
        let mut out_frame = w.alloc_frame();
        mat32f_to_frame(&panorama, &mut out_frame)?;
        w.write_frame(&mut out_frame)?;

        frame_idx += 1;
        progress(StitchProgress { frame: frame_idx, total: total_frames });
    }
}
