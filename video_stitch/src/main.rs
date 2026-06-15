mod converter;
mod stitcher_state;
mod video_reader;
mod video_writer;

use converter::{frame_to_mat32f, mat32f_to_frame};
use ffmpeg_next as ffmpeg;
use open_pano::config::{init_config, init_config_default};
use stitcher_state::StitcherState;
use std::sync::atomic::{AtomicBool, Ordering};
use video_reader::VideoReader;
use video_writer::VideoWriter;

// Set to false by the SIGINT handler so the main loop can finalize cleanly.
static RUNNING: AtomicBool = AtomicBool::new(true);

extern "C" fn on_sigint(_: i32) {
    RUNNING.store(false, Ordering::SeqCst);
    // Re-install so a second Ctrl+C hard-kills.
    unsafe { libc_signal(2, on_sigint as usize) };
}

#[cfg(unix)]
unsafe fn libc_signal(sig: i32, handler: usize) {
    unsafe extern "C" {
        fn signal(sig: i32, handler: usize) -> usize;
    }
    unsafe { signal(sig, handler) };
}
#[cfg(not(unix))]
unsafe fn libc_signal(_: i32, _: usize) {}

fn fps_as_f64(r: ffmpeg::Rational) -> f64 {
    if r.1 == 0 { 30.0 } else { r.0 as f64 / r.1 as f64 }
}

fn print_usage(bin: &str) {
    eprintln!(
        "Usage: {} [config.cfg] --keyframe-interval N <video1> <video2> ... -o <output.mp4>",
        bin
    );
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    ffmpeg::init()?;

    // Install Ctrl+C handler for graceful shutdown.
    unsafe { libc_signal(2, on_sigint as usize) };

    // ── Parse arguments ────────────────────────────────────────────────────────
    let raw: Vec<String> = std::env::args().collect();
    let bin = &raw[0];
    let mut args = raw[1..].iter().peekable();

    let config_path = if args.peek().map(|s| s.ends_with(".cfg")).unwrap_or(false) {
        args.next().map(|s| s.as_str())
    } else {
        None
    };

    let mut keyframe_interval: usize = 1800;
    let mut output_path = String::new();
    let mut input_paths: Vec<String> = Vec::new();

    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--keyframe-interval" | "-k" => {
                let v = args.next().ok_or("--keyframe-interval needs a value")?;
                keyframe_interval =
                    v.parse().map_err(|_| "--keyframe-interval must be an integer")?;
            }
            "-o" | "--output" => {
                output_path = args.next().ok_or("-o needs a value")?.clone();
            }
            other => input_paths.push(other.to_string()),
        }
    }

    if input_paths.len() < 2 {
        print_usage(bin);
        eprintln!("Error: need at least 2 input videos");
        std::process::exit(1);
    }
    if output_path.is_empty() {
        print_usage(bin);
        eprintln!("Error: -o <output> is required");
        std::process::exit(1);
    }

    if let Some(path) = config_path {
        init_config(path);
    } else if std::path::Path::new("config.cfg").exists() {
        init_config("config.cfg");
    } else {
        init_config_default();
    }

    // ── Open readers ───────────────────────────────────────────────────────────
    let mut readers: Vec<VideoReader> = input_paths
        .iter()
        .map(|p| VideoReader::open(p).unwrap_or_else(|e| panic!("Cannot open {}: {}", p, e)))
        .collect();

    let fps = readers[0].fps;
    let fps_f = fps_as_f64(fps);
    let total_frames = readers[0].total_frames;

    eprintln!(
        "[video_stitch] {} inputs | keyframe every {} frames | {:.3} fps | ~{} total frames",
        readers.len(), keyframe_interval, fps_f, total_frames,
    );
    eprintln!("[video_stitch] NOTE: use `cargo run --release` for 10-20× faster blending");

    // ── Main loop ──────────────────────────────────────────────────────────────
    let mut state = StitcherState::new(keyframe_interval);
    let mut writer: Option<VideoWriter> = None;
    let mut frame_idx: usize = 0;
    let loop_start = std::time::Instant::now();
    let mut last_report = std::time::Instant::now();

    loop {
        // Ctrl+C: write trailer and exit cleanly.
        if !RUNNING.load(Ordering::SeqCst) {
            eprintln!("\n[video_stitch] interrupted — writing trailer …");
            if let Some(ref mut w) = writer {
                w.finish()?;
            }
            return Ok(());
        }

        // Decode one frame from every reader.
        let mut yuv_frames = Vec::with_capacity(readers.len());
        for reader in &mut readers {
            match reader.next_frame()? {
                Some(f) => yuv_frames.push(f),
                None => {
                    eprintln!(
                        "[video_stitch] done — {} frames ({:.1}s of video)",
                        frame_idx,
                        frame_idx as f64 / fps_f,
                    );
                    if let Some(ref mut w) = writer {
                        w.finish()?;
                    }
                    return Ok(());
                }
            }
        }

        // Convert to Mat32f.
        let mats: Vec<_> = yuv_frames
            .iter()
            .map(|f| frame_to_mat32f(f).expect("frame conversion failed"))
            .collect();

        // Recompute transform at keyframes.
        if state.should_recompute(frame_idx) {
            let ok = state.compute(mats.clone());
            if !ok && !state.has_transform() {
                eprintln!(
                    "[video_stitch] ERROR: could not compute initial transform.\n\
                     Are the input videos overlapping? Try with still frames first: \
                     `cargo run -p open_pano -- config.cfg frame0.png frame1.png`"
                );
                std::process::exit(1);
            }
        }

        // Warp + blend.
        let panorama = state.apply(mats);

        // Open writer lazily — dimensions are known only after the first blend.
        if writer.is_none() {
            let pw = (panorama.width() as u32).max(2);
            let ph = (panorama.height() as u32).max(2);
            writer = Some(VideoWriter::new(&output_path, pw, ph, fps)?);
            eprintln!("[video_stitch] output {}×{} → {}", pw, ph, output_path);
        }

        let w = writer.as_mut().unwrap();
        let mut out_frame = w.alloc_frame();
        mat32f_to_frame(&panorama, &mut out_frame)?;
        w.write_frame(&mut out_frame)?;

        frame_idx += 1;

        // Progress report every 5 seconds.
        if last_report.elapsed().as_secs() >= 5 {
            let elapsed = loop_start.elapsed().as_secs_f64();
            let enc_fps = frame_idx as f64 / elapsed;
            let eta = if total_frames > 0 && enc_fps > 0.0 {
                let rem = (total_frames as usize).saturating_sub(frame_idx);
                format!("ETA {:.0}s", rem as f64 / enc_fps)
            } else {
                "ETA unknown".to_string()
            };
            eprintln!(
                "[video_stitch] frame {} | {:.2} fps encode | {:.0}s elapsed | {}",
                frame_idx, enc_fps, elapsed, eta,
            );
            last_report = std::time::Instant::now();
        }
    }
}
