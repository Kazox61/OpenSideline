# OpenSideline

A desktop application for generating and viewing virtual camera paths from wide-angle football video. Stitch multiple camera feeds into a panorama, then use YOLOv10 player detection to automatically produce a smooth broadcast-style virtual camera that tracks the action.

![Platform](https://img.shields.io/badge/platform-macOS-lightgrey)
![Rust](https://img.shields.io/badge/rust-2024-orange)
![Tauri](https://img.shields.io/badge/tauri-v2-blue)

![Editor screenshot](public/editor.png)

## What it does

1. **Stitch** — Align and blend multiple overlapping camera feeds (e.g. `cam0.mp4`, `cam1.mp4`, `cam2.mp4`) into a single wide-angle panorama using SIFT feature matching, homography estimation, cylindrical projection, and multi-band blending.
2. **Detect** — Runs YOLOv10 (via ONNX Runtime) on every Nth frame of the panorama video to find player positions.
3. **Track** — Computes a trimmed-mean centroid of foot positions per frame, gap-fills missing frames, and smooths the result with a spring damper.
4. **Generate** — Fits a Catmull-Rom spline through the smoothed centroids and saves a `.vcam.json` keyframe file alongside the source video.
5. **View** — Renders the result in real time using a CSS-transform crop, switchable between full-panorama and virtual-camera mode.
6. **Export** — Encodes the cropped virtual camera view as an H.264 MP4 at 1920×1080.

## Example files

The `public/` directory contains a ready-to-use example:

| File | Description |
|---|---|
| `public/cam0.mp4` | Left camera feed |
| `public/cam1.mp4` | Centre camera feed |
| `public/cam2.mp4` | Right camera feed |
| `public/panorama.mp4` | Stitched panorama produced from the three feeds |
| `public/panorama.vcam.json` | Pre-generated virtual camera path for the panorama |

You can get sample clips from the [Alfheim Dataset](https://datasets.simula.no/alfheim/) (Storås et al., Simula Research Laboratory), a publicly available multi-camera football recording dataset.

## Workspace layout

```
OpenSideline/
├── open_pano/          # Image stitching library (pure Rust, no OpenCV)
│   └── src/
│       ├── feature/             # SIFT keypoint detection & BRIEF descriptors
│       │   ├── sift.rs          # DoG scale-space + orientation
│       │   ├── brief.rs         # Binary descriptor
│       │   └── matcher.rs       # Brute-force + ratio test matching
│       └── stitch/
│           ├── stitcher.rs      # Top-level stitcher API
│           ├── homography.rs    # DLT + RANSAC homography
│           ├── camera_estimator.rs  # Focal length estimation
│           ├── bundle_adjuster.rs   # Levenberg-Marquardt refinement
│           ├── cylstitcher.rs   # Cylindrical projection
│           ├── warp.rs          # Per-pixel warp + alpha mask
│           └── multiband.rs     # Multi-band (Laplacian) blending
│
├── video_stitch/       # Video stitching: decode → stitch frames → encode
│   └── src/
│       ├── lib.rs           # stitch_videos() — progress-aware entry point
│       ├── stitcher_state.rs # Recomputes transform every N keyframes
│       ├── video_reader.rs  # ffmpeg per-stream decoder
│       ├── video_writer.rs  # ffmpeg H.264 encoder
│       ├── warp_map.rs      # Cached pixel warp map for fast apply
│       └── converter.rs     # YUV ↔ float Mat conversion
│
├── virtual_camera/     # Core library: detection, path generation, export
│   └── src/
│       ├── detector.rs          # ffmpeg decode + YOLOv10 inference per frame
│       ├── path_generator.rs    # Smooth damp → Catmull-Rom keyframes
│       ├── virtual_camera_path.rs  # VirtualCameraPath struct, save/load, bbox_at
│       ├── exporter.rs          # ffmpeg encode: crop → scale → H264 MP4
│       └── smooth_damp.rs       # Spring-damper implementation
│
├── yolo_ort/           # YOLOv10 inference wrapper (ONNX Runtime)
│
├── editor/             # Tauri desktop app
│   ├── index.html
│   ├── src/
│   │   ├── main.ts     # App logic, viewer, interpolation, playback
│   │   └── styles.css  # shadcn-style design tokens
│   └── src-tauri/
│       └── src/lib.rs  # Tauri commands + localvideo:// protocol
│
└── models/
    └── football.onnx   # YOLOv10 model
```

## Prerequisites

- **Rust** — stable toolchain (2024 edition)
- **Node.js** — v18+
- **ffmpeg** — system libraries (`libavcodec`, `libavformat`, `libswscale`) — used by `ffmpeg-next`
  - macOS: `brew install ffmpeg`
- **ONNX Runtime** — bundled by the `ort` crate (downloaded automatically on first build)
- A **YOLOv10 ONNX model** trained on football/soccer player detection, placed at `models/football.onnx`

## Running the editor

```bash
cd editor
npm install
npm run tauri dev
```

This starts the Vite dev server and the Tauri shell together.

## Building for distribution

```bash
cd editor
npm run tauri build
```

Produces a native app bundle in `target/release/bundle/`.

## Image & video stitching

### How it works

`open_pano` is a pure-Rust stitching library with no OpenCV dependency:

1. **Feature detection** — SIFT-style DoG scale-space finds keypoints; a BRIEF descriptor encodes each one.
2. **Matching** — Brute-force nearest-neighbour with Lowe's ratio test filters unreliable pairs.
3. **Homography** — DLT + RANSAC estimates the pairwise transform; a bundle adjuster minimises reprojection error globally.
4. **Projection** — Images are warped onto a cylindrical surface to keep straight lines straight across wide fields of view.
5. **Blending** — Multi-band (Laplacian pyramid) blending hides seams across exposure differences.

`video_stitch` wraps this for video: it decodes all input streams in lock-step, recomputes the warp map every N keyframes (configurable), caches it for the frames in between, then re-encodes the panorama stream with ffmpeg.

### Stitching in the editor

Open the **Stitch** panel, click **Select Files…**, and pick two or more overlapping camera recordings (e.g. `cam0.mp4`, `cam1.mp4`, `cam2.mp4`). Progress is reported per encoded frame. The output panorama is saved next to the first input file.

## The `.vcam.json` format

Generated files sit next to the source video with the extension replaced:  
`recording.mp4` → `recording.vcam.json`

```json
{
  "version": 1,
  "source": "/absolute/path/to/panorama.mp4",
  "panorama_size": [2734, 1322],
  "fps": 30.02,
  "frame_count": 1800,
  "aspect": [16, 9],
  "samples": [
    { "i": 0,   "cx": 1550.9, "cy": 541.4, "w": 1094.3, "h": 615.6 },
    { "i": 8,   "cx": 1552.3, "cy": 541.8, "w": 1110.5, "h": 624.6 },
    ...
  ]
}
```

Each sample is a keyframe: `i` is the frame index; `cx/cy` is the crop centre in panorama pixels; `w/h` is the crop size. The viewer interpolates between keyframes in JavaScript using binary search + linear interpolation.

## Editor features

| Feature | Description |
|---|---|
| **Stitch** | Select 2+ overlapping camera files, click Stitch — progress updates per encoded frame |
| **Generate** | Select a panorama video, click Generate — progress updates per detected frame |
| **Viewer — Panorama** | Full-width panorama playback with play/pause and scrubber |
| **Viewer — Virtual Camera** | Real-time CSS-transform crop that follows the camera path |
| **Mode switch** | Dropdown toggles between modes instantly while video plays |
| **Load .vcam.json** | Open a previously generated path without re-running detection |
| **Export** | Save the virtual camera view as a 1920×1080 H.264 MP4 |

## Architecture notes

**No per-frame IPC during playback.** All `.vcam.json` keyframes are loaded into the browser process once. The viewer uses `requestAnimationFrame` to interpolate the crop rect and apply it as a CSS `transform` on the `<video>` element — zero round-trips to Rust while playing.

**Local video protocol.** Rather than configuring Tauri's asset protocol, the app registers a custom `localvideo://` URI scheme that serves files with HTTP range request support. This lets the `<video>` element seek freely without loading the whole file.

**Warp map caching.** `video_stitch` computes the pixel-level warp map once per keyframe interval and reuses it for subsequent frames. This makes per-frame stitching cheap: a single map lookup pass rather than a full feature-match + homography solve every frame.

**Export pipeline.** The Rust exporter decodes the source video with ffmpeg, crops each YUV420P frame in-place with `memcpy` row copies, scales to 1920×1080 with `swscale`, and encodes with `h264_videotoolbox` (macOS) or `libx264` (fallback).
