# OpenSideline

A desktop application for generating and viewing virtual camera paths from wide-angle football video. Point it at a panorama recording, and it uses YOLOv10 player detection to automatically produce a smooth broadcast-style camera that tracks the action.

![Platform](https://img.shields.io/badge/platform-macOS-lightgrey)
![Rust](https://img.shields.io/badge/rust-2024-orange)
![Tauri](https://img.shields.io/badge/tauri-v2-blue)

## What it does

1. **Detect** — Runs YOLOv10 (via ONNX Runtime) on every Nth frame of a panorama video to find player positions.
2. **Track** — Computes a trimmed-mean centroid of foot positions per frame, gap-fills missing frames, and smooths the result with a spring damper.
3. **Generate** — Fits a Catmull-Rom spline through the smoothed centroids and saves a `.vcam.json` keyframe file alongside the source video.
4. **View** — Renders the result in real time using a CSS-transform crop, switchable between full-panorama and virtual-camera mode.
5. **Export** — Encodes the cropped virtual camera view as an H.264 MP4 at 1920×1080.

## Workspace layout

```
OpenSideline/
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

Produces a native app bundle in `editor/src-tauri/target/release/bundle/`.

## The `.vcam.json` format

Generated files sit next to the source video with the extension replaced:  
`recording.mp4` → `recording.vcam.json`

```json
{
  "version": 1,
  "source": "/absolute/path/to/recording.mp4",
  "panorama_size": [5760, 1080],
  "fps": 25.0,
  "frame_count": 3750,
  "aspect": [16, 9],
  "samples": [
    { "i": 0,  "cx": 2880.0, "cy": 540.0, "w": 1024.0, "h": 576.0 },
    { "i": 4,  "cx": 2910.5, "cy": 538.2, "w": 1020.1, "h": 573.8 },
    ...
  ]
}
```

Each sample is a keyframe: `i` is the frame index; `cx/cy` is the crop centre in panorama pixels; `w/h` is the crop size. The viewer interpolates between keyframes in JavaScript using binary search + linear interpolation.

## Editor features

| Feature | Description |
|---|---|
| **Generate** | Select a video, click Generate — progress updates per detected frame |
| **Viewer — Panorama** | Full-width panorama playback with play/pause and scrubber |
| **Viewer — Virtual Camera** | Real-time CSS-transform crop that follows the camera path |
| **Mode switch** | Dropdown toggles between modes instantly while video plays |
| **Load .vcam.json** | Open a previously generated path without re-running detection |
| **Export** | Save the virtual camera view as a 1920×1080 H.264 MP4 |

## Architecture notes

**No per-frame IPC during playback.** All `.vcam.json` keyframes are loaded into the browser process once. The viewer uses `requestAnimationFrame` to interpolate the crop rect and apply it as a CSS `transform` on the `<video>` element — zero round-trips to Rust while playing.

**Local video protocol.** Rather than configuring Tauri's asset protocol, the app registers a custom `localvideo://` URI scheme that serves files with HTTP range request support. This lets the `<video>` element seek freely without loading the whole file.

**Export pipeline.** The Rust exporter decodes the source video with ffmpeg, crops each YUV420P frame in-place with `memcpy` row copies, scales to 1920×1080 with `swscale`, and encodes with `h264_videotoolbox` (macOS) or `libx264` (fallback).
