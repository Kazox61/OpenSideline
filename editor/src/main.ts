import { invoke } from "@tauri-apps/api/core";
import { listen, UnlistenFn } from "@tauri-apps/api/event";
import { open, save } from "@tauri-apps/plugin-dialog";

// ── types ─────────────────────────────────────────────────────────────────────

interface TaskProgress {
  percentage: number;
  step: string;
}

interface VirtualCameraSample {
  i: number;
  cx: number;
  cy: number;
  w: number;
  h: number;
}

interface VirtualCameraData {
  source: string;
  panorama_size: [number, number];
  fps: number;
  frame_count: number;
  aspect: [number, number];
  samples: VirtualCameraSample[];
}

// ── generation elements ───────────────────────────────────────────────────────

let startButton: HTMLButtonElement | null;
let progressContainer: HTMLDivElement | null;
let bar: HTMLDivElement | null;
let progressText: HTMLSpanElement | null;
let logContainer: HTMLDivElement | null;
let videoPathInput: HTMLInputElement | null;
let videoSelectButton: HTMLButtonElement | null;
let outputParagraph: HTMLPreElement | null;
let loadVcamButton: HTMLButtonElement | null;

// ── viewer elements ───────────────────────────────────────────────────────────

let viewerSection: HTMLDivElement | null;
let viewerModeSelect: HTMLSelectElement | null;
let timeDisplay: HTMLSpanElement | null;
let videoWrapper: HTMLDivElement | null;
let video: HTMLVideoElement | null;
let playPauseButton: HTMLButtonElement | null;
let scrubber: HTMLInputElement | null;
let exportButton: HTMLButtonElement | null;
let exportContainer: HTMLDivElement | null;
let exportBar: HTMLDivElement | null;
let exportText: HTMLSpanElement | null;

// ── viewer state ──────────────────────────────────────────────────────────────

let vcamData: VirtualCameraData | null = null;
let viewerMode: "panorama" | "virtual" = "panorama";
let animFrameId: number | null = null;
let isScrubbing = false;

// ── viewer logic ──────────────────────────────────────────────────────────────

function localVideoUrl(filePath: string): string {
  return (
    "localvideo://localhost" +
    filePath
      .split("/")
      .map(encodeURIComponent)
      .join("/")
  );
}

function interpolateSample(
  samples: VirtualCameraSample[],
  frameIdx: number,
): VirtualCameraSample {
  if (samples.length === 0) return { i: 0, cx: 0, cy: 0, w: 1, h: 1 };
  if (frameIdx <= samples[0].i) return { ...samples[0] };
  if (frameIdx >= samples[samples.length - 1].i)
    return { ...samples[samples.length - 1] };

  let lo = 0,
    hi = samples.length - 1;
  while (lo < hi - 1) {
    const mid = (lo + hi) >> 1;
    if (samples[mid].i <= frameIdx) lo = mid;
    else hi = mid;
  }

  const a = samples[lo];
  const b = samples[hi];
  const t = (frameIdx - a.i) / (b.i - a.i);
  return {
    i: frameIdx,
    cx: a.cx + (b.cx - a.cx) * t,
    cy: a.cy + (b.cy - a.cy) * t,
    w: a.w + (b.w - a.w) * t,
    h: a.h + (b.h - a.h) * t,
  };
}

function applyVirtualMode() {
  if (!video || !videoWrapper || !vcamData) return;
  const [panoramaW, panoramaH] = vcamData.panorama_size;
  const frameIdx = Math.round(video.currentTime * vcamData.fps);
  const { cx, cy, w, h } = interpolateSample(vcamData.samples, frameIdx);
  const x0 = cx - w / 2;
  const y0 = cy - h / 2;
  const displayW = videoWrapper.clientWidth;
  const scale = displayW / w;

  video.style.width = `${panoramaW * scale}px`;
  video.style.height = `${panoramaH * scale}px`;
  video.style.transform = `translate(${-x0 * scale}px, ${-y0 * scale}px)`;
  video.style.position = "absolute";
  video.style.top = "0";
  video.style.left = "0";
}

function applyPanoramaMode() {
  if (!video) return;
  video.style.cssText = "width:100%;height:auto;display:block;";
}

function applyCurrentMode() {
  if (!vcamData || !videoWrapper) return;
  if (viewerMode === "virtual") {
    const [aw, ah] = vcamData.aspect;
    videoWrapper.style.aspectRatio = `${aw} / ${ah}`;
    videoWrapper.style.position = "relative";
    videoWrapper.style.overflow = "hidden";
    applyVirtualMode();
  } else {
    const [pw, ph] = vcamData.panorama_size;
    videoWrapper.style.aspectRatio = `${pw} / ${ph}`;
    videoWrapper.style.position = "";
    videoWrapper.style.overflow = "";
    applyPanoramaMode();
  }
}

function startRafLoop() {
  if (animFrameId !== null) return;
  function loop() {
    applyVirtualMode();
    animFrameId = requestAnimationFrame(loop);
  }
  animFrameId = requestAnimationFrame(loop);
}

function stopRafLoop() {
  if (animFrameId !== null) {
    cancelAnimationFrame(animFrameId);
    animFrameId = null;
  }
}

function formatTime(seconds: number): string {
  if (!isFinite(seconds)) return "0:00";
  const m = Math.floor(seconds / 60);
  const s = Math.floor(seconds % 60);
  return `${m}:${s.toString().padStart(2, "0")}`;
}

function updateTimeDisplay() {
  if (!video || !timeDisplay) return;
  timeDisplay.textContent = `${formatTime(video.currentTime)} / ${formatTime(video.duration)}`;
}

function setupVideoEvents() {
  if (!video || !scrubber || !playPauseButton) return;

  video.addEventListener("loadedmetadata", () => {
    if (scrubber && video) scrubber.max = String(video.duration);
    updateTimeDisplay();
    applyCurrentMode();
  });

  video.addEventListener("timeupdate", () => {
    if (!isScrubbing && scrubber && video)
      scrubber.value = String(video.currentTime);
    updateTimeDisplay();
  });

  video.addEventListener("play", () => {
    if (playPauseButton) playPauseButton.textContent = "Pause";
    if (viewerMode === "virtual") startRafLoop();
  });

  video.addEventListener("pause", () => {
    if (playPauseButton) playPauseButton.textContent = "Play";
    stopRafLoop();
    if (viewerMode === "virtual") applyVirtualMode();
  });

  video.addEventListener("seeked", () => {
    if (viewerMode === "virtual") applyVirtualMode();
  });

  video.addEventListener("ended", () => {
    if (playPauseButton) playPauseButton.textContent = "Play";
    stopRafLoop();
  });
}

function showViewer(data: VirtualCameraData) {
  vcamData = data;
  if (!viewerSection || !video) return;
  video.src = localVideoUrl(data.source);
  viewerSection.style.display = "block";
  viewerMode = "panorama";
  if (viewerModeSelect) viewerModeSelect.value = "panorama";
}

// ── export logic ──────────────────────────────────────────────────────────────

async function exportVideo() {
  if (!vcamData || !exportButton) return;

  const outputPath = await save({
    defaultPath: vcamData.source.replace(/\.[^.]+$/, "_virtual.mp4"),
    filters: [{ name: "MP4 Video", extensions: ["mp4"] }],
  });
  if (!outputPath) return;

  exportButton.disabled = true;
  if (exportContainer) exportContainer.style.display = "block";
  if (exportBar) {
    exportBar.style.width = "0%";
    exportBar.classList.remove("success");
  }

  const unlisten: UnlistenFn = await listen<TaskProgress>(
    "export-progress",
    (e) => {
      const { percentage, step } = e.payload;
      if (exportBar) exportBar.style.width = `${percentage}%`;
      if (exportText) exportText.textContent = step;
      if (percentage >= 100) {
        if (exportBar) exportBar.classList.add("success");
        if (exportButton) exportButton.disabled = false;
        unlisten();
      }
    },
  );

  try {
    await invoke("export_virtual_camera", {
      vcam: vcamData,
      outputPath,
    });
  } catch (e) {
    console.error("Export failed:", e);
    if (exportText) exportText.textContent = `Error: ${e}`;
    if (exportButton) exportButton.disabled = false;
    unlisten();
  }
}

// ── generation logic ──────────────────────────────────────────────────────────

async function selectVideo() {
  const file = await open({ multiple: false, directory: false });
  if (file && videoPathInput) videoPathInput.value = file;
}

async function loadVcam() {
  const file = await open({
    multiple: false,
    directory: false,
    filters: [{ name: "Virtual Camera", extensions: ["json"] }],
  });
  if (!file) return;
  try {
    const data: VirtualCameraData = await invoke("load_virtual_camera", {
      jsonPath: file,
    });
    showViewer(data);
  } catch (e) {
    console.error("Failed to load .vcam.json:", e);
  }
}

async function start() {
  if (
    !startButton ||
    !progressContainer ||
    !bar ||
    !videoPathInput ||
    !outputParagraph
  )
    return;

  startButton.disabled = true;
  progressContainer.style.display = "block";
  if (logContainer) logContainer.style.display = "block";

  const unlisten: UnlistenFn = await listen<TaskProgress>(
    "generate-progress",
    (e) => {
      const { percentage, step } = e.payload;
      if (outputParagraph) {
        outputParagraph.textContent += `${step}\n`;
        outputParagraph.scrollTop = outputParagraph.scrollHeight;
      }
      if (progressText) progressText.textContent = step;
      if (!bar) return;
      bar.style.width = `${percentage}%`;
      if (percentage >= 100) {
        bar.classList.add("success");
        if (startButton) startButton.disabled = false;
        unlisten();
      }
    },
  );

  try {
    const data: VirtualCameraData = await invoke("generate_virtual_camera", {
      videoPath: videoPathInput.value,
    });
    showViewer(data);
  } catch (e) {
    console.error(e);
    if (startButton) startButton.disabled = false;
    unlisten();
  }
}

// ── bootstrap ─────────────────────────────────────────────────────────────────

window.addEventListener("DOMContentLoaded", () => {
  // generation
  startButton = document.querySelector<HTMLButtonElement>("#start_button");
  progressContainer =
    document.querySelector<HTMLDivElement>("#progressContainer");
  bar = document.querySelector<HTMLDivElement>("#progressBar");
  progressText = document.querySelector<HTMLSpanElement>("#progressText");
  logContainer = document.querySelector<HTMLDivElement>("#logContainer");
  videoPathInput = document.querySelector<HTMLInputElement>("#video_path");
  videoSelectButton =
    document.querySelector<HTMLButtonElement>("#video_select");
  outputParagraph = document.querySelector<HTMLPreElement>("#output");
  loadVcamButton = document.querySelector<HTMLButtonElement>("#load_vcam");

  // viewer
  viewerSection = document.querySelector<HTMLDivElement>("#viewerSection");
  viewerModeSelect = document.querySelector<HTMLSelectElement>("#viewerMode");
  timeDisplay = document.querySelector<HTMLSpanElement>("#timeDisplay");
  videoWrapper = document.querySelector<HTMLDivElement>("#videoWrapper");
  video = document.querySelector<HTMLVideoElement>("#viewerVideo");
  playPauseButton = document.querySelector<HTMLButtonElement>("#playPause");
  scrubber = document.querySelector<HTMLInputElement>("#scrubber");
  exportButton = document.querySelector<HTMLButtonElement>("#export_button");
  exportContainer = document.querySelector<HTMLDivElement>("#exportContainer");
  exportBar = document.querySelector<HTMLDivElement>("#exportBar");
  exportText = document.querySelector<HTMLSpanElement>("#exportText");

  // wire up events
  videoSelectButton?.addEventListener("click", (e) => {
    e.preventDefault();
    selectVideo();
  });
  loadVcamButton?.addEventListener("click", (e) => {
    e.preventDefault();
    loadVcam();
  });
  startButton?.addEventListener("click", (e) => {
    e.preventDefault();
    start();
  });

  viewerModeSelect?.addEventListener("change", () => {
    viewerMode = (viewerModeSelect?.value ?? "panorama") as
      | "panorama"
      | "virtual";
    stopRafLoop();
    applyCurrentMode();
    if (viewerMode === "virtual" && video && !video.paused) startRafLoop();
  });

  playPauseButton?.addEventListener("click", () => {
    if (!video) return;
    if (video.paused) video.play();
    else video.pause();
  });

  exportButton?.addEventListener("click", (e) => {
    e.preventDefault();
    exportVideo();
  });

  scrubber?.addEventListener("mousedown", () => {
    isScrubbing = true;
  });
  scrubber?.addEventListener("input", () => {
    if (video && scrubber) video.currentTime = parseFloat(scrubber.value);
  });
  scrubber?.addEventListener("mouseup", () => {
    isScrubbing = false;
    if (viewerMode === "virtual") applyVirtualMode();
  });

  setupVideoEvents();
});
