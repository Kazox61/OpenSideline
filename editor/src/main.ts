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

// ── icons ─────────────────────────────────────────────────────────────────────

const PLAY_ICON = `<svg width="13" height="13" viewBox="0 0 13 13" fill="currentColor"><path d="M3 1.5l8 5-8 5z"/></svg>`;
const PAUSE_ICON = `<svg width="13" height="13" viewBox="0 0 13 13" fill="currentColor"><rect x="2" y="1.5" width="3.5" height="10" rx="0.5"/><rect x="7.5" y="1.5" width="3.5" height="10" rx="0.5"/></svg>`;

// ── panel titles ──────────────────────────────────────────────────────────────

const PANEL_TITLES: Record<string, string> = {
  stitch: "Stitch",
  generate: "Generate",
  export: "Export",
};

// ── element refs ──────────────────────────────────────────────────────────────

// view mode HUD
let btnPanorama: HTMLButtonElement | null;
let btnFollow: HTMLButtonElement | null;

// viewer frame shell and inner video wrapper
let viewerFrame: HTMLDivElement | null;
let videoWrapper: HTMLDivElement | null;
let videoPlaceholder: HTMLDivElement | null;
let video: HTMLVideoElement | null;

// playback HUD
let playPauseButton: HTMLButtonElement | null;
let scrubber: HTMLInputElement | null;
let timeDisplay: HTMLSpanElement | null;

// overlay panel
let overlayPanel: HTMLDivElement | null;
let panelTitle: HTMLSpanElement | null;
let panelClose: HTMLButtonElement | null;
let iconBtns: NodeListOf<HTMLButtonElement>;

// stitch panel
let stitchSelectButton: HTMLButtonElement | null;
let stitchFileList: HTMLUListElement | null;
let stitchButton: HTMLButtonElement | null;
let stitchProgressContainer: HTMLDivElement | null;
let stitchProgressBar: HTMLDivElement | null;
let stitchProgressText: HTMLSpanElement | null;

// generate panel
let videoPathInput: HTMLInputElement | null;
let videoSelectButton: HTMLButtonElement | null;
let startButton: HTMLButtonElement | null;
let progressContainer: HTMLDivElement | null;
let progressBar: HTMLDivElement | null;
let progressText: HTMLSpanElement | null;
let logContainer: HTMLDivElement | null;
let outputParagraph: HTMLPreElement | null;
let loadVcamButton: HTMLButtonElement | null;

// export panel
let exportButton: HTMLButtonElement | null;
let exportContainer: HTMLDivElement | null;
let exportBar: HTMLDivElement | null;
let exportText: HTMLSpanElement | null;

// ── state ─────────────────────────────────────────────────────────────────────

let vcamData: VirtualCameraData | null = null;
let viewerMode: "panorama" | "virtual" = "panorama";
let animFrameId: number | null = null;
let isScrubbing = false;
let activePanel: string | null = null;
let stitchInputPaths: string[] = [];

// ── overlay panel ─────────────────────────────────────────────────────────────

function openPanel(id: string) {
  activePanel = id;

  iconBtns?.forEach((btn) =>
    btn.classList.toggle("active", btn.dataset.panel === id),
  );

  document.querySelectorAll<HTMLDivElement>(".panel-content").forEach((el) => {
    el.style.display = el.id === `panel-${id}` ? "flex" : "none";
  });

  if (panelTitle) panelTitle.textContent = PANEL_TITLES[id] ?? id;
  if (overlayPanel) overlayPanel.style.display = "flex";
}

function closePanel() {
  activePanel = null;
  iconBtns?.forEach((btn) => btn.classList.remove("active"));
  if (overlayPanel) overlayPanel.style.display = "none";
}

function togglePanel(id: string) {
  if (activePanel === id) closePanel();
  else openPanel(id);
}

// ── view mode ─────────────────────────────────────────────────────────────────

function setViewMode(mode: "panorama" | "virtual") {
  if (mode === "virtual" && !vcamData) return;
  viewerMode = mode;
  btnPanorama?.classList.toggle("active", mode === "panorama");
  btnFollow?.classList.toggle("active", mode === "virtual");
  stopRafLoop();
  applyCurrentMode();
  if (mode === "virtual" && video && !video.paused) startRafLoop();
}

// ── virtual camera interpolation ──────────────────────────────────────────────

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

// ── render modes ──────────────────────────────────────────────────────────────

function applyVirtualMode() {
  if (!video || !viewerFrame || !videoWrapper || !vcamData) return;
  const displayW = videoWrapper.clientWidth;
  if (displayW === 0) return;

  const [panoramaW, panoramaH] = vcamData.panorama_size;
  const frameIdx = Math.round(video.currentTime * vcamData.fps);
  const { cx, cy, w, h } = interpolateSample(vcamData.samples, frameIdx);
  const x0 = cx - w / 2;
  const y0 = cy - h / 2;
  const scale = displayW / w;

  video.style.cssText = [
    `width:${panoramaW * scale}px`,
    `height:${panoramaH * scale}px`,
    `transform:translate(${-x0 * scale}px,${-y0 * scale}px)`,
    "position:absolute",
    "top:0",
    "left:0",
    "max-width:none",
    "max-height:none",
    "object-fit:fill",
    "display:block",
  ].join(";");
}

function applyPanoramaMode() {
  if (!video) return;
  video.style.cssText =
    "width:100%;height:100%;display:block;object-fit:contain;position:relative;max-width:none;max-height:none;transform:none;";
}

function applyCurrentMode() {
  if (viewerMode === "virtual" && vcamData) {
    applyVirtualMode();
  } else {
    applyPanoramaMode();
  }
}

// ── RAF loop ──────────────────────────────────────────────────────────────────

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

// ── video loading ─────────────────────────────────────────────────────────────

function setPlaybackEnabled(enabled: boolean) {
  if (playPauseButton) playPauseButton.disabled = !enabled;
  if (scrubber) scrubber.disabled = !enabled;
}

function localVideoUrl(filePath: string): string {
  return (
    "localvideo://localhost" +
    filePath
      .split("/")
      .map(encodeURIComponent)
      .join("/")
  );
}

function loadVideoPath(path: string) {
  if (!video || !videoPlaceholder) return;
  vcamData = null;
  viewerMode = "panorama";
  if (btnFollow) btnFollow.disabled = true;
  btnPanorama?.classList.add("active");
  btnFollow?.classList.remove("active");
  video.src = localVideoUrl(path);
  videoPlaceholder.style.display = "none";
  setPlaybackEnabled(true);
  applyPanoramaMode();
}

function showViewer(data: VirtualCameraData) {
  if (!video || !videoPlaceholder) return;
  vcamData = data;
  video.src = localVideoUrl(data.source);
  videoPlaceholder.style.display = "none";
  setPlaybackEnabled(true);
  if (btnFollow) btnFollow.disabled = false;
  viewerMode = "panorama";
  btnPanorama?.classList.add("active");
  btnFollow?.classList.remove("active");
  applyPanoramaMode();
}

// ── video events ──────────────────────────────────────────────────────────────

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
    if (playPauseButton) playPauseButton.innerHTML = PAUSE_ICON;
    if (viewerMode === "virtual") startRafLoop();
  });

  video.addEventListener("pause", () => {
    if (playPauseButton) playPauseButton.innerHTML = PLAY_ICON;
    stopRafLoop();
    if (viewerMode === "virtual") applyVirtualMode();
  });

  video.addEventListener("seeked", () => {
    if (viewerMode === "virtual") applyVirtualMode();
  });

  video.addEventListener("ended", () => {
    if (playPauseButton) playPauseButton.innerHTML = PLAY_ICON;
    stopRafLoop();
  });
}

// ── export ────────────────────────────────────────────────────────────────────

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
    await invoke("export_virtual_camera", { vcam: vcamData, outputPath });
  } catch (e) {
    console.error("Export failed:", e);
    if (exportText) exportText.textContent = `Error: ${e}`;
    if (exportButton) exportButton.disabled = false;
    unlisten();
  }
}

// ── generate ──────────────────────────────────────────────────────────────────

async function selectVideo() {
  const file = await open({ multiple: false, directory: false });
  if (file && videoPathInput) videoPathInput.value = file as string;
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

async function generateVirtualCamera() {
  if (!startButton || !progressContainer || !progressBar || !videoPathInput || !outputParagraph)
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
      if (!progressBar) return;
      progressBar.style.width = `${percentage}%`;
      if (percentage >= 100) {
        progressBar.classList.add("success");
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
    if (outputParagraph) outputParagraph.textContent += `Error: ${e}\n`;
    if (progressText) progressText.textContent = "Failed";
    if (startButton) startButton.disabled = false;
    unlisten();
  }
}

// ── stitch ────────────────────────────────────────────────────────────────────

async function selectStitchFiles() {
  const files = await open({ multiple: true, directory: false });
  if (!files || (files as string[]).length === 0) return;
  stitchInputPaths = files as string[];
  renderStitchFileList();
  if (stitchButton) stitchButton.disabled = stitchInputPaths.length < 2;
}

function renderStitchFileList() {
  if (!stitchFileList) return;
  stitchFileList.innerHTML = "";
  for (const p of stitchInputPaths) {
    const li = document.createElement("li");
    li.textContent = p.split("/").pop() ?? p;
    li.title = p;
    stitchFileList.appendChild(li);
  }
}

async function stitchVideos() {
  if (stitchInputPaths.length < 2 || !stitchButton) return;

  const outputPath = await save({
    defaultPath: "panorama.mp4",
    filters: [{ name: "MP4 Video", extensions: ["mp4"] }],
  });
  if (!outputPath) return;

  stitchButton.disabled = true;
  if (stitchProgressContainer) stitchProgressContainer.style.display = "block";
  if (stitchProgressBar) {
    stitchProgressBar.style.width = "0%";
    stitchProgressBar.classList.remove("success");
  }

  const unlisten: UnlistenFn = await listen<TaskProgress>(
    "stitch-progress",
    (e) => {
      const { percentage, step } = e.payload;
      if (stitchProgressBar) stitchProgressBar.style.width = `${percentage}%`;
      if (stitchProgressText) stitchProgressText.textContent = step;
      if (percentage >= 100) {
        if (stitchProgressBar) stitchProgressBar.classList.add("success");
        if (stitchButton) stitchButton.disabled = false;
        unlisten();
      }
    },
  );

  try {
    await invoke("stitch_videos_cmd", { inputPaths: stitchInputPaths, outputPath });
    if (stitchProgressBar) stitchProgressBar.classList.add("success");
    if (stitchProgressText) stitchProgressText.textContent = "Done";
    if (stitchButton) stitchButton.disabled = false;
    loadVideoPath(outputPath);
    if (videoPathInput) videoPathInput.value = outputPath;
    unlisten();
  } catch (e) {
    console.error("Stitch failed:", e);
    if (stitchProgressText) stitchProgressText.textContent = `Error: ${e}`;
    if (stitchButton) stitchButton.disabled = false;
    unlisten();
  }
}

// ── bootstrap ─────────────────────────────────────────────────────────────────

window.addEventListener("DOMContentLoaded", () => {
  // HUD
  btnPanorama = document.querySelector<HTMLButtonElement>("#btn_panorama");
  btnFollow = document.querySelector<HTMLButtonElement>("#btn_follow");
  viewerFrame = document.querySelector<HTMLDivElement>("#viewerFrame");
  videoWrapper = document.querySelector<HTMLDivElement>("#videoWrapper");
  videoPlaceholder = document.querySelector<HTMLDivElement>("#videoPlaceholder");
  video = document.querySelector<HTMLVideoElement>("#viewerVideo");
  playPauseButton = document.querySelector<HTMLButtonElement>("#playPause");
  scrubber = document.querySelector<HTMLInputElement>("#scrubber");
  timeDisplay = document.querySelector<HTMLSpanElement>("#timeDisplay");

  // overlay panel
  overlayPanel = document.querySelector<HTMLDivElement>("#overlayPanel");
  panelTitle = document.querySelector<HTMLSpanElement>("#panelTitle");
  panelClose = document.querySelector<HTMLButtonElement>("#panelClose");
  iconBtns = document.querySelectorAll<HTMLButtonElement>(".icon-btn");

  // stitch panel
  stitchSelectButton = document.querySelector<HTMLButtonElement>("#stitch_select");
  stitchFileList = document.querySelector<HTMLUListElement>("#stitch_file_list");
  stitchButton = document.querySelector<HTMLButtonElement>("#stitch_button");
  stitchProgressContainer = document.querySelector<HTMLDivElement>("#stitchProgressContainer");
  stitchProgressBar = document.querySelector<HTMLDivElement>("#stitchProgressBar");
  stitchProgressText = document.querySelector<HTMLSpanElement>("#stitchProgressText");

  // generate panel
  videoPathInput = document.querySelector<HTMLInputElement>("#video_path");
  videoSelectButton = document.querySelector<HTMLButtonElement>("#video_select");
  startButton = document.querySelector<HTMLButtonElement>("#start_button");
  progressContainer = document.querySelector<HTMLDivElement>("#progressContainer");
  progressBar = document.querySelector<HTMLDivElement>("#progressBar");
  progressText = document.querySelector<HTMLSpanElement>("#progressText");
  logContainer = document.querySelector<HTMLDivElement>("#logContainer");
  outputParagraph = document.querySelector<HTMLPreElement>("#output");
  loadVcamButton = document.querySelector<HTMLButtonElement>("#load_vcam");

  // export panel
  exportButton = document.querySelector<HTMLButtonElement>("#export_button");
  exportContainer = document.querySelector<HTMLDivElement>("#exportContainer");
  exportBar = document.querySelector<HTMLDivElement>("#exportBar");
  exportText = document.querySelector<HTMLSpanElement>("#exportText");

  // ── wire events ──

  btnPanorama?.addEventListener("click", () => setViewMode("panorama"));
  btnFollow?.addEventListener("click", () => setViewMode("virtual"));

  iconBtns?.forEach((btn) => {
    btn.addEventListener("click", () => {
      const id = btn.dataset.panel;
      if (id) togglePanel(id);
    });
  });

  panelClose?.addEventListener("click", closePanel);

  stitchSelectButton?.addEventListener("click", (e) => { e.preventDefault(); selectStitchFiles(); });
  stitchButton?.addEventListener("click", (e) => { e.preventDefault(); stitchVideos(); });

  videoSelectButton?.addEventListener("click", (e) => { e.preventDefault(); selectVideo(); });
  loadVcamButton?.addEventListener("click", (e) => { e.preventDefault(); loadVcam(); });
  startButton?.addEventListener("click", (e) => { e.preventDefault(); generateVirtualCamera(); });

  exportButton?.addEventListener("click", (e) => { e.preventDefault(); exportVideo(); });

  playPauseButton?.addEventListener("click", () => {
    if (!video) return;
    if (video.paused) video.play();
    else video.pause();
  });

  scrubber?.addEventListener("mousedown", () => { isScrubbing = true; });
  scrubber?.addEventListener("input", () => {
    if (video && scrubber) video.currentTime = parseFloat(scrubber.value);
  });
  scrubber?.addEventListener("mouseup", () => {
    isScrubbing = false;
    if (viewerMode === "virtual") applyVirtualMode();
  });

  setupVideoEvents();
});
