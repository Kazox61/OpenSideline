import { invoke } from "@tauri-apps/api/core";
import { listen, UnlistenFn } from "@tauri-apps/api/event";
import { open } from "@tauri-apps/plugin-dialog";

interface TaskProgress {
  percentage: number;
  step: string;
}

let startButton: HTMLButtonElement | null;
let container: HTMLDivElement | null;
let bar: HTMLDivElement | null;
let progressText: HTMLSpanElement | null;
let logContainer: HTMLDivElement | null;
let videoPathInput: HTMLInputElement | null;
let videoSelectButton: HTMLButtonElement | null;
let outputParagraph: HTMLPreElement | null;

async function selectVideo() {
  const file = await open({
    multiple: false,
    directory: false,
  });
  console.log(file);
  if (file && videoPathInput) {
    videoPathInput.value = file;
  }
}

async function start() {
  if (!startButton || !container || !bar || !videoPathInput || !outputParagraph)
    return;

  startButton.disabled = true;
  container.style.display = "block";
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
    await invoke("generate_virtual_camera", {
      videoPath: videoPathInput.value,
    });
  } catch (e) {
    console.error(e);
    if (startButton) startButton.disabled = false;
    unlisten();
  }
}

window.addEventListener("DOMContentLoaded", () => {
  startButton = document.querySelector<HTMLButtonElement>("#start_button");
  container = document.querySelector<HTMLDivElement>("#progressContainer");
  bar = document.querySelector<HTMLDivElement>("#progressBar");
  progressText = document.querySelector<HTMLSpanElement>("#progressText");
  logContainer = document.querySelector<HTMLDivElement>("#logContainer");
  videoPathInput = document.querySelector<HTMLInputElement>("#video_path");
  videoSelectButton =
    document.querySelector<HTMLButtonElement>("#video_select");
  outputParagraph = document.querySelector<HTMLPreElement>("#output");
  videoSelectButton?.addEventListener("click", (e) => {
    e.preventDefault();
    selectVideo();
  });
  document.querySelector("#start_button")?.addEventListener("click", (e) => {
    e.preventDefault();
    start();
  });
});
