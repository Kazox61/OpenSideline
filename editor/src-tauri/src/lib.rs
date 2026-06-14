use std::path::Path;
use tauri::{AppHandle, Emitter};
use virtual_camera::VirtualCameraPath;

#[tauri::command]
async fn generate_virtual_camera(
    app: AppHandle,
    video_path: String,
) -> Result<VirtualCameraPath, String> {
    // Run on a blocking thread since generate() is CPU-heavy
    tokio::task::spawn_blocking(move || {
        let path = Path::new(&video_path);

        let virtual_camera_path = VirtualCameraPath::generate(path, move |progress| {
            app.emit(
                "generate-progress",
                serde_json::json!({
                    "percentage": progress.percentage,
                    "step": progress.step,
                }),
            )
            .unwrap();
        });
        virtual_camera_path
            .save(&path.with_extension("vcam.json"))
            .unwrap();
        virtual_camera_path
    })
    .await
    .map_err(|e| e.to_string())
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    tauri::Builder::default()
        .plugin(tauri_plugin_dialog::init())
        .plugin(tauri_plugin_opener::init())
        .invoke_handler(tauri::generate_handler![generate_virtual_camera])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
