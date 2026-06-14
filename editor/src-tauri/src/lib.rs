use std::io::{Read, Seek, SeekFrom};
use std::path::Path;
use tauri::{AppHandle, Emitter};
use virtual_camera::{export_video, VirtualCameraPath};

#[tauri::command]
async fn generate_virtual_camera(
    app: AppHandle,
    video_path: String,
) -> Result<VirtualCameraPath, String> {
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

#[tauri::command]
async fn export_virtual_camera(
    app: AppHandle,
    vcam: VirtualCameraPath,
    output_path: String,
) -> Result<(), String> {
    tokio::task::spawn_blocking(move || {
        let video_path = vcam.source.clone();
        let total = vcam.frame_count;
        export_video(
            &vcam,
            Path::new(&video_path),
            Path::new(&output_path),
            |done, _total| {
                app.emit(
                    "export-progress",
                    serde_json::json!({
                        "percentage": (done as f64 / total as f64) * 100.0,
                        "step": format!("Exporting frame {done}/{total}"),
                    }),
                )
                .ok();
            },
        )
        .map_err(|e| e.to_string())
    })
    .await
    .map_err(|e| e.to_string())?
}

#[tauri::command]
async fn load_virtual_camera(json_path: String) -> Result<VirtualCameraPath, String> {
    tokio::task::spawn_blocking(move || {
        VirtualCameraPath::load(Path::new(&json_path)).map_err(|e| e.to_string())
    })
    .await
    .map_err(|e| e.to_string())?
}

/// Percent-decode a URI path (byte-level, handles UTF-8 sequences).
fn percent_decode(s: &str) -> String {
    let mut bytes = Vec::with_capacity(s.len());
    let mut iter = s.bytes();
    while let Some(b) = iter.next() {
        if b == b'%' {
            let h1 = iter.next().unwrap_or(b'0');
            let h2 = iter.next().unwrap_or(b'0');
            let hex = [h1, h2];
            if let Ok(hex_str) = std::str::from_utf8(&hex) {
                if let Ok(decoded) = u8::from_str_radix(hex_str, 16) {
                    bytes.push(decoded);
                    continue;
                }
            }
            bytes.extend_from_slice(&[b'%', h1, h2]);
        } else {
            bytes.push(b);
        }
    }
    String::from_utf8_lossy(&bytes).into_owned()
}

fn local_video_protocol(
    request: tauri::http::Request<Vec<u8>>,
) -> tauri::http::Response<Vec<u8>> {
    use tauri::http::Response;

    // URI: localvideo://localhost/path/to/video.mp4
    let uri = request.uri().to_string();
    let raw_path = match uri.split_once("://localhost") {
        Some((_, p)) => p,
        None => {
            return Response::builder()
                .status(400)
                .body(b"Bad request".to_vec())
                .unwrap()
        }
    };
    let path_str = percent_decode(raw_path);
    let path = Path::new(&path_str);

    let ext = path.extension().and_then(|e| e.to_str()).unwrap_or("");
    let content_type = match ext {
        "mp4" => "video/mp4",
        "webm" => "video/webm",
        "mov" => "video/quicktime",
        "avi" => "video/x-msvideo",
        "mkv" => "video/x-matroska",
        _ => "video/mp4",
    };

    let mut file = match std::fs::File::open(path) {
        Ok(f) => f,
        Err(e) => {
            return Response::builder()
                .status(404)
                .header("Content-Type", "text/plain")
                .body(format!("Not found: {e}").into_bytes())
                .unwrap()
        }
    };
    let total_size = match file.metadata() {
        Ok(m) => m.len(),
        Err(_) => {
            return Response::builder()
                .status(500)
                .body(b"Cannot read file metadata".to_vec())
                .unwrap()
        }
    };

    let range_val = request
        .headers()
        .get("range")
        .and_then(|v| v.to_str().ok())
        .map(|s| s.to_owned());

    if let Some(range_str) = range_val {
        let range_str = range_str.trim_start_matches("bytes=");
        let mut parts = range_str.splitn(2, '-');
        let start: u64 = parts.next().and_then(|s| s.parse().ok()).unwrap_or(0);
        let end: u64 = parts
            .next()
            .and_then(|s| if s.is_empty() { None } else { s.parse().ok() })
            .unwrap_or(total_size.saturating_sub(1));
        let end = end.min(total_size.saturating_sub(1));

        if start > end || start >= total_size {
            return Response::builder()
                .status(416)
                .header("Content-Range", format!("bytes */{total_size}"))
                .body(vec![])
                .unwrap();
        }

        let length = end - start + 1;
        let mut buf = vec![0u8; length as usize];

        if file.seek(SeekFrom::Start(start)).is_err()
            || file.read_exact(&mut buf).is_err()
        {
            return Response::builder()
                .status(500)
                .body(b"Read error".to_vec())
                .unwrap();
        }

        Response::builder()
            .status(206)
            .header("Content-Type", content_type)
            .header("Content-Range", format!("bytes {start}-{end}/{total_size}"))
            .header("Content-Length", length.to_string())
            .header("Accept-Ranges", "bytes")
            .body(buf)
            .unwrap()
    } else {
        // No range header — serve whole file.
        // The browser typically follows up with range requests for video playback.
        let mut buf = Vec::with_capacity(total_size as usize);
        if file.read_to_end(&mut buf).is_err() {
            return Response::builder()
                .status(500)
                .body(b"Read error".to_vec())
                .unwrap();
        }
        Response::builder()
            .status(200)
            .header("Content-Type", content_type)
            .header("Content-Length", total_size.to_string())
            .header("Accept-Ranges", "bytes")
            .body(buf)
            .unwrap()
    }
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    tauri::Builder::default()
        .plugin(tauri_plugin_dialog::init())
        .plugin(tauri_plugin_opener::init())
        .register_uri_scheme_protocol("localvideo", |_ctx, request| {
            local_video_protocol(request)
        })
        .invoke_handler(tauri::generate_handler![
            generate_virtual_camera,
            load_virtual_camera,
            export_virtual_camera
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
