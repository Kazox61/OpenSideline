use std::path::Path;
use virtual_camera::{PathConfig, VirtualCameraPath, compute_virtual_camera_path, detect_players};
use yolo_ort::yolo::yolo_session::YoloSession;

fn main() {
    let video = Path::new("test_video.mp4");
    let mut yolo = YoloSession::new(
        Path::new("models/football.onnx"),
        (640, 640),
        true,
        "yolov10".into(),
    )
    .unwrap();

    let (targets, indices, fps, pano_size, total) =
        detect_players(video, &mut yolo, 6, 0, 0.3, None).unwrap(); // stride 6 ≈ 4 detections/s at 25fps

    let samples = compute_virtual_camera_path(
        &targets,
        &indices,
        pano_size,
        fps,
        total,
        &PathConfig::default(),
    );

    let path = VirtualCameraPath::new(
        video.to_str().unwrap().into(),
        pano_size,
        fps,
        total,
        [16, 9],
        samples,
    );
    path.save(Path::new("test_video.vcam.json")).unwrap();
    println!("Wrote {} keyframes", path.samples.len());
}
