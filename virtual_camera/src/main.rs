use std::path::Path;
use virtual_camera::VirtualCameraPath;

fn main() {
    let video_path = Path::new("test_video.mp4");
    let path = VirtualCameraPath::generate(video_path, Path::new("models/football.onnx"), |p| {
        eprintln!("[{:.0}%] {}", p.percentage, p.step);
    });
    path.save(Path::new("test_video.vcam.json")).unwrap();
    println!("Wrote {} keyframes", path.samples.len());
}
