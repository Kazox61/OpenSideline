pub mod detector;
pub mod exporter;
pub mod path_generator;
pub mod smooth_damp;
pub mod virtual_camera_path;

pub use detector::{detect_players, frame_target, FrameTarget};
pub use exporter::export_video;
pub use path_generator::{compute_virtual_camera_path, PathConfig, ZoomMode};
pub use smooth_damp::SmoothDamp;
pub use virtual_camera_path::{VirtualCameraPath, VirtualCameraSample};
