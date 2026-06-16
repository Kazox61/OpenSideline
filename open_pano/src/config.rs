use std::sync::OnceLock;

/// All configuration values for the stitcher
#[derive(Debug, Clone)]
pub struct Config {
    pub cylinder: bool,
    pub trans: bool,
    pub crop: bool,
    pub focal_length: f32,
    pub estimate_camera: bool,
    pub straighten: bool,
    pub max_output_size: i32,
    pub ordered_input: bool,
    pub lazy_read: bool,

    pub sift_working_size: i32,
    pub num_octave: i32,
    pub num_scale: i32,
    pub scale_factor: f32,

    pub gauss_sigma: f32,
    pub gauss_window_factor: i32,

    pub judge_extrema_diff_thres: f32,
    pub contrast_thres: f32,
    pub pre_color_thres: f32,
    pub edge_ratio: f32,

    pub calc_offset_depth: i32,
    pub offset_thres: f32,

    pub ori_radius: f32,
    pub ori_hist_smooth_count: i32,

    pub desc_hist_scale_factor: i32,
    pub desc_int_factor: i32,

    pub match_reject_next_ratio: f32,

    pub ransac_iterations: i32,
    pub ransac_inlier_thres: f64,
    pub inlier_in_match_ratio: f32,
    pub inlier_in_points_ratio: f32,

    pub slope_plain: f32,
    pub lm_lambda: f32,
    pub multipass_ba: i32,
    pub multiband: i32,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            cylinder: false,
            trans: false,
            crop: true,
            focal_length: 0.0,
            estimate_camera: true,
            straighten: true,
            max_output_size: 12000,
            ordered_input: true,
            lazy_read: true,
            sift_working_size: 1200,
            num_octave: 4,
            num_scale: 7,
            scale_factor: 1.414_213_5,
            gauss_sigma: 1.414_213_5,
            gauss_window_factor: 6,
            judge_extrema_diff_thres: 1e-3,
            contrast_thres: 1e-2,
            pre_color_thres: 2e-2,
            edge_ratio: 12.0,
            calc_offset_depth: 4,
            offset_thres: 0.5,
            ori_radius: 4.5,
            ori_hist_smooth_count: 2,
            desc_hist_scale_factor: 3,
            desc_int_factor: 512,
            match_reject_next_ratio: 0.85,
            ransac_iterations: 5000,
            ransac_inlier_thres: 2.5,
            inlier_in_match_ratio: 0.05,
            inlier_in_points_ratio: 0.02,
            slope_plain: 8e-3,
            lm_lambda: 1.0,
            multipass_ba: 2,
            multiband: 6,
        }
    }
}

// Fixed constants from config.hh
pub const ORI_WINDOW_FACTOR: f32 = 1.5;
pub const ORI_HIST_BIN_NUM: usize = 36;
pub const ORI_HIST_PEAK_RATIO: f32 = 0.8;

pub const DESC_HIST_WIDTH: usize = 4;
pub const DESC_HIST_BIN_NUM: usize = 8;
pub const DESC_LEN: usize = 128; // 4*4*8
pub const DESC_NORM_THRESH: f32 = 0.2;

pub const BRIEF_PATH_SIZE: i32 = 9;
pub const BRIEF_NR_PAIR: i32 = 256;

pub const FLANN_NR_KDTREE: i32 = 6;

static GLOBAL_CONFIG: OnceLock<Config> = OnceLock::new();

pub fn init_config_default() {
    GLOBAL_CONFIG.get_or_init(Config::default);
}

pub fn config() -> &'static Config {
    GLOBAL_CONFIG
        .get()
        .expect("Config not initialized. Call init_config() first.")
}
