use std::collections::HashMap;
use std::fs;
use std::sync::OnceLock;

/// Mirrors C++ ConfigParser - parses "KEY VALUE" pairs from a config file
pub struct ConfigParser {
    data: HashMap<String, f32>,
}

impl ConfigParser {
    pub fn new(fname: &str) -> Self {
        let content = fs::read_to_string(fname)
            .unwrap_or_else(|_| panic!("Cannot find config file: {}", fname));
        let mut data = HashMap::new();
        for line in content.lines() {
            // Strip inline comments, then split into tokens
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let line = line.split('#').next().unwrap_or("").trim();
            let mut tokens = line.split_whitespace();
            if let Some(key) = tokens.next() {
                if let Some(val_str) = tokens.next() {
                    if let Ok(val) = val_str.parse::<f32>() {
                        data.insert(key.to_string(), val);
                    }
                }
            }
        }
        ConfigParser { data }
    }

    pub fn get(&self, key: &str) -> f32 {
        *self
            .data
            .get(key)
            .unwrap_or_else(|| panic!("Option {} not found in config file!", key))
    }
}

/// All configuration values parsed from config.cfg
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
            focal_length: 37.0,
            estimate_camera: true,
            straighten: true,
            max_output_size: 8000,
            ordered_input: false,
            lazy_read: true,
            sift_working_size: 800,
            num_octave: 4,
            num_scale: 7,
            scale_factor: 1.414_213_5,
            gauss_sigma: 1.414_213_5,
            gauss_window_factor: 6,
            judge_extrema_diff_thres: 2e-3,
            contrast_thres: 4e-2,
            pre_color_thres: 5e-2,
            edge_ratio: 6.0,
            calc_offset_depth: 4,
            offset_thres: 0.5,
            ori_radius: 4.5,
            ori_hist_smooth_count: 2,
            desc_hist_scale_factor: 3,
            desc_int_factor: 512,
            match_reject_next_ratio: 0.8,
            ransac_iterations: 1500,
            ransac_inlier_thres: 3.5,
            inlier_in_match_ratio: 0.1,
            inlier_in_points_ratio: 0.04,
            slope_plain: 8e-3,
            lm_lambda: 5.0,
            multipass_ba: 1,
            multiband: 0,
        }
    }
}

impl Config {
    pub fn from_file(path: &str) -> Self {
        let p = ConfigParser::new(path);
        let cfg = |name: &str| p.get(name);
        Config {
            cylinder: cfg("CYLINDER") != 0.0,
            trans: cfg("TRANS") != 0.0,
            crop: cfg("CROP") != 0.0,
            focal_length: cfg("FOCAL_LENGTH"),
            estimate_camera: cfg("ESTIMATE_CAMERA") != 0.0,
            straighten: cfg("STRAIGHTEN") != 0.0,
            max_output_size: cfg("MAX_OUTPUT_SIZE") as i32,
            ordered_input: cfg("ORDERED_INPUT") != 0.0,
            lazy_read: cfg("LAZY_READ") != 0.0,
            sift_working_size: cfg("SIFT_WORKING_SIZE") as i32,
            num_octave: cfg("NUM_OCTAVE") as i32,
            num_scale: cfg("NUM_SCALE") as i32,
            scale_factor: cfg("SCALE_FACTOR"),
            gauss_sigma: cfg("GAUSS_SIGMA"),
            gauss_window_factor: cfg("GAUSS_WINDOW_FACTOR") as i32,
            judge_extrema_diff_thres: cfg("JUDGE_EXTREMA_DIFF_THRES"),
            contrast_thres: cfg("CONTRAST_THRES"),
            pre_color_thres: cfg("PRE_COLOR_THRES"),
            edge_ratio: cfg("EDGE_RATIO"),
            calc_offset_depth: cfg("CALC_OFFSET_DEPTH") as i32,
            offset_thres: cfg("OFFSET_THRES"),
            ori_radius: cfg("ORI_RADIUS"),
            ori_hist_smooth_count: cfg("ORI_HIST_SMOOTH_COUNT") as i32,
            desc_hist_scale_factor: cfg("DESC_HIST_SCALE_FACTOR") as i32,
            desc_int_factor: cfg("DESC_INT_FACTOR") as i32,
            match_reject_next_ratio: cfg("MATCH_REJECT_NEXT_RATIO"),
            ransac_iterations: cfg("RANSAC_ITERATIONS") as i32,
            ransac_inlier_thres: cfg("RANSAC_INLIER_THRES") as f64,
            inlier_in_match_ratio: cfg("INLIER_IN_MATCH_RATIO"),
            inlier_in_points_ratio: cfg("INLIER_IN_POINTS_RATIO"),
            slope_plain: cfg("SLOPE_PLAIN"),
            lm_lambda: cfg("LM_LAMBDA"),
            multipass_ba: cfg("MULTIPASS_BA") as i32,
            multiband: cfg("MULTIBAND") as i32,
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

pub fn init_config(path: &str) {
    GLOBAL_CONFIG
        .set(Config::from_file(path))
        .unwrap_or_else(|_| panic!("Config already initialized"));
}

pub fn init_config_default() {
    GLOBAL_CONFIG
        .set(Config::default())
        .unwrap_or_else(|_| panic!("Config already initialized"));
}

pub fn config() -> &'static Config {
    GLOBAL_CONFIG
        .get()
        .expect("Config not initialized. Call init_config() first.")
}
