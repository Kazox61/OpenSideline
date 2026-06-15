use open_pano::config::{init_config, init_config_default};
use open_pano::imgproc::{crop, write_rgb};
use open_pano::stitch::cylstitcher::CylinderStitcher;
use open_pano::stitch::stitcher::Stitcher;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() <= 1 {
        eprintln!("Usage: {} [config.cfg] <image1> <image2> ...", args[0]);
        std::process::exit(1);
    }

    // Check if first arg is a config file
    let (config_arg, img_start) = if args[1].ends_with(".cfg") {
        (Some(args[1].as_str()), 2)
    } else {
        (None, 1)
    };

    if args.len() <= img_start {
        eprintln!("No image files provided.");
        std::process::exit(1);
    }

    // Load config: explicit arg > config.cfg in cwd > built-in defaults
    if let Some(path) = config_arg {
        init_config(path);
    } else if std::path::Path::new("config.cfg").exists() {
        init_config("config.cfg");
    } else {
        init_config_default();
    }

    let cfg = open_pano::config::config();

    let img_args: Vec<String> = args[img_start..].to_vec();

    let mut res = if cfg.cylinder {
        CylinderStitcher::new(&img_args).build()
    } else {
        Stitcher::new(&img_args).build()
    };

    if cfg.crop {
        res = crop(&res);
    }

    write_rgb("out.png", &res);
    println!("Wrote out.png");
}
