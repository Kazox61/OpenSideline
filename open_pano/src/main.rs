use open_pano::config::init_config_default;
use open_pano::imgproc::{crop, write_rgb};
use open_pano::stitch::cylstitcher::CylinderStitcher;
use open_pano::stitch::stitcher::Stitcher;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() <= 1 {
        eprintln!("Usage: {} <image1> <image2> ...", args[0]);
        std::process::exit(1);
    }

    init_config_default();

    let cfg = open_pano::config::config();

    let mut res = if cfg.cylinder {
        CylinderStitcher::new(&args[1..]).build()
    } else {
        Stitcher::new(&args[1..]).build()
    };

    if cfg.crop {
        res = crop(&res);
    }

    write_rgb("out.png", &res);
    println!("Wrote out.png");
}
