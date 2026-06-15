use crate::imgproc::read_img;
use crate::mat::Mat32f;
use crate::stitch::match_info::Shape2D;

pub struct ImageRef {
    pub fname: String,
    pub img: Option<Mat32f>,
    pub width: i32,
    pub height: i32,
}

impl ImageRef {
    pub fn new(fname: impl Into<String>) -> Self {
        ImageRef {
            fname: fname.into(),
            img: None,
            width: 0,
            height: 0,
        }
    }

    pub fn from_mat(mat: Mat32f) -> Self {
        let width = mat.width() as i32;
        let height = mat.height() as i32;
        ImageRef {
            fname: String::new(),
            img: Some(mat),
            width,
            height,
        }
    }

    pub fn load(&mut self) {
        if self.img.is_some() {
            return;
        }
        let m = read_img(&self.fname);
        self.width = m.width() as i32;
        self.height = m.height() as i32;
        self.img = Some(m);
    }

    pub fn release(&mut self) {
        self.img = None;
    }

    pub fn shape(&self) -> Shape2D {
        Shape2D::new(self.width, self.height)
    }

    pub fn mat(&self) -> &Mat32f {
        self.img.as_ref().expect("ImageRef not loaded")
    }
}
