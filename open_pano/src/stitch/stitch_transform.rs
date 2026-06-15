use crate::mat::Mat32f;
use crate::stitch::homography::Homography;
use crate::stitch::imageref::ImageRef;
use crate::stitch::stitcher_image::{
    ComponentRange, ConnectedImages, ImageComponent, ProjRange, ProjectionMethod,
};

/// Plain-data snapshot of a computed stitching transform — no raw pointers.
/// Cheap to clone and safe to store between frames.
pub struct StitchTransform {
    pub proj_method: ProjectionMethod,
    pub proj_range: ProjRange,
    pub identity_idx: usize,
    pub components: Vec<TransformComponent>,
}

pub struct TransformComponent {
    pub homo: Homography,
    pub homo_inv: Homography,
    pub range: ComponentRange,
    pub img_width: i32,
    pub img_height: i32,
}

impl StitchTransform {
    /// Extract the reusable transform data from a fully-built ConnectedImages.
    pub fn from_bundle(bundle: &ConnectedImages) -> Self {
        let components = bundle
            .component
            .iter()
            .map(|c| TransformComponent {
                homo: c.homo,
                homo_inv: c.homo_inv,
                range: c.range,
                img_width: c.imgref().width,
                img_height: c.imgref().height,
            })
            .collect();
        StitchTransform {
            proj_method: bundle.proj_method,
            proj_range: bundle.proj_range,
            identity_idx: bundle.identity_idx,
            components,
        }
    }

    /// Apply the stored transform to a fresh set of frames (one per camera).
    /// Does NOT re-run SIFT or camera estimation — only warps and blends.
    pub fn apply(&self, images: Vec<Mat32f>) -> Mat32f {
        assert_eq!(
            images.len(),
            self.components.len(),
            "number of frames must match number of cameras in transform"
        );

        // Build owned ImageRef objects so imgptr is valid for the duration of blend().
        let mut imgs: Vec<ImageRef> = images.into_iter().map(ImageRef::from_mat).collect();

        let mut bundle = ConnectedImages::new();
        bundle.proj_method = self.proj_method;
        bundle.proj_range = self.proj_range;
        bundle.identity_idx = self.identity_idx;
        bundle.component = self
            .components
            .iter()
            .zip(imgs.iter_mut())
            .map(|(c, img)| ImageComponent {
                homo: c.homo,
                homo_inv: c.homo_inv,
                imgptr: img as *mut ImageRef,
                range: c.range,
            })
            .collect();

        bundle.blend()
    }
}
