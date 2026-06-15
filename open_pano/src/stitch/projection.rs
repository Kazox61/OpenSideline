use crate::geometry::{Vec2D, Vec3};

pub type Homo2Proj = fn(Vec3) -> Vec2D;
pub type Proj2Homo = fn(Vec2D) -> Vec3;

pub mod flat {
    use crate::geometry::{Vec2D, Vec3};

    pub fn homo2proj(homo: Vec3) -> Vec2D {
        Vec2D::new(homo.x / homo.z, homo.y / homo.z)
    }

    pub fn gradproj(homo: Vec3, grad: Vec3) -> Vec2D {
        let hz_inv = 1.0 / homo.z;
        let hz_sqr_inv = hz_inv * hz_inv;
        Vec2D::new(
            grad.x * hz_inv - grad.z * homo.x * hz_sqr_inv,
            grad.y * hz_inv - grad.z * homo.y * hz_sqr_inv,
        )
    }

    pub fn proj2homo(proj: Vec2D) -> Vec3 {
        Vec3::new(proj.x, proj.y, 1.0)
    }
}

pub mod cylindrical {
    use crate::geometry::{Vec2D, Vec3};

    pub fn homo2proj(homo: Vec3) -> Vec2D {
        Vec2D::new(homo.x.atan2(homo.z), homo.y / homo.x.hypot(homo.z))
    }

    pub fn proj2homo(proj: Vec2D) -> Vec3 {
        Vec3::new(proj.x.sin(), proj.y, proj.x.cos())
    }
}

pub mod spherical {
    use crate::geometry::{Vec2D, Vec3};

    pub fn homo2proj(homo: Vec3) -> Vec2D {
        Vec2D::new(homo.x.atan2(homo.z), homo.y.atan2(homo.x.hypot(homo.z)))
    }

    pub fn gradproj(homo: Vec3, grad: Vec3) -> Vec2D {
        let h_xz = homo.x * homo.x + homo.z * homo.z;
        let h_xz_r = h_xz.sqrt();
        let h_xyz_inv = 1.0 / (h_xz + homo.y * homo.y);
        let h_xz_inv = 1.0 / h_xz;
        Vec2D::new(
            grad.x * homo.z * h_xz_inv - grad.z * homo.x * h_xz_inv,
            -grad.x * homo.x * homo.y * h_xyz_inv / h_xz_r + grad.y * h_xz_r * h_xyz_inv
                - grad.z * homo.y * homo.z * h_xyz_inv / h_xz_r,
        )
    }

    pub fn proj2homo(proj: Vec2D) -> Vec3 {
        Vec3::new(proj.x.sin(), proj.y.tan(), proj.x.cos())
    }
}
