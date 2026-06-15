use crate::geometry::Vec2D;
use crate::stitch::homography::Homography;

pub type PointCorr = (Vec2D, Vec2D);

#[derive(Clone)]
pub struct MatchInfo {
    pub match_pairs: Vec<PointCorr>,
    pub confidence: f32,
    pub homo: Homography,
}

impl MatchInfo {
    pub fn new() -> Self {
        MatchInfo {
            match_pairs: Vec::new(),
            confidence: 0.0,
            homo: Homography::identity(),
        }
    }

    pub fn reverse(&mut self) {
        for p in &mut self.match_pairs {
            std::mem::swap(&mut p.0, &mut p.1);
        }
    }
}

#[derive(Clone, Debug, Copy)]
pub struct Shape2D {
    pub w: i32,
    pub h: i32,
}

impl Shape2D {
    pub fn new(w: i32, h: i32) -> Self {
        Shape2D { w, h }
    }
    pub fn halfw(&self) -> f64 {
        self.w as f64 * 0.5
    }
    pub fn halfh(&self) -> f64 {
        self.h as f64 * 0.5
    }
    pub fn center(&self) -> Vec2D {
        Vec2D::new(self.halfw(), self.halfh())
    }

    pub fn shifted_corners(&self) -> [Vec2D; 4] {
        let hw = self.halfw();
        let hh = self.halfh();
        [
            Vec2D::new(-hw, -hh),
            Vec2D::new(hw, -hh),
            Vec2D::new(-hw, hh),
            Vec2D::new(hw, hh),
        ]
    }

    pub fn shifted_in(&self, p: Vec2D) -> bool {
        p.x >= -self.halfw() && p.x < self.halfw() && p.y >= -self.halfh() && p.y < self.halfh()
    }
}
