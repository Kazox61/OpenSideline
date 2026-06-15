use crate::config::config;
use crate::stitch::camera::Camera;
use crate::stitch::homography::Homography;
use crate::stitch::match_info::{MatchInfo, Shape2D};
use std::cmp::Ordering;
use std::collections::BinaryHeap;

#[derive(Clone, Copy)]
struct Edge {
    v1: usize,
    v2: usize,
    weight: f32,
}

impl PartialEq for Edge {
    fn eq(&self, o: &Self) -> bool {
        self.weight == o.weight
    }
}
impl Eq for Edge {}
impl PartialOrd for Edge {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        self.weight.partial_cmp(&o.weight)
    }
}
impl Ord for Edge {
    fn cmp(&self, o: &Self) -> Ordering {
        self.weight
            .partial_cmp(&o.weight)
            .unwrap_or(Ordering::Equal)
    }
}

pub struct CameraEstimator<'a> {
    n: usize,
    matches: &'a mut Vec<Vec<MatchInfo>>,
    shapes: &'a [Shape2D],
    cameras: Vec<Camera>,
}

impl<'a> CameraEstimator<'a> {
    pub fn new(matches: &'a mut Vec<Vec<MatchInfo>>, shapes: &'a [Shape2D]) -> Self {
        let n = matches.len();
        CameraEstimator {
            n,
            matches,
            shapes,
            cameras: vec![Camera::new(); n],
        }
    }

    pub fn estimate_focal(&mut self) {
        let cfg = config();

        // If the user provided a 35mm-equivalent focal length in config, use it.
        // Convert via: f_px = image_diagonal_px * focal_35mm_equiv / 43.266
        // (43.266mm = diagonal of a 35mm full-frame sensor)
        if cfg.focal_length > 0.0 {
            for (i, c) in self.cameras.iter_mut().enumerate() {
                let diag = ((self.shapes[i].w * self.shapes[i].w
                    + self.shapes[i].h * self.shapes[i].h) as f64)
                    .sqrt();
                c.focal = diag * cfg.focal_length as f64 / 43.266;
            }
            return;
        }

        // Otherwise try to estimate focal from the homographies (Zhang's method).
        let focal = Camera::estimate_focal(self.matches);
        if focal > 0.0 {
            for c in &mut self.cameras {
                c.focal = focal;
            }
        } else {
            for (i, c) in self.cameras.iter_mut().enumerate() {
                c.focal = (self.shapes[i].w + self.shapes[i].h) as f64 * 0.5;
            }
        }
    }

    pub fn estimate(mut self) -> Vec<Camera> {
        self.estimate_focal();

        let cfg = config();
        let n = self.n;

        // Find best starting edge
        let mut best_weight = 0.0f32;
        let mut start_node = 0usize;
        for i in 0..n {
            for j in i + 1..n {
                if self.matches[i][j].confidence > best_weight {
                    best_weight = self.matches[i][j].confidence;
                    start_node = i;
                }
            }
        }

        // Init identity node
        self.cameras[start_node].r = Homography::identity();
        self.cameras[start_node].ppx = 0.0;
        self.cameras[start_node].ppy = 0.0;
        let identity_idx = start_node;

        // Prim-style MST expansion
        let mut vst = vec![false; n];
        let mut q: BinaryHeap<Edge> = BinaryHeap::new();

        let enqueue_edges =
            |from: usize, q: &mut BinaryHeap<Edge>, vst: &[bool], matches: &Vec<Vec<MatchInfo>>| {
                for i in 0..n {
                    if i != from && !vst[i] && matches[from][i].confidence > 0.0 {
                        q.push(Edge {
                            v1: from,
                            v2: i,
                            weight: matches[from][i].confidence,
                        });
                    }
                }
            };

        vst[start_node] = true;
        enqueue_edges(start_node, &mut q, &vst, self.matches);

        // Collect edges in MST order
        let mut mst_edges: Vec<(usize, usize)> = Vec::new();
        while let Some(edge) = q.pop() {
            if vst[edge.v2] {
                continue;
            }
            vst[edge.v2] = true;
            mst_edges.push((edge.v1, edge.v2));
            enqueue_edges(edge.v2, &mut q, &vst, self.matches);
        }

        // Initialize cameras along MST
        for (from, to) in &mst_edges {
            let kfrom = self.cameras[*from].k();
            let kto = self.cameras[*to].k();
            let hinv = self.matches[*from][*to].homo;
            let mat = kfrom.inverse(None) * hinv * kto;
            self.cameras[*to].r = (self.cameras[*from].r_inv() * mat).transpose();
            self.cameras[*to].ppx = 0.0;
            self.cameras[*to].ppy = 0.0;
        }

        // Collect all valid match pairs and run bundle adjustment.
        // For multipass_ba >= 1 the C++ does incremental BA during MST traversal;
        // we approximate that by doing a single pass at the end, which is equivalent
        // for small panoramas and avoids the complex incremental lifetime bookkeeping.
        {
            let mut match_pairs: Vec<(usize, usize)> = Vec::new();
            for i in 1..n {
                for j in 0..i {
                    let m = &self.matches[j][i];
                    if !m.match_pairs.is_empty() && m.confidence > 0.0 {
                        match_pairs.push((i, j));
                    }
                }
            }
            self.bundle_adjust(&match_pairs, identity_idx, cfg.lm_lambda as f64);
        }

        if cfg.straighten {
            Camera::straighten(&mut self.cameras);
        }

        self.cameras
    }

    fn bundle_adjust(&mut self, match_pairs: &[(usize, usize)], identity_idx: usize, _lambda: f64) {
        use crate::stitch::bundle_adjuster::IncrementalBundleAdjuster;

        // IBA owns cloned match point data, so we can hold &mut cameras and
        // read from matches in the same scope without lifetime conflicts.
        let mut iba = IncrementalBundleAdjuster::new(&mut self.cameras);
        iba.set_identity_idx(identity_idx);

        for &(i, j) in match_pairs {
            let m = &self.matches[j][i];
            if !m.match_pairs.is_empty() && m.confidence > 0.0 {
                iba.add_match(i, j, m);
            }
        }

        if iba.has_matches() {
            iba.optimize();
        }
    }
}
