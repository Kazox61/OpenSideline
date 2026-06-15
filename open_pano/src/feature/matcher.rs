use crate::config::config;
use crate::feature::Descriptor;

#[derive(Clone, Debug)]
pub struct MatchData {
    pub data: Vec<(usize, usize)>,
}

impl MatchData {
    pub fn new() -> Self {
        MatchData { data: Vec::new() }
    }
    pub fn size(&self) -> usize {
        self.data.len()
    }
    pub fn reverse(&mut self) {
        for pair in &mut self.data {
            *pair = (pair.1, pair.0);
        }
    }
}

/// Brute-force pairwise feature matcher between two descriptor sets.
pub struct FeatureMatcher<'a> {
    feat1: &'a [Descriptor],
    feat2: &'a [Descriptor],
}

impl<'a> FeatureMatcher<'a> {
    pub fn new(feat1: &'a [Descriptor], feat2: &'a [Descriptor]) -> Self {
        FeatureMatcher { feat1, feat2 }
    }

    pub fn match_features(&self) -> MatchData {
        let cfg = config();
        let reject_ratio_sqr = cfg.match_reject_next_ratio * cfg.match_reject_next_ratio;

        let l1 = self.feat1.len();
        let l2 = self.feat2.len();
        let rev = l1 > l2;

        let (pf1, pf2, ln1, ln2) = if rev {
            (self.feat2, self.feat1, l2, l1)
        } else {
            (self.feat1, self.feat2, l1, l2)
        };

        let mut ret = MatchData::new();

        for k in 0..ln1 {
            let dsc1 = &pf1[k];
            let mut min_idx = 0usize;
            let mut min = f32::MAX;
            let mut next_min = f32::MAX;

            for kk in 0..ln2 {
                let dist = dsc1.euclidean_sqr(&pf2[kk], next_min);
                if dist < min {
                    next_min = min;
                    min = dist;
                    min_idx = kk;
                } else if dist < next_min {
                    next_min = dist;
                }
            }

            if min > reject_ratio_sqr * next_min {
                continue;
            }

            // Bidirectional check: fix min_idx, see if k is distinctive in pf1
            let dsc2 = &pf2[min_idx];
            let mut next_min2 = f32::MAX;
            for kk in 0..ln1 {
                if kk == k {
                    continue;
                }
                let dist = dsc2.euclidean_sqr(&pf1[kk], next_min2);
                if dist < next_min2 {
                    next_min2 = dist;
                }
            }
            if min > reject_ratio_sqr * next_min2 {
                continue;
            }

            ret.data.push((k, min_idx));
        }

        if rev {
            ret.reverse();
        }
        ret
    }
}

/// Pairwise matcher for multiple image feature sets.
/// Builds one index per image and matches pairs using brute-force KNN.
pub struct PairWiseMatcher<'a> {
    feats: &'a [Vec<Descriptor>],
}

impl<'a> PairWiseMatcher<'a> {
    pub fn new(feats: &'a [Vec<Descriptor>]) -> Self {
        PairWiseMatcher { feats }
    }

    /// Match descriptors of image id1 against image id2.
    /// Returns pairs (idx_in_id1, idx_in_id2).
    pub fn match_pair(&self, mut id1: usize, mut id2: usize) -> MatchData {
        let cfg = config();
        let reject_ratio_sqr = cfg.match_reject_next_ratio * cfg.match_reject_next_ratio;

        let rev = self.feats[id1].len() > self.feats[id2].len();
        if rev {
            std::mem::swap(&mut id1, &mut id2);
        }

        let source = &self.feats[id1];
        let target = &self.feats[id2];

        let mut ret = MatchData::new();

        for (i, dsc1) in source.iter().enumerate() {
            // Find top-2 nearest neighbors in target
            let mut min_idx = 0usize;
            let mut mind = f32::MAX;
            let mut mind2 = f32::MAX;

            for (j, dsc2) in target.iter().enumerate() {
                let d = dsc1.euclidean_sqr(dsc2, mind2);
                if d < mind {
                    mind2 = mind;
                    mind = d;
                    min_idx = j;
                } else if d < mind2 {
                    mind2 = d;
                }
            }

            if mind > reject_ratio_sqr * mind2 {
                continue;
            }

            // Bidirectional check
            let dsc_target = &target[min_idx];
            let mut mind2_inv = f32::MAX;
            let mut bidirectional_mini = 0usize;
            let mut mind_inv = f32::MAX;
            for (k, dsc_src) in source.iter().enumerate() {
                let d = dsc_target.euclidean_sqr(dsc_src, mind2_inv);
                if d < mind_inv {
                    mind2_inv = mind_inv;
                    mind_inv = d;
                    bidirectional_mini = k;
                } else if d < mind2_inv {
                    mind2_inv = d;
                }
            }

            if bidirectional_mini != i {
                continue;
            }
            if mind > reject_ratio_sqr * mind2_inv {
                continue;
            }

            ret.data.push((i, min_idx));
        }

        if rev {
            ret.reverse();
        }
        ret
    }
}
