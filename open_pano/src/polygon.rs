use crate::geometry::Vec2D;
use crate::utils::EPS;

pub type Polygon = Vec<Vec2D>;

fn side(a: &Vec2D, b: &Vec2D, p: &Vec2D) -> f64 {
    (*b - *a).cross(&(*p - *a))
}

/// Convex hull (Andrew's monotone chain)
pub fn convex_hull(pts: &mut Vec<Vec2D>) -> Polygon {
    let n = pts.len();
    if n <= 3 {
        return pts.clone();
    }
    pts.sort_by(|a, b| {
        if a.y == b.y {
            a.x.partial_cmp(&b.x).unwrap()
        } else {
            a.y.partial_cmp(&b.y).unwrap()
        }
    });

    let mut ret: Vec<Vec2D> = Vec::new();
    ret.push(pts[0]);
    ret.push(pts[1]);

    // right link (lower hull)
    for i in 2..n {
        while ret.len() >= 2 && side(&ret[ret.len() - 2], ret.last().unwrap(), &pts[i]) <= 0.0 {
            ret.pop();
        }
        ret.push(pts[i]);
    }

    // left link (upper hull)
    let mid = ret.len();
    ret.push(pts[n - 2]);
    for i in (0..n - 2).rev() {
        while ret.len() > mid && side(&ret[ret.len() - 2], ret.last().unwrap(), &pts[i]) <= 0.0 {
            ret.pop();
        }
        ret.push(pts[i]);
    }
    ret
}

pub fn polygon_area(poly: &[Vec2D]) -> f64 {
    let n = poly.len();
    let mut sum = 0.0;
    for i in 0..n {
        let xi = poly[i].x;
        let yi_next = poly[(i + 1) % n].y;
        let yi_prev = poly[(i + n - 1) % n].y;
        sum += xi * (yi_next - yi_prev);
    }
    0.5 * sum.abs()
}

fn get_com(poly: &[Vec2D]) -> Vec2D {
    let mut ret = Vec2D::zero();
    for p in poly {
        ret += *p;
    }
    ret * (1.0 / poly.len() as f64)
}

/// Point-in-polygon query using angle sorting
pub struct PointInPolygon<'a> {
    poly: &'a [Vec2D],
    com: Vec2D,
    slopes: Vec<(f32, usize)>,
}

impl<'a> PointInPolygon<'a> {
    pub fn new(poly: &'a [Vec2D]) -> Self {
        assert!(poly.len() >= 3);
        let com = get_com(poly);
        let mut slopes: Vec<(f32, usize)> = poly
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let k = ((p.y - com.y) as f32).atan2((p.x - com.x) as f32);
                (k, i)
            })
            .collect();
        slopes.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        PointInPolygon { poly, com, slopes }
    }

    pub fn in_polygon(&self, p: Vec2D) -> bool {
        let k = ((p.y - self.com.y) as f32).atan2((p.x - self.com.x) as f32);
        let idx = self.slopes.partition_point(|&(s, _)| s < k);

        let (idx1, idx2) = if idx == self.slopes.len() {
            (self.slopes.last().unwrap().1, self.slopes[0].1)
        } else {
            let i2 = self.slopes[idx].1;
            let i1 = if idx == 0 {
                self.slopes.last().unwrap().1
            } else {
                self.slopes[idx - 1].1
            };
            (i1, i2)
        };

        let p1 = self.poly[idx1];
        let p2 = self.poly[idx2];
        let o1 = side(&p1, &p2, &self.com);
        let o2 = side(&p1, &p2, &p);
        !(o1 * o2 < -EPS)
    }
}
