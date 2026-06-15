use crate::feature::{Descriptor, SSPoint};
use crate::mat::Mat32f;

pub struct BriefPattern {
    pub s: usize,
    pub pattern: Vec<(usize, usize)>,
}

pub struct Brief<'a> {
    img: &'a Mat32f,
    points: &'a [SSPoint],
    pattern: &'a BriefPattern,
}

impl<'a> Brief<'a> {
    pub fn new(img: &'a Mat32f, points: &'a [SSPoint], pattern: &'a BriefPattern) -> Self {
        Brief {
            img,
            points,
            pattern,
        }
    }

    pub fn get_descriptor(&self) -> Vec<Descriptor> {
        let half = self.pattern.s / 2;
        let mut ret = Vec::new();
        for p in self.points {
            let x = (p.real_coor.x * self.img.width() as f64).round() as usize;
            let y = (p.real_coor.y * self.img.height() as f64).round() as usize;
            if x >= half && x + half < self.img.width() && y >= half && y + half < self.img.height()
            {
                ret.push(self.calc_descriptor(p));
            }
        }
        ret
    }

    fn calc_descriptor(&self, p: &SSPoint) -> Descriptor {
        let x = (p.real_coor.x * self.img.width() as f64).round() as i32;
        let y = (p.real_coor.y * self.img.height() as f64).round() as i32;
        let n = self.pattern.pattern.len();
        let half = self.pattern.s as i32 / 2;
        let s = self.pattern.s as i32;

        let pixel = |r: i32, c: i32| -> f32 {
            let p = self.img.pixel(r as usize, c as usize);
            (p[0] + p[1] + p[2]) / 3.0
        };

        let mut bits = vec![false; n];
        for (i, &(p1, p2)) in self.pattern.pattern.iter().enumerate() {
            let y1 = y + p1 as i32 / s - half;
            let x1 = x + p1 as i32 % s - half;
            let y2 = y + p2 as i32 / s - half;
            let x2 = x + p2 as i32 % s - half;
            bits[i] = pixel(y1, x1) > pixel(y2, x2);
        }

        // Pack bits into f32 slots (reinterpreting u32 as f32, matching C++)
        let nwords = n / 32;
        let mut descriptor = vec![0.0f32; nwords];
        for (i, &bit) in bits.iter().enumerate() {
            if bit {
                let idx = i / 32;
                let offset = i % 32;
                let word = descriptor[idx].to_bits();
                descriptor[idx] = f32::from_bits(word | (1u32 << offset));
            }
        }

        Descriptor {
            coor: p.real_coor,
            descriptor,
        }
    }

    pub fn gen_brief_pattern(s: usize, n: usize) -> BriefPattern {
        assert!(s % 2 == 1);
        assert!(n % 32 == 0);

        // Use a simple pseudo-random generator seeded with a fixed value
        // to approximate the C++ normal distribution sampling
        let mut rng_state: u64 = 42;
        let next_u64 = |state: &mut u64| -> u64 {
            *state ^= *state << 13;
            *state ^= *state >> 7;
            *state ^= *state << 17;
            *state
        };

        let mean = s as f64 * 0.5;
        let std = s as f64 * 0.2;

        let get_sample = |state: &mut u64| -> usize {
            loop {
                // Box-Muller transform
                let u1 = (next_u64(state) as f64) / u64::MAX as f64;
                let u2 = (next_u64(state) as f64) / u64::MAX as f64;
                let z = (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos();
                let v = (mean + std * z).round() as i64;
                if v >= 0 && (v as usize) < s {
                    return v as usize;
                }
            }
        };

        let mut pattern = Vec::new();
        let mut n_left = n;
        while n_left > 0 {
            let x1 = get_sample(&mut rng_state);
            let y1 = get_sample(&mut rng_state);
            let x2;
            let y2;
            loop {
                let a = get_sample(&mut rng_state);
                let b = get_sample(&mut rng_state);
                if !(y1 == x1 && b == a) {
                    x2 = a;
                    y2 = b;
                    break;
                }
            }
            pattern.push((y1 * s + x1, y2 * s + x2));
            n_left -= 1;
        }

        BriefPattern { s, pattern }
    }
}
