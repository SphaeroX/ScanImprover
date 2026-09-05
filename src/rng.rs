pub struct Rng {
    s: [u64; 2],
}

impl Rng {
    pub fn new(seed: u64) -> Self {
        let mut z = seed.wrapping_add(0x9E3779B97F4A7C15);
        let mut s = [0u64; 2];
        for slot in s.iter_mut() {
            z = z.wrapping_add(0x9E3779B97F4A7C15);
            let mut x = z;
            x = (x ^ (x >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
            x = (x ^ (x >> 27)).wrapping_mul(0x94D049BB133111EB);
            *slot = x ^ (x >> 31);
        }
        Rng { s }
    }

    pub fn next_u64(&mut self) -> u64 {
        let s0 = self.s[0];
        let mut s1 = self.s[1];
        let result = s0.wrapping_mul(5).rotate_left(7).wrapping_mul(9);
        s1 ^= s0;
        self.s[0] = s0.rotate_left(24) ^ s1 ^ (s1 << 16);
        self.s[1] = s1.rotate_left(37);
        result
    }

    pub fn f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / ((1u64 << 53) as f64))
    }

    pub fn f32(&mut self) -> f32 {
        (self.next_u64() >> 40) as f32 * (1.0 / ((1u64 << 24) as f64) as f32)
    }
}
