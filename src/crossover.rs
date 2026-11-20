//! 2.1ch crossover done on the ESP32: 4th-order Linkwitz-Riley (two cascaded
//! 2nd-order Butterworth sections), high-pass on L/R for the main amplifier
//! and low-pass on the mono sum (L+R)/2 for the woofer amplifier. The two LR4
//! outputs sum flat in magnitude and phase at the crossover frequency.
//! The woofer path also gets a subsonic high-pass (4th order, 24 dB/oct) that
//! removes content below what the woofer can reproduce, which otherwise only
//! drives the cone into excessive excursion (buzzing).
//! A peaking EQ (bass boost) on the woofer path makes up for the small
//! woofer's low-frequency roll-off.
// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Hiroshi Nakajima

use std::f32::consts::{FRAC_1_SQRT_2, PI};

/// Transposed direct form II biquad.
#[derive(Clone, Copy, Default)]
struct Biquad {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    z1: f32,
    z2: f32,
}

impl Biquad {
    /// Butterworth (Q = 1/sqrt(2)) low- or high-pass, RBJ cookbook form.
    fn butterworth(high_pass: bool, freq: f32, rate: f32) -> Self {
        let w0 = 2.0 * PI * freq / rate;
        let (sin, cos) = w0.sin_cos();
        let alpha = sin / (2.0 * FRAC_1_SQRT_2);
        let a0 = 1.0 + alpha;
        let (b0, b1) = if high_pass { ((1.0 + cos) / 2.0, -(1.0 + cos)) } else { ((1.0 - cos) / 2.0, 1.0 - cos) };
        Self {
            b0: b0 / a0,
            b1: b1 / a0,
            b2: b0 / a0,
            a1: -2.0 * cos / a0,
            a2: (1.0 - alpha) / a0,
            z1: 0.0,
            z2: 0.0,
        }
    }

    /// Peaking EQ: `gain_db` at `freq`, bandwidth set by `q`, RBJ cookbook form.
    fn peaking(freq: f32, gain_db: f32, q: f32, rate: f32) -> Self {
        let a = 10f32.powf(gain_db / 40.0);
        let w0 = 2.0 * PI * freq / rate;
        let (sin, cos) = w0.sin_cos();
        let alpha = sin / (2.0 * q);
        let a0 = 1.0 + alpha / a;
        Self {
            b0: (1.0 + alpha * a) / a0,
            b1: -2.0 * cos / a0,
            b2: (1.0 - alpha * a) / a0,
            a1: -2.0 * cos / a0,
            a2: (1.0 - alpha / a) / a0,
            z1: 0.0,
            z2: 0.0,
        }
    }

    /// Takes the coefficients of `new` but keeps the filter state, so the
    /// output does not jump when a setting changes during playback.
    fn retune(&mut self, new: Self) {
        *self = Self { z1: self.z1, z2: self.z2, ..new };
    }

    #[inline]
    fn run(&mut self, x: f32) -> f32 {
        let y = self.b0 * x + self.z1;
        self.z1 = self.b1 * x - self.a1 * y + self.z2;
        self.z2 = self.b2 * x - self.a2 * y;
        y
    }
}

/// Two cascaded Butterworth sections = one LR4 filter.
#[derive(Clone, Copy, Default)]
struct Lr4([Biquad; 2]);

impl Lr4 {
    fn new(high_pass: bool, freq: f32, rate: f32) -> Self {
        let q = Biquad::butterworth(high_pass, freq, rate);
        Self([q, q])
    }

    /// New coefficients, filter state kept (see `Biquad::retune`).
    fn retune(&mut self, new: Self) {
        self.0[0].retune(new.0[0]);
        self.0[1].retune(new.0[1]);
    }

    #[inline]
    fn run(&mut self, x: f32) -> f32 {
        let y = self.0[0].run(x);
        self.0[1].run(y)
    }
}

pub struct Crossover {
    freq: f32,
    rate: f32,
    // 0 = off.
    subsonic_hz: u32,
    hp_l: Lr4,
    hp_r: Lr4,
    lp: Lr4,
    subsonic: Lr4,
    // Bass EQ: (Hz, dB); 0 dB = off.
    eq_hz: u32,
    eq_db: u32,
    eq: Biquad,
}

// Bandwidth of the bass EQ: about 1.5 octaves.
const EQ_Q: f32 = 0.9;

impl Crossover {
    pub fn new(freq_hz: u32, subsonic_hz: u32, rate: u32) -> Self {
        let mut x = Self {
            freq: freq_hz as f32,
            rate: 0.0,
            subsonic_hz,
            hp_l: Lr4::default(),
            hp_r: Lr4::default(),
            lp: Lr4::default(),
            subsonic: Lr4::default(),
            eq_hz: 60,
            eq_db: 0,
            eq: Biquad::default(),
        };
        x.set_rate(rate);
        x
    }

    /// Recomputes the coefficients (and clears the filter state) for a new
    /// sample rate.
    pub fn set_rate(&mut self, rate: u32) {
        let rate = rate as f32;
        self.rate = rate;
        self.hp_l = Lr4::new(true, self.freq, rate);
        self.hp_r = self.hp_l;
        self.lp = Lr4::new(false, self.freq, rate);
        self.subsonic = Lr4::new(true, self.subsonic_hz.max(1) as f32, rate);
        self.eq = Biquad::peaking(self.eq_hz as f32, self.eq_db as f32, EQ_Q, rate);
    }

    /// Changes the woofer bass EQ (centre Hz, boost dB; 0 dB = off); no-op if
    /// unchanged.
    pub fn set_eq(&mut self, hz: u32, db: u32) {
        if (hz, db) != (self.eq_hz, self.eq_db) {
            (self.eq_hz, self.eq_db) = (hz, db);
            self.eq.retune(Biquad::peaking(hz as f32, db as f32, EQ_Q, self.rate));
        }
    }

    /// Changes the crossover frequency; no-op if unchanged.
    pub fn set_freq(&mut self, hz: u32) {
        let freq = hz as f32;
        if freq != self.freq {
            self.freq = freq;
            let hp = Lr4::new(true, freq, self.rate);
            self.hp_l.retune(hp);
            self.hp_r.retune(hp);
            self.lp.retune(Lr4::new(false, freq, self.rate));
        }
    }

    /// Changes the woofer subsonic high-pass (0 = off); no-op if unchanged.
    pub fn set_subsonic(&mut self, hz: u32) {
        if hz != self.subsonic_hz {
            self.subsonic_hz = hz;
            self.subsonic.retune(Lr4::new(true, hz.max(1) as f32, self.rate));
        }
    }

    /// Returns (main L, main R, woofer mono). The subsonic filter and the EQ
    /// always run (a 0 dB peaking EQ passes the signal unchanged), so turning
    /// them on or off does not restart them from a cleared state.
    #[inline]
    pub fn run(&mut self, l: f32, r: f32) -> (f32, f32, f32) {
        let low = self.lp.run(0.5 * (l + r));
        let sub = self.subsonic.run(low);
        let low = if self.subsonic_hz > 0 { sub } else { low };
        let woofer = self.eq.run(low);
        (self.hp_l.run(l), self.hp_r.run(r), woofer)
    }
}

/// Peak limiter that keeps a path below 16-bit full scale. Gain drops at once
/// when a sample would exceed the ceiling and recovers over `RELEASE_S`, so
/// loud peaks are turned down smoothly instead of hard-clipped (which crackles).
pub struct Limiter {
    ceiling: f32,
    gain: f32,
    release: f32,
    /// Samples at which the limiter reduced the gain.
    pub engaged: u32,
}

/// Default ceiling, just below 16-bit full scale.
pub const CEILING: f32 = 32_000.0;
const RELEASE_S: f32 = 0.2;

impl Limiter {
    pub fn new(rate: u32, ceiling: f32) -> Self {
        let mut l = Self { ceiling, gain: 1.0, release: 0.0, engaged: 0 };
        l.set_rate(rate);
        l
    }

    pub fn set_rate(&mut self, rate: u32) {
        self.release = 1.0 - (-1.0 / (RELEASE_S * rate as f32)).exp();
    }

    /// Returns the gain to apply to a sample (or linked stereo pair) whose
    /// absolute peak is `peak`.
    #[inline]
    pub fn gain(&mut self, peak: f32) -> f32 {
        self.gain += (1.0 - self.gain) * self.release;
        if peak * self.gain > self.ceiling {
            self.gain = self.ceiling / peak;
            self.engaged += 1;
        }
        self.gain
    }
}
