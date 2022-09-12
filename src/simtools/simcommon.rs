extern crate nalgebra as na;
use na::{DVector};

use anyhow::{Context};

use std::f64::consts::{PI};

/// 重力加速度[m/s^2]
pub const G: f64 = 9.81;

 // よくある単位変換のメソッドを集めたトレイト
pub trait UnitTrans {
    fn rad2deg(&self) -> Self;
    fn deg2rad(&self) -> Self;
    fn kph2mps(&self) -> Self;
    fn mps2kph(&self) -> Self;
} 

impl UnitTrans for f64 {
    fn rad2deg(&self) -> f64 {
        self * 180.0 / PI
    }

    fn deg2rad(&self) -> f64 {
        self * PI / 180.0
    }

    fn kph2mps(&self) -> f64 {
        self / 3.6
    }

    fn mps2kph(&self) -> f64 {
        self * 3.6
    }
}

/// 積分器
pub struct Integrator {
    sample_time: f64,
    elemnum: usize,
    x: DVector<f64>,
}

impl Integrator {
    pub fn new(elemnum: usize, sample_time: f64) -> Self {
        Self {
            x: DVector::from_element(elemnum, 0.0), 
            elemnum: elemnum,
            sample_time: sample_time,
        }
    }

    pub fn reset(&mut self) {
        self.x = DVector::from_element(self.elemnum, 0.0);
    }

    pub fn calc(&mut self, x: &[f64]) {

    }

}
