extern crate nalgebra as na;

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

pub trait Saturation<T: Clone> {
    fn guard_min(self, guard: T) -> Self;
    fn guard_max(self, guard: T) -> Self;
    fn guard_minmax(self, guard: (T, T)) -> Self;
}

impl Saturation<f64> for f64 {
    fn guard_min(self, guard: f64) -> f64 {
        if self < guard {
            guard
        }
        else {
            self
        }
    }

    fn guard_max(self, guard: f64) -> f64 {
        if self > guard {
            guard
        } 
        else {
            self
        }
    }

    fn guard_minmax(self, guard: (f64, f64)) -> f64 {
        let temp = self.guard_max(guard.1);
        temp.guard_min(guard.0)
    }
}