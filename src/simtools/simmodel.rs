use std::fmt;

use crate::simtools::signal::{*};

extern crate nalgebra as na;
use na::{DMatrix};

/// Modelトレイト
pub trait Model {
    /// シミュレーション時間を1ステップ進める
    fn nextstate(&mut self, delta_t: f64);
    fn output(&self) -> Bus;
}

#[derive(Debug, Clone)]
pub enum SolverType {
    Euler,
    RungeKutta,
}

/// DEModelトレイト
pub trait DEModel: Model{
    fn derivative_func(&self, x: &DMatrix<f64>) -> DMatrix<f64>; // 導関数を定義する
    fn set_state(&mut self, newstate: DMatrix<f64>);
    fn get_state(&self) -> &DMatrix<f64>;
    fn euler_method(&mut self, delta_t: f64) {
        let state = self.get_state();
        let newstate = state + self.derivative_func(state) * delta_t;
        self.set_state(newstate);
    }
    fn rungekutta_method(&mut self, delta_t: f64) {
        let state = self.get_state();
        let d1 = self.derivative_func(state) * delta_t;
        let d2 = self.derivative_func(&(state + &d1 / 2.0)) * delta_t;
        let d3 = self.derivative_func(&(state + &d2 / 2.0)) * delta_t;
        let d4 = self.derivative_func(&(state + &d3)) * delta_t;
        let newstate = state + (d1 + 2.0 * d2 + 2.0 * d3 + d4) / 6.0;
        self.set_state(newstate);
    }
}

/// 状態空間モデル
#[derive(Debug, Clone)]
pub struct SpaceStateModel {
    mat_a: DMatrix<f64>,    // 状態遷移行列A
    mat_b: DMatrix<f64>,    // 入力行列B
    mat_c: DMatrix<f64>,    // 観測行列C
    mat_d: DMatrix<f64>,    // 入力行列D
    state_dim: usize,       // 状態次数
    input_dim: usize,       // 入力次数
    output_dim: usize,      // 出力次数
    x: DMatrix<f64>,        // 状態ベクトル
    u: DMatrix<f64>,        // 入力ベクトル
    solver: SolverType,
}

impl SpaceStateModel {
    pub fn new(sdim: usize, idim: usize, odim: usize, solvertype: SolverType) -> Self {
        SpaceStateModel {
            mat_a: DMatrix::from_element(sdim, sdim, 0.0),
            mat_b: DMatrix::from_element(sdim, idim, 0.0),
            mat_c: DMatrix::from_element(odim, sdim, 0.0),
            mat_d: DMatrix::from_element(odim, idim, 0.0),
            x: DMatrix::from_element(sdim, 1, 0.0),
            u: DMatrix::from_element(idim, 1, 0.0),
            state_dim: sdim,
            input_dim: idim,
            output_dim: odim,
            solver: solvertype,
        }
    }

    // 伝達関数から状態空間モデルを生成する  ⇒　Fromトレイトでの実装に変更すること
    pub fn from_tf<'a> (num: &'a [f64], den: &'a [f64], solvertype: SolverType) -> Result<Self, &'a str> {
        let sdim = den.len() - 1;
        let idim = 1;
        let odim = 1;

        if sdim < 1 {
            return Err("次数が0になりました。")
        }
        if num.len() > den.len() {
            return Err("プロパーな伝達関数ではありません。")
        }

        let mut model = SpaceStateModel::new(sdim, idim, odim, solvertype);
        let an = den[0];

        // A行列の作成
        let mut mat_a = vec![0.0; sdim * sdim];
        
        for r in 0..sdim {
            for c in 0..sdim {
                if c == sdim - 1 {
                    mat_a[r * sdim + c] = -den[sdim - r] / an;
                }
                else {
                    mat_a[r * sdim + c] = 0.0;
                }
            }

            if r > 0 {
                mat_a[r * sdim + r - 1] = 1.0;
            }
        }

        model.set_mat_a(&mat_a);

        // B行列の作成
        let mut mat_b = vec![0.0; sdim * idim];
        let bn = 
            if num.len() - 1 == sdim {
                num[0]
            } else {
                0.0
            };
        
        for r in 0..sdim {
            if r < num.len() {
                mat_b[r] = (num[num.len() - r - 1] - den[sdim - r] * bn ) / an;
            } else {
                mat_b[r] = (-den[sdim - r] * bn ) / an;
            }

        }
        model.set_mat_b(&mat_b);

        // C行列の作成
        let mut mat_c = vec![0.0; sdim * odim];
        mat_c[sdim - 1] = 1.0;
        model.set_mat_c(&mat_c);

        // D行列の作成
        let mut mat_d = vec![bn; odim * idim];
        model.set_mat_d(&mat_d);

        Ok(model)
    }

    pub fn init_state(&mut self, init_state: &[f64]) -> Result<(), &str> {

        if init_state.len() != self.state_dim {
            return Err("状態変数のサイズが違います。");
        }

        for (i, elem) in init_state.iter().enumerate() {
            self.x[i] = *elem;
        }

        Ok(())
    }

    pub fn set_mat_a(&mut self, mat_a: &[f64]) -> Result<(), &str> {

        if mat_a.len() != self.state_dim * self.state_dim {
            return Err("A行列のサイズが違います。");
        }

        for (i, elem) in mat_a.iter().enumerate() {
            self.mat_a[(i / self.state_dim, i % self.state_dim)] = *elem;
        }

        Ok(())
    }

    pub fn set_mat_b(&mut self, mat_b: &[f64]) -> Result<(), &str> {
        
        if mat_b.len() != self.state_dim * self.input_dim {
            return Err("B行列のサイズが違います。");
        }

        for (i, elem) in mat_b.iter().enumerate() {
            self.mat_b[(i / self.input_dim, i % self.input_dim)] = *elem;
        }

        Ok(())
    }

    pub fn set_mat_c(&mut self, mat_c: &[f64]) -> Result<(), &str> {

        if mat_c.len() != self.output_dim * self.state_dim {
            return Err("C行列のサイズが違います。");
        }

        for (i, elem) in mat_c.iter().enumerate() {
            self.mat_c[(i / self.state_dim, i % self.state_dim)] = *elem;
        }

        Ok(())
    }

    pub fn set_mat_d(&mut self, mat_d: &[f64]) -> Result<(), &str> {

        if mat_d.len() != self.output_dim * self.input_dim {
            return Err("D行列のサイズが違います。");
        }

        for (i, elem) in mat_d.iter().enumerate() {
            self.mat_d[(i / self.input_dim, i % self.input_dim)] = *elem;
        }

        Ok(())
    }

    pub fn set_x(&mut self, x: &[f64]) -> Result<(), &str> {
        if x.len() != self.state_dim {
            return Err("状態ベクトルの次数が違います。")
        }
        for (i, elem) in x.iter().enumerate() {
            self.x[i] = *elem;
        }

        Ok(())
    }

    pub fn set_u(&mut self, u: &[f64]) -> Result<(), &str> {
        if u.len() != self.input_dim {
            return Err("入力ベクトルの次数が違います。")
        }
        for (i, elem) in u.iter().enumerate() {
            self.u[i] = *elem;
        }

        Ok(())
    }

    pub fn get_observation(&self) -> DMatrix<f64> {
        &self.mat_c * &self.x + &self.mat_d * &self.u
    }

}

impl Model for SpaceStateModel {
    fn output(&self) -> Bus {
        Bus::new()
    }

    fn nextstate(&mut self, delta_t: f64) {
        match self.solver { 
            SolverType::Euler => self.euler_method(delta_t),
            SolverType::RungeKutta => self.rungekutta_method(delta_t),
        }
    }
}

impl DEModel for SpaceStateModel {
    fn derivative_func(&self, x: &DMatrix<f64>) -> DMatrix<f64> {
        &self.mat_a * x + &self.mat_b * &self.u
    }

    fn set_state(&mut self, newstate: DMatrix<f64>) {
        self.x = newstate; 
    }

    fn get_state(&self) -> &DMatrix<f64> {
        &self.x
    }
}

impl fmt::Display for SpaceStateModel {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        // ここに状態モデルを見やすく表示するための処理を記述する
        let mut str_a = String::from(format!("Matrix A ({0} x {0}): \n", self.state_dim));
        for r in 0..self.state_dim {
            str_a = str_a + &"|";
            for c in 0..self.state_dim {
                str_a = str_a + &format!("{:>15.5} ", self.mat_a[(r, c)]);
            }
            str_a = str_a + &format!("|\n")
        }

        let mut str_b = String::from(format!("Matrix B ({} x {}): \n", self.state_dim, self.input_dim));
        for r in 0..self.state_dim {
            str_b = str_b + &"|";
            for c in 0..self.input_dim {
                str_b = str_b + &format!("{:>15.5} ", self.mat_b[(r, c)]);
            }
            str_b = str_b + &format!("|\n")
        }

        let mut str_c = String::from(format!("Matrix C ({} x {}): \n", self.output_dim, self.state_dim));
        for r in 0..self.output_dim {
            str_c = str_c + &"|";
            for c in 0..self.state_dim {
                str_c = str_c + &format!("{:>15.5} ", self.mat_c[(r, c)]);
            }
            str_c = str_c + &format!("|\n")
        }

        let mut str_d = String::from(format!("Matrix D ({} x {}): \n", self.output_dim, self.input_dim));
        for r in 0..self.output_dim {
            str_d = str_d + &"|";
            for c in 0..self.input_dim {
                str_d = str_d + &format!("{:>15.5} ", self.mat_d[(r, c)]);
            }
            str_d = str_d + &format!("|\n")
        }

        
        write!(f, "{}\n{}\n{}\n{}\n", str_a, str_b, str_c, str_d)
    }
}

/// 伝達関数モデル
pub struct TransFuncModel {
    model: SpaceStateModel, // 内部的には状態空間モデルを持つ
    num: Vec<f64>,          // 分子多項式の係数 2次の例 b2 * s^2 + b1 * s + b0
    den: Vec<f64>,          // 分母多項式の係数 2次の例 a2 * s^2 + a1 * s + a0
}

impl TransFuncModel {
    pub fn new(num_coef: &[f64], den_coef: &[f64], solvertype: SolverType) -> Self {
        let model = SpaceStateModel::from_tf(&num_coef, &den_coef, solvertype).unwrap();
        Self {
            num : num_coef.to_vec(),
            den : den_coef.to_vec(),  
            model : model,
        }
    }

    pub fn set_u(&mut self, u: f64) {
        self.model.set_u(&vec![u]);
    }
}

impl Model for TransFuncModel {
    fn output(&self) -> Bus {
        Bus::new()
    }

    fn nextstate(&mut self, delta_t: f64) {
        self.model.nextstate(delta_t);
    }
}