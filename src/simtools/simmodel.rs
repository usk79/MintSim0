use std::fmt;

use crate::simtools::signal::{*};

extern crate nalgebra as na;
use na::{DMatrix};

use anyhow::{Context};


/// Modelトレイト
pub trait Model {
    /// シミュレーション時間を1ステップ進める
    fn nextstate(&mut self, delta_t: f64);
    /// モデルへのステップごとの入力
    fn interface_in(&mut self, signals: &Bus) -> anyhow::Result<()>;
    /// モデルからのステップごとの出力
    fn interface_out(&mut self) -> &Bus;
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
    name: String,
    mtrx_a: DMatrix<f64>,    // 状態遷移行列A
    mtrx_b: DMatrix<f64>,    // 入力行列B
    mtrx_c: DMatrix<f64>,    // 観測行列C
    mtrx_d: DMatrix<f64>,    // 入力行列D
    state_dim: usize,        // 状態次数
    input_dim: usize,        // 入力次数
    output_dim: usize,       // 出力次数
    x: DMatrix<f64>,         // 状態ベクトル
    u: DMatrix<f64>,         // 入力ベクトル
    solver: SolverType,      // ソルバータイプ
    state_bus: Bus, 
    input_bus: Bus,
    output_bus: Bus,
}

impl SpaceStateModel {
    pub fn new(name: &str, state_def: &Vec<SigDef>, input_def: &Vec<SigDef>, output_def: &Vec<SigDef>, solvertype: SolverType) -> anyhow::Result<Self> {
        let sdim = state_def.len();
        let idim = input_def.len();
        let odim = output_def.len();
        if sdim <= 0 || idim <= 0 || odim <= 0 {
            return Err(anyhow!("状態, 入力, 出力の次数は自然数である必要があります。"));
        }

        let mut state_bus = Bus::new();
        state_def.iter().for_each(|sig| state_bus.push(Signal::new(0.0, sig.name(), sig.unit())).unwrap());
        let mut input_bus = Bus::new();
        input_def.iter().for_each(|sig| input_bus.push(Signal::new(0.0, sig.name(), sig.unit())).unwrap());
        let mut output_bus = Bus::new();
        output_def.iter().for_each(|sig| output_bus.push(Signal::new(0.0, sig.name(), sig.unit())).unwrap());

        Ok(SpaceStateModel {
            name: name.to_string(),
            mtrx_a: DMatrix::from_element(sdim, sdim, 0.0),
            mtrx_b: DMatrix::from_element(sdim, idim, 0.0),
            mtrx_c: DMatrix::from_element(odim, sdim, 0.0),
            mtrx_d: DMatrix::from_element(odim, idim, 0.0),
            x: DMatrix::from_element(sdim, 1, 0.0),
            u: DMatrix::from_element(idim, 1, 0.0),
            state_dim: sdim,
            input_dim: idim,
            output_dim: odim,
            solver: solvertype,
            state_bus: state_bus,
            input_bus: input_bus, 
            output_bus: output_bus,
        })
    }

    pub fn init_state(&mut self, init_state: &[f64]) -> anyhow::Result<()> {
        self.set_x(init_state)?;
        Ok(())
    }

    pub fn set_x(&mut self, x: &[f64]) -> anyhow::Result<()> {
        if x.len() != self.state_dim {
            return Err(anyhow!("状態ベクトルの次数が違います。"))
        }
        x.iter().enumerate().for_each(|(i, e)| self.x[i] = *e);

        Ok(())
    }

    pub fn set_mtrx_a(&mut self, mtrx_a: &[f64]) -> anyhow::Result<()> {

        if mtrx_a.len() != self.state_dim * self.state_dim {
            return Err(anyhow!("A行列のサイズが違います。"));
        }
        mtrx_a.iter().enumerate().for_each(|(i, e)| self.mtrx_a[(i / self.state_dim, i % self.state_dim)] = *e);

        Ok(())
    }

    pub fn set_mtrx_b(&mut self, mtrx_b: &[f64]) -> anyhow::Result<()> {
        
        if mtrx_b.len() != self.state_dim * self.input_dim {
            return Err(anyhow!("B行列のサイズが違います。"));
        }

        mtrx_b.iter().enumerate().for_each(|(i, e)| self.mtrx_b[(i / self.input_dim, i % self.input_dim)] = *e);

        Ok(())
    }

    pub fn set_mtrx_c(&mut self, mtrx_c: &[f64]) -> anyhow::Result<()> {

        if mtrx_c.len() != self.output_dim * self.state_dim {
            return Err(anyhow!("C行列のサイズが違います。"));
        }
        mtrx_c.iter().enumerate().for_each(|(i, e)| self.mtrx_c[(i / self.state_dim, i % self.state_dim)] = *e);

        Ok(())
    }

    pub fn set_mtrx_d(&mut self, mtrx_d: &[f64]) -> anyhow::Result<()> {

        if mtrx_d.len() != self.output_dim * self.input_dim {
            return Err(anyhow!("D行列のサイズが違います。"));
        }
        mtrx_d.iter().enumerate().for_each(|(i, e)| self.mtrx_d[(i / self.input_dim, i % self.input_dim)] = *e);

        Ok(())
    }

    pub fn set_u(&mut self, u: &[f64]) -> anyhow::Result<()> {
        if u.len() != self.input_dim {
            return Err(anyhow!("入力ベクトルの次数が違います。"))
        }
        u.iter().enumerate().for_each(|(i, e)| self.u[i] = *e);

        Ok(())
    }

    pub fn get_observation(&self) -> DMatrix<f64> {
        &self.mtrx_c * &self.x + &self.mtrx_d * &self.u
    }

    pub fn get_statebus(&mut self) -> &Bus {
        // 結果をバスに保存する
        self.x.iter().enumerate().for_each(|(i, elem)| self.state_bus[i].value = *elem);

        &self.state_bus
    }

}

// 伝達関数から状態空間モデルを生成する
pub fn crate_ssm_from_tf<'a> (name: &str, num: &'a [f64], den: &'a [f64], solvertype: SolverType) -> anyhow::Result<SpaceStateModel> {
    let sdim = den.len() - 1;
    let idim = 1;
    let odim = 1;
    let state_def = (0..sdim).map(|x| SigDef::new(format!("s_{}", x), "-")).collect::<Vec<SigDef>>();
    let input_def = vec![SigDef::new("i_1", "-")];
    let output_def = vec![SigDef::new("o_1", "-")];

    if sdim < 1 {
        return Err(anyhow!("状態ベクトルの次数が0になりました。"))
    }
    if num.len() > den.len() {
        return Err(anyhow!("プロパーな伝達関数ではありません。"))
    }

    let mut model = SpaceStateModel::new(name, &state_def, &input_def, &output_def, solvertype)?;
    let an = den[0];

    // A行列の作成
    let mut mtrx_a = vec![0.0; sdim * sdim];
    
    for r in 0..sdim {
        for c in 0..sdim {
            if c == sdim - 1 {
                mtrx_a[r * sdim + c] = -den[sdim - r] / an;
            }
            else {
                mtrx_a[r * sdim + c] = 0.0;
            }
        }

        if r > 0 {
            mtrx_a[r * sdim + r - 1] = 1.0;
        }
    }

    model.set_mtrx_a(&mtrx_a).context("failed at from_tf()")?;

    // B行列の作成
    let mut mtrx_b = vec![0.0; sdim * idim];
    let bn = 
        if num.len() - 1 == sdim {
            num[0]
        } else {
            0.0
        };
    
    for r in 0..sdim {
        if r < num.len() {
            mtrx_b[r] = (num[num.len() - r - 1] - den[sdim - r] * bn ) / an;
        } else {
            mtrx_b[r] = (-den[sdim - r] * bn ) / an;
        }

    }
    model.set_mtrx_b(&mtrx_b).context("failed at from_tf()")?;

    // C行列の作成
    let mut mtrx_c = vec![0.0; sdim * odim];
    mtrx_c[sdim - 1] = 1.0;
    model.set_mtrx_c(&mtrx_c).context("failed at from_tf()")?;

    // D行列の作成
    let mtrx_d = vec![bn; odim * idim];
    model.set_mtrx_d(&mtrx_d).context("failed at from_tf()")?;

    Ok(model)
}


impl Model for SpaceStateModel {
    fn interface_in(&mut self, signals: &Bus) -> anyhow::Result<()> {
        // input_busに設定されている信号名をsignalsから抽出して値をコピーする
        for (i, elem) in self.input_bus.iter_mut().enumerate() { 
            let signal = signals.get_by_name(elem.name())
                .with_context(|| format!("信号名:{}が見つかりません。", elem.name()))?;

            elem.value = signal.value;
            self.u[i] = signal.value;
        };

        Ok(())
    }

    fn interface_out(&mut self) -> &Bus {
        let obs = self.get_observation();
        
        self.output_bus.iter_mut().enumerate().for_each(|(i, elem)| elem.value = obs[i]);

        &self.output_bus
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
        &self.mtrx_a * x + &self.mtrx_b * &self.u
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
                str_a = str_a + &format!("{:>15.5} ", self.mtrx_a[(r, c)]);
            }
            str_a = str_a + &format!("|\n")
        }

        let mut str_b = String::from(format!("Matrix B ({} x {}): \n", self.state_dim, self.input_dim));
        for r in 0..self.state_dim {
            str_b = str_b + &"|";
            for c in 0..self.input_dim {
                str_b = str_b + &format!("{:>15.5} ", self.mtrx_b[(r, c)]);
            }
            str_b = str_b + &format!("|\n")
        }

        let mut str_c = String::from(format!("Matrix C ({} x {}): \n", self.output_dim, self.state_dim));
        for r in 0..self.output_dim {
            str_c = str_c + &"|";
            for c in 0..self.state_dim {
                str_c = str_c + &format!("{:>15.5} ", self.mtrx_c[(r, c)]);
            }
            str_c = str_c + &format!("|\n")
        }

        let mut str_d = String::from(format!("Matrix D ({} x {}): \n", self.output_dim, self.input_dim));
        for r in 0..self.output_dim {
            str_d = str_d + &"|";
            for c in 0..self.input_dim {
                str_d = str_d + &format!("{:>15.5} ", self.mtrx_d[(r, c)]);
            }
            str_d = str_d + &format!("|\n")
        }

        
        write!(f, "{}\n{}\n{}\n{}\n", str_a, str_b, str_c, str_d)
    }
}

/// 伝達関数モデル
#[derive(Debug, Clone)]
pub struct TransFuncModel {
    name: String, 
    model: SpaceStateModel, // 内部的には状態空間モデルを持つ
    num: Vec<f64>,          // 分子多項式の係数 2次の例 b2 * s^2 + b1 * s + b0
    den: Vec<f64>,          // 分母多項式の係数 2次の例 a2 * s^2 + a1 * s + a0
}

impl TransFuncModel {
    pub fn new(name: &str, num_coef: &[f64], den_coef: &[f64], solvertype: SolverType) -> anyhow::Result<Self> {
        let model = crate_ssm_from_tf(name, &num_coef, &den_coef, solvertype).context("Failed to create Trasfer Function Model")?;
        Ok(Self {
            name: name.to_string(),
            num : num_coef.to_vec(),
            den : den_coef.to_vec(),  
            model : model,
        })
    }

    pub fn set_u(&mut self, u: f64) -> anyhow::Result<()> {
        self.model.set_u(&vec![u])
    }
}

impl Model for TransFuncModel {
    fn interface_in(&mut self, signals: &Bus) -> anyhow::Result<()> {
        Ok(self.model.interface_in(signals)?)
    }

    fn interface_out(&mut self) -> &Bus {
        self.model.interface_out()
    }

    fn nextstate(&mut self, delta_t: f64) {
        self.model.nextstate(delta_t);
    }
}

impl fmt::Display for TransFuncModel {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Transfer Function Model -- >> \n\tnum: {:?}\n\tden: {:?}\n", self.num, self.den)
    }
}


#[cfg(test)]
mod simmodel_test {
    use super::*;
    // cargo test -- --test-threads=1　シングルスレッドで実行したいとき
    fn print_typename<T>(_: T) {
        println!("{}", std::any::type_name::<T>());
    }

    #[test] // StateSpaceModelのセット時のテスト 
    fn ssm_settest() {
        let state_def = vec![SigDef::new("s1", "Nm"), SigDef::new("s2", "rpm")];
        let input_def = vec![SigDef::new("i1", "Nm")];
        let output_def = vec![SigDef::new("o1", "rpm")];
        let mut model = SpaceStateModel::new("model", &state_def, &input_def, &output_def, SolverType::Euler).unwrap();

        let mtrx_a = [1.0, 1.0, 1.0, 1.0];
        model.set_mtrx_a(&mtrx_a);
        assert_eq!(model.mtrx_a, DMatrix::from_row_slice(2, 2, &mtrx_a));

        let mtrx_b = [1.0, 1.0];
        model.set_mtrx_b(&mtrx_b);
        assert_eq!(model.mtrx_b, DMatrix::from_row_slice(2, 1, &mtrx_b));

        let mtrx_c = [1.0, 1.0];
        model.set_mtrx_c(&mtrx_c);
        assert_eq!(model.mtrx_c, DMatrix::from_row_slice(1, 2, &mtrx_c));

        let init_state = [1.0, 2.0];
        model.init_state(&init_state);
        assert_eq!(model.x, DMatrix::from_row_slice(2, 1, &init_state));

        let x = [1.0, 2.0];
        model.set_x(&x);
        assert_eq!(model.x, DMatrix::from_row_slice(2, 1, &init_state));

        let mtrx_d = [1.0];
        model.set_mtrx_d(&mtrx_d);
        assert_eq!(model.mtrx_d, DMatrix::from_row_slice(1, 1, &mtrx_d));

        println!("model : {}\n", model);
    }

    #[test]
    fn ssm_interfacetest() {
        let state_bus = Bus::from(vec![
            Signal::new(0.0, "s1", "Nm"),
            Signal::new(0.0, "s2", "V"),
        ]);

        let input_bus = Bus::from(vec![
            Signal::new(1.0, "i1", "Nm"),
        ]);

        let output_bus = Bus::from(vec![
            Signal::new(0.0, "o1", "rpm"),
        ]);

        let mut model = SpaceStateModel::new("model", 
            &state_bus.get_sigdef(), 
            &input_bus.get_sigdef(), 
            &output_bus.get_sigdef(), 
            SolverType::Euler).unwrap();
        model.set_mtrx_a(&[1.0, 0.0, 0.0, 1.0]).unwrap();
        model.set_mtrx_b(&[1.0, 2.0]);
        model.set_mtrx_c(&[1.0, 0.0]);
        
        model.interface_in(&input_bus); // 入力のテスト
        assert_eq!(model.u[0], 1.0);
        
        model.nextstate(1.0);  

        let output = model.interface_out(); // 出力のテスト
        assert_eq!(output.get_by_name("o1").unwrap().value, 1.0);
        
        println!("output => \n{}", output);

        let state = model.get_statebus();
        assert_eq!(state.get_by_name("s1").unwrap().value, 1.0);
        assert_eq!(state.get_by_name("s2").unwrap().value, 2.0);

        println!("state => \n{}", state);
    }

    #[test]
    #[should_panic]
    fn ssm_set_errtest() {
        let state_def = vec![];
        let input_def = vec![];
        let output_def = vec![];
        let model = SpaceStateModel::new("model", &state_def, &input_def, &output_def, SolverType::Euler).unwrap();
    }

    #[test]
    #[should_panic]
    fn ssm_set_mtrx_a_errtest() {
        let state_def = vec![SigDef::new("s1", "Nm"), SigDef::new("s2", "rpm")];
        let input_def = vec![SigDef::new("i1", "Nm")];
        let output_def = vec![SigDef::new("o1", "rpm")];
        let mut model = SpaceStateModel::new("model", &state_def, &input_def, &output_def, SolverType::Euler).unwrap();
        model.set_mtrx_a(&[2.0, 1.0]).unwrap();
    }

    #[test]
    #[should_panic]
    fn ssm_set_mtrx_b_errtest() {
        let state_def = vec![SigDef::new("s1", "Nm"), SigDef::new("s2", "rpm")];
        let input_def = vec![SigDef::new("i1", "Nm")];
        let output_def = vec![SigDef::new("o1", "rpm")];
        let mut model = SpaceStateModel::new("model", &state_def, &input_def, &output_def, SolverType::Euler).unwrap();
        model.set_mtrx_b(&[2.0, 1.0, 2.0]).unwrap();
    }

    #[test]
    #[should_panic]
    fn ssm_set_mtrx_c_errtest() {
        let state_def = vec![SigDef::new("s1", "Nm"), SigDef::new("s2", "rpm")];
        let input_def = vec![SigDef::new("i1", "Nm")];
        let output_def = vec![SigDef::new("o1", "rpm")];
        let mut model = SpaceStateModel::new("model", &state_def, &input_def, &output_def, SolverType::Euler).unwrap();
        model.set_mtrx_c(&[2.0, 1.0, 2.0]).unwrap();
    }

    #[test]
    #[should_panic]
    fn ssm_set_mtrx_d_errtest() {
        let state_def = vec![SigDef::new("s1", "Nm"), SigDef::new("s2", "rpm")];
        let input_def = vec![SigDef::new("i1", "Nm")];
        let output_def = vec![SigDef::new("o1", "rpm")];
        let mut model = SpaceStateModel::new("model", &state_def, &input_def, &output_def, SolverType::Euler).unwrap();
        model.set_mtrx_d(&[2.0, 1.0, 2.0]).unwrap();
    }

    #[test]
    #[should_panic]
    fn ssm_set_x_errtest() {
        let state_def = vec![SigDef::new("s1", "Nm"), SigDef::new("s2", "rpm")];
        let input_def = vec![SigDef::new("i1", "Nm")];
        let output_def = vec![SigDef::new("o1", "rpm")];
        let mut model = SpaceStateModel::new("model", &state_def, &input_def, &output_def, SolverType::Euler).unwrap();
        model.set_x(&[2.0, 1.0, 2.0]).unwrap();
    }

    #[test]
    #[should_panic]
    fn ssm_initstate_errtest() {
        let state_def = vec![SigDef::new("s1", "Nm"), SigDef::new("s2", "rpm")];
        let input_def = vec![SigDef::new("i1", "Nm")];
        let output_def = vec![SigDef::new("o1", "rpm")];
        let mut model = SpaceStateModel::new("model", &state_def, &input_def, &output_def, SolverType::Euler).unwrap();
        model.init_state(&[2.0, 1.0, 2.0]).unwrap();
    }

    
    #[test]
    #[should_panic]
    fn ssm_setu_errtest() {
        let state_def = vec![SigDef::new("s1", "Nm"), SigDef::new("s2", "rpm")];
        let input_def = vec![SigDef::new("i1", "Nm")];
        let output_def = vec![SigDef::new("o1", "rpm")];
        let mut model = SpaceStateModel::new("model", &state_def, &input_def, &output_def, SolverType::Euler).unwrap();
        model.set_u(&[2.0, 1.0, 2.0]).unwrap();
    }

    #[test]
    #[should_panic]
    fn tf_set_errtest1() {
        let model = TransFuncModel::new("model", &[1.0, 0.0, 2.0, 2.0], &[2.0, 1.0, 1.0], SolverType::Euler).unwrap();       
    }

    #[test]
    #[should_panic]
    fn tf_set_errtest2() {
        let model = TransFuncModel::new("model", &[1.0], &[2.0], SolverType::Euler).unwrap();       
    }

    #[test]
    fn tf_settest() {
        let tfmodel = TransFuncModel::new("model", &[2.0, 2.0], &[2.0, 1.0, 1.0], SolverType::Euler).unwrap();
        let state_def = vec![SigDef::new("s1", "Nm"), SigDef::new("s2", "rpm")];
        let input_def = vec![SigDef::new("i1", "Nm")];
        let output_def = vec![SigDef::new("o1", "rpm")];
        let mut ssm = SpaceStateModel::new("model", &state_def, &input_def, &output_def, SolverType::Euler).unwrap();

        ssm.set_mtrx_a(&[0.0, -0.5, 1.0, -0.5]);
        ssm.set_mtrx_b(&[1.0, 1.0]);
        ssm.set_mtrx_c(&[0.0, 1.0]);
        ssm.set_mtrx_c(&[0.0]);

        assert_eq!(tfmodel.model.mtrx_a, ssm.mtrx_a);
        assert_eq!(tfmodel.model.mtrx_b, ssm.mtrx_b);
        assert_eq!(tfmodel.model.mtrx_c, ssm.mtrx_c);
        assert_eq!(tfmodel.model.mtrx_d, ssm.mtrx_d);

        println!("{}\n", tfmodel);
    }
}