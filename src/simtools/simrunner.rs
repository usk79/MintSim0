use anyhow::{*};
//use crate::simtools::simmodel::{*};

use std::f64::consts::{PI};

// 今後の実装メモ
// Modelごとにサンプリングタイムが違う場合でも対応できるように、
// サンプリングの時間が異なる場合を吸収できる方法が必要

#[derive(Debug, Copy, Clone)]
pub struct SimSet {
    simtime: f64, // シミュレーション時間
    delta_t: f64, // 刻み幅Δt
    stepnum: usize, // 全ステップ数
}

impl SimSet {
    pub fn new(simtime: f64, delta_t: f64) -> Self {
        Self {
            simtime: simtime,
            delta_t: delta_t,
            stepnum: (simtime / delta_t + 0.5) as usize,
        }
    }

    pub fn get_simtime(&self) -> f64 {
        self.simtime
    }

    pub fn get_deltat(&self) -> f64 {
        self.delta_t
    }

    pub fn get_stepnum(&self) -> usize {
        self.stepnum
    }
}

pub trait SimTarget {
    /// シミュレーション設定取得
    fn get_simset(&self) -> SimSet;
    /// シミュレーション初期化
    fn init_sim(&mut self) -> anyhow::Result<()> { Ok(()) } // 空のデフォルト定義
    /// ステップ実行
    fn step_sim(&mut self, time: f64) -> anyhow::Result<()>;
    /// シミュレーション完了後の処理
    fn after_sim(&mut self) -> anyhow::Result<()> { Ok(()) } // 空のデフォルト定義
}

#[derive(Debug)]
pub struct SimRunner<T>
    where T:SimTarget
{
    simset: SimSet, // シミュレーション設定
    target: T, // 対象のモデル
}

impl<T> SimRunner<T>
    where T:SimTarget
{
    pub fn new(target: T) -> Self {
        Self {
            simset: target.get_simset(),
            target: target,
        }
    }

    pub fn run_sim(&mut self) -> anyhow::Result<()> {
        let printidx = self.simset.get_stepnum() / 10;

        self.target.init_sim()?;

        for idx in 1..self.simset.get_stepnum() {
            if idx % printidx == 0 {
                println!("processing now ... {}%\n", (idx as f64 / self.simset.get_stepnum() as f64) * 100.0)
            }
            self.target.step_sim(self.simset.get_deltat() * idx as f64)?;
        }

        self.target.after_sim()?;

        Ok(())
    }
}

#[cfg(test)]
mod simrun_test {
    use super::*;
    use crate::simtools::signal::{*};
    use crate::simtools::simscope::{*};
    use crate::simtools::simmodel::{*};
    use crate::simtools::simcommon::{*};

    use nalgebra::DMatrix;

    struct SampleTarget {
        simset: SimSet,
        model: TransFuncModel,
    }
    
    impl SampleTarget {
        fn new() -> Self {
            let model = TransFuncModel::new("test", &[2.0, 2.0], &[2.0, 1.0, 1.0], SolverType::Euler).unwrap();
    
            SampleTarget {
                simset: SimSet::new(10.0, 0.001),
                model: model,
            }
        }
    }
    
    impl SimTarget for SampleTarget {
        fn get_simset(&self) -> SimSet {
            self.simset
        }
    
        fn step_sim(&mut self, time: f64) -> anyhow::Result<()> {
            self.model.nextstate(self.simset.get_deltat());
            Ok(())
        }
    }
    
    #[test]
    fn simplemodel_test() {
        let target = SampleTarget::new();
    
        let mut sim = SimRunner::new(target);
    
        sim.run_sim().unwrap();
    }

    struct RLCCircuit {
        simset: SimSet,
        model: SpaceStateModel,
        bus: Bus,
        scope_out: SimScope,
    }
    
    impl RLCCircuit {
        pub fn new(r: f64, l: f64, c: f64, init_i: f64, init_q: f64) -> Self {
            let input_def = vec![SigDef::new("v", "V")];
            let state_def = vec![SigDef::new("i", "A"), SigDef::new("q", "C")];
            let output_def = vec![SigDef::new("Vr", "V"), SigDef::new("Vc", "V")];

            let mut model = SpaceStateModel::new("RLC", &state_def, &input_def, &output_def, SolverType::RungeKutta).unwrap();
            model.set_mtrx_a(&[-r / l, -1.0 / (l * c), 1.0, 0.0]).unwrap();
            model.set_mtrx_b(&[1.0 / l, 0.0]).unwrap();
            model.set_mtrx_c(&[r, 0.0, 0.0, 1.0 / c]).unwrap();
            model.init_state(&[init_i, init_q]).unwrap();

            let simset = SimSet::new(0.001, 0.0000001);

            let mut bus = Bus::new();
            bus.set_sigdef(&input_def).unwrap();
            bus.set_sigdef(&output_def).unwrap();
            bus.set_sigdef(&state_def).unwrap();

            let scope = SimScope::new(&bus.get_sigdef(), simset.get_stepnum());
    
            Self {
                simset: simset,
                model : model,
                scope_out: scope,
                bus: bus,
            }
        }
    }

    impl SimTarget for RLCCircuit {
        fn get_simset(&self) -> SimSet {
            self.simset
        }
    
        fn init_sim(&mut self) -> anyhow::Result<()> {
            self.model.init_state(&[0.0, 0.0001])?;
            Ok(())
        }
    
        fn step_sim(&mut self, time: f64) -> anyhow::Result<()> {
            let mut input_bus = Bus::new();
            let input_sig = Signal::new(0.10, "v", "V");
            input_bus.push(input_sig)?;

            self.model.interface_in(&input_bus)?;

            self.model.nextstate(self.simset.get_deltat());

            self.bus.update_from_bus(&input_bus);

            let output_bus = self.model.interface_out();
            self.bus.update_from_bus(&output_bus);

            let state_bus = self.model.get_statebus();
            self.bus.update_from_bus(&state_bus);

            self.scope_out.push(time, &self.bus)?;

            Ok(())
        }
    
        fn after_sim(&mut self) -> anyhow::Result<()> {
            self.scope_out.export("test_output\\rlc.csv")?;
            self.scope_out.timeplot_all("test_output\\rlc.png", (500, 500), (3, 2))?;

            Ok(())
        }
    }

    #[test]
    fn rlc_test() {
        let target = RLCCircuit::new(10.0, 0.3e-3, 0.1e-6, 0.0, 0.0);
        let mut sim = SimRunner::new(target);
    
        sim.run_sim().unwrap();
    }

    struct BallAndBeam {
        x: DMatrix<f64>, // 状態ベクトル
        rball: f64, // ボールの半径[m]
        mball: f64, // ボール重量[kg]
        jball: f64, // ボールの慣性モーメント[kg・m^2]
        jbeam: f64, // ビームの慣性モーメント[kg・,^2]
        mu: f64, // ボールの転がり抵抗係数[-]
        k: f64, // 空気抵抗係数[N/(m/s)^2]
        m0: f64, // 途中計算
        m1: f64, // 途中計算
        u: Bus, // 入力バス(入力トルク)
        output_bus: Bus, // 出力バス
        state_bus: Bus, // 状態バス
    }

    impl BallAndBeam {
        pub fn new(init_r: f64, init_v: f64, init_theta: f64, init_omega: f64) -> Self {
            let mut state = DMatrix::from_element(4, 1, 0.0);
            let rball = 0.01; // 1[cm]
            let mball = 0.001; // 1[g]
            let jball = 2.0 / 5.0 * mball * rball * rball; // 2/5mr^2
            let mbeam = 1.0; // 0.1[kg]
            let lbeam = 1.0; // 1[m] ビームの長さ
            let wbeam = 0.05; // 5[cm]　ビームの幅
            let mu = 0.3; // 転がり抵抗係数
            let k = 50.50; // 空気抵抗係数
            let jbeam = mbeam * (lbeam * lbeam + wbeam * wbeam) / 12.0; 
            let m0 = jball / (rball * rball) + mball; // 途中計算用変数
            let m1 = mball / m0;

            state[0] = init_r;
            state[1] = init_v;
            state[2] = init_theta.deg2rad();
            state[3] = init_omega.deg2rad();

            let input_bus = Bus::try_from(vec![SigDef::new("trq", "Nm")]).unwrap();
            let output_bus = Bus::try_from(vec![SigDef::new("ball_pos", "m")]).unwrap();

            let state_bus = Bus::try_from(vec![
                SigDef::new("ball_r", "m"),
                SigDef::new("ball_v", "m/s"),
                SigDef::new("beam_angle", "deg"),
                SigDef::new("beam_rot", "deg/s"),
            ]).unwrap();

            Self {
                x: state,
                rball: rball, 
                mball: mball,
                jball: jball,
                jbeam: jbeam,
                k: k,
                mu: mu,
                m0: m0,
                m1: m1,
                u: input_bus,
                output_bus: output_bus,
                state_bus: state_bus,
            }
        }

        pub fn state_bus(&self) -> &Bus {
            &self.state_bus
        }
    }

    impl Model for BallAndBeam {
        fn input_bus(&mut self) -> &mut Bus {
            &mut self.u
        }

        fn output_bus(&self) -> &Bus {
            &self.output_bus
        }

        fn nextstate(&mut self, delta_t: f64) {
            //self.rungekutta_method(delta_t);
            self.euler_method(delta_t);

            self.output_bus[0].value = self.x[0];
            //self.state_bus.iter_mut().enumerate().for_each(|(i, e)| e.value = self.x[i]);
            self.state_bus[0].value = self.x[0];
            self.state_bus[1].value = self.x[1];
            self.state_bus[2].value = self.x[2].rad2deg();
            self.state_bus[3].value = self.x[3].rad2deg();
        }
    }

    impl DEModel for BallAndBeam {
        fn derivative_func(&self, x: &DMatrix<f64>) -> DMatrix<f64> {
            let mut slope = DMatrix::from_element(4, 1, 0.0);
            
            // r
            slope[0] = x[1];
            // v                                                                 // 抵抗力の項　符号あってる？
            slope[1] = self.m1 * G * x[2].sin() + self.m1 * x[0] * x[3] * x[3];// - self.mu * G * self.m1 * x[2].cos() - self.k * x[1] / self.m0; 
            // theta
            slope[2] = x[3];
            // omega
            let j0 = self.mball * x[0] * x[0] + self.jbeam + self.jball;
            slope[3] = (self.u[0].value - 2.0 * self.mball * x[0] * x[1] * x[3] + self.mball * G * x[0] * x[2].cos()) / j0;

            slope
        }
    
        fn set_state(&mut self, newstate: DMatrix<f64>) {
            self.x = newstate; 
        }
    
        fn get_state(&self) -> &DMatrix<f64> {
            &self.x
        }
    }

    struct SimBAB {
        simset: SimSet,
        bab_model: BallAndBeam,
        controller: PIDController, 
        scope: SimScope,
        mainbus: Bus,
        ctrlbus: Bus,
        ctrlscope: SimScope,
    }

    impl SimBAB {
        fn new() -> Self {
            let simset = SimSet::new(10.0, 0.001);
            let mut model = BallAndBeam::new(0.0, 0.0, 0.0, 0.0);
            let mut bus_for_scope = Bus::new();
            
            let ctrlin = vec![SigDef::new("ball_pos", "m")];
            let ctrlout  = vec![SigDef::new("trq", "Nm")];
            let target = vec![SigDef::new("target", 'm')];
            let mut ctrl = PIDController::new(
                &ctrlin, 
                &ctrlout, 
                &target,
                vec![(1.0, 0.01, 1.0)], 
//                vec![(0.0, 0.0, 0.0)], 
                SolverType::RungeKutta
            ).unwrap();

            bus_for_scope.set_sigdef(&model.input_bus().get_sigdef()).unwrap();
            bus_for_scope.set_sigdef(&model.output_bus().get_sigdef()).unwrap();
            bus_for_scope.set_sigdef(&model.state_bus().get_sigdef()).unwrap();

            let scope = SimScope::new(&bus_for_scope.get_sigdef(), simset.get_stepnum());
            
            let mut ctrlbus = Bus::new();
            ctrlbus.set_sigdef(&ctrl.input_bus().get_sigdef()).unwrap();
            ctrlbus.set_sigdef(&ctrl.output_bus().get_sigdef()).unwrap();

            let ctrlscope = SimScope::new(&ctrlbus.get_sigdef(), simset.get_stepnum());

            Self {
                simset: simset,
                bab_model: model,
                scope: scope,
                mainbus: bus_for_scope,
                controller: ctrl,
                ctrlscope: ctrlscope,
                ctrlbus: ctrlbus,
            }
        }
    }

    impl SimTarget for SimBAB {
        fn get_simset(&self) -> SimSet {
            self.simset
        }

        fn init_sim(&mut self) -> anyhow::Result<()> {
            let mut trgt = Bus::new();
            trgt.set_sigdef(&self.controller.target_bus().get_sigdef())?;
            trgt.get_by_name_mut("target").unwrap().value = 0.0; // [m]
            self.controller.set_target(&trgt)?;

            Ok(())
        }

        fn step_sim(&mut self, time: f64) -> anyhow::Result<()> {
            
            self.controller.interface_in(
                &self.bab_model.interface_out()
            )?;

            self.controller.nextstate(self.simset.get_deltat());

            self.ctrlbus.update_from_bus(&self.controller.input_bus());
            self.ctrlbus.update_from_bus(&self.controller.output_bus());

            self.ctrlscope.push(time, &self.ctrlbus)?;

            self.bab_model.interface_in(
                self.controller.interface_out()
            )?;
            self.bab_model.nextstate(self.simset.get_deltat());

            self.mainbus.update_from_bus(&self.bab_model.input_bus());
            self.mainbus.update_from_bus(&self.bab_model.interface_out());
            self.mainbus.update_from_bus(&self.bab_model.state_bus());
            self.mainbus.update_from_bus(&self.controller.input_bus());
            self.mainbus.update_from_bus(&self.controller.output_bus());

            self.scope.push(time, &self.mainbus)?;

            Ok(())
        }

        fn after_sim(&mut self) -> anyhow::Result<()> {
            self.scope.export("test_output\\bab.csv")?;
            self.scope.timeplot_all("test_output\\bab.png", (500, 500), (3, 2))?;

            self.ctrlscope.timeplot_all("test_output\\ctrl.png", (500, 500), (2, 1))?;

            Ok(())
        }
    }

    #[test]
    fn bab_test() {
        let target = SimBAB::new();
        let mut sim = SimRunner::new(target);

        sim.run_sim().unwrap();
    }

    // Integratorのテスト
    struct SimIntegrator {
        simset: SimSet,
        model: Integrator,
        scope: SimScope,
        bus: Bus,
    }

    impl SimIntegrator {
        fn new() -> Self {
            let simset = SimSet::new(10.0, 0.001);
            let ins = vec![SigDef::new("u1", "-"), SigDef::new("u2", "-"), SigDef::new("u3", "-")];
            let outs: Vec<SigDef> = vec![SigDef::new("o1", "-"), SigDef::new("o2", "-"), SigDef::new("o3", "-")];

            let model = Integrator::new(&ins, &outs, SolverType::Euler);

            let mut bus = Bus::new();

            bus.set_sigdef(&ins).unwrap();
            bus.set_sigdef(&outs).unwrap();

            let scope = SimScope::new(&bus.get_sigdef(), simset.get_stepnum());

            Self {
                simset: simset,
                model: model,
                scope: scope,
                bus: bus,
            }
        }
    }

    impl SimTarget for SimIntegrator {
        fn get_simset(&self) -> SimSet {
            self.simset
        }

        fn step_sim(&mut self, time: f64) -> anyhow::Result<()> {
            let val = self.bus.get_by_name("o1").unwrap().value;

            self.bus.update(&mut Signal::new(1.0, "u1", "-"));
            self.bus.update(&mut Signal::new(val, "u2", "-"));
            self.bus.update(&mut Signal::new((val + PI / 2.0).sin(), "u3", "-"));

            self.model.interface_in(&self.bus).unwrap();
            self.model.nextstate(self.simset.get_deltat());
            
            self.bus.update_from_bus(self.model.output_bus());

            self.scope.push(time, &self.bus).unwrap();

            Ok(())
        }

        fn after_sim(&mut self) -> anyhow::Result<()> {
            self.scope.export("test_output\\integ.csv").unwrap();
            self.scope.timeplot_all("test_output\\integ.png", (800, 500), (2, 3))
        }
    }

    #[test]
    fn integrator_test() {
        let target = SimIntegrator::new();
        let mut sim = SimRunner::new(target);

        sim.run_sim().unwrap();
    }
}