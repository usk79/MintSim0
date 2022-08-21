use anyhow::{*};
//use crate::simtools::simmodel::{*};

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
    fn init_sim(&mut self);
    /// ステップ実行
    fn step_sim(&mut self, time: f64);
    /// シミュレーション完了後の処理
    fn after_sim(&mut self);
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

        self.target.init_sim();

        for idx in 1..self.simset.get_stepnum() {
            if idx % printidx == 0 {
                println!("processing now ... {}%\n", (idx as f64 / self.simset.get_stepnum() as f64) * 100.0)
            }
            self.target.step_sim(self.simset.get_deltat() * idx as f64);
        }

        self.target.after_sim();

        Ok(())
    }
}

#[cfg(test)]
mod simrun_test {
    use super::*;
    use crate::simtools::signal::{*};
    use crate::simtools::simscope::{*};
    use crate::simtools::simmodel::{*};

    use crate::simtools::simconsts::G;
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
    
        fn init_sim(&mut self) {
            
        }
    
        fn step_sim(&mut self, time: f64) {
            self.model.nextstate(self.simset.get_deltat());
        }
    
        fn after_sim(&mut self) {
    
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
    
        fn init_sim(&mut self) {
            self.model.init_state(&[0.0, 0.0001]).unwrap();
        }
    
        fn step_sim(&mut self, time: f64) {
            let mut input_bus = Bus::new();
            let input_sig = Signal::new(0.10, "v", "V");
            input_bus.push(input_sig).unwrap();

            self.model.interface_in(&input_bus).unwrap();

            self.model.nextstate(self.simset.get_deltat());

            self.bus.update_from_bus(&input_bus);

            let output_bus = self.model.interface_out();
            self.bus.update_from_bus(&output_bus);

            let state_bus = self.model.get_statebus();
            self.bus.update_from_bus(&state_bus);

            self.scope_out.push(time, &self.bus).unwrap();
        }
    
        fn after_sim(&mut self) {
            self.scope_out.export("test_output\\rlc.csv").unwrap();
            self.scope_out.timeplot_all("test_output\\rlc.png", (500, 500), (3, 2)).unwrap();
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
            let rball = 0.2; // 20[cm]
            let mball = 0.1; // 100[g]
            let jball = 2.0 / 5.0 * mball * rball * rball; // 2/5mr^2
            let mbeam = 1.0; // 1[kg]
            let lbeam = 1.0; // 1[m] ビームの長さ
            let wbeam = 0.05; // 5[cm]　ビームの幅
            let mu = 0.3; // 転がり抵抗係数
            let k = 50.50; // 空気抵抗係数
            let jbeam = mbeam * (lbeam * lbeam + wbeam * wbeam) / 12.0; 
            let m0 = jball / (rball * rball) + mball; // 途中計算用変数
            let m1 = mball / m0;

            state[0] = init_r;
            state[1] = init_v;
            state[2] = init_theta;
            state[3] = init_omega;

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

        pub fn get_state(&self) -> &Bus {
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
            self.rungekutta_method(delta_t);

            self.output_bus[0].value = self.x[0];
        }
    }

    impl DEModel for BallAndBeam {
        fn derivative_func(&self, x: &DMatrix<f64>) -> DMatrix<f64> {
            let mut slope = DMatrix::from_element(4, 1, 0.0);
            
            // r
            slope[0] = x[1];
            // v                                                                 // 抵抗力の項　符号あってる？
            slope[1] = self.m1 * G * x[2].sin() + self.m1 * x[0] * x[3] * x[3] - self.mu * G * self.m1 * x[2].cos() - self.k * x[1] / self.m0; 
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
}