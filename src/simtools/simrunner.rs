use anyhow::{Context};
use crate::simtools::simmodel::{*};

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
        scope_out: SimScope,
    }
    
    impl RLCCircuit {
        pub fn new(r: f64, l: f64, c: f64, init_i: f64, init_q: f64) -> Self {
            let input_def = vec![SigDef::new("v", "V")];
            let state_def = vec![SigDef::new("i", "A"), SigDef::new("q", "C")];
            let output_def = vec![SigDef::new("Vr", "V"), SigDef::new("Vc", "V")];

            let mut model = SpaceStateModel::new("RLC", state_def, input_def, output_def, SolverType::RungeKutta).unwrap();
            model.set_mtrx_a(&[-r / l, -1.0 / (l * c), 1.0, 0.0]).unwrap();
            model.set_mtrx_b(&[1.0 / l, 0.0]).unwrap();
            model.set_mtrx_c(&[r, 0.0, 0.0, 1.0 / c]).unwrap();
            model.init_state(&[init_i, init_q]).unwrap();

            let simset = SimSet::new(10.0, 0.001);
            let scope = SimScope::new(vec![SigDef::new("Vr", "V"), SigDef::new("Vc", "V")], simset.get_stepnum());
    
            Self {
                simset: simset,
                model : model,
                scope_out: scope,
            }
        }
    }

    impl SimTarget for RLCCircuit {
        fn get_simset(&self) -> SimSet {
            self.simset
        }
    
        fn init_sim(&mut self) {
            
        }
    
        fn step_sim(&mut self, time: f64) {
            let mut input_bus = Bus::new();
            let input_sig = Signal::new(0.0, "v", "V");
            input_bus.push(input_sig);

            self.model.interface_in(&input_bus);

            self.model.nextstate(self.simset.get_deltat());

            self.scope_out.push(time, self.model.interface_out());
        }
    
        fn after_sim(&mut self) {
            self.scope_out.export("test_output\\rlc.csv");
            self.scope_out.timeplot_all("test_output\\rlc.png", (500, 500), (2, 1));
        }
    }

    #[test]
    fn rlc_test() {
        let target = RLCCircuit::new(10.0, 0.3e-3, 0.1e-6, 1.0, 0.0);
        let mut sim = SimRunner::new(target);
    
        sim.run_sim().unwrap();
    }

}