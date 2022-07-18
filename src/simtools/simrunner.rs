use anyhow::{Context};
use crate::simtools::simmodel::{*};

#[derive(Debug)]
pub struct SimRunner<T>
where T:Model
{
    simtime: f64, // シミュレーション時間
    delta_t: f64, // 刻み幅Δt
    stepnum: usize, // 全ステップ数
    model: T, // 対象のモデル
}

impl<T> SimRunner<T>
where T:Model
{
    pub fn new(simtime: f64, delta_t: f64, model: T) -> Self {
        let stepnum = (simtime / delta_t + 0.5) as usize;
        
        Self {
            simtime: simtime,
            delta_t: delta_t,
            stepnum: stepnum,
            model: model,
        }
    }

    pub fn run_sim(&mut self) -> anyhow::Result<()> {
        let printidx = self.stepnum / 10;

        for idx in 1..self.stepnum {
            if idx % printidx == 0 {
                println!("processing now ... {}%\n", (idx as f64 / self.stepnum as f64) * 100.0)
            }
            self.model.nextstate(self.delta_t);
        }

        Ok(())
    }
}
