use std::fmt;

use std::fs;
use std::fs::File;
use std::io::{self, Write, BufWriter};

use crate::simtools::signal::{*};
use std::collections::HashMap;
use anyhow::{Context};

use plotters::prelude::*;
use plotters::coord::Shift;

/**
Busに登録されている信号をシミュレーション時間分保存する。
保存されたデータをグラフにプロットしたり、ファイルへの書き出し機能を有する
*/

#[derive(Debug)]
pub struct SimScope {
    timedata: Vec<f64>, // 時刻情報保管用
    storage: Vec<Vec<f64>>, // データストレージ
    sigdef: Vec<SigDef>,    // 信号定義
    signum: usize,
} 

impl SimScope {
    pub fn new(sigdef: Vec<SigDef>, stepnum: usize) -> Self {
        let signal_num = sigdef.len();

        Self {
            timedata: Vec::with_capacity(stepnum),
            storage: (0..signal_num).map(|_| Vec::with_capacity(stepnum) ).collect::<Vec<Vec<f64>>>(),
            sigdef: sigdef,
            signum: signal_num,
        }
    }

    pub fn push(&mut self, time: f64, signals: &Bus) -> anyhow::Result<()>{
        self.timedata.push(time);
        signals.iter().enumerate().for_each(|(idx, signal)| {
            self.storage[idx].push(signal.value);
        });

        Ok(())
    }

    pub fn export(&self, filepath: &str) {
        let mut file = BufWriter::new(File::create(filepath).unwrap());
        
        // 一行目の信号名の部分を作成
        let mut seriesname = vec!["time[s]".to_string()];
        self.sigdef.iter().for_each(|sig| seriesname.push( sig.to_string() ) );
        
        writeln!(file, "{}", seriesname.join(","));

        // データを1行ずつ出力
        let siglen = self.timedata.len();
        
        for idx in 0..siglen {
            let mut line = vec![self.timedata[idx].to_string()];

            for sigidx in 0..self.signum {
                line.push(self.storage[sigidx][idx].to_string());
            }

            writeln!(file, "{}", line.join(","));
        }
    }

    pub fn timeplot_all(&self, filename: &str, pltsize: (u32, u32), pltdivide: (usize, usize)) -> anyhow::Result<()>{
        let root_area = BitMapBackend::new(filename, pltsize).into_drawing_area();
        let child_areas = root_area.split_evenly(pltdivide);

        root_area.fill(&WHITE).unwrap();

        if self.signum > pltdivide.0 * pltdivide.1 {
            return Err(anyhow!("プロットの分割数が不足しています。"));
        }

        self.storage.iter().enumerate().for_each( |(idx, data)| {
            let sig = &self.sigdef[idx];
            self.timeplot_subfn(&child_areas[idx], &sig.to_string(), &data);
        });

        Ok(())
    }

    fn timeplot_subfn(&self, plt: &DrawingArea<BitMapBackend, Shift>, caption: &str, data: &Vec<f64>) {
        
        plt.fill(&WHITE).unwrap();
    
        let font = ("sans-serif", 20);

        let (y_min, y_max) = data.iter()
                         .fold(
                           (0.0/0.0, 0.0/0.0),
                           |(m,n), v| (v.min(m), v.max(n))
                          ); // f64はNaNがあるためordが実装されていない。min, maxを使うための工夫が必要⇒https://qiita.com/lo48576/items/343ca40a03c3b86b67cb
        let datalen = self.timedata.len();
        let xrange = 0.0..self.timedata[datalen - 1]; 
        let yrange = y_min..y_max;
      
        let mut chart = ChartBuilder::on(&plt)
          .caption(caption, font.into_font()) // キャプションのフォントやサイズ
          .margin(10)                         // 上下左右全ての余白
          .x_label_area_size(16)              // x軸ラベル部分の余白
          .y_label_area_size(42)              // y軸ラベル部分の余白
          .build_cartesian_2d(                // x軸とy軸の数値の範囲を指定する
            xrange,                           // x軸の範囲
            yrange)                           // y軸の範囲
          .unwrap();
    
        // x軸y軸、グリッド線などを描画
        chart.configure_mesh().draw().unwrap();

        let line_series = LineSeries::new(
            self.timedata.iter()
                    .zip(data.iter())
                    .map(|(x, y)| (*x, *y)),
                &RED);

        chart.draw_series(line_series).unwrap();

    }
}

#[cfg(test)]
mod scope_test {
    use super::*;

    #[test]
    fn scope_pushtest() {
        let mut bus = Bus::from(vec![
            Signal::new(0.0, "motor_trq", "Nm"),
            Signal::new(0.0, "motor_volt", "V"),
            Signal::new(0.0, "motor_current", "A"),
        ]);

        let mut scope = SimScope::new(bus.get_sigdef(), 10);

        for i in 0..10 {
            bus[0].value = i as f64;
            bus[1].value = (i * 2) as f64;
            bus[2].value = (i * 3) as f64;

            scope.push(i as f64 * 0.01, &bus);
        }

        scope.export("test_output\\scope_pushtest.csv");
        scope.timeplot_all("test_output\\scope_pushtest.png", (500, 500), (3, 1));

        assert_eq!(scope.storage[0][9], 9.0);
        assert_eq!(scope.storage[1][9], 18.0);
        assert_eq!(scope.storage[2][9], 27.0);

        
    }
}
