use std::fmt;
use std::collections::HashMap;
use std::ops::Index;
use std::ops::IndexMut;
use std::convert::From;
use core::slice::{Iter, IterMut};

extern crate nalgebra as na;
use na::{DMatrix};

use anyhow::{*};

// 今後の実装メモ
// Busについて、複数のバスから必要な信号を抽出して一つのバスを作るメソッドが必要
// Singalについて timeという名前は使用禁止にする処理を追加のこと

/** Signal
シミュレーションで使用する単一の信号表現
値、信号名、信号の単位のデータを保存する。
*/
#[derive(Debug, Clone, PartialEq)]
pub struct Signal {
    pub value: f64,   // 値　値だけは自由に書き換えられるようにしている
    sigdef: SigDef,
}

impl Signal {
    pub fn new(initvalue: f64, name: impl Into<String>, unit: impl Into<String>) -> Self {
        Self {
            value: initvalue,
            sigdef: SigDef::new(name.into(), unit.into()),
        }
    }
    
    /// nameへのアクセッサメソッド
    pub fn name<'a>(&'a self) -> &'a str {
        &self.sigdef.name()
    }

    /// unitへのアクセッサメソッド
    pub fn unit<'a>(&'a self) -> &'a str {
        &self.sigdef.unit()
    }
}

impl fmt::Display for Signal {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}: {} [{}]\n", self.sigdef.name(), self.value, self.sigdef.unit())
    }
}

/// 信号名と単位だけを設定する用のタプル
#[derive(Debug, Clone, PartialEq)]
pub struct SigDef(String, String); // (信号名, 単位)

impl SigDef {
    pub fn new(name: impl Into<String>, unit: impl Into<String>) -> Self {
        Self(name.into(), unit.into())
    }
    pub fn name(&self) -> &str {
        &self.0
    }

    pub fn unit(&self) -> &str {
        &self.1
    }
}

impl fmt::Display for SigDef {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}[{}]", self.0, self.1)
    }
}

/// Signalを複数まとめたものをBusと定義する。
/// シミュレーションで作るモデル同士のインターフェースとして用いる。
/// また、Bus内の信号を抽出するメソッドを中心に実装している。
#[derive(Debug, Clone)]
pub struct Bus {
    signals: Vec<Signal>,
    keytable: HashMap<String, usize> // 名前からインデックスを検索するためのテーブル
}

impl Bus {
    pub fn new() -> Self {
        Self {
            signals: Vec::new(),
            keytable: HashMap::new()
        }
    }

    /// BusにSignalを追加する
    pub fn push(&mut self, signal: Signal) -> anyhow::Result<()> {
        let keyname = signal.name().to_string();
        match self.keytable.contains_key(&keyname) {
            true => {
                return Err(anyhow!(format!("信号名が重複しています。: 信号名{}", keyname)));
            },
            false => {
                self.keytable.insert(keyname, self.signals.len());
                self.signals.push(signal);
            }
        } 
        
        Ok(())
    }

    pub fn update(&mut self, signal: &Signal) {
        if let Some(idx) = self.keytable.get(signal.name()) {
            self.signals[*idx].value = signal.value;
        } // 登録されていない信号に対しては何もしない
    }

    pub fn update_from_bus(&mut self, bus: &Bus) {
        bus.iter().for_each(|sig| self.update(sig));
    }

    /// 信号名から信号を抽出する
    pub fn get_by_name(&self, signame: impl Into<String>) -> anyhow::Result<&Signal> {
        match self.keytable.get(&signame.into()) {
            Some(index) => {
                Ok(&self.signals[*index])
            },
            None => Err(anyhow!("信号が存在しません。"))
        }
    }

    /// get_by_nameのmutable版
    pub fn get_by_name_mut(&mut self, signame: impl Into<String>) -> anyhow::Result<&mut Signal> {
        match self.keytable.get(&signame.into()) {
            Some(index) => {
                Ok(&mut self.signals[*index])
            },
            None => Err(anyhow!("信号が存在しません。"))
        }
    }

    /// Vec<SigDef>の情報を取り込んでバスを作成する
    pub fn set_sigdef(&mut self, sigs: &Vec<SigDef>) -> anyhow::Result<()> {

        for sig in sigs.iter() {
            let keyname = sig.name().to_string();
            match self.keytable.contains_key(&keyname) {
                true => {
                    return Err(anyhow!(format!("信号名が重複しています。: 信号名{}", keyname)));
                },
                false => {
                    self.keytable.insert(keyname, self.signals.len());
                    self.signals.push(Signal::new(0.0, sig.name(), sig.unit()));
                }
            }
        }

        Ok(())
    }

    /// Vec<SigDef>を返す。モデルの作成時に使用する。
    pub fn get_sigdef(&self) -> Vec<SigDef> {
        self.signals.iter().map(|sig| SigDef(sig.name().to_string(), sig.unit().to_string()))
                            .collect::<Vec<SigDef>>()
    }

    pub fn iter(&self) -> BusIter {
        self.signals.iter()
    }

    pub fn iter_mut(&mut self) -> BusIterMut {
        self.signals.iter_mut()
    }
}

/// Indexオペレータのオーバーロード
impl Index<usize> for Bus
{
    type Output = Signal;
    fn index(&self, index: usize) -> &Self::Output {
        &self.signals[index]
    }
}
impl IndexMut<usize> for Bus
{
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.signals[index]
    }
}

/// Displayトレイとの実装
impl fmt::Display for Bus {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let mut liststr = String::new();
        for item in self.signals.iter() {
            let s = format!("{}\n", item);
            liststr.push_str(&s);
        }

        write!(f, "{}", liststr)
    }
}

/// Fromトレイトの実装
impl From<Vec<Signal>> for Bus {
    fn from(signals: Vec<Signal>) -> Self {
        // keytableを事前に作成しておく
        let mut keytable = HashMap::<String, usize>::new();

        for (idx, sig) in signals.iter().enumerate() {
            keytable.insert(sig.name().to_string(), idx);
        }

        Self {
            signals: signals,
            keytable: keytable,
        }
    }
}

impl From<Vec<SigDef>> for Bus {
    fn from(sigdef: Vec<SigDef>) -> Self {
        let mut keytable = HashMap::<String, usize>::new();
        let mut signals = Vec::new();

        for sig in sigdef.iter() {
            let keyname = sig.name().to_string();
            match keytable.contains_key(&keyname) {
                true => {
                    panic!("信号名が重複しています。: 信号名{}", keyname);
                },
                false => {
                    keytable.insert(keyname, signals.len());
                    signals.push(Signal::new(0.0, sig.name(), sig.unit()));
                }
            }
        }

        Self {
            signals: signals,
            keytable: keytable,
        }
    }
}

/// Busのイテレータオブジェクト実装
type BusIter<'a> = Iter<'a, Signal>;
type BusIterMut<'a> = IterMut<'a, Signal>;

#[cfg(test)]
mod signal_test {
    use super::Signal;
    use super::SigDef;
    use super::Bus;

    #[test]
    fn signaltest() {
        let sig = Signal::new(0.0, "a", "A");

        assert_eq!(sig.value, 0.0);
        assert_eq!(sig.name(), "a");
        assert_eq!(sig.unit(), "A");
    }

    #[test]
    fn printtest() {
        // 表示するだけのテストなので cargo test -- --nocapture にて実行
        let a = Signal::new(0.0, "motor_current", "A");

        println!("{}\n", a);
        println!("signal name is {}\n", a.name());

        let b = Signal::new(1.0, "b", "A");

        let mut list = Bus::new();
        list.push(a);
        list.push(b);
        println!("{}", list);
    }

    #[test]
    fn busaccess_test() {
        let a = Signal::new(0.0, "a", "A");
        let b = Signal::new(1.0, "b", "A");

        let mut list = Bus::new();
        list.push(a);
        list.push(b);

        assert_eq!(list[0].value, 0.0);
        assert_eq!(list[1].value, 1.0);

        assert_eq!(list.get_by_name("a").unwrap().value, 0.0);

        list.get_by_name_mut("a").unwrap().value = 3.0;
        assert_eq!(list[0].value, 3.0);
    
        list[1].value = 2.0;
        assert_eq!(list[1].value, 2.0);
    }

    #[test]
    fn buscreate_test() {
        let sigvec = vec![
            Signal::new(0.0, "motor_trq", "Nm"),
            Signal::new(0.0, "motor_volt", "V"),
            Signal::new(0.0, "motor_current", "A"),
        ];
        let copiedsig = sigvec.clone();

        //let bus:Bus = signals.into();　// Bus::fromと同じ意味
        let bus = Bus::from(sigvec);

        for (idx, _sig) in copiedsig.iter().enumerate() {
            assert_eq!(bus[idx], copiedsig[idx]);
        }
        
    }

    #[test]
    fn busiter_test() {
        let sigvec = vec![
            Signal::new(0.0, "motor_trq", "Nm"),
            Signal::new(0.0, "motor_volt", "V"),
            Signal::new(0.0, "motor_current", "A"),
        ];

        let mut bus = Bus::from(sigvec);

        let mut busiter = bus.iter();
        assert_eq!(busiter.next(), Some(&Signal::new(0.0, "motor_trq", "Nm")));
        assert_eq!(busiter.next(), Some(&Signal::new(0.0, "motor_volt", "V")));
        assert_eq!(busiter.next(), Some(&Signal::new(0.0, "motor_current", "A")));
        assert_eq!(busiter.next(), None);

        for (i, sig) in bus.iter_mut().enumerate() {
            sig.value = i as f64;
        }

        let mut busiter_mut = bus.iter_mut();

        assert_eq!(busiter_mut.next(), Some(&mut Signal::new(0.0, "motor_trq", "Nm")));
        assert_eq!(busiter_mut.next(), Some(&mut Signal::new(1.0, "motor_volt", "V")));
        assert_eq!(busiter_mut.next(), Some(&mut Signal::new(2.0, "motor_current", "A")));
        assert_eq!(busiter_mut.next(), None);
    }

    #[test]
    fn setsig_test() {
        let state_def = vec![SigDef::new("s1", "Nm"), SigDef::new("s2", "rpm")];
        let mut bus = Bus::new();
        let testvec = vec![Signal::new(0.0, "s1", "Nm"), Signal::new(0.0, "s2", "rpm")];

        bus.set_sigdef(&state_def);

        assert_eq!(bus.signals, testvec);

        bus.set_sigdef(&state_def);

        assert_eq!(bus.signals, testvec);

    }
}