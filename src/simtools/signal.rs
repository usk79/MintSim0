use std::fmt;
use std::collections::HashMap;
use std::ops::Index;
use std::ops::IndexMut;

#[derive(Debug)]
pub struct Signal {
    pub value: f64,   // 値
    name: String,     // 信号名
    unit: String,     // 単位
}

impl Signal {
    pub fn new(initvalue: f64, signame: impl Into<String>, sigunit: impl Into<String>) -> Self {
        Self {
            value: initvalue,
            name: signame.into(),
            unit: sigunit.into(),
        }
    }    
}

impl fmt::Display for Signal {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}: {} [{}]\n", self.name, self.value, self.unit)
    }
}

/// Busの定義
#[derive(Debug)]
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

    pub fn push(&mut self, signal: Signal) {
        self.signals.push(signal);
        // ここから
    }

   /* pub fn get_by_name(&self, signame: impl Into<String>) -> &Signal {
        // ここから
    }*/
}

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

impl fmt::Display for Bus {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let mut liststr = String::new();
        for item in self.signals.iter() {
            let s = format!("{}: {} [{}]\n", item.name, item.value, item.unit);
            liststr.push_str(&s);
        }

        write!(f, "{}", liststr)
    }
}

#[test]
mod signal_test {
    fn printtest() {
        // 表示するだけのテストなので cargo test -- --nocapture にて実行
        let a = Signal::new(0.0, "a", "A");

        println!("{}", a);

        let b = Signal::new(1.0, "b", "A");

        let mut list = Bus::new();
        list.push(a);
        list.push(b);
        println!("{}", list);
    }

    fn busaccess_test() {
        let a = Signal::new(0.0, "a", "A");
        let b = Signal::new(1.0, "b", "A");

        let mut list = Bus::new();
        list.push(a);
        list.push(b);

        assert_eq!(list[0].value, 0.0);
        assert_eq!(list[1].value, 1.0);
    }
}