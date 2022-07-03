use std::fmt;

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

pub type Bus = Vec<Signal>;