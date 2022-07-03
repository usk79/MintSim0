use mintsim::simtools;
use simtools::{*};

fn main() {
    println!("{}\n", simconsts::G);

    println!("{}\n", signal::Signal::new(0.01, "current", "A"));
    
    let mut bus = signal::Bus::new();
    bus.push(signal::Signal::new(0.01, "current", "A"));
    bus.push(signal::Signal::new(0.02, "current", "A"));
}
