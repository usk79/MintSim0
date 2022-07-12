use mintsim::simtools;
use simtools::{*};
use simmodel::{*};

fn main() {
    println!("{}\n", simconsts::G);

    println!("{}\n", signal::Signal::new(0.01, "current", "A"));
    
    let mut bus = signal::Bus::new();
    bus.push(signal::Signal::new(0.01, "current", "A"));
    bus.push(signal::Signal::new(0.02, "current", "A"));

    //let model = SpaceStateModel::new(3, 1, 1, SolverType::Euler);
    let model = TransFuncModel::new(&[1.0, 0.0], &[2.0, 1.0, 1.0], SolverType::Euler);

    println!("{}", model);
}
