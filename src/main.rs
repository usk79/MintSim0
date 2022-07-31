use std::process;

use mintsim::simtools;
use simtools::{*};
use simrunner::{*};
use simmodel::{*};
use signal::{*};

fn main() {

    simrun_test();
    
    println!("{}\n", simconsts::G);

    println!("{}\n", signal::Signal::new(0.01, "current", "A"));
    
    let mut bus = signal::Bus::new();
    bus.push(signal::Signal::new(0.01, "current", "A"));
    bus.push(signal::Signal::new(0.02, "current", "A"));

    //let model = SpaceStateModel::new(3, 1, 1, SolverType::Euler);
    let model = TransFuncModel::new("TestModel", &[1.0, 0.0], &[2.0, 1.0, 1.0], SolverType::Euler).unwrap();

    println!("{}", model);

    let tes = vec![0, 1, 2, 3];
    let testitr = tes.iter();

    print_typename(testitr);
}

fn print_typename<T>(_: T) {
    println!("{}", std::any::type_name::<T>());
}

fn simrun_test() {
    let model = TransFuncModel::new("test", &[1.0, 0.0, 2.0, 2.0], &[2.0, 1.0, 1.0], SolverType::Euler).unwrap_or_else(|e| {
        println!("{:?}", e);
        process::exit(1);
    });
    let mut sim = SimRunner::new(10.0, 0.001, model);

    sim.run_sim().unwrap_or_else(|e| {
        println!("{:?}", e);
        process::exit(1);
    });
    

}