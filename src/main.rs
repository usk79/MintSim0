use mintsim::simtools;
use simtools::{*};
use simmodel::{*};
use signal::{*};

fn main() {
    
    println!("{}\n", simconsts::G);

    println!("{}\n", signal::Signal::new(0.01, "current", "A"));
    
    let mut bus = signal::Bus::new();
    bus.push(signal::Signal::new(0.01, "current", "A"));
    bus.push(signal::Signal::new(0.02, "current", "A"));

    //let model = SpaceStateModel::new(3, 1, 1, SolverType::Euler);
    let model = TransFuncModel::new(&[1.0, 0.0], &[2.0, 1.0, 1.0], SolverType::Euler).unwrap();

    println!("{}", model);

    let tes = vec![0, 1, 2, 3];
    let testitr = tes.iter();

    print_typename(testitr);
}

fn print_typename<T>(_: T) {
    println!("{}", std::any::type_name::<T>());
}