use crate::robot_veronika::distribution_robot::DistributorRobotFactory;
use crate::oh_crab_visualizer::visualizer::visualizer::{OhCrabVisualizerConfig, OhCrabVisualizer, RunMode};
use robotics_lib::world::tile::Content;

pub fn run_distribution_robot(){
    let robot_factory = DistributorRobotFactory::new(vec![Content::Rock(1).index(), Content::Fish(1).index(), Content::Tree(1).index()]);
    let world_generator = crate::world_gen_utils::load_or_generate_world(128, 420);

    //let config = OhCrabVisualizerConfig::new(RunMode::Interactive, false);
    let config = OhCrabVisualizerConfig::new(RunMode::NonInteractive(500), false);
    let mut visualizer = OhCrabVisualizer::new(robot_factory, world_generator, config);

    visualizer.simulate().unwrap();
    // match visualizer.run() {
    //     Ok(_) => {}
    //     Err(err) => println!("Visualizer run returned error {:?}", err),
    // }
}

