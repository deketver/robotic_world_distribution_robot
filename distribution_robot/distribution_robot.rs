use robotics_lib::{runner::{Robot, Runnable}, world::tile::{Content, Tile}};
use robotics_lib::interface::{Direction};
use crate::{oh_crab_visualizer::visualizer::{visualizable_interfaces::VisualizableInterfaces, visualizable_robot::{RobotCreator, Visulizable}, visualizer_event_listener::VisualizerEventListener}, println_d};
use crate::robot_veronika::partitioning::PartitioningProblem;
use crate::robot_veronika::content_pick::collect::{CollectTool, LibErrorExtended};
use crate::robot_veronika::storage::{StorageInfo, Position};
use robotics_lib::utils::LibError;
use robotics_lib::event::events::Event::EnergyRecharged;
use rust_and_furious_dynamo::dynamo::Dynamo;
use rust_eze_tomtom; //TomTom::get_path_to_coordinates
use strum::IntoEnumIterator;
use std::collections::{BinaryHeap};


pub struct DistributorRobot{
    robot: Robot,
    tick_counter: usize,
    desired_content: Vec<usize>,
    /// By default, exploration phase finished will be set to false.
    exploration_finished: bool,
    /// By default, partitioning solved phase will be set to false.
    partitioning_solved: bool,
    /// Robot's discovered size of the robot, by default set to 0.
    world_size: usize,
    /// Robot's discovered targets to collect.
    targets: BinaryHeap<StorageInfo>,
    //TODO: tests on robots capacity
    markets: Vec<Position>,
    banks: Vec<Position>,
    /// Indexes into which markets should the content go
    //markets_indexes: VecDeque<usize>,
    visualizer_event_listener: VisualizerEventListener
}

impl DistributorRobot{
    pub fn exploration_phase(&mut self, world: &mut robotics_lib::world::World)-> Result<(), LibError> {
        let _ = VisualizableInterfaces::robot_view(self, world);
        let robot_world = VisualizableInterfaces::robot_map(self, world).unwrap();
        if self.world_size == 0 {
            self.world_size = robot_world.len();
        }

        let view_output = VisualizableInterfaces::one_direction_view(self, world, Direction::Up, self.world_size)?;
        let furthest_top_coordinates = &view_output[view_output.len()-1];

        // check if there exists a path going to any of those top high and top bottom coordinates
        let mut top_coordinates: Option<(usize, usize)> = None;
        let mut up_path_len = 0;
        for i in 0..furthest_top_coordinates.len() {
            let coordinate_robot: (usize, usize) = (self.get_coordinate().get_row(), self.get_coordinate().get_col());
            let coordinate_test: (usize, usize) = (coordinate_robot.0  - (view_output.len()), coordinate_robot.1 - i);
            println!("Testing coordinates {:?}, UP", coordinate_test);
            //let path = rust_eze_tomtom::TomTom::get_path_to_coordinates(self, world, false, coordinate_test);
            let path = CollectTool::return_path_to_coordinates(self, world, coordinate_test);
            println!("Got over path");
            if path.is_ok(){
                //at this point, we were able to get to the top
                top_coordinates = Some(coordinate_test);
                up_path_len = path.unwrap().len();
                //up_path_len = path.unwrap().actions.len();
                break;
            }
        }

        let view_output = VisualizableInterfaces::one_direction_view(self, world, Direction::Down, self.world_size)?;
        let furthest_bottom_coordinates = &view_output[view_output.len()-1];
        // check if there exists a path going to any of those top high and top bottom coordinates
        let mut bottom_coordinates: Option<(usize, usize)> = None;
        let mut bottom_path_len = 0;
        for i in 0..furthest_bottom_coordinates.len() {
            let coordinate_robot: (usize, usize) = (self.get_coordinate().get_row(), self.get_coordinate().get_col());
            let coordinate_test: (usize, usize) = (coordinate_robot.0  + (view_output.len()), coordinate_robot.1 - i);
            println!("Testing coordinates {:?}, DOWN", coordinate_test);
            //let path = rust_eze_tomtom::TomTom::get_path_to_coordinates(self, world, false, coordinate_test);
            let path = CollectTool::return_path_to_coordinates(self, world, coordinate_test);
            if path.is_ok(){
                //at this point, we were able to get to the top
                bottom_coordinates = Some(coordinate_test);
                //bottom_path_len = path.unwrap().actions.len();
                bottom_path_len = path.unwrap().len();
                break;
            }
        }

        if top_coordinates.is_some() && bottom_coordinates.is_some() {
            println!("Top and bottom are connected, we can walk up or down to look around");
            let top_coordinates = top_coordinates.unwrap();
            let bottom_coordinates = bottom_coordinates.unwrap();

            // check also right and left
            let _ = VisualizableInterfaces::one_direction_view(self, world, Direction::Left, self.world_size)?;
            let _ = VisualizableInterfaces::one_direction_view(self, world, Direction::Right, self.world_size)?;


            if up_path_len > bottom_path_len{
                let path_to_bottom = CollectTool::return_path_to_coordinates(self,
                                                                             world,
                                                                             bottom_coordinates).unwrap_or(vec![]);
                for direction in path_to_bottom {
                    let _ = VisualizableInterfaces::robot_view(self, world);
                    let _ = VisualizableInterfaces::go(self, world, direction);
                    let _ = VisualizableInterfaces::one_direction_view(self, world, Direction::Left, self.world_size)?;
                    let _ = VisualizableInterfaces::one_direction_view(self, world, Direction::Right, self.world_size)?;
                }

                let path_up = CollectTool::return_path_to_coordinates(self,
                                                                      world,
                                                                      top_coordinates).unwrap_or(vec![]);

                for direction in path_up{
                    let _ = VisualizableInterfaces::robot_view(self, world);
                    let _ = VisualizableInterfaces::go(self, world, direction);
                    let _ = VisualizableInterfaces::one_direction_view(self, world, Direction::Left, self.world_size)?;
                    let _ = VisualizableInterfaces::one_direction_view(self, world, Direction::Right, self.world_size)?;
                }
            }
            else{
                let path_to_top = CollectTool::return_path_to_coordinates(self,
                                                                          world,
                                                                          top_coordinates).unwrap_or(vec![]);
                for direction in path_to_top {
                    let _ = VisualizableInterfaces::robot_view(self, world);
                    let _ = VisualizableInterfaces::go(self, world, direction);
                    let _ = VisualizableInterfaces::one_direction_view(self, world, Direction::Left, self.world_size)?;
                    let _ = VisualizableInterfaces::one_direction_view(self, world, Direction::Right, self.world_size)?;
                }

                let path_bottom = CollectTool::return_path_to_coordinates(self,
                                                                          world,
                                                                          bottom_coordinates).unwrap_or(vec![]);

                for direction in path_bottom{
                    let _ = VisualizableInterfaces::robot_view(self, world);
                    let _ = VisualizableInterfaces::go(self, world, direction);
                    let _ = VisualizableInterfaces::one_direction_view(self, world, Direction::Left, self.world_size)?;
                    let _ = VisualizableInterfaces::one_direction_view(self, world, Direction::Right, self.world_size)?;
                }
            }
        }
        else {
            let view_output = VisualizableInterfaces::one_direction_view(self, world, Direction::Left, self.world_size)?;
            let mut left_coordinates: Option<(usize, usize)> = None;
            let mut left_path_len = 0;
            for i in 0..view_output.len()-1 {
                let second_index = &view_output[i].len() - 1;
                let coordinate_robot: (usize, usize) = (self.get_coordinate().get_row(), self.get_coordinate().get_col());
                let coordinate_test: (usize, usize) = (coordinate_robot.0  - i, coordinate_robot.1 - (second_index));
                println!("Testing coordinates {:?}, LEFT", coordinate_test);
                println!("Robot's position {:?}", coordinate_robot);
                //let path = rust_eze_tomtom::TomTom::get_path_to_coordinates(self, world, false, coordinate_test);
                let path = CollectTool::return_path_to_coordinates(self, world, coordinate_test);
                if path.is_ok(){
                    //at this point, we were able to get to the left
                    left_coordinates = Some(coordinate_test);
                    //left_path_len = path.unwrap().actions.len();
                    left_path_len = path.unwrap().len();
                    break;
                }
            }

            let view_output = VisualizableInterfaces::one_direction_view(self, world, Direction::Right, self.world_size)?;
            let mut right_coordinates: Option<(usize, usize)> = None;
            let mut right_path_len = 0;
            for i in 0..view_output.len()-1 {
                let second_index = &view_output[i].len() - 1;
                let coordinate_robot: (usize, usize) = (self.get_coordinate().get_row(), self.get_coordinate().get_col());
                let coordinate_test: (usize, usize) = (coordinate_robot.0  - i, coordinate_robot.1 + (second_index));
                println!("Testing coordinates {:?}, RIGHT", coordinate_test);
                //let path = rust_eze_tomtom::TomTom::get_path_to_coordinates(self, world, false, coordinate_test);
                let path = CollectTool::return_path_to_coordinates(self, world, coordinate_test);
                if path.is_ok(){
                    right_coordinates = Some(coordinate_test);
                    //right_path_len = path.unwrap().actions.len();
                    right_path_len = path.unwrap().len();
                    break;
                }
            }

            if left_coordinates.is_some() && right_coordinates.is_some(){
                let left_coordinates = left_coordinates.unwrap();
                let right_coordinates = right_coordinates.unwrap();

                if left_path_len < right_path_len{
                    let path_to_left = CollectTool::return_path_to_coordinates(self,
                                                                                 world,
                                                                                 left_coordinates).unwrap_or(vec![]);
                    for direction in path_to_left {
                        let _ = VisualizableInterfaces::robot_view(self, world);
                        let _ = VisualizableInterfaces::go(self, world, direction);
                        let _ = VisualizableInterfaces::one_direction_view(self, world, Direction::Up, self.world_size)?;
                        let _ = VisualizableInterfaces::one_direction_view(self, world, Direction::Down, self.world_size)?;
                    }

                    let path_right = CollectTool::return_path_to_coordinates(self,
                                                                          world,
                                                                          right_coordinates).unwrap_or(vec![]);

                    for direction in path_right{
                        let _ = VisualizableInterfaces::robot_view(self, world);
                        let _ = VisualizableInterfaces::go(self, world, direction);
                        let _ = VisualizableInterfaces::one_direction_view(self, world, Direction::Up, self.world_size)?;
                        let _ = VisualizableInterfaces::one_direction_view(self, world, Direction::Down, self.world_size)?;
                    }

                }
                else{
                    let path_to_right = CollectTool::return_path_to_coordinates(self,
                                                                                world,
                                                                                right_coordinates).unwrap_or(vec![]);
                    for direction in path_to_right{
                        let _ = VisualizableInterfaces::robot_view(self, world);
                        let _ = VisualizableInterfaces::go(self, world, direction);
                        let _ = VisualizableInterfaces::one_direction_view(self, world, Direction::Up, self.world_size)?;
                        let _ = VisualizableInterfaces::one_direction_view(self, world, Direction::Down, self.world_size)?;
                    }
                    let path_to_left = CollectTool::return_path_to_coordinates(self,
                                                                               world,
                                                                               left_coordinates).unwrap_or(vec![]);
                    for direction in path_to_left {
                        let _ = VisualizableInterfaces::robot_view(self, world);
                        let _ = VisualizableInterfaces::go(self, world, direction);
                        let _ = VisualizableInterfaces::one_direction_view(self, world, Direction::Up, self.world_size)?;
                        let _ = VisualizableInterfaces::one_direction_view(self, world, Direction::Down, self.world_size)?;
                    }
                }
            }
        }

        let portion_explored = self.get_quantity_explored_world(world);
        if portion_explored > 0.99{
            self.exploration_finished = true;
            println!("Portion explored is {}", portion_explored);
            return Ok(());
        }
        Err(LibError::OutOfBounds)
    }

    pub fn solve_packaging_problem(&mut self){
        let weights: Vec<u32> = self.extract_storage_into_weights();
        let evolutionary_algo = PartitioningProblem::new(
            weights,
            self.markets.len(),
            100,
            1000,
            0.8,
            0.22,
            0.085,
            5
        );
        let best_solution: Vec<usize> = evolutionary_algo.main_exec("logs/market_distribution.log");
        println!("Best solution is {:?}", best_solution);
        self.partitioning_solved = true;

        let mut new_targets = BinaryHeap::new();

        for item in best_solution{
            let mut target = self.targets.pop().unwrap();
            target.set_market_index(item);
            new_targets.push(target);
        }
        self.targets = new_targets;
    }

    pub fn deliver_content(&mut self, world: &mut robotics_lib::world::World)-> Result<(), LibErrorExtended>{
        while let Some(target) = self.targets.pop() {
            println!("I am distributing the content!");
            //let market_index: usize = self.markets_indexes.pop_front().unwrap();
            // go to collect the item first

            let mut path_to_target = CollectTool::return_path_to_coordinates(self,
                                                                             world,
                                                                             (target.get_position().get_row(),
                                                                              target.get_position().get_col()))?;

            let last_step = path_to_target.pop();
            for direction in path_to_target {
                let _ = VisualizableInterfaces::go(self, world, direction);
            }
            let _ = VisualizableInterfaces::destroy(self, world, last_step.unwrap_or(Direction::Up));
            let maker_position = &self.markets[target.get_market_index()];
            let mut path_to_market = CollectTool::return_path_to_coordinates(self,
                                                                             world,
                                                                             (maker_position.get_row(),
                                                                              maker_position.get_col()))?;
            let last_step = path_to_market.pop();
            for direction in path_to_market {
                let _ = VisualizableInterfaces::go(self, world, direction);
            }

            let content = match target.get_content() {
                0 => Content::Rock(0),
                1 => Content::Tree(0),
                4 => Content::Coin(0),
                10 => Content::Fish(0),
                11 => Content::Market(0),
                _ => Content::None
            };
            // put stuff into market
            let _ = VisualizableInterfaces::put(self, world, content, target.get_quantity(), last_step.unwrap_or(Direction::Up));

            // now we should put money to the bank

            // find the closes bank
            let closest_bank = self.get_closest_bank();
            let mut path_to_bank = CollectTool::return_path_to_coordinates(self,
                                                                           world,
                                                                           (closest_bank.get_row(),
                                                                            closest_bank.get_col()))?;
            let last_step = path_to_bank.pop();
            for direction in path_to_bank {
                let _ = VisualizableInterfaces::go(self, world, direction);
            }

            let coins = Content::Coin(0);
            let quantity: usize = match self.get_backpack().get_contents().get(&coins) {
                Some(quantity) => *quantity,
                None => 0
            };

            let _ = VisualizableInterfaces::put(self, world, coins, quantity, last_step.unwrap_or(Direction::Up));
            break;
        }
        return Ok(());
    }

    pub fn get_quantity_explored_world(&mut self, world: &mut robotics_lib::world::World) -> f32 {
        let robot_world = VisualizableInterfaces::robot_map(self, world).unwrap();
        let number_of_tiles: usize = robot_world.len() * robot_world.len();
        let mut non_none_tiles_counter: u32 = 0;
        for (i,row) in robot_world.iter().enumerate() {
            for (j,col) in row.iter().enumerate() {
                if col.is_none() {
                    continue;
                }
                non_none_tiles_counter += 1;
                let tile = col.clone().unwrap();
                if self.desired_content.contains(&tile.content.index()) {
                    let position = Position::new(i, j);
                    let value = tile.content.get_value().0.unwrap();
                    if value < 1 {
                        continue;
                    }
                    let storage_info = StorageInfo::new(position,
                                                        tile.content.index(),
                                                        tile.content.get_value().0.unwrap());
                    self.targets.push(storage_info);
                }
                else if tile.content.index() == Content::Market(0).index() {
                    let position = Position::new(i, j);
                    if !self.markets.contains(&position) {
                        self.markets.push(position);
                    }
                }
                else if tile.content.index() ==Content::Bank(0..0).index(){
                    let position=Position::new(i, j);
                    if !self.banks.contains(&position){
                        self.banks.push(position);
                    }
                }
            }
        }
        return (non_none_tiles_counter as f32) / (number_of_tiles as f32);
    }

    #[allow(dead_code)]
    fn print_nicer_known_world_map(&self, known_world: &Vec<Vec<Option<Tile>>>) {
        for row in known_world {
            for col in row {
                if col.is_none() {
                    print!("None\t");
                    continue;
                }
                let tile = col.clone().unwrap();
                print!("Type {:?},", tile.tile_type);
                print!("{:?} ", tile.content);
                //print!("(cost {:?})\t", tile.tile_type.properties().cost());
            }
            println!();
        }
    }

    fn extract_storage_into_weights(&self) -> Vec<u32>{
        let mut weights = Vec::with_capacity(self.targets.len());
        let mut targets = self.targets.clone();
        while let Some(target) = targets.pop(){
            weights.push((target.get_quantity() as u32) * target.get_coefficient());
        }
        weights
    }

    fn get_closest_bank(&self) -> Position{
        let mut closest_bank: Option<Position> = None;
        let mut closest_distance: i32 = 100000;
        for bank in &self.banks{
            let distance = (bank.get_row() as i32 - self.get_coordinate().get_row() as i32).abs() +
                (bank.get_col() as i32 - self.get_coordinate().get_col() as i32).abs();
            if distance < closest_distance{
                closest_distance = distance;
                closest_bank = Some(bank.clone());
            }
        }
        closest_bank.unwrap()
    }
}

pub struct DistributorRobotFactory {
    desired_content: Vec<usize>,
    exploration_finished: bool,
    partitioning_solved: bool,
    world_size: usize,
    targets: BinaryHeap<StorageInfo>,
    markets: Vec<Position>,
    banks: Vec<Position>,
    //markets_indexes: VecDeque<usize>,
}

impl DistributorRobotFactory {
    pub fn new(desired_content: Vec<usize>) -> DistributorRobotFactory {
        DistributorRobotFactory{desired_content, exploration_finished: false,
            partitioning_solved: false, world_size: 0,
            targets: BinaryHeap::new(), markets: Vec::new(), banks: Vec::new()}
    }
}

impl RobotCreator for DistributorRobotFactory {
    fn create(&self, data_sender: VisualizerEventListener) -> Box<dyn Runnable> {
        let distributor_robot = DistributorRobot { robot: Robot::new(), tick_counter: 0,
            desired_content: self.desired_content.clone(),
            exploration_finished: self.exploration_finished,
            partitioning_solved: self.partitioning_solved,
            world_size: self.world_size,
            targets: self.targets.clone(),
            markets: self.markets.clone(),
            banks: self.banks.clone(),
            visualizer_event_listener: data_sender };
        Box::new(distributor_robot)
    }
}

impl<'a> Visulizable<'a> for DistributorRobot {
    fn borrow_event_listener(&'a self) -> &'a VisualizerEventListener{
        &self.visualizer_event_listener
    }
}

impl Runnable for DistributorRobot{
    fn process_tick(&mut self, world: &mut robotics_lib::world::World) {
        self.tick_counter+=1;
        println!("CURRENT TICK is {}", self.tick_counter);
        println!("CURRENT SCORE IS {}", VisualizableInterfaces::get_score(self, world));
        println!("Robot's position {:?}", self.robot.coordinate);

        ////// EXPLORATION PHASE
        if self.exploration_finished == false {
            let exploration_output = self.exploration_phase(world);
            if exploration_output.is_err(){
                println!("Exploration didn't go well, I choose random direction");
                // rand index generation
                let random_index = rand::random::<usize>() % Direction::iter().len();
                println!("Random index is {}", random_index);
                let direction = match random_index {
                    0 => Direction::Down,
                    1 => Direction::Up,
                    2 => Direction::Right,
                    3 => Direction::Left,
                    _ => Direction::Up
                };
                let _ = VisualizableInterfaces::go(self, world, direction);
            }
        }
        ////// PARTITIONING PROBLEM SOLUTION PHASE
        else if self.partitioning_solved == false{
            if self.markets.len() < 1 || self.banks.len() < 1{
                println!("I have nothing to do in the world, banks or markets are missing.")
            }
            else{
                println!("I am solving partitioning problem with an evolutionary algorithm!");
                let _ = self.solve_packaging_problem();
            }
        }
        ////// DELIVERY PHASE
        else{
                let output = self.deliver_content(world);
                if output.is_err(){
                    println!("Something went wrong with distribution");
                }
        }

        // check if there are no more targets
        if self.targets.len() == 0{
            println!("I am out of targets, everything is delivered.");
        }
    }

    fn handle_event(&mut self, event: robotics_lib::event::events::Event) {
        println_d!("Example robot received event: {}", event);
        // BEWARE - for a visualizer to work it is necessary to call this method from
        // handle_event method of your robot
        self.visualizer_event_listener.handle_event(&event);
        if self.get_energy().get_energy_level() < 300 {
            let previous_energy = self.get_energy().get_energy_level();
            *self.get_energy_mut()=Dynamo::update_energy();
            self.handle_event(EnergyRecharged(1000-previous_energy));
        }
    }

    fn get_energy(&self) -> &robotics_lib::energy::Energy {
        &self.robot.energy
    }

    fn get_energy_mut(&mut self) -> &mut robotics_lib::energy::Energy {
        &mut self.robot.energy
    }

    fn get_coordinate(&self) -> &robotics_lib::world::coordinates::Coordinate {
        & self.robot.coordinate
    }

    fn get_coordinate_mut(&mut self) -> &mut robotics_lib::world::coordinates::Coordinate {
        &mut self.robot.coordinate
    }

    fn get_backpack(&self) -> &robotics_lib::runner::backpack::BackPack {
        &self.robot.backpack
    }

    fn get_backpack_mut(&mut self) -> &mut robotics_lib::runner::backpack::BackPack {
        &mut self.robot.backpack
    }

}