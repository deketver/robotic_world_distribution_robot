use crate::robot_veronika::content_pick::a_star_search::{
    DirectionWalk, HeuristicProblem, Position, Solution, State,
};
use crate::robot_veronika::content_pick::path_search::{a_start_path, PathSearch};
use crate::robot_veronika::content_pick::robot_search::{a_start, RobotSearch};

use robotics_lib::interface::{destroy, go, robot_map, robot_view, Direction, Tools, look_at_sky};
use robotics_lib::runner::Runnable;
use robotics_lib::utils::{LibError};
use robotics_lib::world::tile::{Content, Tile};
use robotics_lib::world::World;

#[derive(Debug)]
pub enum LibErrorExtended {
    CommonError(LibError),
    NoSolution,
    RobotMapEmpty,
    NoWalkableTile,
    EnergyOutOfLimit
}

/// Tool for collecting desired content into the robot's backpack.
pub struct CollectTool;

impl CollectTool {
    /// Collects the desired content the cheapest way in the robot's known map of the world.
    ///
    /// Function attempts to collect the content from the world via navigating in the world
    /// the way which consumes the least energy (considering tile type, its elevetaion and current
    /// weather conditions), until the robot uses the amount of energy specified,
    /// there is not enough place in the robot's backpack, the amount of content which was aim to collect
    /// is already collected, or there is not any tile which has desired content.
    /// If the required content is located on the same tile as the robot is standing at,
    /// robot will be moved to the cheapest tile around, if the required energy still meets the
    /// specified limit.
    /// Also collects items which are instantly reachable (can be collection without walking).
    ///
    /// The search is done via A* algorithm. Algorithm takes into consideration only path
    /// through the tiles which are walkable.
    ///
    /// Warning: usage of this function can cost energy.
    ///
    /// # Arguments
    /// - `robot` : The robot which is moving
    /// - `world`: The world in which is robot located
    /// - `search_content`: The content you are trying to collect into robot's backpack.
    /// - `quantity`: The amount of content you are trying to collect.
    /// - `energy_limit_cost`: The energy amount the robot can spend on the action (collecting the content).
    ///
    /// # Returns
    /// Returns usize of how many items were collected.
    ///
    /// Result<usize, LibErrorExtended>
    ///
    /// # Example
    /// ```rust
    /// impl Runnable for MockRobot {
    ///     fn process_tick(&mut self, world: &mut robotics_lib::world::World) {
    ///
    ///         // try to collect content
    ///         let search_content = Content::Coin(1);
    ///         let _ = CollectTool::collect_content(self, world, &search_content, 10usize, 100usize);
    /// ```
    /// # Errors:
    /// - `NoSolution`: No solution for given content found (not found in the known map)
    /// - `NotEnoughEnergy`: The robot doesn't have enough energy to do the action
    /// - `CannotDestroy`: The content cannot be destroyed by the robot
    /// - `RobotMapEmpty`: The map of the robot's known world is empty
    /// - `NotEnoughSpace(usize)`: The backpack doesn't have enough space
    /// - `NoWalkableTile`: There is no walkable tile around the robot.
    /// - `EnergyOutOfLimit`: The energy required exceeds the energy limit.
    ///
    /// # Contact:
    /// In case of any qestions or issues, please contanct me on telegram: @deketver
    /// or via email: veronika.deketova@studenti.unitn.it , so we can have a look at it together
    /// or we can update the tool to meet your needs.
    pub fn collect_content(
        robot: &mut impl Runnable,
        world: &mut World,
        search_content: &Content,
        quantity: usize,
        energy_limit_cost: usize,
    ) -> Result<usize, LibErrorExtended> {
        // does destroy cost energy?
        let collect_instant_output = Self::collect_instantly_reachable(robot, world, search_content)?;
        let collect_instant_output = collect_instant_output;
        let robot_energy_limit = robot.get_energy().get_energy_level() - energy_limit_cost;

        // keep track of how many content we have collected
        // get value from instant collection
        let mut content_counter: usize = 0;
        content_counter += collect_instant_output;
        while robot.get_energy().get_energy_level() > robot_energy_limit  && content_counter < quantity {
            let search_result = Self::check_map_for_content(robot, world, search_content);
            if search_result.is_err() {
                return Err(LibErrorExtended::NoSolution);
            }
            let search_result = search_result.unwrap();
            if search_result.cost > energy_limit_cost {
                return Err(LibErrorExtended::EnergyOutOfLimit);
            }
            let actions = search_result.actions;
            if actions.len() < 1 {
                // content is at the same tile as the Robot is standing - so it needs to move to any possible direction
                let moving_option = Self::get_cheapest_walkable_around(robot, world, search_content);
                if moving_option.is_none(){
                    return Err(LibErrorExtended::NoWalkableTile);
                }
                let moving_option = moving_option.unwrap();
                if robot.get_energy().get_energy_level()-moving_option.1 < robot_energy_limit{
                    return Err(LibErrorExtended::EnergyOutOfLimit)
                }
                match moving_option.0 {
                    DirectionWalk::Up { dx: _, dy: _ } => {
                        let direction = Direction::Up;
                        let go_result = go(robot, world, direction);
                        if go_result.is_err() {
                            return Err(LibErrorExtended::CommonError(LibError::NotEnoughEnergy));
                        }
                    }
                    DirectionWalk::Down { dx: _, dy: _ } => {
                        let direction = Direction::Down;
                        let go_result = go(robot, world, direction);
                        if go_result.is_err() {
                            return Err(LibErrorExtended::CommonError(LibError::NotEnoughEnergy));
                        }
                    }
                    DirectionWalk::Left { dx: _, dy: _ } => {
                        let direction = Direction::Left;
                        let go_result = go(robot, world, direction);
                        if go_result.is_err() {
                            return Err(LibErrorExtended::CommonError(LibError::NotEnoughEnergy));
                        }
                    }
                    DirectionWalk::Right { dx: _, dy: _ } => {
                        let direction = Direction::Right;
                        let go_result = go(robot, world, direction);
                        if go_result.is_err() {
                            return Err(LibErrorExtended::CommonError(LibError::NotEnoughEnergy));
                        }
                    }
                }
            }
            else {
                match actions[0].unwrap() {
                    DirectionWalk::Up { dx: _, dy: _ } => {
                        let direction = Direction::Up;
                        let go_result = go(robot, world, direction);
                        if go_result.is_err() {
                            return Err(LibErrorExtended::CommonError(LibError::NotEnoughEnergy));
                        }
                    }
                    DirectionWalk::Down { dx: _, dy: _ } => {
                        let direction = Direction::Down;
                        let go_result = go(robot, world, direction);
                        if go_result.is_err() {
                            return Err(LibErrorExtended::CommonError(LibError::NotEnoughEnergy));
                        }
                    }
                    DirectionWalk::Left { dx: _, dy: _ } => {
                        let direction = Direction::Left;
                        let go_result = go(robot, world, direction);
                        if go_result.is_err() {
                            return Err(LibErrorExtended::CommonError(LibError::NotEnoughEnergy));
                        }
                    }
                    DirectionWalk::Right { dx: _, dy: _ } => {
                        let direction = Direction::Right;
                        let go_result = go(robot, world, direction);
                        if go_result.is_err() {
                            return Err(LibErrorExtended::CommonError(LibError::NotEnoughEnergy));
                        }
                    }
                }
            }
            let collect_instant_output = Self::collect_instantly_reachable(robot, world, search_content);
            if collect_instant_output.is_err() {
                return Err(collect_instant_output.unwrap_err());
            }
            let collect_instant_output = collect_instant_output.unwrap();
            content_counter += collect_instant_output;
        }
        return Ok(content_counter);
    }

    /// This function allows to collect desire Content, which is directly approachable (it is placed
    /// on the tile in direction Up, Down, Left, Right).
    ///
    /// In order for the Content to be collected, it has to be destroyable:
    /// content.properties().destroy() == true
    /// # Arguments
    /// - `robot` : The robot which is moving
    /// - `world`: The world in which is robot located
    /// - `search_content`: The content you are trying to collect into robot's backpack.
    ///
    /// # Returns
    /// Returns usize of how many items were collected.
    /// Result<usize, LibErrorExtended>
    ///
    /// # Example
    /// usage in process_tick() function as following
    /// ```rust
    /// impl Runnable for MockRobot {
    /// fn process_tick(&mut self, world: &mut robotics_lib::world::World) {
    /// //try to collect content
    ///    let search_content = Content::Coin(1);
    ///    let _ = CollectTool::collect_instantly_reachable(self, world, &search_content);
    ///  }
    ///```
    ///
    /// # Errors
    /// - `CannotDestroy`: The content cannot be destroyed by the robot
    /// - `RobotMapEmpty`: The map of the robot's known world is empty
    /// - `NotEnoughSpace(usize)`: The backpack doesn't have enough space
    /// - `EnergyOutOfLimit`: The energy required exceeds the energy limit.
    ///
    /// # Contact:
    /// In case of any qestions or issues, please contanct me on telegram: @deketver
    /// or on email: veronika.deketova@studenti.unitn.it , so we can have a look at it together
    /// or we can update the tool to meet your needs.
    pub fn collect_instantly_reachable(
        robot: &mut impl Runnable,
        world: &mut World,
        search_content: &Content,
    ) -> Result<usize, LibErrorExtended> {
        // also do raise error when there is no more space in the backpack?
        // or the error will be raised be the destroy() interface
        if !Self::check_content_destroyable(search_content) {
            return Err(LibErrorExtended::CommonError(LibError::CannotDestroy));
        }
        let _robot_position = robot.get_coordinate();
        //println!("ROBOT POSITION: {:?}", robot_position);
        // count the amout of content we have collected
        let mut content_counter: usize = 0;

        // robot view also saves what the robot can see into its memory
        let robot_view = robot_view(robot, world);
        for (i, row) in robot_view.iter().enumerate() {
            for (j, col) in row.iter().enumerate() {
                let tile = col.clone();
                if tile.is_none() {
                    continue;
                }
                let tile_content = &tile.unwrap().content;
                //println!("TILE CONTENT: {:?}", tile_content);
                // check if tile content is the same as searched content
                if tile_content.index() == search_content.index() {
                    // get Direction of given tile
                    let direction = CollectTool::create_direction(i, j);
                    if direction.is_none() {
                        continue;
                    }
                    // let's try to destroy given item
                    let destroy_result = destroy(robot, world, direction.unwrap());
                    //println!("DESTROY RESULT: {:?}", destroy_result);
                    let destroy_output = match destroy_result {
                        Ok(output) => output,
                        Err(error) => return match error {
                            LibError::NotEnoughEnergy => Err(LibErrorExtended::CommonError(LibError::NotEnoughEnergy)),
                            LibError::NotEnoughSpace(_) => Err(LibErrorExtended::CommonError(LibError::NotEnoughSpace(0))),
                            _ => Err(LibErrorExtended::CommonError(LibError::CannotDestroy))
                        }
                    };
                    content_counter += destroy_output;
                }
            }
        }
        return Ok(content_counter);
    }

    pub fn return_path_to_coordinates(robot: &mut impl Runnable,world: &mut World, target: (usize, usize))->
    Result<Vec<Direction>, LibErrorExtended>{
        let robot_world = robot_map(world);
        if robot_world.is_none() {
            //println!("Robot map is empty");
            return Err(LibErrorExtended::RobotMapEmpty);
        }
        let robot_world = robot_world.unwrap();

        //println!("KNOWN ROBOT WORLD: ");
        //Self::print_nicer_known_world_map(&robot_world);

        let robot_position = robot.get_coordinate();
        let path_search = PathSearch::new(
            &robot_world,
            robot_position.get_row(),
            robot_position.get_col(),
            target.0,
            target.1,
            look_at_sky(world)
        );
        let solution = a_start_path(&path_search)?;
        //println!("SOLUTION: {:?}", solution);

        let mut converted_actions = Vec::new();
        for action in solution.actions.iter(){
            let converted_action = match action.unwrap() {
                DirectionWalk::Up { dx: _, dy: _ } => Direction::Up,
                DirectionWalk::Down { dx: _, dy: _ } => Direction::Down,
                DirectionWalk::Left { dx: _, dy: _ } => Direction::Left,
                DirectionWalk::Right { dx: _, dy: _ } => Direction::Right,
            };
            converted_actions.push(converted_action);
        }
        return Ok(converted_actions);
    }

    /// Function checks tiles around robot and returns direction to the cheapest tile
    /// to get move to, so it can collect the content
    pub(crate) fn get_cheapest_walkable_around(
        robot: &mut impl Runnable,
        world: &mut World,
        search_content: &Content
    )->Option<(DirectionWalk, usize)>{
        // let robot_coordinate = robot.get_coordinate();
        // let robots_position = Position::new(robot_coordinate.get_row(), robot_coordinate.get_col());
        let robot_world = robot_map(world);
        if robot_world.is_none() {
            return None;
        }
        let robot_world = robot_world.unwrap();

        let robot_position = robot.get_coordinate();
        let robot_position = Position::new(robot_position.get_row(), robot_position.get_col());
        let robot_search = RobotSearch::new(
            &robot_world,
            robot_position.x,
            robot_position.y,
            search_content.index(),
            look_at_sky(world)
        );
        let possible_directions = vec![DirectionWalk::Up {dx: -1, dy: 0}, DirectionWalk::Down { dx: 1, dy: 0 },
                                       DirectionWalk::Left {dx: 0, dy: -1}, DirectionWalk::Right {dx: 0, dy: 1}];
        let mut min_cost: usize = 100000;
        let mut min_cost_direction = None;
        for direction in possible_directions.iter(){
            let dx = match direction {
                DirectionWalk::Up { dx, .. } => *dx,
                DirectionWalk::Down { dx, .. } => *dx,
                DirectionWalk::Left { dx, .. } => *dx,
                DirectionWalk::Right { dx, .. } => *dx,
            };

            let dy = match direction {
                DirectionWalk::Up { dx: _, dy } => *dy,
                DirectionWalk::Down { dx: _, dy } => *dy,
                DirectionWalk::Left { dx: _, dy } => *dy,
                DirectionWalk::Right { dx: _, dy } => *dy,
            };
            // println!("DIRECTION WAS {:?}", direction);
            // println!("Robots position {:?}", robot_position);
            // println!("Dx {:?}, Dy {:?}", dx, dy);
            if robot_search.is_within_bounds(robot_position.x as isize + dx as isize, robot_position.y as isize + dy as isize) {
                let tile = robot_search.known_world[(robot_position.x as i32 + dx) as usize][(robot_position.y as i32 + dy) as usize].clone();
                if tile.clone().unwrap().tile_type.properties().walk(){
                    let tile = tile.clone().unwrap();
                    let robots_state = State::new(robot_position, tile.tile_type, tile.content.index(), tile.content.get_value().0.unwrap_or_default());
                    let cost = robot_search.cost(&robots_state, direction.clone());
                    if cost < min_cost{
                        min_cost = cost;
                        min_cost_direction = Some(direction.clone());
                    }
                }
            }
        }
        if min_cost_direction.is_none(){
            return None
        }
        return Some((min_cost_direction.unwrap(), min_cost));
    }

    // Function instantiates robot's search and does A* algo search
    fn check_map_for_content(
        robot: &mut impl Runnable,
        world: &mut World,
        search_content: &Content,
    ) -> Result<Solution, LibErrorExtended> {
        let robot_world = robot_map(world);
        if robot_world.is_none() {
            //println!("Robot map is empty");
            return Err(LibErrorExtended::RobotMapEmpty);
        }
        let robot_world = robot_world.unwrap();

        //println!("KNOWN ROBOT WORLD: ");
        //Self::print_nicer_known_world_map(&robot_world);

        let robot_position = robot.get_coordinate();
        let robot_search = RobotSearch::new(
            &robot_world,
            robot_position.get_row(),
            robot_position.get_col(),
            search_content.index(),
            look_at_sky(world)
        );
        let solution = a_start(&robot_search);
        //println!("SOLUTION: {:?}", solution);
        solution
    }

    fn create_direction(i_index: usize, j_index: usize) -> Option<Direction> {
        if i_index == 0 && j_index == 1 {
            Some(Direction::Up)
        } else if i_index == 1 {
            if j_index == 0 {
                Some(Direction::Left)
            } else if j_index == 2 {
                Some(Direction::Right)
            } else {
                None
            }
        } else if i_index == 2 && j_index == 1 {
            Some(Direction::Down)
        } else {
            None
        }
    }

    fn check_content_destroyable(content: &Content) -> bool {
        let content_props = content.properties();
        return content_props.destroy();
    }

    #[allow(dead_code)]
    fn print_nicer_world_map(world_map: &Vec<Vec<Tile>>) {
        for row in world_map {
            for col in row {
                let tile = col.clone();
                print!("Tile Type {:?} ", tile.tile_type);
                println!("Tile content {:?}", tile.content);
            }
            println!();
        }
    }

    #[allow(dead_code)]
    fn print_nicer_known_world_map(known_world: &Vec<Vec<Option<Tile>>>) {
        for row in known_world {
            for col in row {
                if col.is_none() {
                    print!("None\t");
                    continue;
                }
                let tile = col.clone().unwrap();
                //print!("Type {:?},", tile.tile_type);
                print!("{:?} ", tile.content);
                print!("(cost {:?})\t", tile.tile_type.properties().cost());
            }
            println!();
        }
    }
}

impl Tools for CollectTool {}
