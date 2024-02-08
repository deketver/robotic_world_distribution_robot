use crate::a_star_search;

use crate::collection::a_star_search::{
    DirectionWalk, HeuristicProblem, Node, Path, Position, Solution, State,
};

use robotics_lib::interface::{destroy, go, robot_map, robot_view, Direction, Tools, look_at_sky};
use robotics_lib::runner::Runnable;
use robotics_lib::utils::{LibError, calculate_cost_go_with_environment};
use robotics_lib::world::tile::Content;
use robotics_lib::world::tile::Tile;
use robotics_lib::world::World;
use std::collections::{BinaryHeap, HashMap};
use robotics_lib::world::environmental_conditions::EnvironmentalConditions;

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
    /// In case of any questions or issues, please contact me on telegram: @deketver
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
        // if collect_instant_output.is_err() {
        //     return Err(collect_instant_output.unwrap_err());
        // }
        // let collect_instant_output = collect_instant_output.unwrap();
        let robot_energy_limit = match robot.get_energy().get_energy_level().checked_sub(energy_limit_cost){
            Some(result) => result,
            None => 0
        };

        // keep track of how many content we have collected
        // get value from instant collection
        let mut content_counter: usize = 0;
        content_counter += collect_instant_output;
        while robot.get_energy().get_energy_level() >= robot_energy_limit && content_counter < quantity {
            let search_result = Self::check_map_for_content(robot, world, search_content)?;

            //println!("Solution found {}, limit is {}", search_result.cost, energy_limit_cost);
            let actions = search_result.actions;
            if actions.len() < 1 {
                // content is at the same tile as the Robot is standing - so it needs to move to any possible direction
                let moving_option = Self::get_cheapest_walkable_around(robot, world, search_content);
                if moving_option.is_none(){
                    return Err(LibErrorExtended::NoWalkableTile);
                }
                let moving_option = moving_option.unwrap();
                let robots_energy_after_move = match robot.get_energy().get_energy_level().checked_sub(moving_option.1){
                    Some(result) => result,
                    None => return Err(LibErrorExtended::EnergyOutOfLimit)
                };
                if robots_energy_after_move < robot_energy_limit{
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
                let energy_after_actions = match robot.get_energy().get_energy_level().checked_sub(search_result.cost){
                    Some(result) => result,
                    None => return Err(LibErrorExtended::EnergyOutOfLimit)
                };
                if energy_after_actions < robot_energy_limit{
                    return Err(LibErrorExtended::EnergyOutOfLimit)
                }
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
            let collect_instant_output = Self::collect_instantly_reachable(robot, world, search_content)?;
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
    /// In case of any questions or issues, please contact me on telegram: @deketver
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
        // count the amount of content we have collected
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

    // Function checks tiles around robot and returns direction to the cheapest tile
    // to get move to, so it can collect the content
    fn get_cheapest_walkable_around(
        robot: &mut impl Runnable,
        world: &mut World,
        search_content: &Content
    )->Option<(DirectionWalk, usize)>{
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
            return Err(LibErrorExtended::RobotMapEmpty);
        }
        let robot_world = robot_world.unwrap();

        let robot_position = robot.get_coordinate();
        let robot_search = RobotSearch::new(
            &robot_world,
            robot_position.get_row(),
            robot_position.get_col(),
            search_content.index(),
            look_at_sky(world)
        );
        let solution = a_start(&robot_search);
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

struct RobotSearch<'a> {
    known_world: &'a Vec<Vec<Option<Tile>>>,
    robot_position: Position,
    goal_state_type: usize,
    environmental_conditions: EnvironmentalConditions,
}
impl<'a> RobotSearch<'a> {
    pub(crate) fn new(
        known_world: &'a Vec<Vec<Option<Tile>>>,
        robot_position_x: usize,
        robot_position_y: usize,
        goal_state_type: usize,
        environmental_conditions: EnvironmentalConditions,
    ) -> Self {
        RobotSearch {
            known_world,
            robot_position: Position::new(robot_position_x, robot_position_y),
            goal_state_type,
            environmental_conditions
        }
    }
    fn is_within_bounds(&self, x: isize, y: isize) -> bool {
        x >= 0
            && y >= 0
            && (x as usize) < self.known_world.len()
            && (y as usize) < self.known_world[x as usize].len()
    }

    fn match_position_action(&self, state: &State, action: &DirectionWalk) -> Option<Position> {
        let mut new_position = state.position;
        match action {
            DirectionWalk::Up { dx, dy } => {
                new_position.x = (state.position.x as i32 + dx) as usize;
                new_position.y = (state.position.y as i32 + dy) as usize;
            }
            DirectionWalk::Down { dx, dy } => {
                new_position.x = (state.position.x as i32 + dx) as usize;
                new_position.y = (state.position.y as i32 + dy) as usize;
            }
            DirectionWalk::Left { dx, dy } => {
                new_position.x = (state.position.x as i32 + dx) as usize;
                new_position.y = (state.position.y as i32 + dy) as usize;
            }
            DirectionWalk::Right { dx, dy } => {
                new_position.x = (state.position.x as i32 + dx) as usize;
                new_position.y = (state.position.y as i32 + dy) as usize;
            }
        }
        Some(new_position)
    }
}

impl<'a> HeuristicProblem for RobotSearch<'a> {
    fn initial_state(&self) -> State {
        let robots_tile = self.known_world[self.robot_position.x][self.robot_position.y]
            .clone()
            .unwrap();
        State::new(
            self.robot_position,
            robots_tile.tile_type,
            robots_tile.content.index(),
            robots_tile.content.get_value().0.unwrap_or_default(),
        )
    }

    fn is_goal(&self, state: &State) -> bool {
        state.tile_content == self.goal_state_type
    }

    fn actions(&self, state: &State) -> Vec<DirectionWalk> {
        let mut actions = Vec::new();
        if self.known_world[state.position.x][state.position.y].is_some() {
            // try to go Up and check what is there, is it walkable?
            if self.is_within_bounds(state.position.x as isize - 1, state.position.y as isize) {
                let tile = self.known_world[state.position.x - 1][state.position.y].clone();
                match tile {
                    Some(tile) => {
                        if tile.tile_type.properties().walk() {
                            actions.push(DirectionWalk::Up { dx: -1, dy: 0 });
                        }
                    }
                    None => {}
                }
            }
            // try to go Down and check what is there, is it walkable?
            if self.is_within_bounds(state.position.x as isize + 1, state.position.y as isize) {
                let tile = self.known_world[state.position.x + 1][state.position.y].clone();
                match tile {
                    Some(tile) => {
                        if tile.tile_type.properties().walk() {
                            actions.push(DirectionWalk::Down { dx: 1, dy: 0 });
                        }
                    }
                    None => {}
                }
            }
            // try to go Left and check what is there, is it walkable?
            if self.is_within_bounds(state.position.x as isize, state.position.y as isize - 1) {
                let tile = self.known_world[state.position.x][state.position.y - 1].clone();
                match tile {
                    Some(tile) => {
                        if tile.tile_type.properties().walk() {
                            actions.push(DirectionWalk::Left { dx: 0, dy: -1 });
                        }
                    }
                    None => {}
                }
            }
            // try to go Right and check what is there, is it walkable?
            if self.is_within_bounds(state.position.x as isize, state.position.y as isize + 1) {
                let tile = self.known_world[state.position.x][state.position.y + 1].clone();
                match tile {
                    Some(tile) => {
                        if tile.tile_type.properties().walk() {
                            actions.push(DirectionWalk::Right { dx: 0, dy: 1 });
                        }
                    }
                    None => {}
                }
            }
        }
        return actions;
    }

    fn result(&self, state: &State, action: DirectionWalk) -> State {
        let new_position = self.match_position_action(state, &action).unwrap();
        let tile = self.known_world[new_position.x][new_position.y]
            .clone()
            .unwrap();
        return State::new(new_position, tile.tile_type, tile.content.index(), tile.content.get_value().0.unwrap_or_default());
    }

    fn cost(&self, state: &State, action: DirectionWalk) -> usize {
        let current_tile = self.known_world[state.position.x][state.position.y].clone().unwrap();
        let new_position = self.match_position_action(state, &action).unwrap();
        let tile = self.known_world[new_position.x][new_position.y]
            .clone()
            .unwrap();
        let mut base_cost = tile.tile_type.properties().cost();
        let mut elevation_cost = 0;
        base_cost = calculate_cost_go_with_environment(base_cost,
                                                       self.environmental_conditions.clone(),
                                                       tile.tile_type);
        let new_elevation = tile.elevation;
        let current_elevation = current_tile.elevation;

        if new_elevation > current_elevation {
            elevation_cost = (new_elevation - current_elevation).pow(2);
        }

        return base_cost + elevation_cost;
    }

    fn estimate(&self, state: &State) -> usize {
        // here we ideally want to get Manhattan distance from Robots position
        // to the state's tile position
        let mut distance = 0;
        let robot_position = self.robot_position;
        let state_position = state.position;
        distance += (robot_position.x as i32 - state_position.x as i32).abs() as usize;
        distance += (robot_position.y as i32 - state_position.y as i32).abs() as usize;
        return distance;
    }
}

fn a_start<'a>(prob: &'a RobotSearch<'a>) -> Result<Solution, LibErrorExtended> //&'a LibError
{
    // initial state is given by robot's current coordinates
    let init = prob.initial_state();
    // alternative to priority queue, here the items with lowest f_cost have the highest priority
    let mut q = BinaryHeap::new();
    q.push(Node::new(init.clone(), 0, 0, None, None));

    let mut visited: HashMap<State, usize> = HashMap::new();

    // pop the node with the highest priority which is suppose tobe processed
    while let Some(node) = q.pop() {
        let state = node.state;
        let f_cost = node.f_cost;

        // if we have visited this state (tile) already, continue with another tile
        if visited.contains_key(&state) {
            continue;
        }
        visited.insert(state.clone(), f_cost);
        if prob.is_goal(&state) {
            // if we have found the goal state, end the search
            let path = Path::new(&node);
            return Ok(Solution::new(path.get_actions(), state.clone(), path.cost));
        }
        // for every action with is possible to do in given state (walk Up, Down, Right, Left
        for action in prob.actions(&state) {
            // get the tile to which given action leads to
            let next_state = prob.result(&state, action.clone());
            // get its cost
            let next_cost = node.cost + prob.cost(&state, action.clone());
            let next_estimate = prob.estimate(&next_state);
            let next_fcost = next_cost + next_estimate;
            q.push(Node::new(
                next_state,
                next_cost,
                next_fcost,
                Some(action),
                Some(Box::new(node.clone())),
            ));
        }
    }
    return Err(LibErrorExtended::NoSolution);
}
