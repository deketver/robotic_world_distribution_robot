use crate::robot_veronika::content_pick::a_star_search::{
    DirectionWalk, HeuristicProblem, Node, Path, Position, Solution, State,
};
use robotics_lib::utils::{calculate_cost_go_with_environment};
use robotics_lib::world::tile::{Tile};
use robotics_lib::world::environmental_conditions::EnvironmentalConditions;
use crate::robot_veronika::content_pick::collect::LibErrorExtended;
use std::collections::{BinaryHeap, HashMap};

pub(crate) struct RobotSearch<'a> {
    pub(crate) known_world: &'a Vec<Vec<Option<Tile>>>,
    pub(crate) robot_position: Position,
    pub(crate) goal_state_type: usize,
    pub(crate) environmental_conditions: EnvironmentalConditions,
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
    pub(crate) fn is_within_bounds(&self, x: isize, y: isize) -> bool {
        x >= 0
            && y >= 0
            && (x as usize) < self.known_world.len()
            && (y as usize) < self.known_world[x as usize].len()
    }

    pub(crate) fn match_position_action(&self, state: &State, action: &DirectionWalk) -> Option<Position> {
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

pub(crate) fn a_start<'a>(prob: &'a RobotSearch<'a>) -> Result<Solution, LibErrorExtended> //&'a LibError
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
            //println!("Path do goal state node is:");
            //println!("Last node cost: {:?}", node.cost);
            //path.print_path();
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
