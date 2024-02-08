use robotics_lib::world::tile::TileType;
use std::cmp::Ordering;

#[derive(Debug, Clone, Copy, Hash)]
pub(crate) struct Position {
    pub(crate) x: usize,
    pub(crate) y: usize,
}

impl Eq for Position {}
impl PartialEq for Position {
    fn eq(&self, other: &Self) -> bool {
        self.x == other.x && self.y == other.y
    }
}

impl Position {
    pub(crate) fn new(x: usize, y: usize) -> Self {
        Position { x, y }
    }
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum DirectionWalk {
    Up { dx: i32, dy: i32 },
    Down { dx: i32, dy: i32 },
    Left { dx: i32, dy: i32 },
    Right { dx: i32, dy: i32 },
}

#[derive(Debug, Clone, Copy, Hash)]
pub(crate) struct State {
    pub(crate) position: Position,
    pub(crate) tile_type: TileType,
    pub(crate) tile_content: usize, // have to do index of the content because it doesn't implement Copy
    pub(crate) tile_content_amount: usize
}

impl Eq for State {}

impl PartialEq for State {
    fn eq(&self, other: &Self) -> bool {
        self.position == other.position
            && self.tile_type == other.tile_type
            && self.tile_content == other.tile_content
    }
}

impl State {
    pub(crate) fn new(position: Position, tile_type: TileType, tile_content: usize, tile_content_amount: usize) -> Self {
        State {
            position,
            tile_type,
            tile_content,
            tile_content_amount
        }
    }
}
#[derive(Debug, Clone)]
pub(crate) struct Node {
    pub(crate) state: State,
    pub(crate) cost: usize,
    pub(crate) f_cost: usize,
    pub(crate) action: Option<DirectionWalk>,
    pub(crate) parent: Option<Box<Node>>,
}

impl Eq for Node {}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.f_cost == other.f_cost
    }
}
impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        // Compare f_cost first
        match other.f_cost.cmp(&self.f_cost) {
            Ordering::Equal => {
                // If f_cost is equal, use tile content amount as a tiebreaker
                self.state.tile_content_amount.cmp(&other.state.tile_content_amount)
            }
            // If f_cost is not equal, return the comparison result
            ordering => ordering,
        }
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Node {
    pub(crate) fn new(
        state: State,
        cost: usize,
        f_cost: usize,
        action: Option<DirectionWalk>,
        parent: Option<Box<Node>>,
    ) -> Node {
        Node {
            state,
            cost,
            f_cost,
            action,
            parent,
        }
    }
}

#[derive(Debug, Clone)]
pub(crate) struct Path {
    pub(crate) cost: usize,
    pub(crate) path: Vec<Node>,
}

impl Path {
    pub(crate) fn new(node: &Node) -> Self {
        // iterating via node.parent as a linked list
        let mut vec = Vec::new();
        let mut iterator = node.clone();
        vec.push(iterator.clone());
        while let Some(parent) = iterator.parent {
            iterator = *parent;
            vec.push(iterator.clone());
        }
        Path {
            cost: node.cost,
            path: vec,
        }
    }

    #[allow(dead_code)]
    pub(crate) fn get_path(&self) -> Vec<Node> {
        self.path.clone()
    }

    #[allow(dead_code)]
    pub(crate) fn print_path(&self) {
        for node in self.path.iter() {
            println!("{:?}", node.action);
        }
    }

    pub(crate) fn get_actions(&self) -> Vec<Option<DirectionWalk>> {
        let mut actions = self
            .path
            .iter()
            .map(|node| node.action.clone())
            .collect::<Vec<_>>();
        //need to do reverse, None action is for initial Node
        let _ = actions.pop();
        actions.reverse();
        actions
    }
}

pub(crate) trait HeuristicProblem {
    fn initial_state(&self) -> State;
    fn is_goal(&self, state: &State) -> bool;
    fn actions(&self, state: &State) -> Vec<DirectionWalk>;
    fn result(&self, state: &State, action: DirectionWalk) -> State;
    fn cost(&self, state: &State, action: DirectionWalk) -> usize;
    fn estimate(&self, state: &State) -> usize;
}

#[allow(dead_code)]
#[derive(Debug, Clone)]
pub(crate) struct Solution {
    pub(crate) actions: Vec<Option<DirectionWalk>>,
    pub(crate) state: State,
    pub(crate) cost: usize,
}

impl Solution {
    pub(crate) fn new(actions: Vec<Option<DirectionWalk>>, state: State, cost: usize) -> Self {
        Solution {
            actions,
            state,
            cost,
        }
    }

    #[allow(dead_code)]
    pub(crate) fn get_action(&self) -> Option<DirectionWalk> {
        self.actions[0]
    }
}
