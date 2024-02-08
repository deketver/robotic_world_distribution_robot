use std::fmt;
use std::cmp::Ordering;

#[derive(Debug, Clone)]
pub struct Position {
    row: usize,
    col: usize,
}

impl Position {
    /// Creates a new instance of `Coordinate`, called only inside of the common crate
    pub(crate) fn new(row: usize, col: usize) -> Self {
        Position { row, col }
    }

    /// Returns the row of the coordinate
    pub fn get_row(&self) -> usize {
        self.row
    }

    /// Returns the column of the coordinate
    pub fn get_col(&self) -> usize {
        self.col
    }
}

impl fmt::Display for Position {
    fn fmt(&self, f: &mut  fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({}, {})", self.row, self.col)
    }
}

impl Eq for Position {}

impl PartialEq for Position {
    fn eq(&self, other: &Self) -> bool {
        self.row == other.row && self.col == other.col
    }
}

impl Ord for Position {
    fn cmp(&self, other: &Self) -> Ordering {
        match self.row.cmp(&other.row) {
            Ordering::Equal => {
                self.col.cmp(&other.col)
            }
            ordering => ordering,
        }
    }
}

impl PartialOrd for Position {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[derive(Debug, Clone)]
pub struct StorageInfo{
    position: Position,
    content: usize,
    quantity: usize,
    coefficient: u32,
    market_index: usize,
}

impl StorageInfo{
    pub fn new(position: Position, content: usize, quantity: usize) -> Self {
        let coefficient = match content {
            // coefficients based on the logic from robotic_lib
            0 => 1,
            1 => 2,
            10 => 5,
            _ => 0
        };
        StorageInfo{
            position,
            content,
            quantity,
            coefficient,
            market_index: 0}
    }

    pub fn get_position(&self) -> Position {
        self.position.clone()
    }

    pub fn get_content(&self) -> usize {
        self.content
    }

    pub fn get_quantity(&self) -> usize {
        self.quantity
    }

    pub fn get_coefficient(&self) -> u32 {
        self.coefficient
    }

    pub fn get_market_index_mut(&mut self) -> &mut usize {
        &mut self.market_index
    }
    pub fn get_market_index(&self) -> usize {
        self.market_index
    }
    pub fn set_market_index(&mut self, market_index: usize) {
        self.market_index = market_index;
    }
}

impl Eq for StorageInfo {}

impl PartialEq for StorageInfo {
    fn eq(&self, other: &Self) -> bool {
        self.position == other.position
    }
}

impl Ord for StorageInfo {
    fn cmp(&self, other: &Self) -> Ordering {
        self.position.cmp(&other.position)
    }
}

impl PartialOrd for StorageInfo {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}