use crate::types::{Meters, ValidationError, WorldPosition, ensure_finite, ensure_positive};
use std::fmt;

const MAX_GRID_CELLS: usize = 16_777_216;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct GridCell {
    column: u32,
    row: u32,
}

impl GridCell {
    pub const fn new(column: u32, row: u32) -> Self {
        Self { column, row }
    }

    pub const fn column(self) -> u32 {
        self.column
    }

    pub const fn row(self) -> u32 {
        self.row
    }
}

#[derive(Clone, Debug)]
pub struct GridMap {
    width: u32,
    height: u32,
    origin: WorldPosition,
    resolution: Meters,
    blocked: Vec<bool>,
}

impl GridMap {
    pub fn new(
        width: u32,
        height: u32,
        origin: WorldPosition,
        resolution_meters: f64,
        blocked_cells: impl IntoIterator<Item = GridCell>,
    ) -> Result<Self, MapError> {
        if width == 0 || height == 0 {
            return Err(MapError::Empty);
        }
        ensure_positive(resolution_meters, "map.resolution")?;
        let cell_count = (width as usize)
            .checked_mul(height as usize)
            .ok_or(MapError::TooLarge)?;
        if cell_count > MAX_GRID_CELLS {
            return Err(MapError::TooLarge);
        }
        let mut blocked = vec![false; cell_count];
        for cell in blocked_cells {
            if cell.column >= width || cell.row >= height {
                return Err(MapError::CellOutOfBounds(cell));
            }
            blocked[(cell.row as usize * width as usize) + cell.column as usize] = true;
        }
        let max_x = origin.x_meters() + f64::from(width) * resolution_meters;
        let max_y = origin.y_meters() + f64::from(height) * resolution_meters;
        ensure_finite(max_x, "map.max_x")?;
        ensure_finite(max_y, "map.max_y")?;
        Ok(Self {
            width,
            height,
            origin,
            resolution: Meters::new(resolution_meters)?,
            blocked,
        })
    }

    pub const fn width(&self) -> u32 {
        self.width
    }

    pub const fn height(&self) -> u32 {
        self.height
    }

    pub const fn origin(&self) -> WorldPosition {
        self.origin
    }

    pub const fn resolution_meters(&self) -> f64 {
        self.resolution.get()
    }

    pub fn grid_to_world(&self, cell: GridCell) -> Result<WorldPosition, MapError> {
        if cell.column >= self.width || cell.row >= self.height {
            return Err(MapError::CellOutOfBounds(cell));
        }
        WorldPosition::new(
            self.origin.x_meters() + (f64::from(cell.column) + 0.5) * self.resolution.get(),
            self.origin.y_meters() + (f64::from(cell.row) + 0.5) * self.resolution.get(),
        )
        .map_err(MapError::from)
    }

    pub fn world_to_grid(&self, position: WorldPosition) -> Result<GridCell, MapError> {
        let relative_x = position.x_meters() - self.origin.x_meters();
        let relative_y = position.y_meters() - self.origin.y_meters();
        let max_x = f64::from(self.width) * self.resolution.get();
        let max_y = f64::from(self.height) * self.resolution.get();
        if relative_x < 0.0 || relative_y < 0.0 || relative_x >= max_x || relative_y >= max_y {
            return Err(MapError::WorldOutOfBounds);
        }
        Ok(GridCell::new(
            (relative_x / self.resolution.get()).floor() as u32,
            (relative_y / self.resolution.get()).floor() as u32,
        ))
    }

    pub fn is_blocked(&self, cell: GridCell) -> Result<bool, MapError> {
        if cell.column >= self.width || cell.row >= self.height {
            return Err(MapError::CellOutOfBounds(cell));
        }
        Ok(self.blocked[(cell.row as usize * self.width as usize) + cell.column as usize])
    }

    /// Conservative broad phase; exact clearance is still checked by the safety kernel.
    pub(crate) fn blocked_cells_in_bounds(
        &self,
        bounds: (f64, f64, f64, f64),
    ) -> impl Iterator<Item = GridCell> + '_ {
        let index = |value: f64, origin: f64, size: u32| {
            ((value - origin) / self.resolution.get())
                .floor()
                .clamp(0.0, f64::from(size - 1)) as u32
        };
        // Include a guard cell on each side for exact grid boundaries and rounding.
        let min_column = index(bounds.0, self.origin.x_meters(), self.width).saturating_sub(1);
        let min_row = index(bounds.1, self.origin.y_meters(), self.height).saturating_sub(1);
        let max_column =
            (index(bounds.2, self.origin.x_meters(), self.width) + 1).min(self.width - 1);
        let max_row =
            (index(bounds.3, self.origin.y_meters(), self.height) + 1).min(self.height - 1);
        (min_row..=max_row).flat_map(move |row| {
            (min_column..=max_column).filter_map(move |column| {
                self.blocked[row as usize * self.width as usize + column as usize]
                    .then_some(GridCell::new(column, row))
            })
        })
    }

    pub(crate) fn cell_bounds(&self, cell: GridCell) -> (f64, f64, f64, f64) {
        let min_x = self.origin.x_meters() + f64::from(cell.column) * self.resolution.get();
        let min_y = self.origin.y_meters() + f64::from(cell.row) * self.resolution.get();
        (
            min_x,
            min_y,
            min_x + self.resolution.get(),
            min_y + self.resolution.get(),
        )
    }

    pub(crate) fn world_bounds(&self) -> (f64, f64, f64, f64) {
        (
            self.origin.x_meters(),
            self.origin.y_meters(),
            self.origin.x_meters() + f64::from(self.width) * self.resolution.get(),
            self.origin.y_meters() + f64::from(self.height) * self.resolution.get(),
        )
    }

    pub(crate) fn blocked_bits(&self) -> &[bool] {
        &self.blocked
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MapError {
    Empty,
    TooLarge,
    CellOutOfBounds(GridCell),
    WorldOutOfBounds,
    Invalid(ValidationError),
}

impl From<ValidationError> for MapError {
    fn from(value: ValidationError) -> Self {
        Self::Invalid(value)
    }
}

impl fmt::Display for MapError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Empty => formatter.write_str("map dimensions must be non-zero"),
            Self::TooLarge => formatter.write_str("map dimensions overflow addressable memory"),
            Self::CellOutOfBounds(cell) => write!(
                formatter,
                "grid cell ({}, {}) is outside the map",
                cell.column, cell.row
            ),
            Self::WorldOutOfBounds => formatter.write_str("world position is outside the map"),
            Self::Invalid(error) => error.fmt(formatter),
        }
    }
}

impl std::error::Error for MapError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn grid_world_conversion_uses_cell_centers_and_positive_axes() {
        let map = GridMap::new(3, 2, WorldPosition::new(-1.0, 2.0).unwrap(), 0.5, []).unwrap();
        let world = map.grid_to_world(GridCell::new(2, 1)).unwrap();
        assert_eq!(world, WorldPosition::new(0.25, 2.75).unwrap());
        assert_eq!(map.world_to_grid(world).unwrap(), GridCell::new(2, 1));
    }

    #[test]
    fn upper_world_boundary_is_outside() {
        let map = GridMap::new(1, 1, WorldPosition::new(0.0, 0.0).unwrap(), 1.0, []).unwrap();
        assert_eq!(
            map.world_to_grid(WorldPosition::new(1.0, 0.5).unwrap()),
            Err(MapError::WorldOutOfBounds)
        );
    }
}
