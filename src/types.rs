use std::fmt;

pub const CONTROL_TICK_MS: i64 = 100;
pub const CONTROL_TICK_SECONDS: f64 = 0.1;
pub const SAFETY_EPSILON_METERS: f64 = 1.0e-9;

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct Meters(f64);

impl Meters {
    pub fn new(value: f64) -> Result<Self, ValidationError> {
        ensure_finite(value, "meters")?;
        Ok(Self(value))
    }

    pub const fn get(self) -> f64 {
        self.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct MetersPerSecond(f64);

impl MetersPerSecond {
    pub fn new(value: f64) -> Result<Self, ValidationError> {
        ensure_finite(value, "meters_per_second")?;
        Ok(Self(value))
    }

    pub const fn get(self) -> f64 {
        self.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct MetersPerSecondSquared(f64);

impl MetersPerSecondSquared {
    pub fn new(value: f64) -> Result<Self, ValidationError> {
        ensure_finite(value, "meters_per_second_squared")?;
        Ok(Self(value))
    }

    pub const fn get(self) -> f64 {
        self.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct MetersPerSecondCubed(f64);

impl MetersPerSecondCubed {
    pub fn new(value: f64) -> Result<Self, ValidationError> {
        ensure_finite(value, "meters_per_second_cubed")?;
        Ok(Self(value))
    }

    pub const fn get(self) -> f64 {
        self.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct Radians(f64);

impl Radians {
    pub fn new(value: f64) -> Result<Self, ValidationError> {
        ensure_finite(value, "radians")?;
        Ok(Self(value))
    }

    pub const fn get(self) -> f64 {
        self.0
    }
}

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct RobotId(String);

impl RobotId {
    pub fn new(value: impl Into<String>) -> Result<Self, ValidationError> {
        let value = value.into();
        if value.is_empty() || value.len() > 128 || value.chars().any(char::is_control) {
            return Err(ValidationError::InvalidId("robot"));
        }
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct ScenarioId(String);

impl ScenarioId {
    pub fn new(value: impl Into<String>) -> Result<Self, ValidationError> {
        let value = value.into();
        if value.is_empty() || value.len() > 128 || value.chars().any(char::is_control) {
            return Err(ValidationError::InvalidId("scenario"));
        }
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct SimulationTimeMs(i64);

impl SimulationTimeMs {
    pub const ZERO: Self = Self(0);

    pub fn new(value: i64) -> Result<Self, ValidationError> {
        if value < 0 {
            return Err(ValidationError::NegativeTime);
        }
        Ok(Self(value))
    }

    pub const fn get(self) -> i64 {
        self.0
    }

    pub fn checked_add_tick(self) -> Result<Self, ValidationError> {
        self.0
            .checked_add(CONTROL_TICK_MS)
            .map(Self)
            .ok_or(ValidationError::TimeOverflow)
    }
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct ControlTick(u64);

impl ControlTick {
    pub const ZERO: Self = Self(0);

    pub const fn get(self) -> u64 {
        self.0
    }

    pub fn checked_next(self) -> Result<Self, ValidationError> {
        self.0
            .checked_add(1)
            .map(Self)
            .ok_or(ValidationError::TickOverflow)
    }
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct MonotonicInstantNs(u64);

impl MonotonicInstantNs {
    pub const fn new(value: u64) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u64 {
        self.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WorldPosition {
    x: Meters,
    y: Meters,
}

impl WorldPosition {
    pub fn new(x_meters: f64, y_meters: f64) -> Result<Self, ValidationError> {
        Ok(Self {
            x: Meters::new(x_meters)?,
            y: Meters::new(y_meters)?,
        })
    }

    pub const fn from_meters(x: Meters, y: Meters) -> Self {
        Self { x, y }
    }

    pub const fn x_meters(self) -> f64 {
        self.x.get()
    }

    pub const fn y_meters(self) -> f64 {
        self.y.get()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Velocity {
    x: MetersPerSecond,
    y: MetersPerSecond,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Acceleration {
    x: MetersPerSecondSquared,
    y: MetersPerSecondSquared,
}

impl Acceleration {
    pub const ZERO: Self = Self {
        x: MetersPerSecondSquared(0.0),
        y: MetersPerSecondSquared(0.0),
    };

    pub fn new(x_mps2: f64, y_mps2: f64) -> Result<Self, ValidationError> {
        Ok(Self {
            x: MetersPerSecondSquared::new(x_mps2)?,
            y: MetersPerSecondSquared::new(y_mps2)?,
        })
    }

    pub const fn x_mps2(self) -> f64 {
        self.x.get()
    }

    pub const fn y_mps2(self) -> f64 {
        self.y.get()
    }

    pub fn magnitude(self) -> f64 {
        self.x.get().hypot(self.y.get())
    }
}

impl Velocity {
    pub const ZERO: Self = Self {
        x: MetersPerSecond(0.0),
        y: MetersPerSecond(0.0),
    };

    pub fn new(x_mps: f64, y_mps: f64) -> Result<Self, ValidationError> {
        Ok(Self {
            x: MetersPerSecond::new(x_mps)?,
            y: MetersPerSecond::new(y_mps)?,
        })
    }

    pub const fn x_mps(self) -> f64 {
        self.x.get()
    }

    pub const fn y_mps(self) -> f64 {
        self.y.get()
    }

    pub fn magnitude(self) -> f64 {
        self.x.get().hypot(self.y.get())
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RobotState {
    position: WorldPosition,
    velocity: Velocity,
    acceleration: Acceleration,
    yaw: Radians,
}

impl RobotState {
    pub fn new(
        position: WorldPosition,
        velocity: Velocity,
        acceleration: Acceleration,
        yaw_radians: f64,
    ) -> Result<Self, ValidationError> {
        Ok(Self {
            position,
            velocity,
            acceleration,
            yaw: Radians::new(yaw_radians)?,
        })
    }

    pub const fn position(self) -> WorldPosition {
        self.position
    }

    pub const fn velocity(self) -> Velocity {
        self.velocity
    }

    pub const fn acceleration(self) -> Acceleration {
        self.acceleration
    }

    pub const fn yaw_radians(self) -> f64 {
        self.yaw.get()
    }

    pub(crate) const fn transitioned(
        self,
        position: WorldPosition,
        velocity: Velocity,
        acceleration: Acceleration,
    ) -> Self {
        Self {
            position,
            velocity,
            acceleration,
            yaw: self.yaw,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ValidationError {
    InvalidId(&'static str),
    DuplicateId(&'static str),
    NonFinite(&'static str),
    NonPositive(&'static str),
    OutOfRange(&'static str),
    NegativeTime,
    TimeOverflow,
    TickOverflow,
}

impl fmt::Display for ValidationError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidId(kind) => write!(formatter, "invalid {kind} id"),
            Self::DuplicateId(kind) => write!(formatter, "duplicate {kind} id"),
            Self::NonFinite(field) => write!(formatter, "{field} must be finite"),
            Self::NonPositive(field) => write!(formatter, "{field} must be positive"),
            Self::OutOfRange(field) => write!(formatter, "{field} is out of range"),
            Self::NegativeTime => formatter.write_str("simulation time cannot be negative"),
            Self::TimeOverflow => formatter.write_str("simulation time overflow"),
            Self::TickOverflow => formatter.write_str("control tick overflow"),
        }
    }
}

impl std::error::Error for ValidationError {}

pub(crate) fn ensure_finite(value: f64, field: &'static str) -> Result<(), ValidationError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(ValidationError::NonFinite(field))
    }
}

pub(crate) fn ensure_positive(value: f64, field: &'static str) -> Result<(), ValidationError> {
    ensure_finite(value, field)?;
    if value > 0.0 {
        Ok(())
    } else {
        Err(ValidationError::NonPositive(field))
    }
}

pub(crate) fn ensure_non_negative(value: f64, field: &'static str) -> Result<(), ValidationError> {
    ensure_finite(value, field)?;
    if value >= 0.0 {
        Ok(())
    } else {
        Err(ValidationError::OutOfRange(field))
    }
}
