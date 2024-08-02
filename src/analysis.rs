use imu_fusion::FusionVector;
use core::f32::consts::PI;
use micromath::F32Ext;
use alloc::{collections::VecDeque, vec::Vec};

type Denoiser = QuantileDenoiser;
// type Denoiser = AverageDenoiser;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MovementDirection {
    Horizontal,
    Vertical,
    Diagonal,
}

impl MovementDirection {
    pub fn as_payload(&self) -> u8 {
        match *self {
            MovementDirection::Vertical => 0,
            MovementDirection::Horizontal => 1,
            MovementDirection::Diagonal => 2,
        }
    }
}

struct Smoothing {
    measurements: VecDeque<FusionVector>,
    smoothing_window_size: usize,
}

impl Smoothing {
    // Adds a measurement and returns smoothed value
    fn add_measurement(&mut self, linear_acceleration: FusionVector) -> FusionVector {
        if self.measurements.len() >= self.smoothing_window_size {
            self.measurements.pop_front();
        }
        self.measurements.push_back(linear_acceleration);

        let sv = self.compute_smoothing_vector();

        linear_acceleration - sv
    }

    fn compute_smoothing_vector(&self) -> FusionVector {
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_z = 0.0;
        for m in self.measurements.iter() {
            sum_x += m.x;
            sum_y += m.y;
            sum_z += m.z;
        }

        let num_elems =  self.measurements.len() as f32;
        FusionVector::new(sum_x / num_elems, sum_y / num_elems, sum_z / num_elems)
    }
}

struct MovementDetection {
    movement_computation: Denoiser,
    acceleration_threshold: f32,
    angle_low_threshold: f32,
    angle_high_threshold: f32,
    prev_direction: Option<MovementDirection>,
}

impl MovementDetection {
    fn add_measurement(&mut self, x: f32, y: f32) -> Option<MovementDirection> {
        let (x, y) = self.movement_computation.add_measurement(x, y);
        self.next_direction(x, y)
    }

    fn next_direction(&mut self, x_accel: f32, y_accel: f32) -> Option<MovementDirection> {
        let mut next_state = self.prev_direction;
        let below_thres = self.below_acceleration_threshold(x_accel, y_accel);

        if below_thres {
            next_state = None;
        } else {
            let angle = y_accel.atan2(x_accel);
            next_state =
                if self.angle_low_threshold < angle && angle < self.angle_high_threshold {
                    Some(MovementDirection::Diagonal)
                } else if angle < self.angle_low_threshold {
                    Some(MovementDirection::Horizontal)
                } else {
                    Some(MovementDirection::Vertical)
                };
        }
        self.prev_direction = next_state;
        next_state
    }

    fn below_acceleration_threshold(&self, x_accel: f32, y_accel: f32) -> bool {
        x_accel < self.acceleration_threshold && y_accel < self.acceleration_threshold
    }
}

struct AverageDenoiser {
    horizontal_measurements: VecDeque<f32>,
    vertical_measurements: VecDeque<f32>,
    detection_window_size: usize,
}

impl AverageDenoiser {
    fn new(detection_window_size: usize) -> Self {
        Self {
            detection_window_size,
            horizontal_measurements: VecDeque::with_capacity(detection_window_size),
            vertical_measurements: VecDeque::with_capacity(detection_window_size),
        }
    }
    fn add_measurement(&mut self, x: f32, y: f32) -> (f32, f32) {
        if self.horizontal_measurements.len() >= self.detection_window_size {
            self.horizontal_measurements.pop_front();
            self.vertical_measurements.pop_front();
        }

        self.horizontal_measurements.push_back(x);
        self.vertical_measurements.push_back(y);

        self.compute_average_detection_accel()
    }

    fn compute_average_detection_accel(&self) -> (f32, f32) {
        let mean_horizontal = self.horizontal_measurements.iter().sum::<f32>()
            / self.horizontal_measurements.len() as f32;
        let mean_vertical = self.vertical_measurements.iter().sum::<f32>()
            / self.horizontal_measurements.len() as f32;

        (mean_horizontal, mean_vertical)
    }
}

const QUANTILE: f32 = 0.75;

struct QuantileDenoiser {
    horizontal_measurements: VecDeque<f32>,
    vertical_measurements: VecDeque<f32>,
    horizontal_measurements_buffer: Vec<f32>,
    vertical_measurements_buffer: Vec<f32>,
    detection_window_size: usize,
}

impl QuantileDenoiser {
    fn new(detection_window_size: usize) -> Self {
        Self {
            detection_window_size,
            horizontal_measurements: VecDeque::with_capacity(detection_window_size),
            vertical_measurements: VecDeque::with_capacity(detection_window_size),
            horizontal_measurements_buffer: Vec::with_capacity(detection_window_size),
            vertical_measurements_buffer: Vec::with_capacity(detection_window_size),
        }
    }

    fn add_measurement(&mut self, x: f32, y: f32) -> (f32, f32) {
        if self.horizontal_measurements.len() >= self.detection_window_size {
            self.horizontal_measurements.pop_front();
            self.vertical_measurements.pop_front();
        }

        self.horizontal_measurements.push_back(x);
        self.vertical_measurements.push_back(y);

        self.compute_quantile_detection_accel()
    }

    fn compute_quantile_detection_accel(&mut self) -> (f32, f32) {
        assert!(!self.horizontal_measurements.is_empty());
        assert!(self.vertical_measurements.len() == self.horizontal_measurements.len());

        self.horizontal_measurements_buffer.clear(); // remove all elements
        self.horizontal_measurements_buffer
            .extend(self.horizontal_measurements.iter()); // add all elements of actual measurements
        self.horizontal_measurements_buffer
            .sort_by(|a, b| a.partial_cmp(b).unwrap());
        self.vertical_measurements_buffer.clear();
        self.vertical_measurements_buffer
            .extend(self.vertical_measurements.iter());
        self.vertical_measurements_buffer
            .sort_by(|a, b| a.partial_cmp(b).unwrap());
        let pos = (self.horizontal_measurements_buffer.len() as f32 * QUANTILE) as usize;

        (
            self.horizontal_measurements_buffer[pos],
            self.vertical_measurements_buffer[pos],
        )
    }
}

pub struct Analysis {
    smoothing: Smoothing,
    movement_detection: MovementDetection,
}

impl Default for Analysis {
    fn default() -> Self {
        let diagonal_low = 0.6 * PI / 4.0;
        let diagonal_high = 1.2 * PI / 4.0;
        Analysis::new(100, 30, 1.5, diagonal_low, diagonal_high)
    }
}

impl Analysis {
    pub fn new(
        smoothing_window_size: usize,
        detection_window_size: usize,
        acceleration_threshold: f32,
        angle_low_threshold: f32,
        angle_high_threshold: f32,
    ) -> Analysis {
        assert!(smoothing_window_size > 0);
        assert!(detection_window_size > 0);
        assert!(detection_window_size < smoothing_window_size);

        Analysis {
            smoothing: Smoothing {
                measurements: VecDeque::with_capacity(smoothing_window_size),
                smoothing_window_size,
            },
            movement_detection: MovementDetection {
                movement_computation: Denoiser::new(detection_window_size),
                acceleration_threshold,
                angle_low_threshold,
                angle_high_threshold,
                prev_direction: None,
            },
        }
    }

    pub fn add_measurement(
        &mut self,
        linear_acceleration: FusionVector,
    ) -> Option<MovementDirection> {
        let smoothed = self.smoothing.add_measurement(linear_acceleration);
        let x = (smoothed.x * smoothed.x + smoothed.y * smoothed.y).sqrt(); // compute euclidean norm of x and y component
        let y = smoothed.z.abs();
        assert!(!x.is_nan());
        assert!(!y.is_nan());
        self.movement_detection.add_measurement(x, y)
    }
}

#[test]
fn test_simple_quantile_movement_computation() {
    let mut movement_detection = QuantileDenoiser::new(30);

    for i in 0..100 {
        let movement = movement_detection.add_measurement(0.0, 0.0);
        assert_eq!(movement, (0.0, 0.0));
    }
}


#[test]
fn test_simple_quantile_movement_computation() {
    let mut movement_detection = QuantileDenoiser::new(30);

    for i in 0..100 {
        let movement = movement_detection.add_measurement(0.0, 0.0);
        assert_eq!(movement, (0.0, 0.0));
    }
}

fn no_invalid_float(f: f32) -> f32 {
    if f.is_normal() {
        f
    } else {
        0.0
    }
}