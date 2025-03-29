use ackermann_msgs::msg::AckermannDriveStamped;
use anyhow::{Error, Result};
use rclrs::{self, Context};
use sensor_msgs::msg::LaserScan;
use std::{
    env,
    f32::consts::PI,
    sync::{Arc, Mutex},
};

// Define driving modes as enum to improve code readability
#[derive(Debug, Clone, Copy)]
enum DrivingMode {
    Test,    // Test environment (faster speeds)
    Normal,  // Normal driving
    Careful, // Careful driving
}

// Define steering angle ranges as enum
#[derive(Debug, Clone, Copy)]
enum SteeringCategory {
    Straight,    // Almost straight (0-5 degrees)
    SlightCurve, // Slight curve (5-10 degrees)
    GentleCurve, // Gentle curve (10-15 degrees)
    MediumCurve, // Medium curve (15+ degrees)
}

impl SteeringCategory {
    // Return category based on steering angle (in degrees)
    fn from_degree(degree: f32) -> Self {
        if degree <= 5.0 {
            SteeringCategory::Straight
        } else if degree <= 10.0 {
            SteeringCategory::SlightCurve
        } else if degree <= 15.0 {
            SteeringCategory::GentleCurve
        } else {
            SteeringCategory::MediumCurve
        }
    }

    // Return speed based on driving mode
    fn speed_for_mode(&self, mode: DrivingMode) -> f32 {
        match (self, mode) {
            // Test mode (faster speeds)
            (SteeringCategory::Straight, DrivingMode::Test) => 2.0,
            (SteeringCategory::SlightCurve, DrivingMode::Test) => 1.5,
            (SteeringCategory::GentleCurve, DrivingMode::Test) => 1.2,
            (SteeringCategory::MediumCurve, DrivingMode::Test) => 0.8,

            // Normal mode
            (SteeringCategory::Straight, DrivingMode::Normal) => 1.2,
            (SteeringCategory::SlightCurve, DrivingMode::Normal) => 1.0,
            (SteeringCategory::GentleCurve, DrivingMode::Normal) => 0.8,
            (SteeringCategory::MediumCurve, DrivingMode::Normal) => 0.5,

            // Careful mode
            (SteeringCategory::Straight, DrivingMode::Careful) => 0.8,
            (SteeringCategory::SlightCurve, DrivingMode::Careful) => 0.6,
            (SteeringCategory::GentleCurve, DrivingMode::Careful) => 0.4,
            (SteeringCategory::MediumCurve, DrivingMode::Careful) => 0.2,
        }
    }
}

// Structure for threshold interpolation
struct ThresholdPoint {
    range: f32,
    threshold: f32,
}

// Structure for steering and speed calculation results
struct SteeringData {
    steering_angle: f32,
    speed: f32,
}

// Manage configuration parameters in one place
struct GapFollowConfig {
    // Vehicle settings
    car_width: f32,
    wheel_base: f32, // Wheelbase for pure pursuit algorithm

    // Scan area settings
    left_wing_angle: f32,  // Left scan angle (radians)
    right_wing_angle: f32, // Right scan angle (radians)

    // Algorithm settings
    max_bubble_points: usize,  // Maximum bubble size
    look_ahead_offset: f32,    // Look ahead offset
    window_size: usize,        // Smoothing window size
    driving_mode: DrivingMode, // Driving mode
}

impl Default for GapFollowConfig {
    fn default() -> Self {
        Self {
            car_width: 0.5,
            wheel_base: 0.32,
            left_wing_angle: PI / 2.7,
            right_wing_angle: -(PI / 2.7),
            max_bubble_points: 850,
            look_ahead_offset: 0.27,
            window_size: 9,
            driving_mode: DrivingMode::Test,
        }
    }
}

struct ReactiveFollowGap {
    // Field to keep ROS subscription alive
    _scan_subscription: Arc<rclrs::Subscription<LaserScan>>,

    // Configuration settings
    config: GapFollowConfig,
}

impl ReactiveFollowGap {
    pub fn new(
        node: &rclrs::Node,
        config: Option<GapFollowConfig>,
    ) -> Result<Self, rclrs::RclrsError> {
        // Use default settings or user-provided settings
        let config = config.unwrap_or_default();

        // Create shared state
        let data_size = Arc::new(Mutex::new(0usize));
        let left_wing_index = Arc::new(Mutex::new(0usize));
        let right_wing_index = Arc::new(Mutex::new(0usize));

        // Create publisher
        let drive_publisher =
            node.create_publisher::<AckermannDriveStamped>("/drive", rclrs::QOS_PROFILE_DEFAULT)?;

        // Clone config for use in closure
        let config_clone = config.clone();

        // Create scan subscription
        let scan_subscription = node.create_subscription::<LaserScan, _>(
            "/scan",
            rclrs::QOS_PROFILE_DEFAULT,
            move |scan_msg: LaserScan| {
                if scan_msg.ranges.is_empty() {
                    println!("Warning: Empty scan message received");
                    return;
                }

                // Update data size
                let range_count = scan_msg.ranges.len();
                {
                    let mut data_size_guard = data_size.lock().unwrap();
                    *data_size_guard = range_count;
                }

                // Update wing indices
                {
                    let left_wing_idx = ((config_clone.left_wing_angle - scan_msg.angle_min)
                        / scan_msg.angle_increment)
                        as usize;
                    let right_wing_idx = ((config_clone.right_wing_angle - scan_msg.angle_min)
                        / scan_msg.angle_increment)
                        as usize;

                    // Ensure indices are within bounds
                    let safe_left = std::cmp::min(left_wing_idx, range_count - 1);
                    let safe_right = std::cmp::min(right_wing_idx, range_count - 1);

                    *left_wing_index.lock().unwrap() = safe_left;
                    *right_wing_index.lock().unwrap() = safe_right;
                }

                // Process each LiDAR scan
                let processed_ranges =
                    ReactiveFollowGap::preprocess_lidar(&scan_msg.ranges, config_clone.window_size);

                // Find closest point to LiDAR
                let (min_range, min_index) = ReactiveFollowGap::find_min_range(&processed_ranges);

                // Calculate bubble size based on car width and distance
                let bubble_size =
                    ((config_clone.car_width / min_range) / scan_msg.angle_increment) as usize;
                let bubble_size = std::cmp::min(bubble_size, config_clone.max_bubble_points);

                // Find max length gap and best point
                let best_point_index = ReactiveFollowGap::find_max_gap(
                    &processed_ranges,
                    min_index,
                    bubble_size,
                    *right_wing_index.lock().unwrap(),
                    *left_wing_index.lock().unwrap(),
                );

                // Ensure best point index is within bounds
                let best_point_index = std::cmp::min(best_point_index, range_count - 1);

                // Calculate steering angle and speed
                let steering_data = ReactiveFollowGap::calculate_steering(
                    best_point_index,
                    &processed_ranges,
                    &scan_msg,
                    &config_clone,
                );

                // Create and publish drive message
                let mut drive_msg = AckermannDriveStamped::default();
                drive_msg.drive.steering_angle = steering_data.steering_angle;
                drive_msg.drive.speed = steering_data.speed;
                let _ = drive_publisher.publish(&drive_msg);
            },
        )?;

        Ok(Self {
            _scan_subscription: scan_subscription,
            config,
        })
    }

    // Find minimum range and index (closest obstacle)
    fn find_min_range(ranges: &[f32]) -> (f32, usize) {
        let mut min_range = f32::MAX;
        let mut min_index = 0usize;

        for (i, &range) in ranges.iter().enumerate() {
            if range < min_range {
                min_range = range;
                min_index = i;
            }
        }

        (min_range, min_index)
    }

    // Calculate steering using pure pursuit algorithm
    fn calculate_steering(
        best_point_index: usize,
        ranges: &[f32],
        scan_msg: &LaserScan,
        config: &GapFollowConfig,
    ) -> SteeringData {
        // Calculate angle of best point
        let angle = scan_msg.angle_min + (scan_msg.angle_increment * best_point_index as f32);
        let steering_degree = (angle * 180.0 / PI).abs();

        // Look ahead distance
        let lookahead = ranges[best_point_index];

        // Convert polar coordinates to cartesian
        let bestpoint_x = lookahead * f32::cos(angle);
        let bestpoint_y = lookahead * f32::sin(angle);

        // Adjust coordinates relative to rear axle
        let lookahead_angle = f32::atan2(bestpoint_y, bestpoint_x + config.look_ahead_offset);
        let mut lookahead_rear =
            f32::sqrt((bestpoint_x + config.look_ahead_offset).powi(2) + bestpoint_y.powi(2));

        // Adjust look ahead distance based on angle
        let lookahead_degree = (lookahead_angle * 180.0 / PI).abs();

        lookahead_rear *= if lookahead_degree <= 5.0 {
            0.8
        } else if lookahead_degree <= 10.0 {
            0.6
        } else if lookahead_degree <= 15.0 {
            0.4
        } else {
            println!(
                "Lookahead Degree: {:.2}°, Distance: {:.2}m",
                lookahead_degree, lookahead_rear
            );
            0.2
        };

        // Calculate steering angle using pure pursuit algorithm
        let pure_pursuit_steer = f32::atan2(
            2.0 * config.wheel_base * f32::sin(lookahead_angle),
            lookahead_rear,
        );

        // Determine steering category and calculate speed
        let category = SteeringCategory::from_degree(steering_degree);
        let speed = category.speed_for_mode(config.driving_mode);

        SteeringData {
            steering_angle: pure_pursuit_steer,
            speed,
        }
    }

    // Preprocess LiDAR data (noise removal and smoothing)
    fn preprocess_lidar(ranges: &[f32], window_size: usize) -> Vec<f32> {
        if ranges.is_empty() {
            return vec![];
        }

        // Find minimum range value
        let min_range = ranges.iter().cloned().fold(f32::MAX, f32::min);

        // Calculate dynamic threshold based on minimum range
        let scan_threshold = ReactiveFollowGap::calculate_threshold(min_range);

        // Smoothing parameters
        let padding = window_size / 2;
        let data_size = ranges.len();

        // Create padded vector with boundary checking
        let mut padded = Vec::with_capacity(data_size + 2 * padding);

        // Left padding
        padded.resize(padding, ranges[0]);

        // Copy original data
        padded.extend_from_slice(ranges);

        // Right padding
        padded.resize(data_size + 2 * padding, *ranges.last().unwrap_or(&0.0));

        // 1. Remove high values (set values above threshold to threshold)
        for value in padded.iter_mut() {
            if *value > scan_threshold {
                *value = scan_threshold;
            }
        }

        // 2. Set each value to window average (smoothing)
        let mut result = vec![0.0; data_size];
        for i in 0..data_size {
            // Calculate window sum
            let window_sum: f32 = padded[i..i + window_size].iter().sum();
            result[i] = window_sum / window_size as f32;
        }

        result
    }

    // Calculate threshold based on minimum range (linear interpolation)
    fn calculate_threshold(min_range: f32) -> f32 {
        // Threshold points (range, threshold) pairs
        let points = [
            ThresholdPoint {
                range: 0.15,
                threshold: 0.4,
            }, // Minimum start point
            ThresholdPoint {
                range: 0.2,
                threshold: 0.8,
            }, // Start point
            ThresholdPoint {
                range: 0.5,
                threshold: 1.4,
            }, // Initial sensitivity increase
            ThresholdPoint {
                range: 0.8,
                threshold: 1.8,
            },
            ThresholdPoint {
                range: 1.1,
                threshold: 2.4,
            },
            ThresholdPoint {
                range: 1.4,
                threshold: 2.8,
            }, // Mid-range threshold
            ThresholdPoint {
                range: 1.7,
                threshold: 3.2,
            },
            ThresholdPoint {
                range: 2.0,
                threshold: 3.6,
            },
            ThresholdPoint {
                range: 2.3,
                threshold: 3.9,
            },
            ThresholdPoint {
                range: 2.6,
                threshold: 4.2,
            },
            ThresholdPoint {
                range: 3.0,
                threshold: 4.5,
            }, // Maximum point
        ];

        // Find interval containing min_range
        let mut i = 0;
        while i < points.len() - 1 && min_range > points[i].range {
            i += 1;
        }

        // Handle out-of-bounds cases
        if min_range <= points[0].range {
            return points[0].threshold;
        } else if min_range >= points[points.len() - 1].range {
            return points[points.len() - 1].threshold;
        }

        // Linear interpolation
        let range_diff = points[i].range - points[i - 1].range;
        let threshold_diff = points[i].threshold - points[i - 1].threshold;
        let ratio = (min_range - points[i - 1].range) / range_diff;

        points[i - 1].threshold + (threshold_diff * ratio)
    }

    // Find maximum gap
    fn find_max_gap(
        ranges: &[f32],
        min_index: usize,
        bubble_size: usize,
        right_wing_index: usize,
        left_wing_index: usize,
    ) -> usize {
        let array_size = ranges.len();

        // Create mutable copy of ranges for bubble processing
        let mut processed_ranges = ranges.to_vec();

        // Calculate half bubble size
        let half_bubble = bubble_size / 2;

        // Apply bubble (set ranges around min_index to zero) - with bounds checking
        let start_idx = if min_index > half_bubble {
            min_index - half_bubble
        } else {
            0
        };
        let end_idx = std::cmp::min(min_index + half_bubble, array_size - 1);

        // Set bubble area to zero (obstacle area)
        for i in start_idx..=end_idx {
            processed_ranges[i] = 0.0;
        }

        // Ensure wing indices are properly ordered and within bounds
        let left_bound = std::cmp::min(left_wing_index, array_size - 1);
        let right_bound = std::cmp::min(right_wing_index, array_size - 1);

        // Find maximum value within range
        let mut max_dist = 0.0f32;

        // Determine scan direction based on wing indices
        let (start_idx, end_idx) = if right_bound <= left_bound {
            (right_bound, left_bound)
        } else {
            // If wings are reversed, search the entire range
            (0, array_size - 1)
        };

        // Find maximum distance
        for i in start_idx..=end_idx {
            if processed_ranges[i] > max_dist {
                max_dist = processed_ranges[i];
            }
        }

        // Special case: if no gaps found
        if max_dist == 0.0 {
            return array_size / 2; // Return middle index
        }

        // Find indices with maximum value
        let mut max_points = Vec::new();
        for i in start_idx..=end_idx {
            if processed_ranges[i] == max_dist {
                max_points.push(i);
            }
        }

        if max_points.is_empty() {
            return array_size / 2; // Return middle index if no max points found
        }

        // Find longest continuous sequence of max points
        let mut current_start = max_points[0];
        let mut current_length = 1;
        let mut max_gap_start = current_start;
        let mut max_gap_length = 1;

        for i in 1..max_points.len() {
            // Check if points are sequential
            if max_points[i] == max_points[i - 1] + 1 {
                current_length += 1;
            } else {
                // If current sequence is longer than max, update max
                if current_length > max_gap_length {
                    max_gap_length = current_length;
                    max_gap_start = current_start;
                }
                // Start new sequence
                current_start = max_points[i];
                current_length = 1;
            }
        }

        // Check final sequence
        if current_length > max_gap_length {
            max_gap_length = current_length;
            max_gap_start = current_start;
        }

        // Calculate middle of longest sequence
        max_gap_start + (max_gap_length / 2)
    }
}

impl Clone for GapFollowConfig {
    fn clone(&self) -> Self {
        Self {
            car_width: self.car_width,
            wheel_base: self.wheel_base,
            left_wing_angle: self.left_wing_angle,
            right_wing_angle: self.right_wing_angle,
            max_bubble_points: self.max_bubble_points,
            look_ahead_offset: self.look_ahead_offset,
            window_size: self.window_size,
            driving_mode: self.driving_mode,
        }
    }
}

fn main() -> Result<(), Error> {
    println!("Starting Reactive Follow Gap Node");
    let context = Context::new(env::args())?;
    let node = rclrs::create_node(&context, "reactive_follow_gap")?;

    // Create node with default settings
    //let _reactive_follow_gap = ReactiveFollowGap::new(&node, None)?;

    // Or create node with custom settings
    let config = GapFollowConfig {
        driving_mode: DrivingMode::Normal,
        car_width: 0.45,
        ..Default::default()
    };
    let _reactive_follow_gap = ReactiveFollowGap::new(&node, Some(config))?;

    while context.ok() {
        let _ = rclrs::spin_once(node.clone(), None);
    }

    Ok(())
}
