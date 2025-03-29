use ackermann_msgs::msg::AckermannDriveStamped;
use anyhow::{Error, Result};
use rclrs::{self, Context};
use sensor_msgs::msg::LaserScan;
use std::{
    env,
    f32::consts::PI,
    sync::{Arc, Mutex},
};

struct ReactiveFollowGap {
    // Keep only the fields we need to prevent unused field warnings
    // The subscription is kept alive as a field to prevent it from being dropped
    _scan_subscription: Arc<rclrs::Subscription<LaserScan>>,
}

impl ReactiveFollowGap {
    pub fn new(node: &rclrs::Node) -> Result<Self, rclrs::RclrsError> {
        // Create shared state - moved inside the closure since they're not used elsewhere
        let data_size = Arc::new(Mutex::new(0usize));
        let left_wing_index = Arc::new(Mutex::new(0usize));
        let right_wing_index = Arc::new(Mutex::new(0usize));

        // Create publisher
        let drive_publisher =
            node.create_publisher::<AckermannDriveStamped>("/drive", rclrs::QOS_PROFILE_DEFAULT)?;

        // Define wing angles
        let left_wing = PI / 2.7;
        let right_wing = -(PI / 2.7);

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
                    let left_wing_idx =
                        ((left_wing - scan_msg.angle_min) / scan_msg.angle_increment) as usize;
                    let right_wing_idx =
                        ((right_wing - scan_msg.angle_min) / scan_msg.angle_increment) as usize;

                    // Ensure indices are within bounds
                    let safe_left = std::cmp::min(left_wing_idx, range_count - 1);
                    let safe_right = std::cmp::min(right_wing_idx, range_count - 1);

                    *left_wing_index.lock().unwrap() = safe_left;
                    *right_wing_index.lock().unwrap() = safe_right;
                }

                // Process each LiDAR scan
                let mut processed_ranges = scan_msg.ranges.clone();
                processed_ranges = ReactiveFollowGap::preprocess_lidar(&processed_ranges);

                // Find closest point to LiDAR
                let mut min_range = f32::MAX;
                let mut min_index = 0usize;

                for (i, &range) in processed_ranges.iter().enumerate() {
                    if range < min_range {
                        min_range = range;
                        min_index = i;
                    }
                }

                // Calculate bubble size based on car width and distance
                let car_width = 0.5f32;
                let bubble_size = ((car_width / min_range) / scan_msg.angle_increment) as usize;
                let bubble_size = std::cmp::min(bubble_size, 850);

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
                let steering_angle =
                    scan_msg.angle_min + (scan_msg.angle_increment * best_point_index as f32);
                let steering_degree = (steering_angle * 180.0 / PI).abs();

                let lookahead = processed_ranges[best_point_index];
                let bestpoint_x = lookahead * f32::cos(steering_angle);
                let bestpoint_y = lookahead * f32::sin(steering_angle);

                let lookahead_angle = f32::atan2(bestpoint_y, bestpoint_x + 0.27);
                let mut lookahead_rear = f32::sqrt(
                    (bestpoint_x + 0.27) * (bestpoint_x + 0.27) + bestpoint_y * bestpoint_y,
                );

                let lookahead_degree = (lookahead_angle * 180.0 / PI).abs();

                // Adjust lookahead distance based on angle
                if lookahead_degree <= 5.0 {
                    lookahead_rear = lookahead_rear * 0.8;
                } else if lookahead_degree <= 10.0 {
                    lookahead_rear = lookahead_rear * 0.6;
                } else if lookahead_degree <= 15.0 {
                    lookahead_rear = lookahead_rear * 0.4;
                    println!(
                        "Lookahead Degree: {:.2}, Lookahead Distance: {:.2}",
                        lookahead_degree, lookahead_rear
                    );
                } else {
                    lookahead_rear = lookahead_rear * 0.2;
                    println!(
                        "Lookahead Degree: {:.2}, Lookahead Distance: {:.2}",
                        lookahead_degree, lookahead_rear
                    );
                }

                let pure_pursuit_steer =
                    f32::atan2(2.0 * 0.32 * f32::sin(lookahead_angle), lookahead_rear);

                // Set speed based on steering angle
                let drive_speed = if steering_degree <= 5.0 {
                    // Almost straight
                    2.0f32
                } else if steering_degree <= 10.0 {
                    // Slight curve
                    1.5f32
                } else if steering_degree <= 15.0 {
                    // Gentle curve
                    1.2f32
                } else {
                    // Medium curve
                    0.8f32
                };

                // Create and publish drive message
                let mut drive_msg = AckermannDriveStamped::default();
                drive_msg.drive.steering_angle = pure_pursuit_steer;
                drive_msg.drive.speed = drive_speed;
                let _ = drive_publisher.publish(&drive_msg);
            },
        )?;

        Ok(Self {
            _scan_subscription: scan_subscription,
        })
    }

    fn preprocess_lidar(ranges: &[f32]) -> Vec<f32> {
        if ranges.is_empty() {
            return vec![];
        }

        // Find min range value
        let min_range = ranges.iter().cloned().fold(f32::MAX, f32::min);

        // Dynamic threshold calculation based on min_range
        let scan_threshold = ReactiveFollowGap::calculate_threshold(min_range);

        // Smoothing parameters
        let window_size = 9;
        let padding = window_size / 2;
        let data_size = ranges.len();

        // Create padded vector with bounds checking
        let mut padded = Vec::with_capacity(data_size + 2 * padding);

        // Left padding
        padded.resize(padding, ranges[0]);

        // Copy original data
        padded.extend_from_slice(ranges);

        // Right padding
        padded.resize(data_size + 2 * padding, *ranges.last().unwrap_or(&0.0));

        // 1. Rejecting high values
        for value in padded.iter_mut() {
            if *value > scan_threshold {
                *value = scan_threshold;
            }
        }

        // 2. Setting each value to the mean over window with safe bounds
        let mut result = vec![0.0; data_size];
        for i in 0..data_size {
            let window_sum: f32 = padded[i..i + window_size].iter().sum();
            result[i] = window_sum / window_size as f32;
        }

        result
    }

    fn calculate_threshold(min_range: f32) -> f32 {
        // Threshold points for linear interpolation
        let points: [(f32, f32); 11] = [
            (0.15, 0.4), // minimum start point
            (0.2, 0.8),  // start point
            (0.5, 1.4),  // more sensitive increase early on
            (0.8, 1.8),
            (1.1, 2.4),
            (1.4, 2.8), // larger threshold increment in middle range
            (1.7, 3.2),
            (2.0, 3.6),
            (2.3, 3.9),
            (2.6, 4.2),
            (3.0, 4.5), // maximum point
        ];

        // Find which interval contains min_range
        let mut i = 0;
        while i < points.len() - 1 && min_range > points[i].0 {
            i += 1;
        }

        // Handle out-of-bounds cases
        if min_range <= points[0].0 {
            return points[0].1;
        } else if min_range >= points[points.len() - 1].0 {
            return points[points.len() - 1].1;
        }

        // Linear interpolation
        let range_diff = points[i].0 - points[i - 1].0;
        let threshold_diff = points[i].1 - points[i - 1].1;
        let ratio = (min_range - points[i - 1].0) / range_diff;

        points[i - 1].1 + (threshold_diff * ratio)
    }

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

        // Apply bubble (set ranges to zero around min_index) with bounds checking
        let start_idx = if min_index > half_bubble {
            min_index - half_bubble
        } else {
            0
        };
        let end_idx = std::cmp::min(min_index + half_bubble, array_size - 1);

        for i in start_idx..=end_idx {
            processed_ranges[i] = 0.0;
        }

        // Ensure wing indices are properly ordered and within bounds
        let left_bound = std::cmp::min(left_wing_index, array_size - 1);
        let right_bound = std::cmp::min(right_wing_index, array_size - 1);

        // Find maximum value within range bounds
        let mut max_dist = 0.0f32;

        // Determine scan direction based on wing indices
        let (start_idx, end_idx) = if right_bound <= left_bound {
            (right_bound, left_bound)
        } else {
            // If wings are reversed, search the entire range
            (0, array_size - 1)
        };

        for i in start_idx..=end_idx {
            if processed_ranges[i] > max_dist {
                max_dist = processed_ranges[i];
            }
        }

        // Special case: if no gaps found
        if max_dist == 0.0 {
            return array_size / 2; // Return middle index
        }

        // Find indices with max value
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

fn main() -> Result<(), Error> {
    println!("Starting Reactive Follow Gap Node");
    let context = Context::new(env::args())?;
    let node = rclrs::create_node(&context, "reactive_follow_gap")?;

    let _reactive_follow_gap = ReactiveFollowGap::new(&node)?;

    while context.ok() {
        let _ = rclrs::spin_once(node.clone(), None);
    }

    Ok(())
}
