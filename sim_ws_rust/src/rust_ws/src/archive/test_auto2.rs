use ackermann_msgs::msg::AckermannDriveStamped;
use anyhow::Result;
use nav_msgs::msg::Odometry;
use rclrs::{self, Context};
use sensor_msgs::msg::LaserScan;
use std::env;
use std::sync::{Arc, Mutex};

// Constants for navigation parameters
const SAFE_DISTANCE: f32 = 0.4;
const TURN_THRESHOLD: f32 = 0.8;
const WALL_FOLLOW_DISTANCE: f32 = 0.6;
const STUCK_TIME_THRESHOLD: f32 = 1.5;
const REVERSE_SPEED: f32 = -0.3;
const FORWARD_SPEED_SLOW: f32 = 0.3;
const FORWARD_SPEED_NORMAL: f32 = 0.4;
const FORWARD_SPEED_FAST: f32 = 0.5;

/// Represents the different sectors of laser scan data
#[derive(Debug)]
struct ScanSectors {
    far_left: f32,
    left: f32,
    center: f32,
    right: f32,
    far_right: f32,
}

impl ScanSectors {
    fn new() -> Self {
        Self {
            far_left: f32::INFINITY,
            left: f32::INFINITY,
            center: f32::INFINITY,
            right: f32::INFINITY,
            far_right: f32::INFINITY,
        }
    }

    fn to_array(&self) -> [f32; 5] {
        [
            self.far_left,
            self.left,
            self.center,
            self.right,
            self.far_right,
        ]
    }
}

/// Main safety controller for autonomous navigation
struct Safety {
    speed: Arc<Mutex<f32>>,
    publisher: Arc<rclrs::Publisher<AckermannDriveStamped>>,
    is_reversing: Arc<Mutex<bool>>,
    stuck_time: Arc<Mutex<f32>>,
}

impl Safety {
    fn new(publisher: Arc<rclrs::Publisher<AckermannDriveStamped>>) -> Self {
        Safety {
            speed: Arc::new(Mutex::new(0.0)),
            publisher,
            is_reversing: Arc::new(Mutex::new(false)),
            stuck_time: Arc::new(Mutex::new(0.0)),
        }
    }

    fn odom_callback(&self, msg: Odometry) {
        let mut speed = self.speed.lock().unwrap();
        *speed = msg.twist.twist.linear.x as f32;
        println!("Speed updated: {:.3} m/s", *speed);
    }

    fn analyze_scan_data(&self, msg: &LaserScan) -> (f32, ScanSectors) {
        let mut min_distance = f32::INFINITY;
        let mut sectors = ScanSectors::new();

        for (i, &distance) in msg.ranges.iter().enumerate() {
            if distance > msg.range_min && distance < msg.range_max {
                let angle = msg.angle_min + (i as f32) * msg.angle_increment;
                min_distance = min_distance.min(distance);

                // Categorize distances into sectors
                match angle {
                    a if a < -0.6 => sectors.far_left = sectors.far_left.min(distance),
                    a if a < -0.2 => sectors.left = sectors.left.min(distance),
                    a if a < 0.2 => sectors.center = sectors.center.min(distance),
                    a if a < 0.6 => sectors.right = sectors.right.min(distance),
                    _ => sectors.far_right = sectors.far_right.min(distance),
                }
            }
        }

        (min_distance, sectors)
    }

    fn calculate_steering(&self, sectors: &ScanSectors, in_narrow_space: bool) -> f32 {
        if in_narrow_space {
            // More aggressive steering in narrow spaces
            let left_space = sectors.far_left + sectors.left;
            let right_space = sectors.right + sectors.far_right;
            let space_diff = right_space - left_space;

            let steering_factor = if space_diff.abs() > 1.0 {
                space_diff * 0.5 // Stronger steering for large differences
            } else {
                space_diff * 0.3 // Gentler steering for small differences
            };
            steering_factor.clamp(-0.8, 0.8)
        } else {
            // Normal steering in open spaces
            let steering_factor =
                ((sectors.right + sectors.far_right) - (sectors.far_left + sectors.left)) * 0.3;
            steering_factor.clamp(-0.7, 0.7)
        }
    }

    fn handle_wall_following(&self, sectors: &ScanSectors) -> (f32, f32) {
        let left_wall = sectors.left < WALL_FOLLOW_DISTANCE;
        let right_wall = sectors.right < WALL_FOLLOW_DISTANCE;

        match (left_wall, right_wall) {
            (true, false) => (FORWARD_SPEED_NORMAL, 0.5), // Follow right wall
            (false, true) => (FORWARD_SPEED_NORMAL, -0.5), // Follow left wall
            _ => (FORWARD_SPEED_FAST, self.calculate_steering(sectors, false)),
        }
    }

    fn scan_callback(&self, msg: LaserScan) {
        let speed = *self.speed.lock().unwrap();
        let mut is_reversing = self.is_reversing.lock().unwrap();
        let mut stuck_time = self.stuck_time.lock().unwrap();
        let (min_distance, sectors) = self.analyze_scan_data(&msg);

        // Update stuck status
        if speed.abs() < 0.1 {
            *stuck_time += 0.1;
        } else {
            *stuck_time = 0.0;
        }

        let mut drive_msg = AckermannDriveStamped::default();

        // Determine driving behavior
        if sectors.center < SAFE_DISTANCE {
            // Obstacle avoidance mode
            *is_reversing = true;
            drive_msg.drive.speed = REVERSE_SPEED;
            let left_space = sectors.far_left.min(sectors.left);
            let right_space = sectors.right.min(sectors.far_right);
            drive_msg.drive.steering_angle = if left_space > right_space { -1.0 } else { 1.0 };
        } else {
            *is_reversing = false;
            if sectors.center > TURN_THRESHOLD {
                // Open space navigation
                let (speed, steering) = self.handle_wall_following(&sectors);
                drive_msg.drive.speed = speed;
                drive_msg.drive.steering_angle = steering;
            } else {
                // Narrow space navigation
                drive_msg.drive.speed = FORWARD_SPEED_SLOW;
                drive_msg.drive.steering_angle = self.calculate_steering(&sectors, true);
            }
        }

        // Handle stuck condition
        if *stuck_time > STUCK_TIME_THRESHOLD {
            self.handle_stuck_condition(&mut drive_msg, &mut is_reversing, &mut stuck_time);
        }

        self.log_status(
            &drive_msg,
            min_distance,
            &sectors,
            *is_reversing,
            *stuck_time,
        );
        self.publish_drive_command(drive_msg);
    }

    fn handle_stuck_condition(
        &self,
        drive_msg: &mut AckermannDriveStamped,
        is_reversing: &mut bool,
        stuck_time: &mut f32,
    ) {
        *is_reversing = !*is_reversing;
        *stuck_time = 0.0;
        if *is_reversing {
            drive_msg.drive.speed = REVERSE_SPEED;
            drive_msg.drive.steering_angle *= -1.0;
        }
    }

    fn log_status(
        &self,
        drive_msg: &AckermannDriveStamped,
        min_distance: f32,
        sectors: &ScanSectors,
        is_reversing: bool,
        stuck_time: f32,
    ) {
        println!("=== Driving Summary ===");
        println!("Speed: {:.3} m/s", drive_msg.drive.speed);
        println!("Steering: {:.3} rad", drive_msg.drive.steering_angle);
        println!("Minimum distance: {:.3} m", min_distance);
        println!(
            "Mode: {}",
            if is_reversing { "Reversing" } else { "Forward" }
        );

        let sector_array = sectors.to_array();
        println!(
            "Sectors (FL/L/C/R/FR): {:.2}/{:.2}/{:.2}/{:.2}/{:.2}",
            sector_array[0], sector_array[1], sector_array[2], sector_array[3], sector_array[4]
        );
        println!("Stuck time: {:.1}s", stuck_time);
        println!("==================\n");
    }

    fn publish_drive_command(&self, drive_msg: AckermannDriveStamped) {
        if let Err(e) = self.publisher.publish(&drive_msg) {
            println!("Failed to publish drive command: {:?}", e);
        }
    }
}

fn main() -> Result<()> {
    println!("Starting Autonomous Navigation Node");
    let context = Context::new(env::args())?;
    let node = rclrs::create_node(&context, "autonomous_navigator")?;

    let publisher = Arc::new(
        node.create_publisher::<AckermannDriveStamped>("/drive", rclrs::QOS_PROFILE_DEFAULT)?,
    );

    let safety = Arc::new(Safety::new(Arc::clone(&publisher)));

    // Set up odometry subscription
    let safety_clone_odom = Arc::clone(&safety);
    let _odom_sub = node.create_subscription::<Odometry, _>(
        "/ego_racecar/odom",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: Odometry| safety_clone_odom.odom_callback(msg),
    )?;

    // Set up laser scan subscription
    let safety_clone_scan = Arc::clone(&safety);
    let _scan_sub = node.create_subscription::<LaserScan, _>(
        "/scan",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: LaserScan| safety_clone_scan.scan_callback(msg),
    )?;

    println!("All systems initialized. Starting autonomous navigation...");
    rclrs::spin(node)?;
    Ok(())
}
