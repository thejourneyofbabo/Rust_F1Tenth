use anyhow::Result;
use rclrs::{self, Context};
use sensor_msgs::msg::LaserScan;
use std::env;

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

    fn display(&self) {
        println!("=== Sector Distances ===");
        println!("Far Left:  {:.3} m", self.far_left);
        println!("Left:      {:.3} m", self.left);
        println!("Center:    {:.3} m", self.center);
        println!("Right:     {:.3} m", self.right);
        println!("Far Right: {:.3} m", self.far_right);
        println!("=====================\n");
    }
}

fn analyze_scan_data(msg: &LaserScan) -> ScanSectors {
    let mut sectors = ScanSectors::new();

    for (i, &distance) in msg.ranges.iter().enumerate() {
        if distance > msg.range_min && distance < msg.range_max {
            let angle = msg.angle_min + (i as f32) * msg.angle_increment;

            // Updated angle ranges to match visualization
            match angle {
                a if a < -1.0 => sectors.far_right = sectors.far_right.min(distance),
                a if a < -0.5 => sectors.right = sectors.right.min(distance),
                a if a < 0.5 => sectors.center = sectors.center.min(distance),
                a if a < 1.0 => sectors.left = sectors.left.min(distance),
                _ => sectors.far_left = sectors.far_left.min(distance),
            }
        }
    }

    sectors
}

fn scan_callback(msg: LaserScan) {
    let sectors = analyze_scan_data(&msg);
    sectors.display();
}

fn main() -> Result<()> {
    println!("Starting Sector Distance Reader Node");
    let context = Context::new(env::args())?;
    let node = rclrs::create_node(&context, "sector_reader")?;

    // Set up laser scan subscription
    let _scan_sub = node.create_subscription::<LaserScan, _>(
        "/scan",
        rclrs::QOS_PROFILE_DEFAULT,
        scan_callback,
    )?;

    println!("Sector reader initialized. Waiting for laser scan data...");
    rclrs::spin(node)?;
    Ok(())
}
