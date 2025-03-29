use ackermann_msgs::msg::AckermannDriveStamped;
use anyhow::{Error, Result};
use rclrs::{self, Context};
use sensor_msgs::msg::LaserScan;
use std::{
    env,
    sync::{Arc, Mutex},
};

fn main() -> Result<(), Error> {
    println!("This is LiDAR-based drive control node");

    let context = Context::new(env::args())?;
    let node = rclrs::create_node(&context, "lidar_drive_controller")?;

    let publisher =
        node.create_publisher::<AckermannDriveStamped>("/drive", rclrs::QOS_PROFILE_DEFAULT)?;

    let drive_msg = Arc::new(Mutex::new(AckermannDriveStamped::default()));
    let drive_msg_clone = Arc::clone(&drive_msg);

    let _subscriber = node.create_subscription::<LaserScan, _>(
        "scan",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: LaserScan| {
            let mut drive = drive_msg_clone.lock().unwrap();

            let min_distance = msg
                .ranges
                .iter()
                .filter(|&&r| r > msg.range_min && r < msg.range_max)
                .fold(f32::INFINITY, |a, &b| a.min(b));

            println!("min_distance: {}", min_distance);

            if min_distance < 0.8 {
                drive.drive.speed = 0.0;
                println!("stop!!");
            } else {
                drive.drive.speed = 1.0;
            }

            // Create a copy of the message to publish
            let msg_to_publish = drive.clone();
            // Drop the MutexGuard before publishing
            drop(drive);

            let _ = publisher.publish(&msg_to_publish);
        },
    )?;

    rclrs::spin(node).map_err(|err| err.into())
}
