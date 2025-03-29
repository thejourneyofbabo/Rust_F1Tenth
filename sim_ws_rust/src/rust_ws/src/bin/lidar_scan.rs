use anyhow::{Error, Result};
use rclrs::{self, Context};
use sensor_msgs::msg::LaserScan;
use std::env;

fn main() -> Result<(), Error> {
    println!("This is LiDAR Scan node");
    let context = Context::new(env::args())?;
    let node = rclrs::create_node(&context, "lidar_scanner")?;
    let _subscriber = node.create_subscription::<LaserScan, _>(
        "scan",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: LaserScan| {
            println!("{:?}", msg);
        },
    )?;

    rclrs::spin(node).map_err(|err| err.into())
}
