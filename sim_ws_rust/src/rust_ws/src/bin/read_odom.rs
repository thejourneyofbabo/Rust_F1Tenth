use anyhow::{Error, Result};
use nav_msgs::msg::Odometry;
use rclrs::{self, Context};
use std::env;

fn main() -> Result<(), Error> {
    println!("test_move node");

    let context = Context::new(env::args())?;
    let node = rclrs::create_node(&context, "odom_scan")?;
    let _subscriber = node.create_subscription::<Odometry, _>(
        "ego_racecar/odom",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: Odometry| {
            //println!("{:?}", msg);
            println!(
                "
                pose_x: {:.3}\n
                pose_y: {:.3}\n
                pose_z: {:.3}\n
                twist_linear.x {:.3}\n 
                twist_linear.y {:.3}\n 
                twist_linear.z {:.3}\n 
                twist_angular.x {:.3}\n 
                twist_angular.y {:.3}\n 
                twist_angular.z {:.3}\n 
                -----------",
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z,
            )
        },
    )?;

    rclrs::spin(node).map_err(|err| err.into())
}
