use ackermann_msgs::msg::AckermannDriveStamped;
use anyhow::{Error, Result};
use rclrs::ParameterValue;
use rclrs::{self, Context};
use std::{
    env,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread,
    time::Duration,
};

fn main() -> Result<(), Error> {
    println!("This is test move node");
    let context = Context::new(env::args())?;
    let node = rclrs::create_node(&context, "test_param")?;
    let publisher =
        node.create_publisher::<AckermannDriveStamped>("/drive", rclrs::QOS_PROFILE_DEFAULT)?;
    let mut msg = AckermannDriveStamped::default();

    // Create a mandatory parameter with a default value
    let int_param = node
        .declare_parameter("my_int_param")
        .default(42)
        .mandatory()
        .unwrap();

    // Create an optional parameter (can be unset)
    let optional_param = node
        .declare_parameter("my_optional_param")
        .default(10.3)
        .optional()
        .unwrap();

    // Create a read-only parameter
    let readonly_param = node
        .declare_parameter("my_readonly_param")
        .default(10.5)
        .read_only()
        .unwrap();

    // Check the value
    let man_par = int_param.get();
    let optional_par = optional_param.get();
    let read_only_par = readonly_param.get();

    println!(
        "man_par: {}\nopt_par: {:?}\nread_par: {}",
        man_par, optional_par, read_only_par
    );

    // Create an executor to handle service callbacks
    let executor = rclrs::SingleThreadedExecutor::new();
    let _ = executor.add_node(&node.clone());

    // Use a flag to signal when the drive thread is done
    let thread_finished = Arc::new(AtomicBool::new(false));
    let thread_finished_clone = thread_finished.clone();

    // Start a separate thread for the driving commands
    let publisher_clone = publisher.clone();
    let drive_thread = thread::spawn(move || {
        let mut msg = AckermannDriveStamped::default();

        // Forward motion
        for _ in 0..100 {
            msg.drive.speed = 1.0;
            thread::sleep(Duration::from_millis(10));
            let _ = publisher_clone.publish(&msg);
        }

        // Backward motion
        for _ in 0..100 {
            msg.drive.speed = -1.0;
            thread::sleep(Duration::from_millis(10));
            let _ = publisher_clone.publish(&msg);
        }

        // Stop
        msg.drive.speed = 0.0;
        let _ = publisher_clone.publish(&msg);

        // Signal that we're done
        thread_finished_clone.store(true, Ordering::SeqCst);
    });

    // Spin the executor to process service callbacks
    while !thread_finished.load(Ordering::SeqCst) {
        match executor.spin_once(Some(Duration::from_millis(100))) {
            Ok(_) => {}
            Err(e) => {
                // Ignore timeout errors which are expected
                if !e.to_string().contains("Timeout") {
                    return Err(e.into());
                }
            }
        }
    }

    // Wait for the drive thread to complete
    drive_thread.join().unwrap();

    println!("Node shutting down");
    Ok(())
}
