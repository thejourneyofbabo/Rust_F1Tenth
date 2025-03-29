use anyhow::{Error, Result};
use rclrs::{self, Context};
use std::env;

fn main() -> Result<(), Error> {
    println!("This is test basic param_node");
    let context = Context::new(env::args())?;
    let node = rclrs::create_node(&context, "basic_param")?;

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

    println!("Node shutting down");
    Ok(())
}
