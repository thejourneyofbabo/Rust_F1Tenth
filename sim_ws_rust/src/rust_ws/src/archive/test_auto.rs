use ackermann_msgs::msg::AckermannDriveStamped;
use anyhow::Result;
use nav_msgs::msg::Odometry;
use rclrs::{self, Context};
use sensor_msgs::msg::LaserScan;
use std::env;
use std::sync::{Arc, Mutex};

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

    fn scan_callback(&self, msg: LaserScan) {
        let speed = *self.speed.lock().unwrap();
        let mut is_reversing = self.is_reversing.lock().unwrap();
        let mut stuck_time = self.stuck_time.lock().unwrap();

        let mut min_distance = f32::INFINITY;
        let mut sectors = vec![f32::INFINITY; 5]; // 5개 섹터로 늘림 (far_left, left, center, right, far_right)

        // 스캔 데이터 분석
        for (i, &distance) in msg.ranges.iter().enumerate() {
            if distance > msg.range_min && distance < msg.range_max {
                let angle = msg.angle_min + (i as f32) * msg.angle_increment;

                min_distance = min_distance.min(distance);

                // 더 세밀한 섹터 분석
                if angle < -0.6 {
                    sectors[0] = sectors[0].min(distance); // far left
                } else if angle < -0.2 {
                    sectors[1] = sectors[1].min(distance); // left
                } else if angle < 0.2 {
                    sectors[2] = sectors[2].min(distance); // center
                } else if angle < 0.6 {
                    sectors[3] = sectors[3].min(distance); // right
                } else {
                    sectors[4] = sectors[4].min(distance); // far right
                }
            }
        }

        let mut drive_msg = AckermannDriveStamped::default();

        // 정지 상태 확인
        if speed.abs() < 0.1 {
            *stuck_time += 0.1;
        } else {
            *stuck_time = 0.0;
        }

        // 향상된 주행 로직
        const SAFE_DISTANCE: f32 = 0.4;
        const TURN_THRESHOLD: f32 = 0.8;
        const WALL_FOLLOW_DISTANCE: f32 = 0.6;

        if sectors[2] < SAFE_DISTANCE {
            // 전방 장애물이 너무 가까움
            *is_reversing = true;
            drive_msg.drive.speed = -0.3;

            // 더 넓은 공간쪽으로 회전 (조향각 증가)
            let left_space = sectors[0].min(sectors[1]);
            let right_space = sectors[3].min(sectors[4]);
            drive_msg.drive.steering_angle = if left_space > right_space { -1.0 } else { 1.0 };
        } else {
            *is_reversing = false;

            // 전진 시 장애물 회피 로직 개선
            if sectors[2] > TURN_THRESHOLD {
                // 벽 따라가기 로직 추가
                let left_wall = sectors[1] < WALL_FOLLOW_DISTANCE;
                let right_wall = sectors[3] < WALL_FOLLOW_DISTANCE;

                if left_wall {
                    // 왼쪽 벽이 가까우면 오른쪽으로 조향
                    drive_msg.drive.speed = 0.4;
                    drive_msg.drive.steering_angle = 0.5;
                } else if right_wall {
                    // 오른쪽 벽이 가까우면 왼쪽으로 조향
                    drive_msg.drive.speed = 0.4;
                    drive_msg.drive.steering_angle = -0.5;
                } else {
                    // 양쪽이 모두 열려있으면 더 넓은 쪽으로 이동
                    let steering_factor =
                        ((sectors[3] + sectors[4]) - (sectors[0] + sectors[1])) * 0.3;
                    drive_msg.drive.speed = 0.5;
                    drive_msg.drive.steering_angle = steering_factor.clamp(-0.7, 0.7);
                }
            } else {
                // 좁은 공간에서의 주행
                drive_msg.drive.speed = 0.3;

                // 더 적극적인 회피 동작
                let left_space = sectors[0] + sectors[1]; // 왼쪽 공간의 총합
                let right_space = sectors[3] + sectors[4]; // 오른쪽 공간의 총합
                let space_diff = right_space - left_space;

                // 공간 차이에 따라 더 강하게 조향
                let steering_factor = if space_diff.abs() > 1.0 {
                    space_diff * 0.5 // 큰 차이가 있을 때 더 강하게 조향
                } else {
                    space_diff * 0.3 // 작은 차이일 때는 부드럽게 조향
                };
                drive_msg.drive.steering_angle = steering_factor.clamp(-0.8, 0.8);
            }
        }

        // 오랫동안 막혀있을 때의 탈출 로직
        if *stuck_time > 1.5 {
            *is_reversing = !*is_reversing;
            *stuck_time = 0.0;
            if *is_reversing {
                drive_msg.drive.speed = -0.3;
                drive_msg.drive.steering_angle *= -1.0; // 반대 방향으로 조향
            }
        }

        println!("=== Driving Summary ===");
        println!("Speed: {:.3} m/s", drive_msg.drive.speed);
        println!("Steering: {:.3} rad", drive_msg.drive.steering_angle);
        println!("Minimum distance: {:.3} m", min_distance);
        println!(
            "Mode: {}",
            if *is_reversing {
                "Reversing"
            } else {
                "Forward"
            }
        );
        println!(
            "Sectors (FL/L/C/R/FR): {:.2}/{:.2}/{:.2}/{:.2}/{:.2}",
            sectors[0], sectors[1], sectors[2], sectors[3], sectors[4]
        );
        println!("Stuck time: {:.1}s", *stuck_time);
        println!("==================\n");

        // 주행 명령 발행
        match self.publisher.publish(&drive_msg) {
            Ok(_) => (),
            Err(e) => println!("Failed to publish drive command: {:?}", e),
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

    let safety = Safety::new(Arc::clone(&publisher));
    let safety_arc = Arc::new(safety);

    let safety_clone_odom = Arc::clone(&safety_arc);
    let _odom_sub = node.create_subscription::<Odometry, _>(
        "/ego_racecar/odom",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: Odometry| {
            safety_clone_odom.odom_callback(msg);
        },
    )?;

    let safety_clone_scan = Arc::clone(&safety_arc);
    let _scan_sub = node.create_subscription::<LaserScan, _>(
        "/scan",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: LaserScan| {
            safety_clone_scan.scan_callback(msg);
        },
    )?;

    println!("All systems initialized. Starting autonomous navigation...");
    rclrs::spin(node)?;
    Ok(())
}
