use anyhow::Result;
use clap::Clap;
use hamilton::{
    driver::{BodyConfig, HamiltonLssDriver},
    holonomic_controller::HolonomicWheelCommand,
    map::Map,
    motion_controller::{Controller, Pose},
};
use nalgebra as na;
use remote_controller::{start_remote_controller_server_with_map, Action, ActionList, AreaSize};
use std::{
    net::SocketAddrV4,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::Duration,
};
use tokio::{
    self, spawn,
    sync::Mutex,
    time::{interval, sleep, timeout},
};
use tracing::*;
use tracing_subscriber::filter::LevelFilter;

#[derive(Clap)]
#[clap()]
struct Args {
    #[clap(about = "Serial port to use")]
    port: String,
    #[clap(long = "config", about = "Config path")]
    config: Option<String>,
    #[clap(short, long, default_value = "239.0.0.22:7070")]
    address: SocketAddrV4,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    tracing_subscriber::fmt()
        .pretty()
        .with_env_filter("hamilton=info")
        .with_max_level(LevelFilter::INFO)
        .init();
    let body_config = if let Some(path) = args.config {
        info!("Loading configuration from {}", path);
        BodyConfig::load(&path)?
    } else {
        info!("Loading default configuration");
        BodyConfig::load_from_default()?
    };

    let map = Map::included_room();
    let mut navigation_controller = Controller::default();

    let lss_driver = Arc::new(Mutex::new(lss_driver::LSSDriver::new(&args.port)?));
    let hamilton_driver = HamiltonLssDriver::new(lss_driver, body_config).await?;
    let shared_driver = Arc::new(Mutex::new(hamilton_driver));

    let cloned_driver = Arc::clone(&shared_driver);
    let (area_height, area_width) = map.get_size();

    // why did I design it this way? This is dumb
    let actions = ActionList::new(vec![
        Action::new("save_waypoint_a".to_owned(), "Save waypoint A".to_owned()),
        Action::new("save_waypoint_b".to_owned(), "Save waypoint B".to_owned()),
        Action::new("go_to_waypoint_a".to_owned(), "Go to waypoint A".to_owned()),
        Action::new("go_to_waypoint_b".to_owned(), "Go to waypoint B".to_owned()),
    ]);

    let mut controller_state = start_remote_controller_server_with_map(
        ([0, 0, 0, 0], 8080),
        AreaSize::new(area_width, area_height),
        actions,
    );

    spawn(async move {
        let mut reading_rate = interval(Duration::from_secs(1));
        loop {
            reading_rate.tick().await;
            let mut driver = shared_driver.lock().await;
            if let Ok(voltage) = driver.read_voltage().await {
                info!("Current voltage is {}", voltage);
                let color = if voltage < 3.0 * 3.6 {
                    lss_driver::LedColor::Red
                } else {
                    lss_driver::LedColor::Magenta
                };
                if driver.set_color(color).await.is_err() {
                    error!("Failed to set color");
                }
            } else {
                error!("Failed to read voltage");
            }
        }
    });

    let running = Arc::new(AtomicBool::new(true));
    let running_handle = running.clone();

    ctrlc::set_handler(move || {
        running_handle.store(false, Ordering::Release);
        println!("Caught interrupt\nExiting...");
    })?;

    let mut localization_rx =
        hamilton::localiser::create_localization_subscriber(args.address).await?;

    let mut target_a = na::Point2::origin();
    let mut target_b = na::Point2::origin();

    // this loop is getting super convoluted
    // refactor
    // pattern match here would decrease the indentation
    // but I don't think it would make this cleaner
    // this is complicated logic so it looks dense
    // refactor to structure
    loop {
        if !running.load(Ordering::Acquire) {
            break;
        }
        if let Ok(message) = timeout(Duration::from_millis(500), localization_rx.recv()).await {
            if let Some(message) = message {
                if let Some((position, yaw)) = message.get_tracker_pose() {
                    // set start position
                    navigation_controller.update_current_pose(Pose::from_na(position, yaw));

                    if let Some(action_id) = controller_state.check_new_actions()? {
                        match action_id.as_str() {
                            "save_waypoint_a" => {
                                if let Some(position) = message.get_any_controller_pose() {
                                    target_a = position;
                                }
                            }
                            "save_waypoint_b" => {
                                if let Some(position) = message.get_any_controller_pose() {
                                    target_b = position;
                                }
                            }
                            "go_to_waypoint_a" => {
                                navigation_controller.update_target_pose(Pose::from_na(
                                    target_a,
                                    na::Rotation2::identity(),
                                ));
                            }
                            "go_to_waypoint_b" => {
                                navigation_controller.update_target_pose(Pose::from_na(
                                    target_b,
                                    na::Rotation2::identity(),
                                ));
                            }
                            _ => (),
                        }
                    }

                    if let Some(move_command) = navigation_controller.calculate_drive() {
                        cloned_driver.lock().await.send(move_command).await?;
                    } else {
                        cloned_driver
                            .lock()
                            .await
                            .send(HolonomicWheelCommand::stopped())
                            .await?;
                    }
                } else {
                    let move_command = HolonomicWheelCommand::stopped();
                    cloned_driver.lock().await.send(move_command).await?;
                    error!("No tracker pose found");
                }
            } else {
                error!("UDP channel closed");
                break;
            }
        } else {
            let move_command = HolonomicWheelCommand::stopped();
            cloned_driver.lock().await.send(move_command).await?;
            error!("No udp messages");
        }
    }

    let move_command = HolonomicWheelCommand::stopped();
    cloned_driver.lock().await.send(move_command).await?;
    sleep(Duration::from_secs(1)).await;
    Ok(())
}
