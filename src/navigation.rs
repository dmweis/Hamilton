use crate::{driver::HamiltonLssDriver, localiser::TrackedObjects, motion_controller::Controller};
use crate::{holonomic_controller::HolonomicWheelCommand, motion_controller::Pose};
use anyhow::Result;
use last_message_channel::Receiver;
use std::{sync::Arc, time::Duration};
use tokio::{
    spawn,
    sync::Mutex,
    time::{interval, timeout},
};
use tracing::*;

pub struct Navigation {
    driver: Arc<Mutex<HamiltonLssDriver>>,
    motion_controller: Controller,
    localization_receiver: Receiver<TrackedObjects>,
}

impl Navigation {
    pub fn new(
        driver: HamiltonLssDriver,
        motion_controller: Controller,
        localization_receiver: Receiver<TrackedObjects>,
    ) -> Self {
        let driver = Arc::new(Mutex::new(driver));
        start_voltage_monitor(&driver);
        Self {
            driver,
            motion_controller,
            localization_receiver,
        }
    }

    pub async fn move_to(&mut self, target: Pose) -> Result<()> {
        self.motion_controller.update_target_pose(target);
        loop {
            if let Ok(message) = timeout(
                Duration::from_millis(500),
                self.localization_receiver.recv(),
            )
            .await
            {
                if let Ok(message) = message {
                    if let Some((position, yaw)) = message.get_tracker_pose() {
                        self.motion_controller
                            .update_current_pose(Pose::from_na(position, yaw));
                        if let Some((move_command, moved)) =
                            self.motion_controller.calculate_drive_or_reached()
                        {
                            if !moved {
                                self.motion_controller.clear_target();
                                return Ok(());
                            }
                            self.driver.lock().await.send(move_command).await?;
                        } else {
                            self.driver
                                .lock()
                                .await
                                .send(HolonomicWheelCommand::stopped())
                                .await?;
                        }
                    } else {
                        self.driver
                            .lock()
                            .await
                            .send(HolonomicWheelCommand::stopped())
                            .await?;
                        error!("No tracker pose found");
                    }
                } else {
                    error!("UDP channel closed");
                    break;
                }
            } else {
                let move_command = HolonomicWheelCommand::stopped();
                self.driver.lock().await.send(move_command).await?;
                error!("No udp messages");
            }
        }
        Ok(())
    }
}

fn start_voltage_monitor(driver: &Arc<Mutex<HamiltonLssDriver>>) {
    let driver_clone = Arc::clone(driver);
    spawn(async move {
        let mut reading_rate = interval(Duration::from_secs(1));
        loop {
            reading_rate.tick().await;
            let mut driver = driver_clone.lock().await;
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
}
