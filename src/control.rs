use heapless::Vec;
use const_format::formatcp;
use crate::config::FIRMWARE_CONFIG;

#[derive(Clone)]
pub enum SysCommands {
    Restart,
}
pub enum MessageTopics {
    Event,
}

impl MessageTopics {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Event => formatcp!("{}/event", FIRMWARE_CONFIG.mqtt_id),
            Self::Report => formatcp!("{}/report", FIRMWARE_CONFIG.mqtt_id),
        }
    }
}

pub const MAX_SIZE: usize = 10;

pub struct MQTTMessage {
    pub topic: MessageTopics,
    pub payload: Vec<u8, MAX_SIZE>,
}
