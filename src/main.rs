#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::{mem::MaybeUninit, ptr::addr_of_mut, str::{self, FromStr}};

use alloc::string::String;
use esp_backtrace as _;
use esp_hal::{
    clock::{ClockControl, CpuClock},
    cpu_control::{CpuControl, Stack as CPUStack},
    gpio::{GpioPin, Input, Io, Level, Output, Pull},
    i2c::I2C,
    peripherals::{Peripherals, I2C0},
    prelude::*,
    rmt::Rmt,
    rng::Rng,
    system::SystemControl,
    timer::OneShotTimer,
    Async, Blocking,
};

use esp_hal_embassy::Executor;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either, select3, Either3};
use embassy_time::{Delay, Duration, Instant, Timer};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{Channel, Receiver, Sender},
    pubsub::{PubSubChannel, Publisher, Subscriber, WaitResult}
};

use esp_println::println;
use heapless::Vec;
use static_cell::{make_static, StaticCell};

extern crate alloc;

use esp_wifi::wifi::{
    AuthMethod, WifiController, WifiDevice, WifiEvent, WifiStaDevice, WifiState
};
use embassy_net::{dns::DnsQueryType, tcp::TcpSocket, Config, Stack, StackResources};
use rust_mqtt::{
    client::{client::MqttClient, client_config::ClientConfig},
    packet::v5::{
        reason_codes::ReasonCode,
        publish_packet::QualityOfService,
    },
    utils::rng_generator::CountingRng,
};

use esp_hal_smartled::{
    smartLedBuffer,
    SmartLedsAdapter,
};
use smart_leds::{
    brightness, gamma,
    hsv::{hsv2rgb, Hsv},
    SmartLedsWrite,
};

use icm20948_async::{AccRange, AccDlp, AccUnit, GyrDlp, GyrRange, GyrUnit, IcmError, Icm20948};
use imu_fusion::{FusionMatrix, FusionVector};

const NUM_BLOCKS: usize = 2;
use core::f32::consts::PI;

mod analysis;
mod config;
mod control;
mod imu_tracker;

use crate::config::FIRMWARE_CONFIG;
use imu_tracker::ImuTracker;
use analysis::Analysis;
use control::{
    SysCommands,
    SysStates,
    MessageTopics,
    MAX_SIZE,
    MQTTMessage
};

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

static mut APP_CORE_STACK: CPUStack<10000> = CPUStack::new();


#[embassy_executor::task]
async fn motion_analysis(
    mut i2c: I2C<'static, I2C0, Async>,
    event_sender: Sender<'static, CriticalSectionRawMutex, MQTTMessage, NUM_BLOCKS>,
    mut cmd_receiver: Subscriber<'static, CriticalSectionRawMutex, SysCommands, 1, 3, 2>,
    mut flag_pin: Output<'static, GpioPin<2>>
) {
    'full: loop {
        // Create and await IMU object
        let imu_configured = Icm20948::new_i2c(&mut i2c, Delay)
            // Configure accelerometer
            .acc_range(AccRange::Gs8)
            .acc_dlp(AccDlp::Hz111)
            .acc_unit(AccUnit::Gs)
            // Configure gyroscope
            .gyr_range(GyrRange::Dps1000)
            .gyr_dlp(GyrDlp::Hz120)
            .gyr_unit(GyrUnit::Dps)
            // Final initialization
            .set_address(0x69);
        let imu_result = imu_configured.initialize_9dof().await;

        // Unpack IMU result safely and print error if necessary
        let mut imu = match imu_result {
            Ok(imu) => imu,
            Err(error) => {
                match error {
                    IcmError::BusError(_)   => log::error!("IMU_READER : IMU encountered a communication bus error"),
                    IcmError::ImuSetupError => log::error!("IMU_READER : IMU encountered an error during setup"),
                    IcmError::MagSetupError => log::error!("IMU_READER : IMU encountered an error during mag setup")
                } panic!("Could not init IMU!");
            }
        };

        // Calibrate gyroscope offsets using 100 samples
        log::info!("Calibrating gyroscopes, keep still...");
        let _gyr_cal = imu.gyr_calibrate(250).await.is_ok();
        log::info!("... done calibrating gyros.");

        // Setup motion analysis
        const IMU_SAMPLE_PERIOD: Duration = Duration::from_hz(200);

        let acc_misalignment = FusionMatrix::identity();
        let acc_offset = FusionVector::zero();
        let acc_sensitivity = FusionVector::ones();
        let gyr_offset = FusionVector::zero();
        let mut tracker = ImuTracker::new(IMU_SAMPLE_PERIOD, Instant::now(), 1000.0f32,
                                        acc_misalignment, acc_sensitivity, acc_offset, gyr_offset);
        //let mut analysis = Analysis::default();
        const DIAGONAL_BAND_DEG: f32 = 25.0;
        let mut analysis = Analysis::new(60, 30, 0.12,
                                                DIAGONAL_BAND_DEG*PI/180.0,
                                                (90.0 - DIAGONAL_BAND_DEG)*PI/180.0);
        // Main loop: reading the sensor and sending movement detection data to the broker

        // modulus to send motion direction samples at a low rate
        const DETECTION_REPORT_FREQ: Duration = Duration::from_hz(8);
        const MOD_DETECTION: u32 = (DETECTION_REPORT_FREQ.as_ticks() / IMU_SAMPLE_PERIOD.as_ticks()) as u32;

        let mut id: u32 = 0;
        'sample: loop {

            let futures = select(
                Timer::after(IMU_SAMPLE_PERIOD),
                cmd_receiver.next_message()
            ).await;
            match futures {
                Either::First(_) => {
                    id += 1;
                    let should_send_sample = id % MOD_DETECTION == 0;

                    let now = Instant::now();
                    flag_pin.set_high();
                    match imu.read_9dof().await {
                        Ok(meas) => {
                            let acc = FusionVector::new(meas.acc.x, meas.acc.y, meas.acc.z);
                            let gyr = FusionVector::new(meas.gyr.x, meas.gyr.y, meas.gyr.z);

                            // Magnetometer axes are reflected along X axis, as per the datasheet
                            let mag = FusionVector::new(meas.mag.x, -meas.mag.y, -meas.mag.z);

                            tracker.update(now, acc, gyr, mag);
                            let new_direction = analysis.add_measurement(tracker.linear_accel);
                            flag_pin.set_low();
                            if should_send_sample {
                                if let Some(dir) = new_direction {
                                    let value: u8 = 0x30 + dir.as_digit();
                                    //let mark = (id % 100) as u8;
                                    //let payload: SampleBuffer = [mark, value];
                                    let event = MQTTMessage {
                                        topic: MessageTopics::Event,
                                        payload: Vec::<u8, MAX_SIZE>::from_slice(&[value]).unwrap(),
                                    };
                                    event_sender.send(event).await;
                                }
                            }
                        },
                        Err(e) => {
                            log::error!("Reading IMU {e:?}");
                            break 'sample;
                        }
                    }
                }
                Either::Second(received_cmd) => {
                    if let WaitResult::Message(SysCommands::Restart) = received_cmd {
                        log::info!("Restarting!");
                        continue 'full;
                    }
                }
            }
        }
    }
}


#[main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    init_heap();

    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock160MHz).freeze();
    let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut rng = Rng::new(peripherals.RNG);

    let systimer = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(
        &clocks,
        make_static!(
            [
             OneShotTimer::new(systimer.alarm0.into()),
             OneShotTimer::new(systimer.alarm1.into()),
             ]
        )
    );

    // Message channel for task orchestration
    static CHANNEL_MSGS: StaticCell<PubSubChannel<CriticalSectionRawMutex, SysCommands, 1, 3, 2>> = StaticCell::new();
    let channel_evts = CHANNEL_MSGS.init(PubSubChannel::new());
    let mut pusher_msgs = channel_evts.publisher().unwrap();

    // Message channel for payload passing to MQTT pub messages
    static CHANNEL_MQTT_PUB: StaticCell<Channel<CriticalSectionRawMutex, MQTTMessage, NUM_BLOCKS>> = StaticCell::new();
    let channel_mqtt_pub = CHANNEL_MQTT_PUB.init(Channel::new());
    let sender_samples = channel_mqtt_pub.sender();
    let receiver_samples = channel_mqtt_pub.receiver();

    // Power handling via GPIO pins
    let enable_pin_out = Output::new(io.pins.gpio4, Level::High);
    let low_batt_pin_in = Input::new(io.pins.gpio3, Pull::Down);
    spawner.spawn(
        power_handling(
            channel_evts.subscriber().unwrap(),
            channel_mqtt_pub.sender(),
            enable_pin_out,
            low_batt_pin_in)
    ).ok();

    let mut flag = Output::new(io.pins.gpio2, Level::Low);
    flag.set_low();

    let led_pin = io.pins.gpio48;
    let freq = 80.MHz();
    let rmt = Rmt::new(peripherals.RMT, freq, &clocks, None).unwrap();
    let rmt_buffer = smartLedBuffer!(1);
    let led  = SmartLedsAdapter::new(rmt.channel0, led_pin, rmt_buffer, &clocks);

    // Message channel for LED setting
    static CHANNEL_LED: StaticCell<Channel<CriticalSectionRawMutex, u8, 1>> = StaticCell::new();
    let channel_led = CHANNEL_LED.init(Channel::new());
    let sender_led = channel_led.sender();

    spawner.spawn(led_driving(led, channel_led.receiver())).ok();

    // WiFi PHY start
    let seed = ((rng.random() as u64) << 32) | (rng.random() as u64);
    let timer = esp_hal::timer::PeriodicTimer::new(
        esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG1, &clocks, None)
            .timer0
            .into(),
    );
    let init = esp_wifi::initialize(
        esp_wifi::EspWifiInitFor::Wifi,
        timer,
        rng,
        peripherals.RADIO_CLK,
        &clocks,
    )
    .unwrap();
    let (wifi_interface, controller) =
        esp_wifi::wifi::new_with_mode(&init, peripherals.WIFI, WifiStaDevice).unwrap();

    // I2C to IMU start
    let sclk = io.pins.gpio8;
    let mosi = io.pins.gpio10;  // SDA on IMU board
    //let miso = io.pins.gpio7;    // SDO on IMU board
    //let cs = io.pins.gpio5;
    let i2c0 = I2C::new_async(
        peripherals.I2C0,
        mosi,
        sclk,
        400.kHz(),
        &clocks,
    );
    // Offload IMU reading and motion analysis to second core
    let msg_recv = channel_evts.subscriber().unwrap();
    let _guard = cpu_control
        .start_app_core(unsafe { &mut *addr_of_mut!(APP_CORE_STACK) }, move || {
            static EXECUTOR: StaticCell<Executor> = StaticCell::new();
            let executor = EXECUTOR.init(Executor::new());
            executor.run(|spawner| {
                spawner.spawn(motion_analysis(i2c0, sender_samples, msg_recv, flag)).unwrap();
            });
        })
        .unwrap();

    // Init network stack
    let mut dhcp_config: embassy_net::DhcpConfig = Default::default();
    dhcp_config.hostname = Some(heapless::String::from_str(FIRMWARE_CONFIG.mqtt_id).unwrap());
    let stack = &*make_static!(Stack::new(
        wifi_interface,
        Config::dhcpv4(dhcp_config),
        make_static!(StackResources::<3>::new()),
        seed
    ));

    spawner.spawn(connection(controller, channel_led.sender())).ok();
    spawner.spawn(net_task(stack)).ok();

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];

    // Outer loop that maintains WiFi connectivity
    'conn: loop {
        sender_led.send(SysStates::BringUp as u8).await;

        let futures = select(
            Timer::after(Duration::from_millis(1000)),
            stack.wait_config_up()
        ).await;
        match futures {
            Either::First(_) => {
                log::info!("Bringing network link up...");
                continue 'conn;
            }
            Either::Second(_) => {
                if let Some(config) = stack.config_v4() {
                    log::info!("Got IP: {}", config.address);
                }
            }
        }

        // Inner loop that maintains connectivity to the MQTT broker
        'mqtt: loop {
            sender_led.send(SysStates::ConnectingNet as u8).await;
            let host = match stack.dns_query(FIRMWARE_CONFIG.mqtt_host, DnsQueryType::A).await {
                Ok(r) => {
                    log::info!("DNS query response: {:?}", r);
                    r[0]
                }
                Err(e) => {
                    log::warn!("DNS query error: {:?}", e);
                    continue 'mqtt;
                    //IpAddress::from_str(FIRMWARE_CONFIG.mqtt_host).unwrap()
                }
            };
            let remote_endpoint = (
                host,
                FIRMWARE_CONFIG.mqtt_port.parse::<u16>().unwrap()
            );

            log::info!("Connecting...");
            let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
            socket.set_timeout(Some(Duration::from_secs(10)));
            Timer::after(Duration::from_millis(500)).await;
            if let Err(e) = socket.connect(remote_endpoint).await {
                log::error!("connecting: {:?}", e);
                continue 'mqtt;
            }
            log::info!("Connected!");
            sender_led.send(SysStates::ConnectingBroker as u8).await;

            let mut config = ClientConfig::new(
                rust_mqtt::client::client_config::MqttVersion::MQTTv5,
                CountingRng(20000),
            );
            const KEEP_ALIVE: u16 = 5;
            config.add_max_subscribe_qos(rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS1);
            config.add_client_id(FIRMWARE_CONFIG.mqtt_id);
            config.add_username(FIRMWARE_CONFIG.mqtt_id);
            config.add_password(FIRMWARE_CONFIG.mqtt_pass);
            config.keep_alive = KEEP_ALIVE;
            config.max_packet_size = 100;
            let mut recv_buffer = [0; 128];
            let mut write_buffer = [0; 128];
            let mut client = MqttClient::<_, 5, _>::new(
                socket,
                &mut write_buffer, 128,
                &mut recv_buffer, 128,
                config,
            );
            log::info!("Attempting broker connection...");
            'broker: while let Err(result) = client.connect_to_broker().await {
                if result == ReasonCode::Success {
                    log::info!("Connected!");
                    break 'broker;
                }
                else {
                    log::error!("Could not contact broker because {result}")
                }
                Timer::after(Duration::from_millis(500)).await;
            }
            log::info!("Connected to broker!");
            sender_led.send(SysStates::ConnectedBroker as u8).await;

            // Remote control via MQTT
            const CONTROL_TOPIC: &str = const_format::formatcp!("{}/cmd", FIRMWARE_CONFIG.mqtt_id);
            if let Err(result) = client.subscribe_to_topic(CONTROL_TOPIC).await {
                if result == ReasonCode::Success {
                    continue 'mqtt;
                }
                else {
                    log::error!("Could not subscribe because {result}")
                }
                Timer::after(Duration::from_millis(500)).await;
            }
            log::info!("Subscribed!");

            // Main loop: sending motion samples via 'client'

            // moduli to keep a healthy load for the MQTT link
            const MQTT_PING_PERIOD: Duration = Duration::from_secs(KEEP_ALIVE as u64*7/8);

            loop {
                let futures = select3(
                    receiver_samples.receive(),
                    client.receive_message(),
                    Timer::after(MQTT_PING_PERIOD),
                ).await;
                match futures {
                    Either3::First(buf) => {
                        if !publish_msg(&mut client, buf).await {
                            continue 'mqtt;
                        }
                    }
                    Either3::Second(msg) => {
                        if let Err(e) = process_mqtt_incoming(msg, &mut pusher_msgs).await {
                            log::error!("Problem receiving message: {:?}", e);
                            continue 'mqtt;
                        }
                    }
                    Either3::Third(_) => {
                        // Timeout expired: send MQTT ping!
                        if let Err(result) = client.send_ping().await {
                            if result != ReasonCode::Success {
                                log::error!("Could not send ping because {result}; Restarting connection!");
                                continue 'mqtt;
                            }
                        }
                    }
                }
            }
        }
    }
}

async fn publish_msg(
    mqtt_client: &mut MqttClient<'_, TcpSocket<'_>, 5, CountingRng>,
    msg: MQTTMessage
) -> bool
{
    let mut succeeded = true;
    let topic = msg.topic.as_str();
    // Receive a buffer from the channel
    if let Err(result) = mqtt_client
        .send_message(
            topic,
            &msg.payload,
            QualityOfService::QoS0,
            false,
        )
        .await {
        if result != ReasonCode::Success {
            log::error!("Could not publish because {result}; Restarting connection!");
            succeeded = false;
        }
    }
    println!("{}", String::from_utf8_lossy(&msg.payload));

    succeeded
}

#[embassy_executor::task]
async fn connection(
    mut controller: WifiController<'static>,
    sender_led: Sender<'static, CriticalSectionRawMutex, u8, 1>,
) {
    loop {
        if esp_wifi::wifi::get_wifi_state() == WifiState::StaConnected {
            // wait until we're no longer connected
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
        }
        // We've been disconnected
        sender_led.send(SysStates::ConnectingPhy as u8).await;
        Timer::after(Duration::from_millis(2000)).await;

        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = esp_wifi::wifi::Configuration::Client(esp_wifi::wifi::ClientConfiguration {
                ssid: FIRMWARE_CONFIG.wifi_ssid.parse().unwrap(),
                password: FIRMWARE_CONFIG.wifi_psk.parse().unwrap(),
                auth_method: AuthMethod::WPA2Personal,
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            log::info!("Starting wifi");
            controller.start().await.unwrap();
            log::info!("Wifi started!");
        }
        log::info!("About to connect...");
        sender_led.send(30).await;

        match controller.connect().await {
            Ok(_) => {
                sender_led.send(SysStates::ConnectedPhy as u8).await;
                log::info!("Wifi connected!");
            },
            Err(e) => {
                log::error!("Failed to connect to wifi: {e:?}");
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<WifiDevice<'static, WifiStaDevice>>) {
    stack.run().await
}

async fn process_mqtt_incoming<'b>(
    message_opt: Result<(&'b str, &'b [u8]), ReasonCode>,
    msg_sender: &mut Publisher<'static, CriticalSectionRawMutex, SysCommands, 1, 3, 2>,
) -> Result<(), ReasonCode>{
    match message_opt {
        Ok((topic, raw_payload)) => {
            match str::from_utf8(raw_payload) {
                Ok(payload) => {
                    log::info!("Got '{}' on '{}'", payload, topic);
                    if let Some(cmd) = dispatch_incoming_mqtt_message(topic, payload) {
                        msg_sender.publish(cmd).await;
                    }
                    else {
                        log::warn!("Unknown topic/payload!");
                    }
                }
                Err(e) => {
                    log::error!("Invalid payload: {e}");
                }
            };
            Ok(())
        }
        Err(err) => {
            Err(err)
        }
    }
}

fn dispatch_incoming_mqtt_message(
    topic: &str,
    payload: &str,
) -> Option<SysCommands> {
    let subtopic = topic.split("/").nth(1).unwrap();
    match subtopic {
        "cmd" => {
            match payload {
                "reset" => Some(SysCommands::Restart),
                "off" => Some(SysCommands::PowerOff),
                _ => None
            }
        }
        _ => None
    }
}

#[embassy_executor::task]
async fn power_handling(
    mut cmd_receiver: Subscriber<'static, CriticalSectionRawMutex, SysCommands, 1, 3, 2>,
    event_sender: Sender<'static, CriticalSectionRawMutex, MQTTMessage, NUM_BLOCKS>,
    mut enable_pin: Output<'static, GpioPin<4>>,
    mut low_battery_pin: Input<'static, GpioPin<3>>
) {
    loop {
        let futures = select(
            low_battery_pin.wait_for_rising_edge(),
            cmd_receiver.next_message()
        ).await;
        match futures {
            Either::First(_) => {
                let event = MQTTMessage {
                    topic: MessageTopics::Report,
                    payload: Vec::<u8, MAX_SIZE>::from_slice("low-batt".as_bytes()).unwrap(),
                };
                event_sender.send(event).await;
                log::warn!("Low battery detected!");
            }
            Either::Second(cmd) => {
                if let WaitResult::Message(SysCommands::PowerOff) = cmd {
                    log::warn!("Shutting down!");
                    enable_pin.set_low();
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn led_driving (
    mut led: SmartLedsAdapter<esp_hal::rmt::Channel<Blocking, 0>, 25>,
    cmd_receiver: Receiver<'static, CriticalSectionRawMutex, u8, 1>,
)
{
    let mut color = Hsv {
        hue: 0,
        sat: 255,
        val: 255,
    };
    let mut data;

    loop {
        let hue = cmd_receiver.receive().await;
        color.hue = hue;
        // Convert from the HSV color space (where we can easily transition from one
        // color to the other) to the RGB color space that we can then send to the LED
        data = [hsv2rgb(color)];
        // When sending to the LED, we do a gamma correction first (see smart_leds
        // documentation for details) and then limit the brightness to 10 out of 255 so
        // that the output it's not too bright.
        if let Err(e) = led.write(brightness(gamma(data.iter().cloned()), 20)) {
            log::error!("Driving LED: {:?}", e);
        }
    }
}
