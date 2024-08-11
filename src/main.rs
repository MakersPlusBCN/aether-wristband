#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::{mem::MaybeUninit, ptr::addr_of_mut, str::FromStr};

use esp_backtrace as _;
use esp_hal::{
    clock::{ClockControl, CpuClock},
    cpu_control::{CpuControl, Stack as CPUStack},
    gpio::{Io, Level, Output, GpioPin},
    i2c::I2C,
    peripherals::{Peripherals, I2C0},
    prelude::*,
    rng::Rng,
    system::SystemControl,
    timer::OneShotTimer,
    Async
};

use esp_hal_embassy::Executor;
use embassy_executor::Spawner;
use embassy_time::{with_timeout, Delay, Duration, Instant, Ticker, Timer};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{Channel, Sender},
};

use esp_println::println;
use static_cell::{make_static, StaticCell};

extern crate alloc;

use esp_wifi::wifi::{
    AuthMethod, WifiController, WifiDevice, WifiEvent, WifiStaDevice, WifiState
};
use embassy_net::{tcp::TcpSocket, Config, Ipv4Address, Stack, StackResources};
use rust_mqtt::{
    client::{client::MqttClient, client_config::ClientConfig},
    packet::v5::{
        reason_codes::ReasonCode,
        publish_packet::QualityOfService,
    },
    utils::rng_generator::CountingRng,
};

use icm20948_async::{AccRange, AccDlp, AccUnit, GyrDlp, GyrRange, GyrUnit, IcmError, Icm20948};
use imu_fusion::{FusionMatrix, FusionVector};

const BLOCK_SIZE: usize = 1;
type SampleBuffer = [u8; BLOCK_SIZE];
const NUM_BLOCKS: usize = 2;
use core::f32::consts::PI;

mod analysis;
mod config;
mod imu_tracker;

use crate::config::FIRMWARE_CONFIG;
use imu_tracker::ImuTracker;
use analysis::Analysis;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

static mut APP_CORE_STACK: CPUStack<8192> = CPUStack::new();

#[embassy_executor::task]
async fn motion_analysis(
    i2c: I2C<'static, I2C0, Async>,
    sender: Sender<'static, CriticalSectionRawMutex, SampleBuffer, NUM_BLOCKS>,
    mut flag_pin: Output<'static, GpioPin<2>>
) {
    // Create and await IMU object
    let imu_configured = Icm20948::new_i2c(i2c, Delay)
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
    log::info!("IMU_READER : Reading gyroscopes, keep still");
    let _gyr_cal = imu.gyr_calibrate(100).await.is_ok();

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

    let mut ticker = Ticker::every(IMU_SAMPLE_PERIOD);
    let mut id: u32 = 0;
    loop {
        ticker.next().await;
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
                        let payload: SampleBuffer = [value];

                        sender.send(payload).await;
                    }
                }
            },
            Err(e) => {
                log::error!("Reading IMU {e:?}");
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

    let mut flag = Output::new(io.pins.gpio2, Level::Low);
    flag.set_low();

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

    // Message channel for IMU->MQTT payload passing
    static CHANNEL: StaticCell<Channel<CriticalSectionRawMutex, SampleBuffer, NUM_BLOCKS>> = StaticCell::new();
    let channel = CHANNEL.init(Channel::new());
    let sender = channel.sender();
    let receiver = channel.receiver();

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
    let _guard = cpu_control
        .start_app_core(unsafe { &mut *addr_of_mut!(APP_CORE_STACK) }, move || {
            static EXECUTOR: StaticCell<Executor> = StaticCell::new();
            let executor = EXECUTOR.init(Executor::new());
            executor.run(|spawner| {
                spawner.spawn(motion_analysis(i2c0, sender, flag)).unwrap();
            });
        })
        .unwrap();

    // Init network stack
    let stack = &*make_static!(Stack::new(
        wifi_interface,
        Config::dhcpv4(Default::default()),
        make_static!(StackResources::<3>::new()),
        seed
    ));

    spawner.spawn(connection(controller)).ok();
    spawner.spawn(net_task(stack)).ok();

    let remote_endpoint = (
        Ipv4Address::from_str(FIRMWARE_CONFIG.mqtt_host).unwrap(),
        FIRMWARE_CONFIG.mqtt_port.parse::<u16>().unwrap()
    );
    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];

    // Outer loop that maintains WiFi connectivity
    'conn: loop {
        log::info!("Bringing network link up...");
        Timer::after(Duration::from_millis(500)).await;
        if !stack.is_link_up() {
            continue 'conn;
        }

        log::info!("Waiting to get IP address...");
        'ip: loop {
            if let Some(config) = stack.config_v4() {
                log::info!("Got IP: {}", config.address);
                break 'ip;
            }
            Timer::after(Duration::from_millis(500)).await;
        }

        // Inner loop that maintains connectivity to the MQTT broker
        'mqtt: loop {
            let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
            socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));


            log::info!("Connecting...");
            Timer::after(Duration::from_millis(500)).await;
            if let Err(e) = socket.connect(remote_endpoint).await {
                log::error!("connecting: {:?}", e);
                continue 'mqtt;
            }
            log::info!("Connected!");

            let mut config = ClientConfig::new(
                rust_mqtt::client::client_config::MqttVersion::MQTTv5,
                CountingRng(20000),
            );
            const KEEP_ALIVE: u16 = 5;
            config.add_max_subscribe_qos(rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS1);
            config.add_client_id(FIRMWARE_CONFIG.mqtt_id);
            config.add_username(FIRMWARE_CONFIG.mqtt_user);
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

            // Main loop: sending motion samples via 'client'

            // moduli to keep a healthy load for the MQTT link
            const MQTT_PING_PERIOD: Duration = Duration::from_secs(KEEP_ALIVE as u64*7/8);
            const EVENT_TOPIC: &str = const_format::formatcp!("{}/event", FIRMWARE_CONFIG.mqtt_id);
            loop {
                let op = with_timeout(MQTT_PING_PERIOD,
                    async {
                        // Receive a buffer from the channel
                        let buf: SampleBuffer = receiver.receive().await;

                        if let Err(result) = client
                            .send_message(
                                EVENT_TOPIC,
                                &buf,
                                QualityOfService::QoS0,
                                false,
                            )
                            .await {
                            if result != ReasonCode::Success {
                                log::error!("Could not publish because {result}; Restarting connection!");
                                // TODO bubble the error up!
                            }
                        }
                        println!("{}", buf[0]);
                }).await;

                if op.is_err() {
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

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    log::info!("start connection task");
    log::info!("Device capabilities: {:?}", controller.get_capabilities());
    loop {
        if esp_wifi::wifi::get_wifi_state() == WifiState::StaConnected {
            // wait until we're no longer connected
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            Timer::after(Duration::from_millis(5000)).await
        }
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

        match controller.connect().await {
            Ok(_) => log::info!("Wifi connected!"),
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