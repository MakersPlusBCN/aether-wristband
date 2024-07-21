#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl, system::SystemControl,
    peripherals::Peripherals,
    gpio::Io,
    rmt::Rmt,
    timer::{ErasedTimer, OneShotTimer},
    prelude::*,
};

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

use static_cell::make_static;
use embassy_net::{tcp::TcpSocket, Config, Ipv4Address, Stack, StackResources};
use esp_wifi::wifi::{
        AuthMethod, WifiController, WifiDevice, WifiEvent, WifiStaDevice, WifiState
    };

use esp_hal_smartled::{smartLedBuffer, SmartLedsAdapter};
use smart_leds::{
    brightness,
    gamma,
    hsv::{hsv2rgb, Hsv},
    SmartLedsWrite,
};

// The constant `CONFIG` is auto-generated by `toml_config`.
#[toml_cfg::toml_config]
pub struct FirmwareConfig {
    #[default("")]
    wifi_ssid: &'static str,
    #[default("")]
    wifi_psk: &'static str,
}

#[main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);

    let clocks = ClockControl::max(system.clock_control).freeze();

    let timer = esp_hal::timer::PeriodicTimer::new(
        esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG1, &clocks, None)
            .timer0
            .into(),
    );
    let init = esp_wifi::initialize(
        esp_wifi::EspWifiInitFor::Wifi,
        timer,
        esp_hal::rng::Rng::new(peripherals.RNG),
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
            [OneShotTimer::new(systimer.alarm0.into())]
        ),
    );

    // Init network stack
    let seed = 1234; // very random, very secure seed
    let stack = &*make_static!(Stack::new(
        wifi_interface,
        Config::dhcpv4(Default::default()),
        make_static!(StackResources::<3>::new()),
        seed
    ));

    spawner.spawn(connection(controller)).ok();
    spawner.spawn(net_task(stack)).ok();

    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    log::info!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            log::info!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let led_pin = io.pins.gpio48;
    let rmt = Rmt::new(peripherals.RMT, 80.MHz(), &clocks, None).unwrap();

    // We use one of the RMT channels to instantiate a `SmartLedsAdapter` which can
    // be used directly with all `smart_led` implementations
    let rmt_buffer = smartLedBuffer!(1);
    let mut led = SmartLedsAdapter::new(rmt.channel0, led_pin, rmt_buffer, &clocks);

    let mut color = Hsv {
        hue: 0,
        sat: 255,
        val: 255,
    };
    let mut data;

    spawner.spawn(print_loop()).ok();

    loop {
        // Iterate over the rainbow!
        for hue in 0..=255 {
            color.hue = hue;
            // Convert from the HSV color space (where we can easily transition from one
            // color to the other) to the RGB color space that we can then send to the LED
            data = [hsv2rgb(color)];
            // When sending to the LED, we do a gamma correction first (see smart_leds
            // documentation for details) and then limit the brightness to 10 out of 255 so
            // that the output it's not too bright.
            led.write(brightness(gamma(data.iter().cloned()), 10))
                .unwrap();
            Timer::after(Duration::from_millis(20)).await;
        }
    }
}

#[embassy_executor::task]
async fn print_loop() {
    let mut i = 0;
    loop {
        i += 1;
        log::info!("Hello world from embassy using esp-hal-async! ({i})");
        Timer::after(Duration::from_millis(1_000)).await;
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