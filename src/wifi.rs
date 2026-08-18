use anyhow::Result;
use esp_idf_hal::modem::Modem;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::{AuthMethod, BlockingWifi, ClientConfiguration, Configuration, EspWifi};

/// Set at build time so credentials never land in source control, e.g.
/// `FAIRY_LIGHTS_WIFI_SSID="Home Wifi" FAIRY_LIGHTS_WIFI_PASSWORD=some-password cargo build`.
const WIFI_SSID: &str = env!("FAIRY_LIGHTS_WIFI_SSID");
const WIFI_PASSWORD: &str = env!("FAIRY_LIGHTS_WIFI_PASSWORD");

/// Association with the AP occasionally fails on the first try (weak signal, a busy
/// channel, router taking a moment to respond, ...). Each attempt below waits out
/// esp-idf-svc's own ~15s connect timeout before we retrigger association, so this
/// gives Wi-Fi up to a couple of minutes to come up before we give up entirely.
const CONNECT_ATTEMPTS: u32 = 5;

/// Joins your existing Wi-Fi network so the lights can be controlled from any phone
/// already on that network, without switching networks first.
pub fn connect(
    modem: Modem<'static>,
    sysloop: EspSystemEventLoop,
    nvs: EspDefaultNvsPartition,
) -> Result<BlockingWifi<EspWifi<'static>>> {
    let mut wifi = BlockingWifi::wrap(EspWifi::new(modem, sysloop.clone(), Some(nvs))?, sysloop)?;

    wifi.set_configuration(&Configuration::Client(ClientConfiguration {
        ssid: WIFI_SSID.try_into().unwrap(),
        auth_method: AuthMethod::WPA2Personal,
        password: WIFI_PASSWORD.try_into().unwrap(),
        ..Default::default()
    }))?;

    wifi.start()?;

    for attempt in 1..=CONNECT_ATTEMPTS {
        match wifi.connect() {
            Ok(()) => break,
            Err(err) if attempt < CONNECT_ATTEMPTS => {
                log::warn!(
                    "Wi-Fi connect attempt {attempt}/{CONNECT_ATTEMPTS} to \"{WIFI_SSID}\" failed: {err}; retrying"
                );
                wifi.disconnect()?;
            }
            Err(err) => return Err(err.into()),
        }
    }

    wifi.wait_netif_up()?;

    let ip = wifi.wifi().sta_netif().get_ip_info()?.ip;
    log::info!("Connected to \"{WIFI_SSID}\" - browse to http://{ip}/");

    Ok(wifi)
}
