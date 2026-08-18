use std::fs;
use std::path::Path;

fn main() {
    load_dotenv();
    embuild::espidf::sysenv::output();
}

/// Cargo doesn't read `.env` files itself, and an env var set inside a build script's
/// own process never reaches the `rustc` invocation that compiles this crate - only
/// `cargo:rustc-env=KEY=VALUE` lines printed to stdout do that. So this loads `.env`
/// (if present) at the crate root and re-emits each entry that way, which is what lets
/// `env!("FAIRY_LIGHTS_WIFI_SSID")` in `src/wifi.rs` pick it up from a plain
/// `cargo build` instead of requiring the vars to be exported in the shell first.
fn load_dotenv() {
    let path = Path::new(".env");
    println!("cargo:rerun-if-changed={}", path.display());

    let Ok(contents) = fs::read_to_string(path) else {
        return;
    };

    for line in contents.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let Some((key, value)) = line.split_once('=') else {
            continue;
        };
        let key = key.trim();
        let value = value.trim().trim_matches('"').trim_matches('\'');
        println!("cargo:rustc-env={key}={value}");
    }
}
