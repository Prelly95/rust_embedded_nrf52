use std::sync::Arc;

use anyhow::Result;
use esp_idf_hal::io::Write;
use esp_idf_svc::http::server::{Configuration, EspHttpServer};
use esp_idf_svc::http::Method;

use crate::pattern::Pattern;
use crate::state::SharedState;

const INDEX_HTML: &str = include_str!("web/index.html");
/// Parsing the HTML/JS response needs more stack than the httpd default.
const STACK_SIZE: usize = 10240;

/// Starts the HTTP server and registers all routes. The returned server must be kept
/// alive (not dropped) for as long as it should keep serving requests.
pub fn start_server(state: Arc<SharedState>) -> Result<EspHttpServer<'static>> {
    let mut server = EspHttpServer::new(&Configuration {
        stack_size: STACK_SIZE,
        ..Default::default()
    })?;

    server.fn_handler::<anyhow::Error, _>("/", Method::Get, |req| {
        req.into_response(200, Some("OK"), &[("Content-Type", "text/html")])?
            .write_all(INDEX_HTML.as_bytes())
            .map_err(Into::into)
    })?;

    let state_for_get = state.clone();
    server.fn_handler::<anyhow::Error, _>("/api/state", Method::Get, move |req| {
        let body = format!(
            r#"{{"on":{},"brightness":{},"pattern":"{}","breatheMs":{},"twinkleMs":{}}}"#,
            state_for_get.is_on(),
            state_for_get.brightness_percent(),
            state_for_get.pattern().as_str(),
            state_for_get.breathe_ms(),
            state_for_get.twinkle_interval_ms(),
        );
        req.into_response(200, Some("OK"), &[("Content-Type", "application/json")])?
            .write_all(body.as_bytes())
            .map_err(Into::into)
    })?;

    let state_for_on = state.clone();
    server.fn_handler::<anyhow::Error, _>("/api/on", Method::Get, move |req| {
        state_for_on.set_on(true);
        req.into_ok_response()?.write_all(b"ok").map_err(Into::into)
    })?;

    let state_for_off = state.clone();
    server.fn_handler::<anyhow::Error, _>("/api/off", Method::Get, move |req| {
        state_for_off.set_on(false);
        req.into_ok_response()?.write_all(b"ok").map_err(Into::into)
    })?;

    let state_for_brightness = state.clone();
    server.fn_handler::<anyhow::Error, _>("/api/brightness", Method::Get, move |req| {
        if let Some(percent) = query_param(req.uri(), "value").and_then(|v| v.parse::<u8>().ok())
        {
            state_for_brightness.set_brightness_percent(percent);
        }
        req.into_ok_response()?.write_all(b"ok").map_err(Into::into)
    })?;

    let state_for_pattern = state.clone();
    server.fn_handler::<anyhow::Error, _>("/api/pattern", Method::Get, move |req| {
        if let Some(pattern) = query_param(req.uri(), "value").and_then(Pattern::from_str) {
            state_for_pattern.set_pattern(pattern);
        }
        req.into_ok_response()?.write_all(b"ok").map_err(Into::into)
    })?;

    let state_for_breathe_speed = state.clone();
    server.fn_handler::<anyhow::Error, _>("/api/breathe-speed", Method::Get, move |req| {
        if let Some(ms) = query_param(req.uri(), "value").and_then(|v| v.parse::<u16>().ok()) {
            state_for_breathe_speed.set_breathe_ms(ms);
        }
        req.into_ok_response()?.write_all(b"ok").map_err(Into::into)
    })?;

    server.fn_handler::<anyhow::Error, _>("/api/twinkle-speed", Method::Get, move |req| {
        if let Some(ms) = query_param(req.uri(), "value").and_then(|v| v.parse::<u16>().ok()) {
            state.set_twinkle_interval_ms(ms);
        }
        req.into_ok_response()?.write_all(b"ok").map_err(Into::into)
    })?;

    Ok(server)
}

/// Pulls `key=value` out of a request URI's query string. No percent-decoding since we
/// only ever send plain ASCII digits over it.
fn query_param<'a>(uri: &'a str, key: &str) -> Option<&'a str> {
    let query = uri.split_once('?')?.1;
    query.split('&').find_map(|pair| {
        let (k, v) = pair.split_once('=')?;
        (k == key).then_some(v)
    })
}
