#pragma once

#include <WiFi.h>
#include <esp_http_server.h>
#include "control.h"
#include "web_debug_page.h"
#include "boot_log.h"

namespace web_debug {
struct Snapshot {
	control::Telemetry data;
	uint32_t timeMs = 0;
	bool initFailed = false;
	bool testMode = false;
};
static Snapshot latest;
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;

inline void publish(bool failed, bool testing) {
	Snapshot next;
	next.data = control::getTelemetry();
	next.timeMs = millis();
	next.initFailed = failed;
	next.testMode = testing;
	portENTER_CRITICAL(&lock);
	latest = next;
	portEXIT_CRITICAL(&lock);
}

inline esp_err_t telemetry(httpd_req_t *req) {
	Snapshot s;
	portENTER_CRITICAL(&lock);
	s = latest;
	portEXIT_CRITICAL(&lock);
	const auto &d = s.data;
	String json;
	json.reserve(400);
	json = "{\"ms\":";
	json += s.timeMs;
	json += ",\"status\":";
	json += static_cast<int>(d.status);
	json += ",\"issues\":";
	json += static_cast<int>(d.sensorIssues);
	json += ",\"mode\":";
	json += s.initFailed ? "\"failed\"" : s.testMode ? "\"test\"" : "\"control\"";
	auto field = [&](const char *key, float value) {
		json += ",\"";
		json += key;
		json += "\":";
		json += isfinite(value) ? String(value, 4) : String("null");
	};
	field("front", d.frontDistance);
	field("left", d.tofLeft);
	field("right", d.tofRight);
	field("raw", d.rawOffset);
	field("offset", d.offset);
	field("rate", d.offsetRate);
	field("p", d.pTerm);
	field("d", d.dTerm);
	field("desired", d.desiredPhi);
	field("applied", d.appliedPhi);
	field("servo", d.servoDegrees);
	field("a", d.pwmA);
	field("b", d.pwmB);
	json += '}';
	httpd_resp_set_type(req, "application/json");
	httpd_resp_set_hdr(req, "Cache-Control", "no-store");
	return httpd_resp_send(req, json.c_str(), json.length());
}

inline esp_err_t page(httpd_req_t *req) {
	httpd_resp_set_type(req, "text/html; charset=utf-8");
	return httpd_resp_send(req, dashboardPage, sizeof(dashboardPage)-1);
}

inline esp_err_t boots(httpd_req_t *req) {
	const String json = boot_log::json();
	httpd_resp_set_type(req, "application/json; charset=utf-8");
	httpd_resp_set_hdr(req, "Cache-Control", "no-store");
	return httpd_resp_send(req, json.c_str(), json.length());
}

inline void begin() {
	if (!WiFi.softAP("S3-Car-Debug", "s3carview")) {
		Serial.println("Debug Wi-Fi unavailable");
		return;
	}
	httpd_handle_t server = nullptr;
	httpd_config_t config = HTTPD_DEFAULT_CONFIG();
	config.lru_purge_enable = true;
	httpd_uri_t home = {}, api = {}, history = {};
	home.uri = "/";
	home.method = HTTP_GET;
	home.handler = page;
	api.uri = "/telemetry";
	api.method = HTTP_GET;
	api.handler = telemetry;
	history.uri = "/boots";
	history.method = HTTP_GET;
	history.handler = boots;
	if (httpd_start(&server, &config) != ESP_OK ||
	    httpd_register_uri_handler(server, &home) != ESP_OK ||
	    httpd_register_uri_handler(server, &api) != ESP_OK ||
	    httpd_register_uri_handler(server, &history) != ESP_OK) {
		if (server) httpd_stop(server);
		WiFi.softAPdisconnect(true);
		Serial.println("Debug web server unavailable");
		return;
	}
	Serial.print("Debug page: http://");
	Serial.println(WiFi.softAPIP());
}
}
