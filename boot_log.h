#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <esp_partition.h>
#include <esp_system.h>
#include <time.h>

namespace boot_log {

constexpr unsigned CAPACITY = 32; // 固定槽位，写满后覆盖最旧记录。
constexpr uint32_t FORMAT = 0x424C0001;
struct Record {
	uint64_t sequence;
	int64_t epoch; // UTC Unix 秒；0 表示系统时间未知，不使用编译时间冒充。
	uint32_t reason;
	uint32_t format;
};
static Record records[CAPACITY] = {};
static unsigned count = 0;
static const char *storageStatus = "not_started";

inline const char *reasonName(uint32_t reason) {
	switch (reason) {
		case ESP_RST_POWERON: return "POWER_ON";
		case ESP_RST_EXT: return "EXTERNAL_RESET";
		case ESP_RST_SW: return "SOFTWARE_RESET";
		case ESP_RST_PANIC: return "PANIC";
		case ESP_RST_INT_WDT: return "INTERRUPT_WATCHDOG";
		case ESP_RST_TASK_WDT: return "TASK_WATCHDOG";
		case ESP_RST_WDT: return "WATCHDOG";
		case ESP_RST_DEEPSLEEP: return "DEEP_SLEEP";
		case ESP_RST_BROWNOUT: return "BROWNOUT";
		case ESP_RST_SDIO: return "SDIO";
		default: return "OTHER_OR_UNKNOWN";
	}
}

// 仅在 setup 中调用一次，早于 Wi-Fi 和传感器初始化；模块重试不记为重启。
inline void begin() {
	static bool attempted = false;
	if (attempted) return;
	attempted = true;
	const esp_partition_t *partition = esp_partition_find_first(
		ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, "nvs");
	// 连整个共享 NVS 分区都限制在 Flash 的 5% 内，涵盖元数据和垃圾回收开销。
	// 不修改分区表，不清空已有 Wi-Fi 配置；不满足限制时停止写日志。
	if (!partition || !ESP.getFlashChipSize() || partition->size > ESP.getFlashChipSize() / 20) {
		storageStatus = "partition_exceeds_budget_or_missing";
		Serial.println(F("Boot log: NVS partition missing or exceeds 5% Flash budget"));
		return;
	}
	Preferences prefs;
	if (!prefs.begin("car_boots", false)) {
		storageStatus = "open_failed";
		Serial.println(F("Boot log: cannot open NVS"));
		return;
	}
	uint64_t latest = 0;
	for (unsigned i = 0; i < CAPACITY; ++i) {
		char key[8];
		snprintf(key, sizeof(key), "r%02u", i);
		Record record = {};
		if (prefs.getBytesLength(key) == sizeof(record) &&
			prefs.getBytes(key, &record, sizeof(record)) == sizeof(record) &&
			record.format == FORMAT && record.sequence > 0) {
			records[count++] = record;
			if (record.sequence > latest) latest = record.sequence;
		}
	}
	// 启动阶段排序，之后网页只读取缓存，不反复访问 Flash。
	for (unsigned i = 1; i < count; ++i) {
		Record record = records[i];
		unsigned j = i;
		while (j && records[j - 1].sequence > record.sequence) {
			records[j] = records[j - 1];
			--j;
		}
		records[j] = record;
	}
	const time_t now = time(nullptr);
	Record current = {latest + 1, now >= 1704067200 ? static_cast<int64_t>(now) - millis() / 1000 : 0,
		static_cast<uint32_t>(esp_reset_reason()), FORMAT};
	char key[8];
	snprintf(key, sizeof(key), "r%02u", static_cast<unsigned>((current.sequence - 1) % CAPACITY));
	// 单条记录一次提交，不另写游标，避免断电造成游标与记录不一致。
	const bool saved = prefs.putBytes(key, &current, sizeof(current)) == sizeof(current);
	prefs.end();
	storageStatus = saved ? "ok" : "write_failed";
	if (saved) {
		if (count == CAPACITY) {
			for (unsigned i = 1; i < count; ++i) records[i - 1] = records[i];
			--count;
		}
		records[count++] = current;
	}
	Serial.printf("Boot log: #%llu reason=%s (%lu) epoch=%lld saved=%s\n",
		static_cast<unsigned long long>(current.sequence), reasonName(current.reason),
		static_cast<unsigned long>(current.reason), static_cast<long long>(current.epoch), storageStatus);
}

inline void printRecords() {
	Serial.printf("Boot history: storage=%s count=%u/%u (oldest first)\n", storageStatus, count, CAPACITY);
	for (unsigned i = 0; i < count; ++i) {
		const Record &r = records[i];
		char timestamp[32] = "UNKNOWN";
		if (r.epoch) {
			const time_t epoch = static_cast<time_t>(r.epoch);
			const struct tm *utc = gmtime(&epoch);
			if (utc)
				strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S UTC", utc);
		}
		Serial.printf("#%llu time=%s reason=%s (%lu)\n",
			static_cast<unsigned long long>(r.sequence), timestamp,
			reasonName(r.reason), static_cast<unsigned long>(r.reason));
	}
	Serial.println(F("End boot history."));
}

inline String json() {
	String result;
	result.reserve(4096);
	result = "{\"storage\":\"";
	result += storageStatus;
	result += "\",\"capacity\":32,\"records\":[";
	for (unsigned i = 0; i < count; ++i) {
		const Record &r = records[i];
		char line[200], timestamp[24];
		if (r.epoch) snprintf(timestamp, sizeof(timestamp), "%lld", static_cast<long long>(r.epoch));
		else snprintf(timestamp, sizeof(timestamp), "null");
		snprintf(line, sizeof(line), "%s{\"sequence\":%llu,\"epoch_utc\":%s,\"reason_code\":%lu,\"reason\":\"%s\"}",
			i ? "," : "", static_cast<unsigned long long>(r.sequence), timestamp,
			static_cast<unsigned long>(r.reason), reasonName(r.reason));
		result += line;
	}
	result += "]}";
	return result;
}

} // namespace boot_log
