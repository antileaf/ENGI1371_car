"""在临时目录模拟 NVS，验证启动日志滚动和写入失败，不访问真实 Flash。"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory(prefix="car-boot-log-") as directory:
	work = Path(directory)
	(work / "Arduino.h").write_text(r'''
#pragma once
#include <cstdint>
#include <cstdio>
#include <string>
using String = std::string;
#define F(s) s
inline unsigned long millis() { return 100; }
struct { unsigned getFlashChipSize() { return 4 * 1024 * 1024; } } ESP;
struct {
 void println(const char *) {}
 template<class... Args> void printf(const char *, Args...) {}
} Serial;
''')
	(work / "Preferences.h").write_text(r'''
#pragma once
#include <map>
#include <vector>
#include <cstring>
#include <string>
static std::map<std::string, std::vector<unsigned char>> disk;
static bool failWrite = false;
static int writes = 0;
struct Preferences {
 bool begin(const char *, bool) { return true; }
 void end() {}
 size_t getBytesLength(const char *key) {
  auto it = disk.find(key); return it == disk.end() ? 0 : it->second.size();
 }
 size_t getBytes(const char *key, void *out, size_t size) {
  if (getBytesLength(key) != size) return 0;
  memcpy(out, disk[key].data(), size); return size;
 }
 size_t putBytes(const char *key, const void *data, size_t size) {
  ++writes;
  if (failWrite) return 0;
  const auto *bytes = static_cast<const unsigned char *>(data);
  disk[key] = std::vector<unsigned char>(bytes, bytes + size); return size;
 }
};
''')
	(work / "esp_partition.h").write_text(r'''
#pragma once
constexpr int ESP_PARTITION_TYPE_DATA = 1, ESP_PARTITION_SUBTYPE_DATA_NVS = 2;
struct esp_partition_t { unsigned size; } partition = {0x5000};
inline const esp_partition_t *esp_partition_find_first(int, int, const char *) { return &partition; }
''')
	(work / "esp_system.h").write_text(r'''
#pragma once
enum { ESP_RST_POWERON, ESP_RST_EXT, ESP_RST_SW, ESP_RST_PANIC, ESP_RST_INT_WDT,
 ESP_RST_TASK_WDT, ESP_RST_WDT, ESP_RST_DEEPSLEEP, ESP_RST_BROWNOUT, ESP_RST_SDIO };
inline int esp_reset_reason() { return ESP_RST_BROWNOUT; }
''')
	(work / "check.cpp").write_text(r'''
#include "boot_log.h"
#include <cassert>
int main(int argc, char **argv) {
 using namespace boot_log;
 const std::string mode = argv[1];
 if (mode == "full" || mode == "failure") {
  Preferences prefs;
  for (unsigned i = 1; i <= CAPACITY; ++i) {
   Record r = {i, 0, ESP_RST_POWERON, FORMAT};
   char key[8]; snprintf(key, sizeof(key), "r%02u", i - 1);
   prefs.putBytes(key, &r, sizeof(r));
  }
 }
 writes = 0;
 if (mode == "failure") failWrite = true;
 if (mode == "budget") partition.size = 1024 * 1024;
 begin(); begin(); // 同一次启动不能重复写入。
 if (mode == "budget") { assert(writes == 0 && count == 0); return 0; }
 assert(writes == 1);
 if (mode == "failure") {
  assert(count == CAPACITY && records[0].sequence == 1);
  assert(std::string(storageStatus) == "write_failed"); return 0;
 }
 assert(std::string(storageStatus) == "ok");
 assert(records[count - 1].reason == ESP_RST_BROWNOUT);
 if (mode == "full") {
  assert(count == CAPACITY && records[0].sequence == 2);
  assert(records[count - 1].sequence == 33 && disk.size() == CAPACITY);
  Record saved = {}; memcpy(&saved, disk["r00"].data(), sizeof(saved));
  assert(saved.sequence == 33);
 } else assert(count == 1 && records[0].sequence == 1);
 records[count - 1].epoch = 0;
 assert(json().find("\"epoch_utc\":null") != std::string::npos);
 assert(json().find("BROWNOUT") != std::string::npos);
}
''')
	executable = work / "check.exe"
	subprocess.run(["g++", "-std=c++11", "-I", str(work), "-I", str(root),
		str(work / "check.cpp"), "-o", str(executable)], check=True)
	for mode in ("empty", "full", "failure", "budget"):
		subprocess.run([str(executable), mode], check=True)
	print("Boot log: first boot, rollover, failed write, budget and unknown time checks passed")
