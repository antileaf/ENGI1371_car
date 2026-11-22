#pragma once

#include "servo.h"
#include <string.h>

namespace serial_commands {

static void (*printBootLog)() = nullptr;

// 两种模式共用一个串口接收器，避免 boot 命令与舵机角度争抢输入。
inline void update(bool allowServo) {
	static char line[16];
	static unsigned length = 0;
	static bool overflow = false;
	while (Serial.available() > 0) {
		const int c = Serial.read();
		if (c != '\r' && c != '\n') {
			if (length < sizeof(line) - 1) line[length++] = static_cast<char>(c);
			else overflow = true;
			continue;
		}
		if (!length && !overflow) continue;
		line[length] = '\0';
		if (!overflow && length == 4 && strcmp(line, "boot") == 0) {
			if (printBootLog) printBootLog();
			else Serial.println(F("Boot log unavailable."));
		}
		else {
			unsigned angle = 0;
			bool valid = allowServo && !overflow;
			for (unsigned i = 0; valid && i < length; ++i) {
				if (line[i] < '0' || line[i] > '9') valid = false;
				else {
					angle = angle * 10 + (line[i] - '0');
					valid = angle <= 180;
				}
			}
			if (valid) {
				servo::write(angle);
				Serial.print(F("servo_deg="));
				Serial.println(angle);
			}
			else Serial.println(allowServo ? F("Expected boot or servo angle 0-180.") : F("Expected boot."));
		}
		length = 0;
		overflow = false;
	}
}

} // namespace serial_commands
