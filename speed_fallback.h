#pragma once

#include <math.h>
#include <Arduino.h>

namespace speed_fallback {

constexpr float MISSING_SECONDS = 0.30f;
constexpr int RECOVERY_FRAMES = 3;
constexpr float MIN_TARGET_SPEED = 0.02f;

// 每侧独立保存测速状态；估算只适用于当前前进控制模式。
struct Wheel {
	bool active = false;
	bool referenceMeasured = false;
	float missingSeconds = 0;
	float learnedGain = 0; // 曾有连续有效反馈时的米/秒/PWM 比例
	float fallbackGain = 0;
	int goodFrames = 0;
	int previousPwm = 0;

	// observed 是编码器实测速度；pwm/target 对应刚结束采样窗口内的指令。
	// 单帧零值可能只是低速量化，不立刻认定掉线。
	float observe(float observed, int pwm, float target, float dt) {
		if (pwm <= 0 || target <= MIN_TARGET_SPEED || observed < 0) {
			active = false;
			missingSeconds = 0;
			goodFrames = 0;
			previousPwm = pwm;
			return observed;
		}
		if (observed > 0) {
			missingSeconds = 0;
			if (active) {
				if (++goodFrames >= RECOVERY_FRAMES) {
					active = false;
					goodFrames = 0;
				}
			} else {
				// 同一 PWM 下连续取得正轮速后才学习，减少加速过程的偏差。
				goodFrames = pwm == previousPwm ? goodFrames + 1 : 1;
				if (goodFrames >= RECOVERY_FRAMES) {
					const float gain = observed / pwm;
					learnedGain = learnedGain > 0 ? 0.8f * learnedGain + 0.2f * gain : gain;
					goodFrames = RECOVERY_FRAMES;
				}
			}
		} else {
			goodFrames = 0;
			missingSeconds = fminf(MISSING_SECONDS, missingSeconds + dt);
			if (!active && missingSeconds >= MISSING_SECONDS) {
				active = true;
				referenceMeasured = learnedGain > 0;
				// 从未有可用测速时，只能用最后目标速度/PWM 建立临时参考，
				// 不能据此证明车轮真的转动，也不是已标定的电机模型。
				fallbackGain = referenceMeasured ? learnedGain : target / pwm;
			}
		}
		previousPwm = pwm;
		return active ? pwm * fallbackGain : observed;
	}

	// fallback 不把估算速度送入 PI；使用相同参考比例直接生成开环 PWM。
	int openLoopPwm(float target, int maxPwm) const {
		if (target <= 0 || !isfinite(fallbackGain) || fallbackGain <= 0)
			return 0;
		const float pwm = target / fallbackGain;
		return static_cast<int>(lroundf(constrain(pwm, 0, maxPwm)));
	}
};

} // namespace speed_fallback
