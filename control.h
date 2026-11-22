#pragma once

#include <math.h>
#include "base.h"
#include "tof.h"
#include "ultrasonic.h"
#include "servo.h"
#include "motor.h"

namespace control {

using calibration::SteeringCalibrationPoint;
using calibration::steeringCalibration;
using calibration::STEERING_POINT_COUNT;
using calibration::TOF1_IS_LEFT;
using calibration::MOTOR_A_DIRECTION;
using calibration::MOTOR_B_DIRECTION;
using calibration::DEFAULT_PWM_A;
using calibration::DEFAULT_PWM_B;
using calibration::MAX_STEER;

// PCHIP 保形三次插值经过所有实测点，连接处一阶导数连续。
// 随着前轮向左转，舵机指令角度可以递增或递减，
// 但必须严格单调，才能进行双向转换。
inline bool steeringCalibrationReady() {
	float direction = 0.0f;
	for (int i = 0; i < STEERING_POINT_COUNT; i++) {
		const SteeringCalibrationPoint &p = steeringCalibration[i];

		if (!isfinite(p.wheelDeg) || !isfinite(p.servoDeg) ||
				p.servoDeg < 0.0f || p.servoDeg > 180.0f)
			return false;

		if (i == 0)
			continue;

		if (p.wheelDeg <= steeringCalibration[i - 1].wheelDeg)
			return false;

		const float difference = p.servoDeg - steeringCalibration[i - 1].servoDeg;
		if (difference == 0.0f)
			return false;
		
		if (i == 1)
			direction = difference;
		
		if ((difference > 0.0f) != (direction > 0.0f))
			return false;
	}

	return true;
}

namespace steeringDetail {
// 标定表已通过检查后，计算各节点共享的切线斜率。
inline void calculateSlopes(float slopes[STEERING_POINT_COUNT]) {
	float h[STEERING_POINT_COUNT - 1], secant[STEERING_POINT_COUNT - 1];

	for (int i = 0; i < STEERING_POINT_COUNT - 1; i++) {
		h[i] = steeringCalibration[i + 1].wheelDeg - steeringCalibration[i].wheelDeg;
		secant[i] = (steeringCalibration[i + 1].servoDeg - steeringCalibration[i].servoDeg) / h[i];
	}

	// 内部节点采用加权调和平均，兼容不等间距标定点。
	for (int i = 1; i < STEERING_POINT_COUNT - 1; i++) {
		const float w1 = 2 * h[i] + h[i - 1];
		const float w2 = h[i] + 2 * h[i - 1];
		slopes[i] = (w1 + w2) / (w1 / secant[i - 1] + w2 / secant[i]);
	}

	// 两端使用单边三点估计，并限制方向和幅度以保持单调。
	const int last = STEERING_POINT_COUNT - 2;
	slopes[0] = ((2 * h[0] + h[1]) * secant[0] - h[0] * secant[1]) / (h[0] + h[1]);
	slopes[STEERING_POINT_COUNT - 1] = ((2 * h[last] + h[last - 1]) *
			secant[last] - h[last] * secant[last - 1]) / (h[last] + h[last - 1]);
	
	for (int end = 0; end < 2; end++) {
		const int node = end == 0 ? 0 : STEERING_POINT_COUNT - 1;
		const float d = end == 0 ? secant[0] : secant[last];

		if (slopes[node] * d <= 0)
			slopes[node] = 0;
		else if (fabsf(slopes[node]) > 3 * fabsf(d))
			slopes[node] = 3 * d;
	}
}

// 在指定区间计算三次 Hermite 曲线；仅供已验证范围的内部调用。
inline float evaluate(int interval, float wheelDeg, const float slopes[]) {
	const SteeringCalibrationPoint &a = steeringCalibration[interval];
	const SteeringCalibrationPoint &b = steeringCalibration[interval + 1];
	const float h = b.wheelDeg - a.wheelDeg;
	const float t = (wheelDeg - a.wheelDeg) / h;
	const float change = b.servoDeg - a.servoDeg;
	const float startSlope = h * slopes[interval];
	const float endSlope = h * slopes[interval + 1];
	// 使用增量形式的霍纳求值，减少大角度基值之间相消造成的误差。
	const float value = a.servoDeg + t * (startSlope
			+ t * (3 * change - 2 * startSlope - endSlope
			+ t * (-2 * change + startSlope + endSlope)));
	// 抑制浮点舍入引起的极小越界。
	const float lower = fminf(a.servoDeg, b.servoDeg);
	const float upper = fmaxf(a.servoDeg, b.servoDeg);
	return constrain(value, lower, upper);
}
} // namespace steeringDetail

// 输入超出标定范围时返回失败，不进行外推。
// 正向使用 PCHIP，反向在同一条单调曲线上二分求解。
// 失败时输出为 NAN；调用方必须检查返回值，成功后才能使用输出。
inline bool interpolateSteering(float input, bool wheelToServo, float &output) {
	output = NAN;
	if (!isfinite(input) || !steeringCalibrationReady())
			return false;
	
	float slopes[STEERING_POINT_COUNT];
	steeringDetail::calculateSlopes(slopes);

	for (int i = 1; i < STEERING_POINT_COUNT; i++) {
		const SteeringCalibrationPoint &a = steeringCalibration[i - 1];
		const SteeringCalibrationPoint &b = steeringCalibration[i];
		const float x0 = wheelToServo ? a.wheelDeg : a.servoDeg;
		const float x1 = wheelToServo ? b.wheelDeg : b.servoDeg;
		if (input < fminf(x0, x1) || input > fmaxf(x0, x1)) continue;
		if (input == x0) { output = wheelToServo ? a.servoDeg : a.wheelDeg; return true; }
		if (input == x1) { output = wheelToServo ? b.servoDeg : b.wheelDeg; return true; }
		if (wheelToServo) {
			output = steeringDetail::evaluate(i-1, input, slopes);
		}
		else {
			float low = a.wheelDeg, high = b.wheelDeg;
			const bool increasing = b.servoDeg > a.servoDeg;
			for (int step = 0; step < 24; step++) {
				const float middle = low + (high - low) * 0.5f;
				if (middle == low || middle == high)
					break;
				
				const float value = steeringDetail::evaluate(i - 1, middle, slopes);

				if ((value < input) == increasing)
					low = middle;
				else
					high = middle;
			}

			output = low + (high - low) * 0.5f;
		}

		return true;
	}

	return false;
}

// 将 等效前轮转角 转换为舵机角度；调用 write() 时将 servoDeg 四舍五入。
inline bool wheelAngleToServo(float phiRad, float &servoDeg) {
	return interpolateSteering(phiRad * (180.0f / PI),
			true, servoDeg);
}

// 根据舵机指令估计等效前轮转角；此结果不是传感器反馈的实际转角。
inline bool servoAngleToWheel(float servoDeg, float &phiRad) {
	float wheelDeg;

	if (!interpolateSteering(servoDeg, false, wheelDeg)) {
		phiRad = NAN;
		return false;
	}

	phiRad = wheelDeg * (PI / 180.0f);

	return true;
}

constexpr uint32_t CONTROL_PERIOD_US = 60000UL;
constexpr uint32_t MAX_CONTROL_GAP_US = 200000UL;
constexpr float STOP_DISTANCE = 0.25f; // 需按实测制动距离调整
constexpr uint8_t OBSTACLE_CONFIRM_FRAMES = 3; // 连续近距离有效回波达到此帧数才停车
static_assert(OBSTACLE_CONFIRM_FRAMES > 0, "Obstacle confirmation requires at least one frame");
constexpr float RESUME_DISTANCE = 0.30f; // 恢复阈值高于停车阈值，避免反复启停
static_assert(RESUME_DISTANCE > STOP_DISTANCE, "Resume distance must exceed stop distance");
constexpr uint32_t RESTART_WAIT_US = 2000000UL; // 连续畅通 2 秒后重新采样起步
constexpr uint32_t OBSTACLE_LOCK_US = 60000000UL; // 连续受阻 60 秒后锁定停机

// PWM 表示驱动力而非实测车速；两侧独立配置前进指令绝对值。
constexpr int MAX_PWM = 160;
static_assert(calibration::MIN_DRIVE_PWM > 0 && calibration::MIN_DRIVE_PWM <= MAX_PWM,
	"Minimum drive PWM must be within the allowed drive range");
constexpr float PWM_RAMP_PER_SECOND = 40.0f;
constexpr float MAX_STEER_RATE = 45.0f * PI / 180.0f;
// P：看位置 D：看趋势
constexpr float OFFSET_KP = 2.5f, OFFSET_KD = 0.8f;
constexpr float OFFSET_DEADBAND = 0.01f; // 仅 P 项使用死区，过中心时保留 D 项
constexpr float OFFSET_FILTER_SECONDS = 0.10f, RATE_FILTER_SECONDS = 0.12f;
constexpr float MAX_OFFSET_RATE = 1.0f; // 限制突变引起的 D 项
constexpr int HISTORY_SIZE = 5;

// 保留原编号，删除已不使用的轮速相关状态。
enum class Status {
	Stopped = 0,
	WarmingUp = 1,
	Running = 3,
	Obstacle = 4,
	WaitingToRestart = 5,
	CalibrationMissing = 6,
	SensorFault = 7,
	TimingFault = 8,
	InvalidTarget = 11,
	ObstacleLocked = 12,
};

enum SensorIssue : uint8_t { ToF1Invalid = 1, ToF2Invalid = 2, FrontNoEcho = 4, FrontInvalid = 8 };

struct Telemetry {
	uint8_t sensorIssues = 0;
	Status status = Status::Stopped;
	float frontDistance = NAN, tofLeft = NAN, tofRight = NAN;
	float rawOffset = 0, offset = 0, offsetRate = 0;
	float pTerm = 0, dTerm = 0, desiredPhi = 0, appliedPhi = 0, servoDegrees = calibration::SERVO_CENTRE_DEG;
	int pwmA = 0, pwmB = 0; // PWM 绝对值，停车为零；方向仅在电机输出时应用
};

namespace runtime {
static Telemetry telemetry;
static bool active = false;
static int sampleCount = 0;
static float offsets[HISTORY_SIZE] = {};
static uint32_t timestamps[HISTORY_SIZE] = {};
static uint8_t obstacleFrames = 0;
static uint32_t lastSampleUs = 0, lastObstacleCheckUs = 0, clearSinceUs = 0, blockedSinceUs = 0;
static bool trackingBlocked = false;
static int requestedA = DEFAULT_PWM_A, requestedB = DEFAULT_PWM_B;
static float ramp = 0;
}

inline void recordTofInitFailure(bool firstReady, bool secondReady) {
	runtime::telemetry.sensorIssues = (firstReady ? 0 : ToF1Invalid) | (secondReady ? 0 : ToF2Invalid);
}

inline const Telemetry &getTelemetry() { return runtime::telemetry; }

// 仅障碍停车自动恢复，其他故障需显式 start()。
inline void stop(Status reason = Status::Stopped) {
	motor::stopMotors();
	runtime::active = false; runtime::ramp = 0; runtime::obstacleFrames = 0;
	runtime::lastObstacleCheckUs = static_cast<uint32_t>(micros());
	runtime::clearSinceUs = 0;
	runtime::trackingBlocked = reason == Status::Obstacle;
	runtime::blockedSinceUs = runtime::lastObstacleCheckUs;
	runtime::telemetry.status = reason;
	runtime::telemetry.pwmA = runtime::telemetry.pwmB = 0;
}

// 输入是前进 PWM 的绝对值，不再是 m/s。
inline bool start(int pwmA = DEFAULT_PWM_A, int pwmB = DEFAULT_PWM_B) {
	stop();
	if (pwmA <= 0 || pwmB <= 0 || pwmA > MAX_PWM || pwmB > MAX_PWM) {
		stop(Status::InvalidTarget);
		return false;
	}

	float centre, limit;
	if (!wheelAngleToServo(0, centre) || !wheelAngleToServo(MAX_STEER, limit) || !wheelAngleToServo(-MAX_STEER, limit)) {
		stop(Status::CalibrationMissing);
		return false;
	}

	devices::tof1.setTimeout(25);
	devices::tof2.setTimeout(25);

	servo::enable(static_cast<int>(lroundf(centre)));

	runtime::telemetry = Telemetry{};
	runtime::telemetry.servoDegrees = centre;
	runtime::telemetry.status = Status::WarmingUp;

	runtime::sampleCount = 0;
	runtime::requestedA = pwmA; runtime::requestedB = pwmB;
	runtime::lastSampleUs = static_cast<uint32_t>(micros());
	runtime::active = true;

	return true;
}

// A missing echo can also mean the path is outside the measurable range.
inline bool sampleFront() {
	auto &t = runtime::telemetry;
	t.sensorIssues &= ~(FrontNoEcho | FrontInvalid);
	if (!ultrasonic::readDistance(t.frontDistance, 12000UL)) {
		t.sensorIssues |= FrontNoEcho;
		return false;
	}
	constexpr float minimumEchoMetres = 0.02f;
	if (!isfinite(t.frontDistance) || t.frontDistance < minimumEchoMetres) {
		t.frontDistance = NAN;
		t.sensorIssues |= FrontInvalid;
		return false;
	}
	return true;
}

inline void updateObstacleRecovery() {
	const uint32_t now = static_cast<uint32_t>(micros());
	const uint32_t gap = now - runtime::lastObstacleCheckUs;

	if (gap < CONTROL_PERIOD_US)
		return;
	// 未持续观察到的时间不计入畅通等待时间。

	if (gap > MAX_CONTROL_GAP_US) {
		runtime::telemetry.status = Status::Obstacle;
		runtime::clearSinceUs = 0;
		runtime::trackingBlocked = false; // 未观测的间隔不计入连续受阻时间
	}

	const bool hasEcho = sampleFront();
	const uint32_t sampledAt = static_cast<uint32_t>(micros());
	runtime::lastObstacleCheckUs = sampledAt;


	if (sampledAt - now > MAX_CONTROL_GAP_US) {
		stop(Status::TimingFault);
		return;
	}

	if (runtime::telemetry.sensorIssues & FrontInvalid) {
		runtime::telemetry.status = Status::Obstacle;
		runtime::trackingBlocked = false;
		runtime::clearSinceUs = 0;
		return;
	}
	if (hasEcho && runtime::telemetry.frontDistance <= RESUME_DISTANCE) {
		runtime::telemetry.status = Status::Obstacle;
		runtime::clearSinceUs = 0;

		if (!runtime::trackingBlocked) {
			runtime::trackingBlocked = true;
			runtime::blockedSinceUs = sampledAt;
		}

		if (sampledAt - runtime::blockedSinceUs >= OBSTACLE_LOCK_US) {
			stop(Status::ObstacleLocked);
			servo::disable();
		}

		return;
	}

	// 远距离或无回波仍按畅通处理，连续受阻计时重新开始。
	runtime::trackingBlocked = false;
	if (runtime::telemetry.status == Status::Obstacle) {
		runtime::telemetry.status = Status::WaitingToRestart;
		runtime::clearSinceUs = sampledAt;
		return;
	}
	
	if (sampledAt - runtime::clearSinceUs >= RESTART_WAIT_US) {
		// 重新检查标定并清除ToF 历史，再从零缓慢起步。
		start(runtime::requestedA, runtime::requestedB);
	}
}


// 向左偏为正，前轮左转为正，反馈取负号。拟合使用真实时间间隔。
inline void observeOffset(float offset, uint32_t sampledAt, float dt) {
	using namespace runtime;
	if (sampleCount == HISTORY_SIZE) {
		for (int i = 1; i < HISTORY_SIZE; ++i) {
			offsets[i - 1] = offsets[i];
			timestamps[i - 1] = timestamps[i];
		}
	}
	else
		sampleCount++;

	offsets[sampleCount - 1] = offset;
	timestamps[sampleCount - 1] = sampledAt;
	telemetry.rawOffset = offset;

	if (sampleCount == 1)
		telemetry.offset = offset;
	else
		telemetry.offset += dt / (OFFSET_FILTER_SECONDS + dt) * (offset - telemetry.offset);

	float sumT = 0, sumX = 0, sumTT = 0, sumTX = 0;
	for (int i = 0; i < sampleCount; i++) {
		// 无符号时间差兼容 micros() 回绕。
		const float t = -static_cast<float>(sampledAt - timestamps[i]) * 1e-6f;
		sumT += t;
		sumX += offsets[i];
		sumTT += t * t;
		sumTX += t * offsets[i];
	}

	const float denominator = sampleCount * sumTT - sumT * sumT;
	const float slope = denominator > 1e-9f ? (sampleCount * sumTX - sumT * sumX) / denominator : 0;
	const float bounded = constrain(slope, -MAX_OFFSET_RATE, MAX_OFFSET_RATE);
	telemetry.offsetRate += dt / (RATE_FILTER_SECONDS + dt) * (bounded - telemetry.offsetRate);
}

inline float steeringDemand(float offset, float rate) {
	// 连续死区；不能因为位置居中而清除 D 项。
	const float error = offset > OFFSET_DEADBAND ? offset - OFFSET_DEADBAND :
	 offset < -OFFSET_DEADBAND ? offset + OFFSET_DEADBAND : 0;

	runtime::telemetry.pTerm = -OFFSET_KP * error;
	runtime::telemetry.dTerm = -OFFSET_KD * rate;

	const float desiredPhi = runtime::telemetry.pTerm + runtime::telemetry.dTerm;
	return constrain(desiredPhi, -MAX_STEER, MAX_STEER);
}

inline void update() {
	using namespace runtime;

	if (telemetry.status == Status::Obstacle ||
			telemetry.status == Status::WaitingToRestart) {
		updateObstacleRecovery();
		return;
	}

	if (!active)
		return;
	
	const uint32_t now = static_cast<uint32_t>(micros());
	if (now - lastSampleUs < CONTROL_PERIOD_US)
		return;

	if (now - lastSampleUs > MAX_CONTROL_GAP_US) {
		stop(Status::TimingFault);
		return;
	}

	float first, second;
	if (sampleFront() && telemetry.frontDistance <= STOP_DISTANCE) {
		if (++obstacleFrames >= OBSTACLE_CONFIRM_FRAMES) {
			stop(Status::Obstacle);
			return;
		}
	}
	else
		obstacleFrames = 0;
	
	tof::readDistances(first, second);
	telemetry.sensorIssues &= ~(ToF1Invalid | ToF2Invalid);
	if (!isfinite(first) || first <= 0 || first > 2) {
		telemetry.sensorIssues |= ToF1Invalid;
		first = NAN;
	}
	if (!isfinite(second) || second <= 0 || second > 2) {
		telemetry.sensorIssues |= ToF2Invalid;
		second = NAN;
	}
	telemetry.tofLeft = TOF1_IS_LEFT ? first : second;
	telemetry.tofRight = TOF1_IS_LEFT ? second : first;


	const uint32_t sampledAt = static_cast<uint32_t>(micros());
	const uint32_t elapsed = sampledAt-lastSampleUs;
	if (elapsed > MAX_CONTROL_GAP_US) {
		stop(Status::TimingFault);
		return;
	}

	const float dt = elapsed * 1e-6f;
	lastSampleUs = sampledAt;
	const bool tofFault = telemetry.sensorIssues & (ToF1Invalid | ToF2Invalid);
	if (tofFault) {
		sampleCount = 0;
		telemetry.rawOffset = telemetry.offset = telemetry.offsetRate = NAN;
		telemetry.pTerm = telemetry.dTerm = 0;
		telemetry.desiredPhi = 0;
	} else {
		if (sampleCount == 0) telemetry.offsetRate = 0;
		const float totalClearance = telemetry.tofRight + telemetry.tofLeft;
		observeOffset((telemetry.tofRight - telemetry.tofLeft) / totalClearance, sampledAt, dt);
	}
	const bool steeringReady = sampleCount == HISTORY_SIZE;
	// Never launch from rest with missing history. While moving, steer toward centre.
	if (!steeringReady && telemetry.pwmA == 0) return;
	telemetry.status = Status::Running;
	telemetry.desiredPhi = steeringReady ? steeringDemand(telemetry.offset, telemetry.offsetRate) : 0;

	const float step = MAX_STEER_RATE * dt;
	const float requestedChange = telemetry.desiredPhi - telemetry.appliedPhi;
	const float phi = telemetry.appliedPhi + constrain(requestedChange, -step, step);
	
	float degrees;
	if (!wheelAngleToServo(phi, degrees)) {
		stop(Status::CalibrationMissing);
		return;
	}
	
	servo::write(static_cast<int>(lroundf(degrees)));
	telemetry.appliedPhi = phi;
	telemetry.servoDegrees = degrees;

	// 从最低有效 PWM 起步，只对高于下限的部分做斜坡。
	const int floorPwm = calibration::MIN_DRIVE_PWM;
	const int spanA = requestedA > floorPwm ? requestedA - floorPwm : 0;
	const int spanB = requestedB > floorPwm ? requestedB - floorPwm : 0;
	const int span = spanA > spanB ? spanA : spanB;
	if (!steeringReady) ramp = 0;
	else {
		// 即使直行目标等于最低 PWM，转弯增速也要经过起步斜坡。
		const int rampSpan = span > 0 ? span : MAX_PWM - floorPwm;
		ramp = rampSpan > 0 ? fminf(1.0f, ramp + PWM_RAMP_PER_SECOND * dt / rampSpan) : 1.0f;
	}
	telemetry.pwmA = !steeringReady ? floorPwm : floorPwm + static_cast<int>(lroundf(spanA * ramp));
	telemetry.pwmB = !steeringReady ? floorPwm : floorPwm + static_cast<int>(lroundf(spanB * ramp));
	// 依据限速后的前轮指令做开环差速：左转内轮 B，右转内轮 A。
	if (steeringReady && phi != 0) {
		int &innerPwm = phi > 0 ? telemetry.pwmB : telemetry.pwmA;
		int &outerPwm = phi > 0 ? telemetry.pwmA : telemetry.pwmB;
		const float turn = fabsf(phi) / MAX_STEER;
		const float reduction = calibration::MAX_INNER_PWM_REDUCTION * turn;
		const int reducedPwm = static_cast<int>(lroundf(innerPwm * (1.0f - reduction)));
		innerPwm = reducedPwm < floorPwm ? floorPwm : reducedPwm;
		// 满舵时外轮目标为 MAX_PWM，增量与起步进度同步。
		const int outerTarget = phi > 0 ? floorPwm + spanA : floorPwm + spanB;
		const int boost = static_cast<int>(lroundf((MAX_PWM - outerTarget) * turn * ramp));
		outerPwm = constrain(outerPwm + boost, floorPwm, MAX_PWM);
	}
	motor::motorA(telemetry.pwmA * MOTOR_A_DIRECTION);
	motor::motorB(telemetry.pwmB * MOTOR_B_DIRECTION);
}

} // namespace control
