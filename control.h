#pragma once

#include <math.h>

#include "base.h"

#include "state.h"
#include "pure_pursuit.h"

#include "tof.h"
#include "ultrasonic.h"
#include "encoder.h"
#include "servo.h"
#include "motor.h"

#include "speed_fallback.h"

namespace control {

struct SteeringCalibrationPoint {
	float wheelDeg; // 等效前轮转角，左转为正，单位：度
	float servoDeg; // 传给 Servo::write() 的舵机指令角度，单位：度
};

// 标定时统一使用自行车模型的等效前轮转角，
// 并非阿克曼转向机构的内外侧前轮实际转角。
const SteeringCalibrationPoint steeringCalibration[] = {
	{-20.0f, 120.0}, // 右转 20 度
	{-10.0f, 100.0}, // 右转 10 度
	{  0.0f, 75.0}, // 前轮归中
	{ 10.0f, 50.0}, // 左转 10 度
	{ 20.0f, 40.0}  // 左转 20 度
};
constexpr int STEERING_POINT_COUNT = sizeof(steeringCalibration) / sizeof(steeringCalibration[0]);

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
	return fmaxf(fminf(a.servoDeg, b.servoDeg),
				 fminf(value, fmaxf(a.servoDeg, b.servoDeg)));
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
		
		if (input < fminf(x0, x1) || input > fmaxf(x0, x1))
			continue;
		
		if (input == x0) {
			output = wheelToServo ? a.servoDeg : a.wheelDeg;
			return true;
		}

		if (input == x1) {
			output = wheelToServo ? b.servoDeg : b.wheelDeg;
			return true;
		}

		if (wheelToServo) {
			output = steeringDetail::evaluate(i - 1, input, slopes);
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

// 将 BicycleResult::phi 转换为舵机角度；调用 write() 时将 servoDeg 四舍五入。
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

// Bicycle Model：前轮转向、后轮驱动，假设车轮纯滚动且无侧滑。
// 以后轴中心为参考点；距离单位为米，速度为米/秒，角度为弧度。
// 转角和偏航角速度以左转为正，车速以前进为正。
struct BicycleResult {
	float phi;       // 限幅后的等效前轮转角，不是舵机角度，单位：弧度
	float curvature; // 车辆轨迹曲率，不是道路曲率，单位：1/米
	float alpha;     // 偏航角速度，单位：弧度/秒
	float vl;        // 左后轮目标线速度，单位：米/秒
	float vr;        // 右后轮目标线速度，单位：米/秒
};

// 传入 pure_pursuit 的 SteeringResult::phi，不要传入 SteeringResult::delta。
// wheelbase = 轴距 L，trackWidth = 后轮轮距 W；maxSteer 为标定的机械转角上限。
// 输入无效时返回 false 并清零输出，调用方应在失败时停止电机。
// 本函数只计算，不直接输出 PWM 或舵机指令。
inline bool calculateBicycleModel(
		float v, float phi, float wheelbase, float trackWidth,
		float maxSteer, BicycleResult &result) {
	result = BicycleResult{};

	if (!isfinite(v) || !isfinite(phi) || !isfinite(wheelbase) ||
			!isfinite(trackWidth) || !isfinite(maxSteer) ||
			wheelbase <= 0.0f || trackWidth <= 0.0f ||
			maxSteer < 0.0f || maxSteer >= HALF_PI) {
		return false;
	}

	BicycleResult next;
	next.phi = fmaxf(-maxSteer, fminf(phi, maxSteer));
	next.curvature = tanf(next.phi) / wheelbase;
	next.alpha = v * next.curvature;

	// v = (vl + vr) / 2; alpha = (vr - vl) / W.
	// 根据限幅后的转角计算轮速，确保左右轮速度与转向一致。
	const float halfDifference = next.alpha * trackWidth * 0.5f;
	next.vl = v - halfDifference;
	next.vr = v + halfDifference;
	if (!isfinite(next.curvature) || !isfinite(next.alpha) ||
			!isfinite(next.vl) || !isfinite(next.vr)) {
		return false;
	}

	result = next;
	return true;
}

struct PoseRate {
	float xDot;     // 全局坐标系 X 方向速度，单位：米/秒
	float yDot;     // 全局坐标系 Y 方向速度，单位：米/秒
	float thetaDot; // 全局航向角变化率，单位：弧度/秒
};

// 后轴中心在全局坐标系中的位姿变化率：
// X_dot = v*cos(theta), Y_dot = v*sin(theta), theta_dot = v*tan(phi)/L.
// 此处为全局坐标和航向角，与 CarState::x（相对道路的横向偏移）
// 及 CarState::theta（相对道路的航向偏角）含义不同。
inline bool calculatePoseRate(
	float v, float heading, float phi, float wheelbase,
	float trackWidth, float maxSteer, PoseRate &rate
) {
	rate = PoseRate{};
	BicycleResult model;
	if (!isfinite(heading) ||
			!calculateBicycleModel(v, phi, wheelbase, trackWidth, maxSteer, model)) {
		return false;
	}

	rate.xDot = v * cosf(heading);
	rate.yDot = v * sinf(heading);
	rate.thetaDot = model.alpha;
	return true;
}

// 低速前进控制参数：左右位置和电机正方向需按实车确认。
constexpr bool TOF1_IS_LEFT = false;
constexpr bool MOTOR_A_IS_LEFT = false;
constexpr int LEFT_MOTOR_DIRECTION = -1, RIGHT_MOTOR_DIRECTION = +1;
constexpr uint32_t CONTROL_PERIOD_US = 60000UL;
constexpr uint32_t MAX_CONTROL_GAP_US = 200000UL;
constexpr float STOP_DISTANCE = 0.30f; // 需按实测制动距离调整
constexpr uint8_t OBSTACLE_CONFIRM_FRAMES = 3; // 连续近距离有效回波达到此帧数才停车
static_assert(OBSTACLE_CONFIRM_FRAMES > 0, "Obstacle confirmation requires at least one frame");
constexpr float RESUME_DISTANCE = 0.35f; // 恢复阈值高于停车阈值，避免反复启停
static_assert(RESUME_DISTANCE > STOP_DISTANCE, "Resume distance must exceed stop distance");
constexpr uint32_t RESTART_WAIT_US = 2000000UL; // 连续畅通 2 秒后重新采样起步
constexpr uint32_t OBSTACLE_LOCK_US = 60000000UL; // 连续受阻 60 秒后锁定停机
constexpr float MAX_TARGET_SPEED = 0.50f;
constexpr float REVERSE_SPEED_LIMIT = 0.10f; // 任一轮低于 -0.10 m/s 才判反向异常
constexpr float ESTIMATION_FALLBACK_SPEED_LIMIT = 0.20f; // 估计失败时允许简化纠偏的速度上限
constexpr float ACCELERATION = 0.15f; // m/s²
constexpr float MAX_STEER = 0.34f; // 弧度，约 19.5 度，留在当前 ±20 度标定范围内
constexpr float MAX_STEER_RATE = 30.0f * PI / 180.0f; // 前轮转角最大变化率：30 度/秒

// 启发式优化：如果持续接近道路中心，则减弱转向幅度
constexpr float CENTRE_OFFSET_LIMIT = 0.03f; // 历史样本均在中心 ±3 cm 内
constexpr float CENTRE_OFFSET_SPREAD = 0.004f; // 历史偏移极差不超过 4 mm
constexpr float CENTRE_STABLE_SECONDS = 0.5f; // 至少观察 0.5 秒，避免仅凭单帧居中
constexpr float CENTRED_CURVATURE_WEIGHT = 0.20f; // 居中稳定时保留 20% 曲率影响
constexpr float CURVATURE_FADE_SECONDS = 0.40f; // 平滑减弱时间常数
constexpr float CURVATURE_RECOVER_SECONDS = 0.15f; // 离开稳定区域时更快恢复曲率影响

// PI 增益为初始调试值，尚未按实际 5 V 电机标定；输出单位为 PWM。
constexpr float SPEED_KP = 400.0f, SPEED_KI = 100.0f;
constexpr int MAX_PWM = 150;

enum class Status {
	Stopped,
	WarmingUp,
	LowSpeed,
	Running,

	Obstacle, // 检测到障碍物，停车
	WaitingToRestart, // 障碍物已移开，正在倒计时
	
	CalibrationMissing,
	SensorFault,
	TimingFault,
	EstimationFault,
	DriveFault,
	InvalidTarget,

	ObstacleLocked // 长时间受阻：停止电机和舵机脉冲，不自动恢复
};

struct Telemetry { // 遥测
	Status status;
	state::CarState car;
	float frontDistance;
	float tofLeft = NAN, tofRight = NAN; // 实际左/右侧测距，单位：米；未读取或超时为 NAN
	float measuredLeft, measuredRight;
	float encoderLeft = NAN, encoderRight = NAN; // 原始实测值，掉线时可能为零
	bool fallbackLeft = false, fallbackRight = false;
	bool fallbackLeftMeasured = false, fallbackRightMeasured = false;
	float targetLeft, targetRight;
	float servoDegrees;
	float curvatureWeight = 1.0f;
	float appliedPhi = 0; // 限制变化率后的前轮目标转角，弧度，不是实测角度
	int pwmLeft, pwmRight;
};

namespace runtime {
static Telemetry telemetry = {};
static bool active = false;
static int sampleCount = 0;
static uint8_t obstacleFrames = 0;
static uint32_t timestamps[K] = {};
static uint32_t lastSampleUs = 0;
static uint32_t lastObstacleCheckUs = 0, clearSinceUs = 0;
static uint32_t blockedSinceUs = 0;
static bool trackingBlocked = false;
static float requestedSpeed = 0, rampedSpeed = 0, filteredKappa = 0;
static float leftIntegral = 0, rightIntegral = 0, stallSeconds = 0;
static float curvatureWeight = 1.0f, appliedPhi = 0;
static speed_fallback::Wheel fallbackLeft, fallbackRight;
static int encoderOnlySamples = 0;
}

inline const Telemetry &getTelemetry() {
	return runtime::telemetry;
}

// 停车清除积分及输出目标，保留 requestedSpeed 供障碍解除后恢复。
// 只有障碍停车允许自动恢复；手动停车和其他故障仍需显式 start()。
inline void stop(Status reason = Status::Stopped) {
	motor::stopMotors();

	runtime::active = false;
	runtime::fallbackLeft = speed_fallback::Wheel{};
	runtime::fallbackRight = speed_fallback::Wheel{};
	runtime::encoderOnlySamples = 0;
	runtime::obstacleFrames = 0;
	runtime::lastObstacleCheckUs = static_cast<uint32_t>(micros());
	runtime::clearSinceUs = 0;
	runtime::trackingBlocked = reason == Status::Obstacle;
	runtime::blockedSinceUs = runtime::lastObstacleCheckUs;
	runtime::leftIntegral = runtime::rightIntegral = runtime::rampedSpeed = 0;
	runtime::telemetry.status = reason;
	runtime::telemetry.targetLeft = runtime::telemetry.targetRight = 0;
	runtime::telemetry.pwmLeft = runtime::telemetry.pwmRight = 0;
}

enum class DriveFaultCause { EncoderSetup, SpeedRead, ReverseFeedback, NoMotion };

// 先停车再打印故障瞬间的数据，避免 stop() 清零 PWM 后丢失诊断依据。
inline void failDrive(DriveFaultCause cause) {
	const int pwmLeft = runtime::telemetry.pwmLeft;
	const int pwmRight = runtime::telemetry.pwmRight;
	const float targetLeft = runtime::telemetry.targetLeft;
	const float targetRight = runtime::telemetry.targetRight;

	stop(Status::DriveFault);

	Serial.print(F("DriveFault reason="));
	switch (cause) {
		case DriveFaultCause::EncoderSetup: Serial.print(F("ENCODER_SETUP")); break;
		case DriveFaultCause::SpeedRead: Serial.print(F("SPEED_READ_FAILED")); break;
		case DriveFaultCause::ReverseFeedback: Serial.print(F("NEGATIVE_SPEED")); break;
		case DriveFaultCause::NoMotion: Serial.print(F("NO_MOTION_2S")); break;
	}

	Serial.print(F(" vL_mps=")); Serial.print(runtime::telemetry.measuredLeft, 4);
	Serial.print(F(" vR_mps=")); Serial.print(runtime::telemetry.measuredRight, 4);
	Serial.print(F(" previous_targetL=")); Serial.print(targetLeft, 4);
	Serial.print(F(" previous_targetR=")); Serial.print(targetRight, 4);
	Serial.print(F(" previous_pwmL=")); Serial.print(pwmLeft);
	Serial.print(F(" previous_pwmR=")); Serial.println(pwmRight);
}

// 底层初始化完成后调用，目标速度单位为米/秒，仅支持前进。
// 先停车收满 K 组样本，再缓慢起步。
inline bool start(float targetSpeed) {
	stop();

	if (!isfinite(targetSpeed) || targetSpeed <= 0 || targetSpeed > MAX_TARGET_SPEED) {
		stop(Status::InvalidTarget);
		return false;
	}

	float centre, leftLimit, rightLimit;
	if (!encoder::calibrationReady()) {
		Serial.println(F("Control start: invalid encoder calibration."));
		stop(Status::CalibrationMissing);
		return false;
	}
	if (!steeringCalibrationReady()) {
		Serial.println(F("Control start: invalid steering calibration table."));
		stop(Status::CalibrationMissing);
		return false;
	}
	if (!wheelAngleToServo(0, centre)) {
		Serial.println(F("Control start: steering calibration does not cover centre."));
		stop(Status::CalibrationMissing);
		return false;
	}
	if (!wheelAngleToServo(MAX_STEER, leftLimit)
		|| !wheelAngleToServo(-MAX_STEER, rightLimit)) {
		Serial.println(F("Control start: MAX_STEER exceeds calibrated wheel angles."));
		stop(Status::CalibrationMissing);
		return false;
	}

	if (!encoder::setup()) {
		failDrive(DriveFaultCause::EncoderSetup);
		return false;
	}

	devices::tof1.setTimeout(25);
	devices::tof2.setTimeout(25);

	servo::enable(static_cast<int>(lroundf(centre)));

	encoder::resetSpeedWindow();

	runtime::telemetry = Telemetry{};
	runtime::telemetry.servoDegrees = centre;
	runtime::curvatureWeight = 1.0f;
	runtime::appliedPhi = 0;
	runtime::telemetry.status = Status::WarmingUp;
	runtime::sampleCount = 0;
	runtime::filteredKappa = runtime::stallSeconds = 0;
	runtime::lastSampleUs = static_cast<uint32_t>(micros());
	runtime::requestedSpeed = targetSpeed;
	runtime::active = true;

	return true;
}

// 条件积分抗饱和；仅驱动前进，减速时最低输出为零。
inline int speedPI(float target, float measured, float dt, float &integral) {
	if (target <= 0) {
		integral = 0;
		return 0;
	}

	const float error = target - measured;
	const float proposed = integral + SPEED_KI * error * dt;
	const float raw = SPEED_KP * error + proposed;

	if ((raw >= 0 && raw <= MAX_PWM) || (raw > MAX_PWM && error < 0) || (raw < 0 && error > 0))
		integral = proposed;

	integral = fmaxf(-MAX_PWM, fminf(integral, MAX_PWM));
	return static_cast<int>(lroundf(fmaxf(0, fminf(MAX_PWM, SPEED_KP * error + integral))));
}

// 每次 loop 调用即可，不要再插入 delay 或阻塞测试。
// 底层测距仍会阻塞，软件只能在读取返回后执行故障停车。
// 停车期间独立调度测距，不使用正常行驶的 lastSampleUs。
inline void updateObstacleRecovery() {
	const uint32_t now = static_cast<uint32_t>(micros());
	const uint32_t gap = now - runtime::lastObstacleCheckUs;
	if (gap < CONTROL_PERIOD_US) return;
	// 未持续观察到的时间不计入畅通等待时间。
	if (gap > MAX_CONTROL_GAP_US) {
		runtime::telemetry.status = Status::Obstacle;
		runtime::clearSinceUs = 0;
		runtime::trackingBlocked = false; // 未观测的间隔不计入连续受阻时间
	}
	float front;
	// 无回波按畅通处理，NAN 仅表示没有实际距离读数。
	const bool hasEcho = ultrasonic::readDistance(front, 12000UL);
	if (!hasEcho) front = NAN;
	if (hasEcho && (!isfinite(front) || front <= 0)) {
		stop(Status::SensorFault); return;
	}
	const uint32_t sampledAt = static_cast<uint32_t>(micros());
	runtime::lastObstacleCheckUs = sampledAt;
	runtime::telemetry.frontDistance = front;
	if (sampledAt - now > MAX_CONTROL_GAP_US) {
		stop(Status::TimingFault); return;
	}
	if (hasEcho && front <= RESUME_DISTANCE) {
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
		// 重新检查标定并清除历史、积分、测速窗口，再从零缓慢起步。
		start(runtime::requestedSpeed);
	}
}

// 仅凭 ToF 历史判断稳定居中，不使用可能受干扰的轮速差。
inline bool centredAndStable() {
	if (runtime::sampleCount < K) return false;
	const float span = (runtime::timestamps[K-1] - runtime::timestamps[0]) * 1e-6f;
	if (span < CENTRE_STABLE_SECONDS) return false;
	float minimum = INFINITY, maximum = -INFINITY;
	for (int i = 0; i < K; i++) {
		const float offset = (dr[i] - dl[i]) * 0.5f;
		if (!isfinite(offset) || fabsf(offset) > CENTRE_OFFSET_LIMIT) return false;
		minimum = fminf(minimum, offset);
		maximum = fmaxf(maximum, offset);
	}
	return maximum - minimum <= CENTRE_OFFSET_SPREAD;
}

inline float updateCurvatureWeight(float dt) {
	const bool stable = centredAndStable();
	const float target = stable ? CENTRED_CURVATURE_WEIGHT : 1.0f;
	const float tau = stable ? CURVATURE_FADE_SECONDS : CURVATURE_RECOVER_SECONDS;
	runtime::curvatureWeight += dt / (tau + dt) * (target - runtime::curvatureWeight);
	return runtime::curvatureWeight;
}

// 按实际经过时间限制前轮目标角变化；左右轮目标速度也必须使用该角度。
inline float limitSteeringRate(float desiredPhi, float dt) {
	const float maximumChange = MAX_STEER_RATE * dt;
	const float change = fmaxf(-maximumChange,
		fminf(desiredPhi - runtime::appliedPhi, maximumChange));
	return fmaxf(-MAX_STEER, fminf(runtime::appliedPhi + change, MAX_STEER));
}

inline void update() {
	if (runtime::telemetry.status == Status::Obstacle ||
			runtime::telemetry.status == Status::WaitingToRestart) {
		updateObstacleRecovery();
		return;
	}

	if (!runtime::active)
		return;

	const uint32_t now = static_cast<uint32_t>(micros());

	if (now - runtime::lastSampleUs < CONTROL_PERIOD_US)
		return;

	if (now - runtime::lastSampleUs > MAX_CONTROL_GAP_US) { // 寄
		stop(Status::TimingFault);
		return;
	}

	float front, first, second, speedLeft, speedRight;

	// 无回波按畅通处理，NAN 仅表示没有实际距离读数。
	const bool hasEcho = ultrasonic::readDistance(front, 12000UL);
	if (!hasEcho)
		front = NAN;
	
	if (hasEcho && (!isfinite(front) || front <= 0)) {
		stop(Status::SensorFault);
		return;
	}

	runtime::telemetry.frontDistance = front;

	if (hasEcho && front <= STOP_DISTANCE) {
		if (runtime::obstacleFrames < OBSTACLE_CONFIRM_FRAMES)
			runtime::obstacleFrames++;
		
		if (runtime::obstacleFrames >= OBSTACLE_CONFIRM_FRAMES) {
			stop(Status::Obstacle);
			return;
		}
	}
	else {
		// 远距离或无回波打断连续性，下次近距离回波从第一帧重新计数。
		runtime::obstacleFrames = 0;
	}

	const bool validTof = tof::readDistances(first, second);
	// 先缓存本次读数，故障时也能看到异常值；按实际左右输出而不是通道编号。
	runtime::telemetry.tofLeft = TOF1_IS_LEFT ? first : second;
	runtime::telemetry.tofRight = TOF1_IS_LEFT ? second : first;

	if (!validTof || !isfinite(first) || !isfinite(second) ||
			first <= 0 || second <= 0 || first > 2.0f || second > 2.0f) {
		stop(Status::SensorFault);
		return;
	}

	// 编码器接口固定返回电机 A/B；在控制入口统一转换为车身左/右。
	float speedMotorA, speedMotorB;
	const bool validSpeeds = encoder::readSpeeds(speedMotorA, speedMotorB);
	speedLeft = MOTOR_A_IS_LEFT ? speedMotorA : speedMotorB;
	speedRight = MOTOR_A_IS_LEFT ? speedMotorB : speedMotorA;
	
	if (!validSpeeds ||
			!isfinite(speedLeft) || !isfinite(speedRight)) {
		runtime::telemetry.measuredLeft = speedLeft;
		runtime::telemetry.measuredRight = speedRight;
		failDrive(DriveFaultCause::SpeedRead);
		return;
	}

	const uint32_t sampledAt = static_cast<uint32_t>(micros());
	const uint32_t elapsedUs = sampledAt - runtime::lastSampleUs;

	if (elapsedUs > MAX_CONTROL_GAP_US) {
		stop(Status::TimingFault);
		return;
	}

	const float dt = elapsedUs * 1e-6f;

	runtime::lastSampleUs = sampledAt;
	runtime::telemetry.encoderLeft = speedLeft;
	runtime::telemetry.encoderRight = speedRight;

	const bool wasFallbackLeft = runtime::fallbackLeft.active;
	const bool wasFallbackRight = runtime::fallbackRight.active;

	speedLeft = runtime::fallbackLeft.observe(speedLeft, runtime::telemetry.pwmLeft, runtime::telemetry.targetLeft, dt);
	speedRight = runtime::fallbackRight.observe(speedRight, runtime::telemetry.pwmRight, runtime::telemetry.targetRight, dt);
	
	// 恢复实测时清除该侧旧积分，避免从故障前的积分突然恢复输出。
	if (wasFallbackLeft != runtime::fallbackLeft.active)
		runtime::leftIntegral = 0;
	if (wasFallbackRight != runtime::fallbackRight.active)
		runtime::rightIntegral = 0;
	
	runtime::telemetry.fallbackLeft = runtime::fallbackLeft.active;
	runtime::telemetry.fallbackRight = runtime::fallbackRight.active;
	runtime::telemetry.fallbackLeftMeasured = runtime::fallbackLeft.referenceMeasured;
	runtime::telemetry.fallbackRightMeasured = runtime::fallbackRight.referenceMeasured;

	const bool degradedSpeed = runtime::fallbackLeft.active || runtime::fallbackRight.active ||
		runtime::fallbackLeft.missingSeconds > 0 || runtime::fallbackRight.missingSeconds > 0;
	if (degradedSpeed)
		runtime::encoderOnlySamples = 0;
	else if (runtime::encoderOnlySamples < K)
		runtime::encoderOnlySamples++;
	
	const float left = TOF1_IS_LEFT ? first : second;
	const float right = TOF1_IS_LEFT ? second : first;

	// 复用已有历史数组，最旧样本在下标 0。
	if (runtime::sampleCount == K) {
		for (int i = 1; i < K; i++) {
			dl[i - 1] = dl[i];
			dr[i - 1] = dr[i];
			vl[i - 1] = vl[i];
			vr[i - 1] = vr[i];
			runtime::timestamps[i - 1] = runtime::timestamps[i];
		}
	}
	else
		runtime::sampleCount++;
	
	const int tail = runtime::sampleCount - 1;
	dl[tail] = left;
	dr[tail] = right;
	vl[tail] = speedLeft;
	vr[tail] = speedRight;
	runtime::timestamps[tail] = sampledAt;
	runtime::telemetry.measuredLeft = speedLeft;
	runtime::telemetry.measuredRight = speedRight;
	
	if (runtime::sampleCount < K)
		return;

	state::CarState estimated = {};
	// 历史中含估算/疑似掉线数据时，不用轮速差推断道路曲率。
	const bool encoderHistoryReady = runtime::encoderOnlySamples >= K;
	const bool fullEstimate = encoderHistoryReady && state::estimateTimedState(dl, dr, vl, vr,
			runtime::timestamps, estimated);

	if (!fullEstimate) {
		// 估计失败且速度低于配置上限时，以横向偏移纠偏；偏角和曲率置零为近似。
		const float measuredSpeed = (speedLeft + speedRight) * 0.5f;

		if (encoderHistoryReady && fabsf(measuredSpeed) >= ESTIMATION_FALLBACK_SPEED_LIMIT) {
			stop(Status::EstimationFault);
			return;
		}

		estimated.x = (right - left) * 0.5f;
		estimated.v = measuredSpeed;
		estimated.alpha = encoderHistoryReady ? (speedRight - speedLeft) / W : 0;
		runtime::filteredKappa = 0;
	}
	else {
		const float weight = dt / (0.3f + dt);
		runtime::filteredKappa += weight * (estimated.kappa - runtime::filteredKappa);
	}

	runtime::telemetry.car = estimated;
	runtime::telemetry.status = fullEstimate ? Status::Running : Status::LowSpeed;

	const float curvatureWeight = updateCurvatureWeight(dt);
	const float steeringKappa = runtime::filteredKappa * curvatureWeight;

	const float lookahead = pure_pursuit::calculateLookahead(estimated.v, steeringKappa);
	const pure_pursuit::SteeringResult steering = pure_pursuit::calculateSteering(
		estimated.x, estimated.theta, steeringKappa, L, lookahead, MAX_STEER);
	
	if (!isfinite(steering.phi) || !isfinite(steering.curvature)) {
		stop(Status::EstimationFault);
		return;
	}

	runtime::rampedSpeed = fminf(runtime::requestedSpeed,
							   runtime::rampedSpeed + ACCELERATION * dt);

	BicycleResult command;
	float servoDegrees;
	const float limitedPhi = limitSteeringRate(steering.phi, dt);

	if (!calculateBicycleModel(runtime::rampedSpeed, limitedPhi, L, W, MAX_STEER, command) ||
			!wheelAngleToServo(command.phi, servoDegrees)) {
		stop(Status::EstimationFault);
		return;
	}

	// 同比限制两轮速度，保留内外轮速度比例。
	const float largest = fmaxf(command.vl, command.vr);
	if (largest > MAX_TARGET_SPEED) {
		command.vl *= MAX_TARGET_SPEED / largest;
		command.vr *= MAX_TARGET_SPEED / largest;
	}
	
	if (speedLeft < -REVERSE_SPEED_LIMIT || speedRight < -REVERSE_SPEED_LIMIT) {
		failDrive(DriveFaultCause::ReverseFeedback);
		return;
	}

	// 调试期间暂停无运动超时检查，方便拿起小车测试；需要时取消注释恢复。
	// if ((command.vl > 0.04f && speedLeft < 0.01f)
	//     || (command.vr > 0.04f && speedRight < 0.01f)) runtime::stallSeconds += dt;
	// else runtime::stallSeconds = 0;
	// if (runtime::stallSeconds > 2.0f) {
	//     failDrive(DriveFaultCause::NoMotion); return;
	// }

	const int pwmLeft = runtime::fallbackLeft.active ?
			runtime::fallbackLeft.openLoopPwm(command.vl, MAX_PWM) :
			runtime::fallbackLeft.missingSeconds > 0 ?
					runtime::telemetry.pwmLeft :
					speedPI(command.vl, speedLeft, dt, runtime::leftIntegral);
	const int pwmRight = runtime::fallbackRight.active ?
			runtime::fallbackRight.openLoopPwm(command.vr, MAX_PWM) :
			runtime::fallbackRight.missingSeconds > 0 ?
					runtime::telemetry.pwmRight :
					speedPI(command.vr, speedRight, dt, runtime::rightIntegral);
	
	servo::write(static_cast<int>(lroundf(servoDegrees)));

	runtime::appliedPhi = command.phi;
	runtime::telemetry.appliedPhi = command.phi;
	runtime::telemetry.curvatureWeight = curvatureWeight;

	const int leftOutput = pwmLeft * LEFT_MOTOR_DIRECTION;
	const int rightOutput = pwmRight * RIGHT_MOTOR_DIRECTION;
	motor::motorA(MOTOR_A_IS_LEFT ? leftOutput : rightOutput);
	motor::motorB(MOTOR_A_IS_LEFT ? rightOutput : leftOutput);

	runtime::telemetry.targetLeft = command.vl;
	runtime::telemetry.targetRight = command.vr;
	runtime::telemetry.servoDegrees = servoDegrees;
	runtime::telemetry.pwmLeft = pwmLeft;
	runtime::telemetry.pwmRight = pwmRight;
}

} // namespace control
