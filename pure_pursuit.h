#pragma once

const float MIN_LOOKAHEAD = 0.15f;
const float MAX_LOOKAHEAD = 0.50f;

const float PREDICTION_TIME = 0.5f;

namespace pure_pursuit {

struct SteeringResult {
	float delta;     // 车辆期望的转向角 (rad)
	float curvature; // 车辆期望的行驶轨迹曲率 (1/m)
	float phi;       // Bicycle-model 计算出的前轮中心转角 (rad)
};

float calculateLookahead(float v, float kappa) { // 自适应决定 lookahead
	const float base = 0.30f;
	const float T = 0.60f;
	const float lambda = 0.75f;
	// lambda 越大，弯道的 lookahead 越小

	float speedLookahead = base + T * fabsf(v);

	float lookahead = speedLookahead /
		(1.0f + lambda * fabsf(kappa) * speedLookahead);

	return constrain(
		lookahead,
		MIN_LOOKAHEAD,
		MAX_LOOKAHEAD
	);
}

float calculateDelta(
	float x,
	float theta,
	float kappa,
	float v,
	float lookahead
) {
	// Feedback: correct lateral and heading errors
	float deltaFB = -x / lookahead - theta;

	// Feedforward: anticipate road curvature
	float deltaFF = v * kappa * PREDICTION_TIME;

	// Final target heading relative to current heading
	float delta = deltaFB + deltaFF;

	return delta; // radians
}

SteeringResult calculateSteering(
	float x,
	float theta,
	float kappa,
	float L,
	float lookahead,
	float maxSteer
) {
	float Xt, Yt;
	float a = kappa * lookahead;

	// Target point on road centerline
	if (fabsf(a) < 0.001f) {
		Xt = lookahead;
		Yt = 0.5f * kappa * lookahead * lookahead;
	}
	else {
		Xt = sinf(a) / kappa;
		Yt = (1.0f - cosf(a)) / kappa;
	}

	// Target position relative to rear axle
	float dx = Xt;
	float dy = Yt - x;

	// Transform to vehicle coordinates
	float ct = cosf(theta);
	float st = sinf(theta);

	float forward = dx * ct + dy * st;
	float lateral = dy * ct - dx * st;

	float D2 = forward * forward + lateral * lateral;

	SteeringResult result;

	// Desired heading relative to current heading
	result.delta = atan2f(lateral, forward);

	// Pure Pursuit curvature
	result.curvature = 2.0f * lateral / D2;

	// Bicycle model
	float phi = atanf(L * result.curvature);

	result.phi = constrain(
		phi,
		-maxSteer,
		maxSteer
	);

	return result;
}

}
