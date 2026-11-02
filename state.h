#pragma once

#include "base.h"
#include <math.h>
#include <stdint.h>

namespace state {

struct CarState {
	float x; // 横向偏移
	float v;
	float theta; // 行驶方向偏角
	float kappa; // 道路曲率
	float alpha; // 车辆角速度
};

// 使用真实时间戳拟合，允许传感器阻塞造成的不等间隔采样。
// 数组按时间从旧到新排列；失败时不修改输出状态。
inline bool estimateTimedState(const float left[K], const float right[K],
		const float speedLeft[K], const float speedRight[K],
		const uint32_t timeUs[K], CarState &output) {
	if (K < 3 || W <= 0)
		return false;

	const uint32_t spanUs = timeUs[K - 1] - timeUs[0];
	if (spanUs == 0 || spanUs > 10000000UL)
		return false;

	const float span = spanUs * 1e-6f;
	
	float moments[5] = {}, rhs[3] = {};
	float sumV = 0, sumUV = 0, sumA = 0, sumUA = 0;
	for (int i = 0; i < K; i++) {
		if (!isfinite(left[i]) || !isfinite(right[i]) ||
				!isfinite(speedLeft[i]) || !isfinite(speedRight[i]))
			return false;
		
		const uint32_t offset = timeUs[i] - timeUs[0];
		if (offset > spanUs || (i > 0 && offset <= timeUs[i - 1] - timeUs[0]))
			return false;
		
		// 将时间归一化到 [-1, 0]，避免微秒量级造成矩阵病态。
		const float u = static_cast<float>(offset) / spanUs - 1.0f;
		const float x = (right[i] - left[i]) * 0.5f;
		const float v = (speedLeft[i] + speedRight[i]) * 0.5f;
		const float a = (speedRight[i] - speedLeft[i]) / W;
		float power = 1;

		for (int j = 0; j < 5; ++j) {
			moments[j] += power;
			if (j < 3) rhs[j] += power * x;
			power *= u;
		}

		sumV += v;
		sumUV += u * v;
		sumA += a;
		sumUA += u * a;
	}

	float matrix[3][4];
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++)
			matrix[i][j] = moments[i + j];
		
		matrix[i][3] = rhs[i];
	}

	// 带选主元的消元法，求 x = A + (B * u) + (C * u * u)
	for (int column = 0; column < 3; column++) {
		int pivot = column;
		for (int row = column + 1; row < 3; row++)
			if (fabsf(matrix[row][column]) > fabsf(matrix[pivot][column]))
				pivot = row;
		
		if (fabsf(matrix[pivot][column]) < 1e-6f)
			return false;
		
		for (int j = 0; j < 4; ++j) {
			float temp = matrix[column][j];
			matrix[column][j] = matrix[pivot][j];
			matrix[pivot][j] = temp;
		}

		const float divisor = matrix[column][column];

		for (int j = column; j < 4; j++)
			matrix[column][j] /= divisor;
		
		for (int row = 0; row < 3; ++row) {
			if (row == column)
				continue;
			
			const float factor = matrix[row][column];
			for (int j = column; j < 4; j++)
				matrix[row][j] -= factor * matrix[column][j];
		}
	}

	const float denominator = K * moments[2] - moments[1] * moments[1];
	if (denominator <= 1e-6f)
		return false;
	
	const float vSlope = (K * sumUV - moments[1] * sumV) / denominator;
	const float aSlope = (K * sumUA - moments[1] * sumA) / denominator;

	CarState next;
	next.x = matrix[0][3];
	next.v = (sumV - vSlope * moments[1]) / K;
	next.alpha = (sumA - aSlope * moments[1]) / K;

	if (fabsf(next.v) < 0.1f)
		return false;
	
	const float xDot = matrix[1][3] / span;
	const float xDDot = 2 * matrix[2][3] / (span * span);
	next.theta = xDot / next.v;
	next.kappa = next.alpha / next.v - xDDot / (next.v * next.v)
		+ xDot * (vSlope / span) / (next.v * next.v * next.v);
	
	if (!isfinite(next.x) || !isfinite(next.v) || !isfinite(next.alpha) ||
			!isfinite(next.theta) || !isfinite(next.kappa)) 
		return false;
	
	output = next;
	return true;
}

bool estimateState(const float dl[K], const float dr[K],
		const float vl[K], const float vr[K],
		/* int tail, */ CarState &state) {

	const float dt = DATA_SEP / 1000.0f;

	// if (dt <= 0 || W <= 0) return false;

	const float T = (K - 1) * 0.5f * dt;

	// Calculate the average of u^2
	float m2 = 0;
	for (int i = 0; i < K; i++) {
		float u = i * dt - T;
		m2 += u * u;
	}
	m2 /= K;

	float sumU2 = 0;
	float sumZ2 = 0;

	float sumX = 0, sumV = 0, sumA = 0;
	float sumUX = 0, sumZX = 0;
	float sumUV = 0, sumUA = 0;

	for (int i = 0; i < K; i++) {
		float u = i * dt - T;
		float z = u * u - m2;

		float x = (dr[i] - dl[i]) * 0.5f;
		float v = (vl[i] + vr[i]) * 0.5f;
		float a = (vr[i] - vl[i]) / W;

		sumU2 += u * u;
		sumZ2 += z * z;

		sumX += x;
		sumV += v;
		sumA += a;

		sumUX += u * x;
		sumZX += z * x;
		sumUV += u * v;
		sumUA += u * a;
	}

	// Quadratic regression for x
	float A = sumX / K;
	float B = sumUX / sumU2;
	float C = sumZX / sumZ2;

	// Linear regression for v and alpha
	float vDot = sumUV / sumU2;
	float aSlope = sumUA / sumU2;

	state.x = A + B * T + C * (T * T - m2);
	state.v = sumV / K + vDot * T;
	state.alpha = sumA/K + aSlope * T;

	float xDot = B + 2 * C * T;
	float xDDot = 2 * C;

	// Avoid division by nearly zero speed
	if (fabsf(state.v) < 0.1f)
		return false;

	state.theta = xDot / state.v;

	float v = state.v;
	state.kappa =
		state.alpha / v
		- xDDot / (v * v)
		+ xDot * vDot / (v * v * v);

	return true;
}

} // namespace state
