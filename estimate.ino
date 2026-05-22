// 基于陀螺仪和加速度计的姿态计算
// Attitude estimation from gyro and accelerometer

#include "quaternion.h"
#include "vector.h"
#include "lpf.h"
#include "util.h"

float accWeight = 0.003;
float levelWeight = 0.0002;
float levelMaxTilt = radians(30); // rad, level correction fades out at this tilt angle (matches TILT_MAX)
LowPassFilter<Vector> ratesFilter(0.2); // cutoff frequency ~ 40 Hz

void estimate() {
	applyGyro();
	applyAcc();
	applyLevel();
}

void applyGyro() {
	// filter gyro to get angular rates
	rates = ratesFilter.update(gyro);

	// apply rates to attitude
	attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(rates * dt));
}

void applyAcc() {
	// test should we apply accelerometer gravity correction
	float accNorm = acc.norm();
	landed = !motorsActive() && abs(accNorm - ONE_G) < ONE_G * 0.1f;

	if (!landed) return;

	// calculate accelerometer correction
	Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
	Vector correction = Vector::rotationVectorBetween(acc, up) * accWeight;

	// apply correction
	attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(correction));
}

void applyLevel() {
	if (landed) return;

	// assume the pilot keeps the drone more or less level in flight
	// weight fades to zero as tilt approaches levelMaxTilt, so correction
	// does not fight intentional large-angle flight while still suppressing
	// gyro drift during normal hover
	Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
	float tilt = acos(constrain(up.z, -1.0f, 1.0f));
	float dynamicWeight = levelWeight * constrain(1.0f - tilt / levelMaxTilt, 0.0f, 1.0f);

	Vector correction = Vector::rotationVectorBetween(Vector(0, 0, 1), up) * dynamicWeight;
	attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(correction));
}
