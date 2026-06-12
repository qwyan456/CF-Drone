// 飞行控制

#include "vector.h"
#include "quaternion.h"
#include "pid.h"
#include "lpf.h"
#include "util.h"

#if WEB_RC_ENABLED
#include "control.h"
#endif

// 参数适配118mm轴距的微型四轴飞行器
// ============== 角速率环（内环）参数 ==============
#define PITCHRATE_P 0.06 // 增大P值提高响应速度
#define PITCHRATE_I 0.1 // 中等I值补偿电机差异
#define PITCHRATE_D 0.001 // 小D值抑制震荡
#define PITCHRATE_I_LIM 0.3 // 限制积分积累
#define ROLLRATE_P PITCHRATE_P // 横滚和俯仰使用相同参数
#define ROLLRATE_I PITCHRATE_I 
#define ROLLRATE_D PITCHRATE_D
#define ROLLRATE_I_LIM PITCHRATE_I_LIM
#define YAWRATE_P 0.3 // 偏航需要更高的P值（惯性较小）
#define YAWRATE_I 0.01 // 中等I值补偿
#define YAWRATE_D 0.01 // 小D值
#define YAWRATE_I_LIM 0.3
// ============== 角度环（外环）参数 ==============
#define ROLL_P 6 // 较高的P值快速响应
#define ROLL_I 0 // 外环 I 项
#define ROLL_D 0 // 角度环通常不需要D项
#define ROLL_I_LIM radians(5.0f) // 外环横滚积分限幅（rad/s），约 5°/s，防止低油门/切模式时积分发散
#define PITCH_P ROLL_P // 横滚和俯仰相同
#define PITCH_I ROLL_I
#define PITCH_D ROLL_D
#define PITCH_I_LIM ROLL_I_LIM
#define YAW_P 3 // 偏航响应稍慢

// // 参数适配50mm轴距的微型四轴飞行器
// // ============== 角速率环（内环）参数 ==============
// #define PITCHRATE_P 0.05 // 增大P值提高响应速度
// #define PITCHRATE_I 0.05 // 中等I值补偿电机差异
// #define PITCHRATE_D 0.001 // 小D值抑制震荡
// #define PITCHRATE_I_LIM 0.2 // 限制积分积累
// #define ROLLRATE_P PITCHRATE_P // 横滚和俯仰使用相同参数
// #define ROLLRATE_I PITCHRATE_I 
// #define ROLLRATE_D PITCHRATE_D
// #define ROLLRATE_I_LIM PITCHRATE_I_LIM
// #define YAWRATE_P 0.3 // 偏航需要更高的P值（惯性较小）
// #define YAWRATE_I 0.01 // 中等I值补偿
// #define YAWRATE_D 0.01 // 小D值
// #define YAWRATE_I_LIM 0.3
// // ============== 角度环（外环）参数 ==============
// #define ROLL_P 1 // 较高的P值快速响应
// #define ROLL_I 0 // 角度环通常不需要I项
// #define ROLL_D 0 // 角度环通常不需要D项
// #define ROLL_I_LIM radians(5.0f) // 外环横滚积分限幅（rad/s），约 5°/s，防止低油门/切模式时积分发散
// #define PITCH_P ROLL_P // 横滚和俯仰相同
// #define PITCH_I ROLL_I
// #define PITCH_D ROLL_D
// #define PITCH_I_LIM ROLL_I_LIM
// #define YAW_P 3 // 偏航响应稍慢

// ============== 限制值 ==============
#define PITCHRATE_MAX radians(360) // 高转速限制（1000°/s）
#define ROLLRATE_MAX radians(360)
#define YAWRATE_MAX radians(300) // 偏航转速稍低
#define TILT_MAX radians(30) // 最大倾斜角30°
#define ALTHOLD_HOVER_THRUST 0.5f   // 定高悬停基础推力（stub，待气压计实现后替换）
#define ARM_THROTTLE_LIMIT   0.05f  // 解锁油门上限（归一化后 0~1），5%，超过此值禁止解锁
#define RATES_D_LPF_ALPHA 0.2 // cutoff frequency ~ 40 Hz

float motThrMin = 0.10f;  // 推力下限（摇杆最低位的输出），可通过参数 MOT_THR_MIN 调节
float motThrMax = 0.9f;   // 推力上限（摇杆最高位的输出），可通过参数 MOT_THR_MAX 调节；保留 10% 余量供姿态修正

// ============== 每电机补偿参数 ==============
// 用于补偿马达/桨叶推力不一致：scale 缩放系数调整推力增益，offset 偏移量调整推力基线。
// 公式：motor_output = motor_mixer * scale + offset
//
// scale > 1.0 = 该电机推力偏弱需放大；scale < 1.0 = 该电机推力偏强需缩小
// offset > 0  = 增加该电机基线推力；offset < 0 = 减少基线推力
// 建议步长：scale 0.05，offset 0.01
//
// 调整方法（串口/Web 控制台）：
//   1. 悬停观察：松杆看飞机往哪偏
//   2. 单电机测试：mfl/mfr/mrl/mrr 命令观察各电机转速/声音
//   3. 缩放补偿（推力增益差异）：
//      飞机向左滚 → 左侧电机偏强 → p MOT_SCALE_FL 0.95 或 p MOT_SCALE_RL 0.95
//      飞机向右滚 → 右侧电机偏强 → p MOT_SCALE_FR 0.95 或 p MOT_SCALE_RR 0.95
//   4. 偏移补偿（推力基线差异）：
//      某电机怠速不转或转速明显低 → p MOT_OFF_xx 0.02
//   5. 组合微调直到悬停稳
//
float motScale[4] = {1.0f, 1.0f, 1.0f, 1.0f};  // 每电机推力缩放系数，1.0=无补偿
float motOffset[4] = {0.0f, 0.0f, 0.0f, 0.0f};  // 每电机推力偏移量，0=无补偿

const int RAW = 0, ACRO = 1, STAB = 2, ALTHOLD = 3, AUTO = 4; // flight modes
int mode = STAB;
bool armed = false;
int flightModes[] = {STAB, STAB, STAB}; // RC模式拨杆三挡对应的飞行模式，可通过参数 CTL_FLT_MODE_0/1/2 配置

#if WEB_RC_ENABLED
extern uint16_t webRCButtons;
#endif

PID rollRatePID(ROLLRATE_P, ROLLRATE_I, ROLLRATE_D, ROLLRATE_I_LIM, RATES_D_LPF_ALPHA);
PID pitchRatePID(PITCHRATE_P, PITCHRATE_I, PITCHRATE_D, PITCHRATE_I_LIM, RATES_D_LPF_ALPHA);
PID yawRatePID(YAWRATE_P, YAWRATE_I, YAWRATE_D);
PID rollPID(ROLL_P, ROLL_I, ROLL_D, ROLL_I_LIM);
PID pitchPID(PITCH_P, PITCH_I, PITCH_D, PITCH_I_LIM);
PID yawPID(YAW_P, 0, 0);
Vector maxRate(ROLLRATE_MAX, PITCHRATE_MAX, YAWRATE_MAX);
float tiltMax = TILT_MAX;

Quaternion attitudeTarget;
Vector ratesTarget;
Vector ratesExtra; // feedforward rates
Vector torqueTarget;
float thrustTarget;

// ============== 软件配平参数 ==============
// 用于补偿机械不对称（重心偏移、电机/桨叶推力差异、IMU 安装偏斜等）引起的固定方向漂移。
// 单位：弧度（rad）。
//
// 调整方法（串口/Web 控制台，无需重新烧录）：
//   命令格式：p <参数名> <值>
//   每次建议步长 0.005 rad（约 0.3°），逐步收敛至松杆不漂为止。
//   参数自动存储到 Flash，断电后继续生效。
//
// 漂移方向与参数对照（松杆后观察）：
//   飞机向左漂 → p CTL_TRIM_ROLL  +0.01  （正值，命令右倾以产生向右分力抵消左漂）
//   飞机向右漂 → p CTL_TRIM_ROLL  -0.01  （负值，命令左倾以产生向左分力抵消右漂）
//   飞机向前漂 → p CTL_TRIM_PITCH -0.01  （负值，命令抬头以产生向后分力抵消前漂）
//   飞机向后漂 → p CTL_TRIM_PITCH +0.01  （正值，命令低头以产生向前分力抵消后漂）
//
// 注意：电量变化、负载改变后配平值可能略有偏移，需偶尔重调。
float trimRoll  = 0.0f; // 横滚配平角（rad），参数名：CTL_TRIM_ROLL
float trimPitch = 0.0f; // 俯仰配平角（rad），参数名：CTL_TRIM_PITCH

extern const int MOTOR_REAR_LEFT, MOTOR_REAR_RIGHT, MOTOR_FRONT_RIGHT, MOTOR_FRONT_LEFT;
extern float motors[4];
extern float controlRoll, controlPitch, controlThrottle, controlYaw, controlMode;
extern float batteryVoltage;  // battery.ino

void control() {
	interpretControls();
#if WEB_RC_ENABLED
	interpretWebRC();
#endif
	failsafe();
	controlAttitude();
	controlRates();
	controlTorque();
}

void interpretControls() {
	if (controlMode < 0.25) mode = flightModes[0];
	else if (controlMode <= 0.75) mode = flightModes[1];
	else if (controlMode > 0.75) mode = flightModes[2];

	if (mode == AUTO) return; // pilot is not effective in AUTO mode

#if WEB_RC_ENABLED
	if (!isUsingWebRC()) { // SBUS手势解锁仅当WebRC未活跃时生效
#endif
	static bool armWarnNotified = false;  // 防刷屏：低电禁止解锁提示
	if (controlThrottle < 0.05 && controlYaw > 0.95) { // arm gesture
		if (batteryVoltage > VBAT_ABSENT_THRESHOLD && batteryVoltage < VBAT_WARN_THRESHOLD) {
			// L1 及以下：禁止解锁，状态变化时提示
			if (!armWarnNotified) {
				print("电量低(%.2fV)，禁止解锁\n", batteryVoltage);
#if WEB_RC_ENABLED
				char warnBuf[64];
				snprintf(warnBuf, sizeof(warnBuf), "电量低(%.2fV) 禁止解锁", batteryVoltage);
				setWebRCWarn(warnBuf);
#endif
				armWarnNotified = true;
			}
		} else {
			armed = true;
			armWarnNotified = false; // 换新电池后重置
		}
	}
	if (controlThrottle < 0.05 && controlYaw < -0.95) armed = false; // disarm gesture
#if WEB_RC_ENABLED
	}
#endif

	if (abs(controlYaw) < 0.1) controlYaw = 0; // yaw dead zone

	if (controlThrottle < 0.05f) {
		thrustTarget = 0.0f;   // 底部死区 → 怠速，PID 不运行
	} else {
		thrustTarget = mapf(controlThrottle, 0.05f, 1.0f, motThrMin, motThrMax);
	}

	if (mode == STAB) {
		float yawTarget = attitudeTarget.getYaw();
		if (!armed || invalid(yawTarget) || controlYaw != 0) yawTarget = attitude.getYaw(); // reset yaw target
		// trimRoll/trimPitch 叠加到摇杆指令上，补偿机械不对称引起的固定漂移
		// 调整方式见变量声明处注释，或通过 CLI: set CTL_TRIM_ROLL / CTL_TRIM_PITCH
		attitudeTarget = Quaternion::fromEuler(Vector(controlRoll * tiltMax + trimRoll, controlPitch * tiltMax + trimPitch, yawTarget));
		ratesExtra = Vector(0, 0, -controlYaw * maxRate.z); // positive yaw stick means clockwise rotation in FLU
	}

	if (mode == ACRO) {
		attitudeTarget.invalidate(); // skip attitude control
		ratesTarget.x = controlRoll * maxRate.x;
		ratesTarget.y = controlPitch * maxRate.y;
		ratesTarget.z = -controlYaw * maxRate.z; // positive yaw stick means clockwise rotation in FLU
	}

	if (mode == RAW) { // direct torque control
		attitudeTarget.invalidate(); // skip attitude control
		ratesTarget.invalidate(); // skip rate control
		torqueTarget = Vector(controlRoll, controlPitch, -controlYaw) * 0.1;
	}
}

void controlAttitude() {
	if (!armed || attitudeTarget.invalid() || thrustTarget < motThrMin) return; // skip attitude control

	const Vector up(0, 0, 1);
	Vector upActual = Quaternion::rotateVector(up, attitude);
	Vector upTarget = Quaternion::rotateVector(up, attitudeTarget);

	Vector error = Vector::rotationVectorBetween(upTarget, upActual);

	ratesTarget.x = rollPID.update(error.x) + ratesExtra.x;
	ratesTarget.y = pitchPID.update(error.y) + ratesExtra.y;

	float yawError = wrapAngle(attitudeTarget.getYaw() - attitude.getYaw());
	ratesTarget.z = yawPID.update(yawError) + ratesExtra.z;
}


void controlRates() {
	if (!armed || ratesTarget.invalid() || thrustTarget < motThrMin) return; // skip rates control

	Vector error = ratesTarget - rates;

	// Calculate desired torque, where 0 - no torque, 1 - maximum possible torque
	torqueTarget.x = rollRatePID.update(error.x);
	torqueTarget.y = pitchRatePID.update(error.y);
	torqueTarget.z = yawRatePID.update(error.z);
}

void controlTorque() {
	if (!torqueTarget.valid()) return; // skip torque control

	if (!armed) {
		memset(motors, 0, sizeof(motors)); // stop motors if disarmed
		return;
	}

	if (thrustTarget < motThrMin) {
		for (int i = 0; i < 4; i++) motors[i] = motThrMin; // idle thrust
		return;
	}

	motors[MOTOR_FRONT_LEFT] = thrustTarget + torqueTarget.x - torqueTarget.y + torqueTarget.z;
	motors[MOTOR_FRONT_RIGHT] = thrustTarget - torqueTarget.x - torqueTarget.y - torqueTarget.z;
	motors[MOTOR_REAR_LEFT] = thrustTarget + torqueTarget.x + torqueTarget.y - torqueTarget.z;
	motors[MOTOR_REAR_RIGHT] = thrustTarget - torqueTarget.x + torqueTarget.y + torqueTarget.z;

	// 每电机补偿（缩放+偏移），补偿马达/桨叶推力不一致
	for (int i = 0; i < 4; i++) {
		motors[i] = motors[i] * motScale[i] + motOffset[i];
	}

	desaturate(motors[MOTOR_FRONT_LEFT], motors[MOTOR_FRONT_RIGHT], motors[MOTOR_REAR_LEFT], motors[MOTOR_REAR_RIGHT]);

	motors[0] = constrain(motors[0], 0, 1);
	motors[1] = constrain(motors[1], 0, 1);
	motors[2] = constrain(motors[2], 0, 1);
	motors[3] = constrain(motors[3], 0, 1);
}

void desaturate(float& a, float& b, float& c, float& d) {
	// avg ≈ thrustTarget（力矩分量之和为零），保持 avg 不变，等比缩减力矩偏差
	float avg = (a + b + c + d) * 0.25f;
	float maxVal = max(max(a, b), max(c, d));
	float minVal = min(min(a, b), min(c, d));

	float scale = 1.0f;
	if (maxVal > 1.0f && maxVal > avg) {
		scale = min(scale, (1.0f - avg) / (maxVal - avg));
	}
	if (minVal < 0.0f && avg > minVal) {
		scale = min(scale, avg / (avg - minVal));
	}

	if (scale < 1.0f) {
		a = avg + (a - avg) * scale;
		b = avg + (b - avg) * scale;
		c = avg + (c - avg) * scale;
		d = avg + (d - avg) * scale;
	}
}

const char* getModeName() {
	switch (mode) {
		case RAW:     return "RAW";
		case ACRO:    return "ACRO";
		case STAB:    return "STAB";
		case ALTHOLD: return "ALTHOLD";
		case AUTO:    return "AUTO";
		default:      return "UNKNOWN";
	}
}

#if WEB_RC_ENABLED
// Web遥控器按钮处理（仅负责ARM/DISARM和模式切换）
void interpretWebRC() {
	if (!isUsingWebRC()) return;

	// 处理解锁/上锁按钮（上升沿检测，避免每个控制周期重复触发）
	static uint16_t lastWebRCButtons = 0;
	uint16_t risingEdge = webRCButtons & ~lastWebRCButtons;
	lastWebRCButtons = webRCButtons;

	// 处理解锁/上锁状态变化日志
	static bool lastArmedState = false;
	if (armed != lastArmedState) {
		print(armed ? "Web RC: 已解锁\n" : "Web RC: 已上锁\n");
		lastArmedState = armed;
	}

	// 按鈕。0：解锁（上升沿）
	if (risingEdge & 0x0001) {
		if (controlThrottle < ARM_THROTTLE_LIMIT) {
			armed = true;
		} else {
			setWebRCWarn("油门过高，无法解锁");
		}
	}

	// 按钮1：上锁（上升沿）
	if (risingEdge & 0x0002) {
		armed = false;
	}

	// 按钮2：急停（上升沿）
	if (risingEdge & 0x0004) {
		armed = false;
		thrustTarget = 0.0f;
	}

	// 按钮6：STAB模式（上升沿）
	if (risingEdge & 0x0040) {
		mode = STAB;
	}

	// 按钮7：ACRO模式（上升沿）
	if (risingEdge & 0x0080) {
		mode = ACRO;
	}

	// 按鈕。8：ALTHOLD定高模式（上升沿）- 暂未实现，跳过定高切到STAB，避免前端模式循环卡死
	if (risingEdge & 0x0100) {
		mode = STAB;
		setWebRCWarn("定高模式暂不支持，已切换为自稳");
	}

	// 模式切换日志
	static int lastMode = STAB;
	if (mode != lastMode) {
		print("Web RC: 模式切换到 %s\n", getModeName());
		lastMode = mode;
	}
}
#endif

// ============== 电机自动校准 ==============

// L1 地面静态校准：拆桨放桌面，逐个电机发送相同油门，用陀螺仪感受力矩差异
// 原理：四轴X型混控中，每个电机对 roll/pitch/yaw 三轴力矩的贡献是固定的。
//       给单个电机加推力 → 陀螺仪检测到角速度变化 → 可推算该电机在各轴上的实际推力效果。
//       对4个电机分别测量后，对比理论值即可算出每电机的 scale 和 offset。
//
// X型混控矩阵（电机 → 力矩）：
//   FL: +roll, -pitch, +yaw
//   FR: -roll, -pitch, -yaw
//   RL: +roll, +pitch, -yaw
//   RR: -roll, +pitch, +yaw
//
// 若某电机推力偏弱，其贡献的力矩就偏小，PID 内环 I 项会积分出补偿量。
// 我们直接测量各电机单独运行时产生的角速率响应，反推推力差异。

void motcalGround() {
	print("===== L1 地面静态校准 =====\n");
	print("请确认：已拆桨！飞机已放平在桌面上！\n");
	print("3秒后开始...\n");
	pause(3);

	// 先确保 IMU 稳定
	readIMU();
	step();
	estimate();

	const float testThrust = 0.3f;    // 测试油门（不要太高，MOSFET直驱时0.3足够感受到力矩）
	const int testDurationMs = 800;   // 每个电机测试持续毫秒数
	const int settleMs = 500;         // 电机停止后等待稳定毫秒数
	const int sampleCount = 200;      // 每个电机采样次数

	// 记录每个电机单独运行时，陀螺仪感受到的角速率积分（= 角度变化量）
	// 角速率积分 ∝ 推力产生的力矩 × 时间
	Vector gyroIntegral[4] = {Vector(0,0,0), Vector(0,0,0), Vector(0,0,0), Vector(0,0,0)};

	for (int mot = 0; mot < 4; mot++) {
		print("测试电机 %d/%d...\n", mot + 1, 4);

		// 清零所有电机
		for (int i = 0; i < 4; i++) motors[i] = 0;
		sendMotors();

		// 等待静止
		delay(settleMs);

		// 记录初始陀螺仪读数（减去静态偏置）
		readIMU();
		step();
		estimate();

		// 启动目标电机
		motors[mot] = testThrust;
		sendMotors();
		delay(50); // 等待电机启动

		// 采样陀螺仪，累加角速率
		Vector sum(0, 0, 0);
		for (int s = 0; s < sampleCount; s++) {
			readIMU();
			step();
			estimate();
			sum = sum + rates; // rates 是滤波后的角速率（已减去陀螺偏置）
			delay(1);
		}

		gyroIntegral[mot] = sum / sampleCount; // 平均角速率（rad/s）

		// 停止电机
		motors[mot] = 0;
		sendMotors();
		delay(settleMs);

		print("  电机%d: roll=%.4f pitch=%.4f yaw=%.4f rad/s\n",
			mot, gyroIntegral[mot].x, gyroIntegral[mot].y, gyroIntegral[mot].z);
	}

	// ---- 分析：用角速率响应反推 scale ----
	// 理论上，4个电机推力相同时：
	//   FL 和 RL 的 roll 响应应相同（都是+roll方向）
	//   FR 和 RR 的 roll 响应应相同（都是-roll方向）
	//   FL 和 FR 的 pitch 响应应相同（都是-pitch方向）
	//   RL 和 RR 的 pitch 响应应相同（都是+pitch方向）
	//
	// 实际做法：取每个电机 roll/pitch/yaw 三个分量的绝对值作为"实际推力效果"，
	//           与4个电机的均值比较，得出 scale。

	print("\n----- 校准结果 -----\n");

	// 计算每个电机的"推力效果"（取roll/pitch/yaw响应的平均绝对值）
	float effect[4];
	for (int i = 0; i < 4; i++) {
		effect[i] = (fabsf(gyroIntegral[i].x) + fabsf(gyroIntegral[i].y) + fabsf(gyroIntegral[i].z)) / 3.0f;
	}

	float avgEffect = (effect[0] + effect[1] + effect[2] + effect[3]) / 4.0f;

	if (avgEffect < 0.001f) {
		print("警告：陀螺仪响应过小，请确认已拆桨并放平桌面，或调高测试油门\n");
		print("校准中止。可尝试：p MOT_THR_MIN 0.2 后重试\n");
		return;
	}

	for (int i = 0; i < 4; i++) {
		// scale = 理论效果 / 实际效果 = avg / effect
		// 但我们要的是"让弱电机放大输出"，所以 scale = avg / effect
		float newScale = avgEffect / effect[i];
		// 限制 scale 在合理范围 [0.7, 1.3]
		newScale = constrain(newScale, 0.7f, 1.3f);
		motScale[i] = newScale;
		print("  电机%d: effect=%.4f scale=%.3f\n", i, effect[i], newScale);
	}

	// offset: 基于scale的启发式估算
	// 如果 scale > 1，说明电机偏弱，给一点 offset 补偿低端推力
	for (int i = 0; i < 4; i++) {
		if (motScale[i] > 1.02f) {
			motOffset[i] = (motScale[i] - 1.0f) * motThrMin * 0.5f;
		} else if (motScale[i] < 0.98f) {
			motOffset[i] = (motScale[i] - 1.0f) * motThrMin * 0.5f;
		} else {
			motOffset[i] = 0.0f;
		}
		print("  电机%d: offset=%.3f\n", i, motOffset[i]);
	}

	print("\n参数已更新，将在上锁后自动保存到 Flash。\n");
	print("如需微调：p MOT_SCALE_FL <值> / p MOT_OFF_FL <值>\n");
	print("如需重新校准：motcal ground\n");
	print("===== L1 校准完成 =====\n");
}

// L2 悬停自动校准：悬停松杆3~5秒，根据各轴PID积分量算出补偿值
// 原理：悬停松杆后，如果某方向持续漂移，PID 内环 I 项会积分出补偿量。
//       这个积分量直接反映了各轴力矩的恒定偏差，可转换为 motScale/motOffset 修正值。
//
// 注意：飞机不需要完全静止，漂一点没关系，算法用的是积分均值。

void motcalHover() {
	print("===== L2 悬停自动校准 =====\n");

	if (!armed) {
		print("错误：请先解锁并悬停后再执行此命令\n");
		return;
	}
	if (mode != STAB) {
		print("错误：请切换到 STAB（自稳）模式后再执行\n");
		return;
	}
	if (thrustTarget < motThrMin) {
		print("错误：油门过低，请先推油门悬停\n");
		return;
	}

	print("请松开摇杆，保持悬停 5 秒...\n");
	print("飞机会轻微漂移，这是正常的，算法会自动计算补偿\n");

	// 重置内环 PID 积分，从零开始记录
	rollRatePID.reset();
	pitchRatePID.reset();
	yawRatePID.reset();

	// 记录 5 秒内的 PID 积分累积
	const float duration = 5.0f;
	float startTime = t;

	// 让主循环继续运行（control() 在 loop() 中被调用），
	// 我们只记录积分量，不干预控制
	// 每秒打印进度
	float lastPrint = 0;
	while (t - startTime < duration) {
		if (t - lastPrint >= 1.0f) {
			lastPrint = t;
			print("  校准中... %.0f/%.0f 秒\n", t - startTime, duration);
		}
		delay(100);
	}

	// 读取 PID 内环积分量
	// integral 是误差×时间的累积，正值 = 实际角速率低于目标（电机推力不足导致旋转慢）
	float rollIntegral  = rollRatePID.integral;   // +roll 方向的积分（FL/RL 不足 或 FR/RR 过强）
	float pitchIntegral = pitchRatePID.integral;   // +pitch 方向的积分（RL/RR 不足 或 FL/FR 过强）
	float yawIntegral   = yawRatePID.integral;     // +yaw 方向的积分（FL/RR 不足 或 FR/RL 过强）

	print("\n----- 积分数据 -----\n");
	print("  roll I: %.4f  pitch I: %.4f  yaw I: %.4f\n", rollIntegral, pitchIntegral, yawIntegral);

	// ---- 将积分量转换为电机补偿 ----
	// 混控矩阵（力矩 → 电机）：
	//   FL = thrust + roll - pitch + yaw
	//   FR = thrust - roll - pitch - yaw
	//   RL = thrust + roll + pitch - yaw
	//   RR = thrust - roll + pitch + yaw
	//
	// 逆推：将三轴修正按混控矩阵的逆分配到各电机
	float avgThrust = thrustTarget;
	float scaleStep = 0.1f;  // 每次校准最大调整10%，防止过调

	// roll 积分 > 0：+roll 方向推力不足 → FL/RL 需增大 或 FR/RR 需减小
	float rollCorrection = constrain(rollIntegral * rollRatePID.i / avgThrust, -scaleStep, scaleStep);
	// pitch 积分 > 0：+pitch 方向推力不足 → RL/RR 需增大 或 FL/FR 需减小
	float pitchCorrection = constrain(pitchIntegral * pitchRatePID.i / avgThrust, -scaleStep, scaleStep);
	// yaw 积分 > 0：+yaw 方向推力不足 → FL/RR 需增大 或 FR/RL 需减小
	float yawCorrection = constrain(yawIntegral * yawRatePID.i / avgThrust, -scaleStep, scaleStep);

	// 按混控矩阵的逆，将三轴修正分配到各电机
	float corrections[4] = {
		+rollCorrection + pitchCorrection + yawCorrection,   // FL
		-rollCorrection + pitchCorrection - yawCorrection,   // FR
		+rollCorrection - pitchCorrection - yawCorrection,   // RL
		-rollCorrection - pitchCorrection + yawCorrection    // RR
	};

	print("\n----- 校准结果 -----\n");
	for (int i = 0; i < 4; i++) {
		motScale[i] = constrain(motScale[i] + corrections[i], 0.7f, 1.3f);
		// 同步更新 offset
		if (motScale[i] > 1.02f) {
			motOffset[i] = (motScale[i] - 1.0f) * motThrMin * 0.5f;
		} else if (motScale[i] < 0.98f) {
			motOffset[i] = (motScale[i] - 1.0f) * motThrMin * 0.5f;
		}
		print("  电机%d: scale=%.3f offset=%.3f\n", i, motScale[i], motOffset[i]);
	}

	print("\n参数已更新，上锁后自动保存到 Flash。\n");
	print("可重复执行 motcal hover 逐步收敛，直到悬停稳定。\n");
	print("===== L2 校准完成 =====\n");
}
