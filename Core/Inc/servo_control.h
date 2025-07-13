/*
 * servo_control.h
 *
 *  Created on: Oct 11, 2024
 *      Author: MRactuator
 */

#ifndef __SERVO_CONTROL_H
#define __SERVO_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "as5048a.h"
#include "ma702.h"
#include "encoder_base.h"

typedef enum {
	forward,
	reverse,
	stop
}ControlMode;

class ServoController {
private:
    TIM_HandleTypeDef* htim_pwm;   // PWM用タイマーのハンドル
    uint32_t channel_in1;          // PWMのIN1チャンネル
    uint32_t channel_in2;          // PWMのIN2チャンネル
    float kp, ki, kd;              // PIDゲイン
    float previous_diff;
    float integral;
    float target_angle;
    float current_angle;
    float output;
    uint16_t output_limit;
    EncoderBase* encoder;
	MA702* ma702;
	float zero_position_map;

public:
    // コンストラクタ
	ServoController(
		TIM_HandleTypeDef* htim,
		uint32_t ch1,
		uint32_t ch2,
		float _kp,
		float _ki,
		float _kd,
		float _target_angle,
		uint16_t _output_limit,
		EncoderBase* _encoder,
		bool state
	);

    // PID計算とPWM出力
    void control();

	void onOffControl(ControlMode control_mode);

	void flowControl(float flow_rate, ControlMode control_mode);

    void setZeroPosition(float zero_position_value);

    // 目標角度を設定
    void setTargetAngle(float angle);

    // 現在の角度を取得
    float getCurrentAngle();

    float getOutput();
};

/*class OnOffServoController {
private:
	GPIO_TypeDef* servo_gpio1;
	uint16_t servo_pin1;
	GPIO_TypeDef* servo_gpio2;
	uint16_t servo_pin2;

public:
	OnOffServoController(GPIO_TypeDef* gpio1, uint16_t pin1, GPIO_TypeDef* gpio2, uint16_t pin2, bool state);

	void control(ControlMode control_mode);
};


class AngleSetServoController {
private:
	GPIO_TypeDef* servo_gpio1;
	uint16_t servo_pin1;
	GPIO_TypeDef* servo_gpio2;
	uint16_t servo_pin2;
	float target_angle;
    float current_angle;
	AS5048A* encoder;              // 角度センサのインスタンスを保持
	uint16_t zero_position;
	float zero_position_map;

public:
	AngleSetServoController(GPIO_TypeDef* gpio1, uint16_t pin1, GPIO_TypeDef* gpio2, uint16_t pin2, float _target_angle, AS5048A* encoder_, bool state);

	void control(ControlMode control_mode);

	void angleControl();

	void setZeroPosition();

	void setTargetAngle(float angle);

    float getCurrentAngle();
};*/


#ifdef __cplusplus
}
#endif

#endif /* INC_SERVO_CONTROL_H_ */
