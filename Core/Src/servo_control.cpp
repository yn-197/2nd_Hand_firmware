#include "servo_control.h"

ServoController::ServoController(
	TIM_HandleTypeDef* htim, uint32_t ch1, uint32_t ch2,
	float _kp, float _ki, float _kd, float _target_angle,
	uint16_t _output_limit, EncoderBase* _encoder, bool state, float _scale
) {
	if (state == 0) {
		//pwm用設定
		channel_in1 = ch1;
		channel_in2 = ch2;
	} else {
		channel_in1 = ch2;
		channel_in2 = ch1;
	}
	
	htim_pwm = htim;
	kp = _kp * scale; //Pのズレを修正
	ki = _ki;
	kd = _kd;
	previous_diff = 0.0f;
	integral = 0.0f;
	target_angle = _target_angle;
	current_angle = 0.0f;
	output = 0.0f;
	output_limit = _output_limit;
	encoder = _encoder;
	zero_position_map = 0.0f;
	scale = _scale;
}

void ServoController::control(){
	current_angle = encoder->normalize(encoder->read2angle(encoder->getRawRotation()) - zero_position_map);

	float diff = target_angle - current_angle;

	integral += diff;
	float derivative = diff - previous_diff;
	previous_diff = diff;
	output = kp * diff + ki * integral + kd * derivative;
	if(output >= output_limit){
		output = output_limit;
	}else if(output <= -output_limit){
		output = -output_limit;
	}

	if(output > 0){
		__HAL_TIM_SET_COMPARE(htim_pwm, channel_in1, static_cast<uint32_t>(output));
		__HAL_TIM_SET_COMPARE(htim_pwm, channel_in2, 0);
	}else {
		__HAL_TIM_SET_COMPARE(htim_pwm, channel_in1, 0);
		__HAL_TIM_SET_COMPARE(htim_pwm, channel_in2, static_cast<uint32_t>(-output));
	}
}

void ServoController::onOffControl(ControlMode control_mode)
{
    switch(control_mode){
	case forward:
		__HAL_TIM_SET_COMPARE(htim_pwm, channel_in1, static_cast<uint32_t>(output_limit));
		__HAL_TIM_SET_COMPARE(htim_pwm, channel_in2, 0);
		break;
	case reverse:
		__HAL_TIM_SET_COMPARE(htim_pwm, channel_in1, 0);
		__HAL_TIM_SET_COMPARE(htim_pwm, channel_in2, static_cast<uint32_t>(output_limit));
		break;
	case stop:
		__HAL_TIM_SET_COMPARE(htim_pwm, channel_in1, 0);
		__HAL_TIM_SET_COMPARE(htim_pwm, channel_in2, 0);
		break;
	default:
		break;
	}
}

void ServoController::flowControl(float flow_rate, ControlMode control_mode)
{
    switch(control_mode){
	case forward:
		__HAL_TIM_SET_COMPARE(htim_pwm, channel_in1, static_cast<uint32_t>(flow_rate));
		__HAL_TIM_SET_COMPARE(htim_pwm, channel_in2, 0);
		break;
	case reverse:
		__HAL_TIM_SET_COMPARE(htim_pwm, channel_in1, 0);
		__HAL_TIM_SET_COMPARE(htim_pwm, channel_in2, static_cast<uint32_t>(flow_rate));
		break;
	case stop:
		__HAL_TIM_SET_COMPARE(htim_pwm, channel_in1, 0);
		__HAL_TIM_SET_COMPARE(htim_pwm, channel_in2, 0);
		break;
	default:
		break;
	}
}

void ServoController::setZeroPosition(float zero_position_value){
	zero_position_map = zero_position_value;
}

void ServoController::setTargetAngle(float angle){
	angle *= scale; // スケールを適用
	if(angle < 0) angle = 0;
	if(angle > 180) angle = 180;
	target_angle = angle;
}

float ServoController::getCurrentAngle(){
	return current_angle;
}

float ServoController::getOutput(){
	return output;
}

/*
//OnOff servo control
OnOffServoController::OnOffServoController(GPIO_TypeDef* gpio1, uint16_t pin1, GPIO_TypeDef* gpio2, uint16_t pin2, bool state){
	if(state == 0){
		servo_gpio1 = gpio1;
		servo_pin1 = pin1;
		servo_gpio2 = gpio2;
		servo_pin2 = pin2;
	}else{
		servo_gpio1 = gpio2;
		servo_pin1 = pin2;
		servo_gpio2 = gpio1;
		servo_pin2 = pin1;
	}
}

void OnOffServoController::control(ControlMode control_mode){
	switch(control_mode){
	case forward:
		HAL_GPIO_WritePin(servo_gpio1, servo_pin1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(servo_gpio2, servo_pin2, GPIO_PIN_RESET);
		break;
	case reverse:
		HAL_GPIO_WritePin(servo_gpio1, servo_pin1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(servo_gpio2, servo_pin2, GPIO_PIN_SET);
		break;
	case stop:
		HAL_GPIO_WritePin(servo_gpio1, servo_pin1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(servo_gpio2, servo_pin2, GPIO_PIN_RESET);
		break;
	default:
		break;
	}
}



AngleSetServoController::AngleSetServoController(GPIO_TypeDef* gpio1, uint16_t pin1, GPIO_TypeDef* gpio2, uint16_t pin2, float _target_angle, AS5048A* _encoder, bool state){
	if(state == 0){
		servo_gpio1 = gpio1;
		servo_pin1 = pin1;
		servo_gpio2 = gpio2;
		servo_pin2 = pin2;
	}else{
		servo_gpio1 = gpio2;
		servo_pin1 = pin2;
		servo_gpio2 = gpio1;
		servo_pin2 = pin1;
	}
	target_angle = _target_angle;
	current_angle = 0;
	encoder = _encoder;
	zero_position = 0;
	zero_position_map = 0;
}

void AngleSetServoController::control(ControlMode control_mode){
	switch(control_mode){
	case forward:
		HAL_GPIO_WritePin(servo_gpio1, servo_pin1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(servo_gpio2, servo_pin2, GPIO_PIN_RESET);
		break;
	case reverse:
		HAL_GPIO_WritePin(servo_gpio1, servo_pin1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(servo_gpio2, servo_pin2, GPIO_PIN_SET);
		break;
	case stop:
		HAL_GPIO_WritePin(servo_gpio1, servo_pin1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(servo_gpio2, servo_pin2, GPIO_PIN_RESET);
		break;
	default:
		break;
	}
}

void AngleSetServoController::angleControl(){
	current_angle = encoder->normalize(encoder->read2angle(encoder->getRawRotation()) - zero_position_map);

	if(current_angle < target_angle - 5){
		control(forward);
	}else if(current_angle > target_angle + 5){
		control(reverse);
	}else {
		control(stop);
	}
}

void AngleSetServoController::setZeroPosition(){
	zero_position = encoder->getRawRotation();
	zero_position_map = encoder->read2angle(zero_position);
}

void AngleSetServoController::setTargetAngle(float angle){
	target_angle = angle;
}

float AngleSetServoController::getCurrentAngle(){
	current_angle = encoder->normalize(encoder->read2angle(encoder->getRawRotation()) - zero_position_map);
	return current_angle;
}

*/
