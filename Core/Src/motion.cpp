#include "motion.h"
#include "servo_control.h"

extern bool isServoPidOn[10];

MotionController::MotionController(ServoController* _mp1, ServoController* _pip1, ServoController* _mp2, ServoController* _pip2,
			ServoController* _mp3, ServoController* _pip3, ServoController* _cm, ServoController* _mp4, ServoController* _pip4, ServoController* _abd){
	mp1 = _mp1;
	pip1 = _pip1;
	mp2 = _mp2;
	pip2 = _pip2;
	mp3 = _mp3;
	pip3 = _pip3;
	cm = _cm;
	mp4 = _mp4;
	pip4 = _pip4;
	abd = _abd;
}

void MotionController::setPosition(float mp1_angle, float pip1_angle, float mp2_angle, float pip2_angle,
                                   float mp3_angle, float pip3_angle, float cm_angle, float mp4_angle, float pip4_angle, float abd_angle){
    for (int i = 0; i < 10; ++i) {
        isServoPidOn[i] = true;
    }
    mp1->setTargetAngle(mp1_angle);
    pip1->setTargetAngle(pip1_angle);
    mp2->setTargetAngle(mp2_angle);
    pip2->setTargetAngle(pip2_angle);
    mp3->setTargetAngle(mp3_angle);
    pip3->setTargetAngle(pip3_angle);
    cm->setTargetAngle(cm_angle);
	mp4->setTargetAngle(mp4_angle);
    pip4->setTargetAngle(pip4_angle);
    abd->setTargetAngle(abd_angle);
}

void MotionController::setMotion(HandMotion hand_motion){
    for (int i = 0; i < 10; ++i) {
        isServoPidOn[i] = false;
    }// Disable PID control for all servos
	switch(hand_motion){
	case OPEN:

		mp1->onOffControl(reverse);
		mp2->onOffControl(reverse);
		mp3->onOffControl(reverse);
		mp4->onOffControl(reverse);
		pip1->onOffControl(reverse);
		pip2->onOffControl(reverse);
		pip3->onOffControl(reverse);
		pip4->onOffControl(reverse);
		cm->onOffControl(reverse);
		abd->onOffControl(stop);

		break;
	case CLOSE:

		mp2->onOffControl(forward);
		mp1->onOffControl(forward);
		mp3->onOffControl(forward);
		mp4->onOffControl(forward);
		pip1->onOffControl(forward);
		pip2->onOffControl(forward);
		pip3->onOffControl(forward);
		pip4->onOffControl(forward);
		cm->onOffControl(reverse);
		abd->onOffControl(stop);

		break;
	case GRASP:

		cm->onOffControl(forward);
		HAL_Delay(800);
		mp4->onOffControl(forward);
		HAL_Delay(300);
		mp1->onOffControl(forward);
		mp2->onOffControl(forward);
		mp3->onOffControl(forward);
		pip2->onOffControl(forward);
		pip3->onOffControl(forward);
		pip4->onOffControl(forward);
		abd->onOffControl(stop);
		pip1->onOffControl(forward);

		break;
	case GRIP:
		cm->onOffControl(forward);
		HAL_Delay(300);
		mp1->onOffControl(forward);
		mp2->onOffControl(forward);
		mp3->onOffControl(forward);
		HAL_Delay(300);
		mp4->onOffControl(forward);
		HAL_Delay(300);
		pip1->onOffControl(forward);
		pip2->onOffControl(forward);
		pip3->onOffControl(forward);
		pip4->onOffControl(forward);

		break;
	case PEACE:
		mp1->onOffControl(reverse);
		mp2->onOffControl(reverse);
		mp3->onOffControl(reverse);
		mp4->onOffControl(reverse);
		pip1->onOffControl(reverse);
		pip2->onOffControl(reverse);
		pip3->onOffControl(reverse);
		pip4->onOffControl(reverse);
		cm->onOffControl(reverse);
		abd->onOffControl(forward);
		HAL_Delay(500);
		abd->onOffControl(reverse);
		HAL_Delay(500);
		abd->onOffControl(forward);
		HAL_Delay(500);
		abd->onOffControl(reverse);
		HAL_Delay(500);

		break;
	case ORDER:
		mp1->onOffControl(forward);
		HAL_Delay(500);
		pip1->onOffControl(forward);
		HAL_Delay(500);
		mp2->onOffControl(forward);
		HAL_Delay(500);
		pip2->onOffControl(forward);
		HAL_Delay(500);
		mp3->onOffControl(forward);
		HAL_Delay(500);
		pip3->onOffControl(forward);
		HAL_Delay(500);
		cm->onOffControl(forward);
		HAL_Delay(500);
		mp4->onOffControl(forward);
		HAL_Delay(500);
		pip4->onOffControl(forward);
		HAL_Delay(2000);

		pip4->onOffControl(reverse);
		HAL_Delay(500);
		mp4->onOffControl(reverse);
		HAL_Delay(500);
		cm->onOffControl(reverse);
		HAL_Delay(500);
		pip3->onOffControl(reverse);
		HAL_Delay(500);
		mp3->onOffControl(reverse);
		HAL_Delay(500);
		pip2->onOffControl(reverse);
		HAL_Delay(500);
		mp2->onOffControl(reverse);
		HAL_Delay(500);
		pip1->onOffControl(reverse);
		HAL_Delay(500);
		mp1->onOffControl(reverse);

		break;
	case KNOB:
		mp2->onOffControl(reverse);
		mp3->onOffControl(reverse);
		pip2->onOffControl(reverse);
		pip3->onOffControl(reverse);
		cm->onOffControl(reverse);
		HAL_Delay(300);
		mp1->onOffControl(forward);
		mp4->onOffControl(forward);
		HAL_Delay(300);
		pip1->onOffControl(forward);
		pip4->onOffControl(forward);
        break;

	case MOUTH:
		mp1->onOffControl(forward);
		mp2->onOffControl(forward);
		mp3->onOffControl(forward);
		mp4->onOffControl(reverse);
		pip1->onOffControl(reverse);
		pip2->onOffControl(reverse);
		pip3->onOffControl(reverse);
		pip4->onOffControl(reverse);
		cm->onOffControl(reverse);
	default:
		break;
	}
}

void MotionController::stopMotion(){
	for (int i = 0; i < 10; ++i) {
        isServoPidOn[i] = false;
    }
	mp1->onOffControl(stop);
	mp2->onOffControl(stop);
	mp3->onOffControl(stop);
	mp4->onOffControl(stop);
	pip1->onOffControl(stop);
	pip2->onOffControl(stop);
	pip3->onOffControl(stop);
	pip4->onOffControl(stop);
	cm->onOffControl(stop);
	abd->onOffControl(stop);
}