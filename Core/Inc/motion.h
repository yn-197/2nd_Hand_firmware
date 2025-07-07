#ifndef INC_MOTION_H_
#define INC_MOTION_H_

#ifdef __cplusplus
extern "C" {
#endif

//include
#include "main.h"
#include "servo_control.h"
#include "as5048a.h"

typedef enum {
	OPEN, //パー
	CLOSE, //グー
	GRASP, //物の把持
	GRIP, //つかみ
	PEACE, //ピース
	ORDER, //指連続稼働
	KNOB, //つまみ
	MOUTH

}HandMotion;
//class
class MotionController {
private:
	ServoController* mp1;
	ServoController* pip1;
	ServoController* mp2;
	ServoController* pip2;
	ServoController* mp3;
	ServoController* pip3;
	ServoController* mp4;
	ServoController* cm;
	ServoController* pip4;
	ServoController* abd;

public:
	MotionController(ServoController* _mp1, ServoController* _pip1, ServoController* _mp2, ServoController* _pip2,
			ServoController* _mp3, ServoController* _pip3, ServoController* _mp4, ServoController* _cm, ServoController* _pip4, ServoController* _abd);

    void setPosition(float mp1_angle, float pip1_angle, float mp2_angle, float pip2_angle,
                    float mp3_angle, float pip3_angle, float cm_angle, float mp4_angle, float pip4_angle, float abd_angle);

	void setMotion(HandMotion hand_motion);

	void stopMotion();

};


#ifdef __cplusplus
}
#endif

#endif /* INC_MOTION_H_ */