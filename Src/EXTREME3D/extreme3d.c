#include "extreme3d.h"

Extreme3dStruct extreme3d; 

uint16_t Compass = 0;

void extreme3dConfig(void){
	extreme3d.joystickX.getJoystickState = 32768;
	extreme3d.joystickY.getJoystickState = 32768;
	extreme3d.joystickRotationZ.getJoystickState = 32768;
}




