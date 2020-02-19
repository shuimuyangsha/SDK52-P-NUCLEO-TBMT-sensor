#ifndef EXTREME3D_H
#define EXTREME3D_H

//#include "sys.h"
#include "stdint.h"
typedef struct buttons{
	uint8_t getButtonsState;
	uint8_t setButtonsState;
}Buttons;

typedef struct slider{
	uint16_t getSliderState;
	uint16_t setSliderState;
}Slider;

typedef  struct joystick{
	uint16_t getJoystickState;
	uint16_t setjoystickState;
}Joystick;

typedef  struct point_of_view{
	short int getPointOfViewState;
	short int setPointOfViewState;
}PointOfView;

typedef  struct extreme3dStruct{
	Buttons buttons0;
	Buttons buttons1;
	Buttons buttons2;
	Buttons buttons3;
	Buttons buttons4;
	Buttons buttons5;
	Buttons buttons6;
	Buttons buttons7;
	Buttons buttons8;
	Buttons buttons9;
	Buttons buttons10;
	Buttons buttons11;

	Slider  slider0;
	Joystick joystickX;
	Joystick joystickY;
	Joystick joystickRotationZ;

	PointOfView pointOfView;
}Extreme3dStruct;

extern uint16_t Compass;

extern Extreme3dStruct extreme3d;

void extreme3dConfig(void);

#endif // EXTREME3D_H
