/*
	Author: bitluni 2019
	License:
	Creative Commons Attribution ShareAlike 4.0
	https://creativecommons.org/licenses/by-sa/4.0/

	For further details check out:
		https://youtube.com/bitlunislab
		https://github.com/bitluni
		http://bitluni.net
*/
#pragma once
#include "VGA.h"
#include "../Graphics/GraphicsR2G2B2S2Swapped.h"
#include "../I2S/DMABufferDescriptor.h"

class VGA6Bit : public VGA, public GraphicsR2G2B2S2Swapped
{
  public:

	bool VGA6Bit_useinterrupt;

	VGA6Bit() : VGA(1) { //8 bit based modes only work with I2S1
	}

	bool init(int mode, const int *redPins, const int *greenPins, const int *bluePins, const int hsyncPin, const int vsyncPin, const int clockPin = -1)	{
		int pinMap[8];
		for (int i = 0; i < 2; i++)
		{
			pinMap[i] = redPins[i];
			pinMap[i + 2] = greenPins[i];
			pinMap[i + 4] = bluePins[i];
		}
		pinMap[6] = hsyncPin;
		pinMap[7] = vsyncPin;
		return VGA::init(mode, pinMap, 8, clockPin);
	}

	virtual void propagateResolution(const int xres, const int yres) {
		setResolution(xres, yres);
	}

	virtual Color **allocateFrameBuffer() {
		return (Color **)DMABufferDescriptor::allocateDMABufferArray(yres, vidmodes[mode][vmodeproperties::hRes], true);
	}

  virtual Color *allocateLineBuffer() {
		return (Color *)DMABufferDescriptor::allocateBuffer(vidmodes[mode][vmodeproperties::hRes], true);
	}
};
