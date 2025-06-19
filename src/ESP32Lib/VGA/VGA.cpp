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
#include "VGA.h"
#include <stdio.h>

VGA::VGA(const int i2sIndex) {}

bool VGA::init(int mode, const int *pinMap, const int bitCount, const int clockPin) {

	this->mode = mode;
	int xres = vidmodes[mode][vmodeproperties::hRes];
	int yres = vidmodes[mode][vmodeproperties::vRes] / vidmodes[mode][vmodeproperties::vDiv];

  printf("VGA mode %d: %dx%d\n", mode, xres, yres);

	propagateResolution(xres, yres);

	return true;

}
