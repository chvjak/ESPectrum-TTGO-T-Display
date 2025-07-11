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
#include "Graphics.h"

class GraphicsR2G2B2S2Swapped: public Graphics<unsigned char>
{
	public:
	typedef unsigned char Color;
	static const Color RGBAXMask = 0x3f;
	Color SBits;

	GraphicsR2G2B2S2Swapped()
	{
		SBits = 0xc0;
		frontColor = 0xff;
	}

	virtual int R(Color c) const
	{
		return (((int)c & 3) * 255 + 1) / 3;
	}
	virtual int G(Color c) const
	{
		return (((int)(c >> 2) & 3) * 255 + 1) / 3;
	}
	virtual int B(Color c) const
	{
		return (((int)(c >> 4) & 3) * 255 + 1) / 3;
	}
	virtual int A(Color c) const
	{
		return (((int)(c >> 6) & 3) * 255 + 1) / 3;
	}

	virtual Color RGBA(int r, int g, int b, int a = 255) const
	{
		return ((r >> 6) & 0b11) | ((g >> 4) & 0b1100) | ((b >> 2) & 0b110000) | (a & 0b11000000);
	}

	virtual void dotFast(int x, int y, Color color)
	{
		frameBuffer[y][x^2] = (color & RGBAXMask) | SBits;
	}

	virtual void dot(int x, int y, Color color)
	{
		if ((unsigned int)x < xres && (unsigned int)y < yres)
			frameBuffer[y][x^2] = (color & RGBAXMask) | SBits;
	}

	virtual void dotAdd(int x, int y, Color color)
	{
		if ((unsigned int)x < xres && (unsigned int)y < yres)
		{
			int c0 = frameBuffer[y][x^2];
			int c1 = color;
			int r = (c0 & 0b11) + (c1 & 0b11);
			if(r > 0b11) r = 0b11;
			int g = (c0 & 0b1100) + (c1 & 0b1100);
			if(g > 0b1100) g = 0b1100;
			int b = (c0 & 0b110000) + (c1 & 0b110000);
			if(b > 0b110000) b = 0b110000;
			frameBuffer[y][x^2] = r | (g & 0b1100) | (b & 0b110000) | SBits;
		}
	}

	virtual void dotMix(int x, int y, Color color)
	{
		if ((unsigned int)x < xres && (unsigned int)y < yres && (color >> 6) != 0)
		{
			unsigned int ai = (3 - ((int)color >> 6)) * (65536 / 3);
			unsigned int a = 65536 - ai;
			unsigned int co = frameBuffer[y][x^2];
			unsigned int ro = (co & 0b11) * ai;
			unsigned int go = (co & 0b1100) * ai;
			unsigned int bo = (co & 0b110000) * ai;
			unsigned int r = (color & 0b11) * a + ro;
			unsigned int g = ((color & 0b1100) * a + go) & 0b11000000000000000000;
			unsigned int b = ((color & 0b110000) * a + bo) & 0b1100000000000000000000;
			frameBuffer[y][x^2] = ((r | g | b) >> 16) | SBits;
		}
	}

	virtual Color get(int x, int y)
	{
		if ((unsigned int)x < xres && (unsigned int)y < yres)
			return frameBuffer[y][x^2] & RGBAXMask;
		return 0;
	}

	virtual void clear(Color color = 0)
	{
		for (int y = 0; y < this->yres; y++)
			for (int x = 0; x < this->xres; x++)
				frameBuffer[y][x^2] = (color & RGBAXMask) | SBits;
	}

};