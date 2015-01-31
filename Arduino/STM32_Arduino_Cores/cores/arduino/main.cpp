/*
  Copyright (c) 2014 MakerLab.me & Andy Sze(andy.sze.mail@gmail.com)  All right reserved.
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#define ARDUINO_MAIN
#include "Arduino.h"

__IO uint32_t TimingMillis;


void TimeTick_Increment(void)
{
  TimingMillis++;
}

/*
 * \brief Main entry point of Arduino application
 */
int main( void )
{
	init();
  TimingMillis=0;

	//delay(1);

#if defined(USBCON)
	//USBDevice.attach();
#endif

	setup();

	for (;;)
	{

		loop();
		if (serialEventRun) serialEventRun();
	}

	return 0;
}
