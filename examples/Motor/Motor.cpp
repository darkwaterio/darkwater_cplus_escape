/*
Example code is placed under the BSD license.
Written by Team Dark Water (team@darkwater.io)
Copyright (c) 2014, Dark Water
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Emlid Limited nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL EMLID LIMITED BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "darkwater/DWESCAPE.h"
#include "darkwater/Util.h"
#include <stdlib.h>
//=============================================================================

int main()
{
    if (check_apm()) {
        return 1;
    }
	//-------------------------------------------------------------------------
	DWESCAPE dw;
	dw.initialize();

	DW_Motor *dw1 = dw.getMotor(1);
	DW_Motor *dw2 = dw.getMotor(2);
	DW_Motor *dw3 = dw.getMotor(3);
	DW_Motor *dw4 = dw.getMotor(4);
	DW_Motor *dw5 = dw.getMotor(5);
	DW_Motor *dw6 = dw.getMotor(6);

	dw1->off();
	dw2->off();
	dw3->off();
	dw4->off();
	dw5->off();
	dw6->off();
	usleep(1000000);

	printf("Set forward - \n");
	printf("Motor 1\n");
	dw1->setMotorSpeed( 255 );
	usleep(1000000);
	printf("Motor 2\n");
	dw2->setMotorSpeed( 255 );
	usleep(1000000);
	printf("Motor 3\n");
	dw3->setMotorSpeed( 255 );
	usleep(1000000);
	printf("Motor 4\n");
	dw4->setMotorSpeed( 255 );
	usleep(1000000);
	printf("Motor 5\n");
	dw5->setMotorSpeed( 255 );
	usleep(1000000);
	printf("Motor 6\n");
	dw6->setMotorSpeed( 255 );
	usleep(1000000);
	printf("Stopping - \n");
	printf("Motor 1\n");
	dw1->setMotorSpeed( 0 );
	usleep(1000000);
	printf("Motor 2\n");
	dw2->setMotorSpeed( 0 );
	usleep(1000000);
	printf("Motor 3\n");
	dw3->setMotorSpeed( 0 );
	usleep(1000000);
	printf("Motor 4\n");
	dw4->setMotorSpeed( 0 );
	usleep(1000000);
	printf("Motor 5\n");
	dw5->setMotorSpeed( 0 );
	usleep(1000000);
	printf("Motor 6\n");
	dw6->setMotorSpeed( 0 );
	usleep(1000000);
	printf("Set reverse - \n");
	printf("Motor 1\n");
	dw1->setMotorSpeed( -255 );
	usleep(1000000);
	printf("Motor 2\n");
	dw2->setMotorSpeed( -255 );
	usleep(1000000);
	printf("Motor 3\n");
	dw3->setMotorSpeed( -255 );
	usleep(1000000);
	printf("Motor 4\n");
	dw4->setMotorSpeed( -255 );
	usleep(1000000);
	printf("Motor 5\n");
	dw5->setMotorSpeed( -255 );
	usleep(1000000);
	printf("Motor 6\n");
	dw6->setMotorSpeed( -255 );
	usleep(1000000);
	printf("All off \n");
	dw1->off();
	dw2->off();
	dw3->off();
	dw4->off();
	dw5->off();
	dw6->off();

}
