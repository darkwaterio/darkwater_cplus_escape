/*
Dark Water 640 driver code is placed under the BSD license.
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

#include "DWESCAPE.h"

/** PCA9685 constructor.
 * @param address I2C address
 * @see DWESCAPE_DEFAULT_ADDRESS
 */
DWESCAPE::DWESCAPE(uint8_t address) {
    this->devAddr = address;
}

/** Power on and prepare for general usage.
 * This method reads prescale value stored in PCA9685 and calculate frequency based on it.
 * Then it enables auto-increment of register address to allow for faster writes.
 * And finally the restart is performed to enable clocking.
 */
bool DWESCAPE::initialize() {
    this->pwm = new PCA9685( this->devAddr );
    this->pwm->initialize();
    if (!testConnection() ) {
    	printf("No ESCAPE board found\n");
    	return 0;		
    }
    // set the default frequency
    setFrequency( 100 );
    
}

/** Verify the I2C connection.
 * @return True if connection is valid, false otherwise
 */
bool DWESCAPE::testConnection() {
    return this->pwm->testConnection();
}

/** Calculate prescale value based on the specified frequency and write it to the device.
 * @return Frequency in Hz
 */
float DWESCAPE::getFrequency() {
    return this->pwm->getFrequency();
}


/** Calculate prescale value based on the specified frequency and write it to the device.
 * @param Frequency in Hz
 */
void DWESCAPE::setFrequency(float frequency) {
    this->pwm->setFrequency( frequency );
}

/** Set channel start offset of the pulse and it's length
 * @param Channel number (0-15)
 * @param Offset (0-4095)
 * @param Length (0-4095)
 */
void DWESCAPE::setPWM(uint8_t channel, uint16_t offset, uint16_t length) {
	this->pwm->setPWM( channel, offset, length );
}

/** Set channel's pulse length
 * @param Channel number (0-15)
 * @param Length (0-4095)
 */
void DWESCAPE::setPWM(uint8_t channel, uint16_t length) {
    this->pwm->setPWM(channel, length);
}

/** Set channel's pulse length in milliseconds
 * @param Channel number (0-15)
 * @param Length in milliseconds
 */
void DWESCAPE::setPWMmS(uint8_t channel, float length_mS) {
    this->pwm->setPWMmS(channel, length_mS);
}

/** Set channel's pulse length in microseconds
 * @param Channel number (0-15)
 * @param Length in microseconds
 */
void DWESCAPE::setPWMuS(uint8_t channel, float length_uS) {
    this->pwm->setPWMuS(channel, length_uS);
}

/** Set start offset of the pulse and it's length for all channels
 * @param Offset (0-4095)
 * @param Length (0-4095)
 */
void DWESCAPE::setAllPWM(uint16_t offset, uint16_t length) {
	this->pwm->setAllPWM(offset, length);
}

/** Set pulse length for all channels
 * @param Length (0-4095)
 */
void DWESCAPE::setAllPWM(uint16_t length) {
    this->pwm->setAllPWM(length);
}

/** Set pulse length in milliseconds for all channels
 * @param Length in milliseconds
 */
void DWESCAPE::setAllPWMmS(float length_mS) {
	this->pwm->setAllPWMmS(length_mS);
}

/** Set pulse length in microseconds for all channels
 * @param Length in microseconds
 */
void DWESCAPE::setAllPWMuS(float length_uS) {
	this->pwm->setAllPWMuS(length_uS);
}

void DWESCAPE::setPin(uint8_t channel, uint8_t value) {

	if( channel < 0 || channel > 15 ) {
		fprintf(stderr, "PWM pin must be between 0 and 15 inclusive");
	}

	if( value == 0 ) {
		setPWM( channel, 0, 4096 );
	} else if( value == 1 ) {
		setPWM( channel, 4096, 0 );
	} else {
		fprintf(stderr, "Pin value must be 0 or 1!");
	}
}

void DWESCAPE::setAllPin(uint8_t value) {

	if( value == 0 ) {
		setAllPWM( 0, 4096 );
	} else if( value == 1 ) {
		setAllPWM( 4096, 0 );
	} else {
		fprintf(stderr, "Pin value must be 0 or 1!");
	}

}

void DWESCAPE::allOff() {
	setAllPin( 0 );
}

/* DC Motor specific code */

DW_Motor *DWESCAPE::getMotor(uint8_t motor) {

	uint8_t pin;

	motor--;

	// Get the motor
	switch(motor) {
		case 0:
				pin = 0;
				break;
		case 1:
				pin = 1;
				break;
		case 2:
				pin = 2;
				break;
		case 3:
				pin = 3;
				break;
		case 4:
				pin = 4;
				break;
		case 5:
				pin = 5;
				break;
		default:
				fprintf(stderr, "Motor number must be between 1 and 6 inclusive");
	}

	if(motors[motor].motor == 0) {
		// We don't have one yet
		motors[motor].motor = motor; // how many motors on one line?!?
		motors[motor].DWC = this;

		motors[motor].pin = pin;
	}

	return &motors[motor];

}

DW_Servo *DWESCAPE::getServo(uint8_t servo) {

	uint8_t pin;

	servo--;

	// Get the servo
	switch(servo) {
		case 0:
				pin = 6; // Servo 1 is on PWM 0
				break;
		case 1:
				pin = 7; // Servo 2 is on PWM 1
				break;
		case 2:
				pin = 8;
				break;
		case 3:
				pin = 9;
				break;
		case 4:
				pin = 10;
				break;
		case 5:
				pin = 11;
				break;
		default:
				fprintf(stderr, "Servo number must be between 1 and 6 inclusive");
	}

	if(servos[servo].servo == 0) {
		// We don't have one yet
		servos[servo].servo = servo; // how many servos on one line?!?
		servos[servo].DWC = this;

		servos[servo].pin = pin;
	}

	return &servos[servo];

}

/* Motors code */

DW_Motor::DW_Motor(void) {
  DWC = NULL;
  motor = 0;
  pin = 0;
}

void DW_Motor::setMotorSpeed(int16_t speed) {

	// Speed deciphering for the two control modes
	if( speed >= 1000 && speed < 1500 ) {
		run( DW_REVERSE, speed );
		} else if( speed > 1500 && speed <= 2000 ) {
			run( DW_FORWARD, speed );	
			} else if( speed > 0 && speed <= 255 ) {
				run( DW_FORWARD, map(speed, 0, 255, 1500, 2000 ) );
				} else if( speed < 0 && speed >= -255 ) {
					run( DW_REVERSE, map( abs(speed, 0, 255, 1500, 1000) );
					} else if( speed == 0 || speed == 1500 ) {
						run( DW_STOP, 1500 );
					}

	

}

void DW_Motor::off(void) {
	setMotorSpeed( 0 );
}

void DW_Motor::run( uint8_t control, uint16_t speed ) {

	// The if statement isn't really needed - but it's there in case we want to add more functionality later
	if( control == DW_FORWARD ) {
		setPWMuS( speed );
		} else if( control == DW_REVERSE ) {
			setPWMuS( speed );
			} else if( control == DW_STOP ) {
				setPWMuS( speed );
				}

}

uint16_t DW_Motor::map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void DW_Motor::setPWMmS(float length_mS) {

	DWC->setPWMmS( pin, length_mS ); // Servo 1 is on PWM 0

}

void DW_Motor::setPWMuS(float length_uS) {

	DWC->setPWMuS( pin, length_uS ); // Servo 1 is on PWM 0
	

}

/* Servo code */

DW_Servo::DW_Servo(void) {
  DWC = NULL;
  servo = 0;
  pin = 0;
}

void DW_Servo::off(void) {

	DWC->setPin( pin, 0 );

}
    
void DW_Servo::setPWMmS(float length_mS) {

	DWC->setPWMmS( pin, length_mS ); // Servo 1 is on PWM 0

}

void DW_Servo::setPWMuS(float length_uS) {

	DWC->setPWMuS( pin, length_uS ); // Servo 1 is on PWM 0
	

}
