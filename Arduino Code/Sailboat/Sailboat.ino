#include <sail_controller.h>
#include <rudder_controller.h>
#include <Wire.h>
#include <Servo.h>
#include <TinyGPSPlus.h>

#define _DEBUG TRUE

const unsigned int COMPASS_I2C_ADDRESS = 0x60;

const unsigned int DEBUG_SERIAL_BAUDRATE = 57600;
const unsigned int GPS_SERIAL_BAUDRATE = 57600;

const unsigned int GPS_VALID_DATA_LIMIT_AGE = 5000;

const unsigned long GPS_GOAL_LATITUDE = 90500000;		// 90°30’00″
const unsigned long GPS_GOAL_LONGITUDE = 90500000;		// 90°30’00″

const unsigned int ROTARY_ENCODER_PIN = 8;
const unsigned int SAIL_SERVO_PIN = 49;
const unsigned int RUDDER_SERVO_PIN = 53;

const unsigned int ROTARY_ENC_MIN_PULSE_WIDTH = 1000;
const unsigned int ROTARY_ENC_MAX_PULSE_WIDTH = 4000;
const unsigned int SAIL_MIN_PULSE_WIDTH = 1000;
const unsigned int SAIL_MAX_PULSE_WIDTH = 2000;
const unsigned int RUDDER_MIN_PULSE_WIDTH = 1000;
const unsigned int RUDDER_MAX_PULSE_WIDTH = 2000;

TinyGPSPlus gps;

Servo rudder;
Servo sail;

void setup()
{

#ifdef _DEBUG
	Serial.begin(DEBUG_SERIAL_BAUDRATE);
#endif // _DEBUG

	Serial3.begin(GPS_SERIAL_BAUDRATE);

	Wire.begin();

	pinMode(ROTARY_ENCODER_PIN, INPUT_PULLUP);

	rudder.attach(RUDDER_SERVO_PIN);
	sail.attach(SAIL_SERVO_PIN);

	rudder.writeMicroseconds(1500);
	sail.writeMicroseconds(1500);

	while (!gps.location.isValid())
		retrieveGPSData();

#ifdef _DEBUG
	Serial.println("GPS Position Fixed !");
#endif // DEBUG

}

void loop()
{
	byte highByte, lowByte;
	char pitch, roll;
	int bearing;
	double rudderActuationValue;
	double sailActuationValue;
	double goalAlignment;
	double windAlignment;

	retrieveCompassData(&bearing, &pitch, &roll);
	retrieveGPSData();

	if (gps.location.age() < GPS_VALID_DATA_LIMIT_AGE)
	{
		goalAlignment = gps.courseTo(gps.location.lat(), gps.location.lng(), GPS_GOAL_LATITUDE, GPS_GOAL_LONGITUDE);
		windAlignment = abs(retrievePositionEncoderData());
		rudder_controllerInferenceEngine(goalAlignment, &rudderActuationValue);
		sail_controllerInferenceEngine(abs(roll), goalAlignment, windAlignment, &sailActuationValue);
		rudder.writeMicroseconds(map(rudderActuationValue, 0, 100, RUDDER_MIN_PULSE_WIDTH, RUDDER_MAX_PULSE_WIDTH));
		sail.writeMicroseconds(map(sailActuationValue, 0, 100, SAIL_MIN_PULSE_WIDTH, SAIL_MAX_PULSE_WIDTH));

#ifdef _DEBUG
		Serial.print("Position Encoder : "); Serial.println(retrievePositionEncoderData());
		Serial.print("Roll : "); Serial.println(roll, DEC);
		Serial.print("Bearing : "); Serial.println(bearing);
		Serial.print("GPS Location : "); Serial.print(gps.location.lat(), 6); Serial.print(F(",")); Serial.println(gps.location.lng(), 6);
		Serial.print("Goal Position Alignment : "); Serial.println(goalAlignment);
		Serial.print("Wind Alignment : "); Serial.println(windAlignment);
		Serial.print("Sail Actuation Value : "); Serial.println(sailActuationValue);
		Serial.print("Rudder Actuation Value : "); Serial.println(rudderActuationValue);
#endif // DEBUG

	}
}

void retrieveCompassData(int *bearing, char *pitch, char *roll)
{
	byte highByte, lowByte;

	Wire.beginTransmission((int)COMPASS_I2C_ADDRESS);
	Wire.write(2);
	Wire.endTransmission();

	Wire.requestFrom(COMPASS_I2C_ADDRESS, 4);

	while (Wire.available() < 4);
	highByte = Wire.read();
	lowByte = Wire.read();
	*pitch = (int)Wire.read();
	*roll = (int)Wire.read();

	*bearing = word(highByte, lowByte) / 10.0;
}

void retrieveGPSData()
{
	while (Serial3.available() > 0)
		gps.encode(Serial3.read());
}

int retrievePositionEncoderData()
{
	return map(pulseIn(ROTARY_ENCODER_PIN, HIGH), ROTARY_ENC_MIN_PULSE_WIDTH, ROTARY_ENC_MAX_PULSE_WIDTH, -180, 180);
}