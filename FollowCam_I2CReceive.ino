#include <Wire.h>

#define SLAVE_ADDRESS 0x06
int number = 0;
int state = 0;
byte a;
byte b;

void setup() 
{
	pinMode(13, OUTPUT);
	Serial.begin(9600); // start serial for output
	// initialize i2c as slave
	Wire.begin(SLAVE_ADDRESS);

	// define callbacks for i2c communication
	Wire.onReceive(receiveData);
	Wire.onRequest(sendData);

	Serial.println("Ready!");
}

void loop() 
{
	delay(100);
}

// callback for received data
void receiveData(int byteCount)
{
	while(Wire.available()) 
	{
		a = Wire.read();
		b = Wire.read();
		number = a;
		number = (number << 8) | b;
		Serial.print("data received: ");
		Serial.println(number);
	}
}

// callback for sending data
void sendData()
{
	Wire.write(number);
}