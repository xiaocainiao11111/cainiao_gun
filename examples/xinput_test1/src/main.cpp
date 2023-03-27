#include <Arduino.h>

#include <XInput.h>

const uint8_t LED_Pin = LED_BUILTIN;
const uint8_t Button_Pin = 6;

void setup() {
	pinMode(Button_Pin, INPUT_PULLUP);  // Set button pin as input w/ pullup

	pinMode(LED_Pin, OUTPUT);    // Set LED pin as output
	digitalWrite(LED_Pin, LOW);  // Turn LED off

	// Set callback function. Function must have a 'void' return type
	// and take a single uint8_t as an argument
	XInput.setReceiveCallback(rumbleCallback);

	XInput.begin();
}

void loop() {
	boolean buttonState = digitalRead(Button_Pin);
	
	if(buttonState == LOW) { 
		XInput.press(TRIGGER_RIGHT);
	}
	else {
		XInput.release(TRIGGER_RIGHT);
	}
}

void rumbleCallback(uint8_t packetType) {
	// If we have an LED packet (0x01), do nothing
	if (packetType == (uint8_t) XInputReceiveType::LEDs) {
		return;
	}

	// If we have a rumble packet (0x00), see our rumble data on the LED
	else if (packetType == (uint8_t) XInputReceiveType::Rumble) {
		uint8_t rumbleValue = XInput.getRumbleLeft() | XInput.getRumbleRight();
		analogWrite(LED_Pin, rumbleValue);
	}
}