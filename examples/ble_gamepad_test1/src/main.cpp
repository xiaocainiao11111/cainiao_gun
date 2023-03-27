#include <Arduino.h>

#include <BleGamepad.h>

BleGamepad bleGamepad("ESP32 BLE Gamepad","Espressif",100);

void setup()
{
    Serial.begin(115200);
    Serial.println("Starting BLE work!");
    bleGamepad.begin();
    // The default bleGamepad.begin() above enables 16 buttons, all axes, one hat, and no simulation controls or special buttons
}

void loop()
{
    if (bleGamepad.isConnected())
    {
        Serial.println("Press buttons 5, 16 and start. Move all enabled axes to max. Set DPAD (hat 1) to down right.");

        bleGamepad.press(BUTTON_16);
        bleGamepad.pressStart();
        bleGamepad.setAxes(32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767);
        bleGamepad.setHat1(HAT_DOWN_RIGHT);
        // All axes, sliders, hats etc can also be set independently. See the IndividualAxes.ino example
        delay(500);

        Serial.println("Release button 5 and start. Move all axes to min. Set DPAD (hat 1) to centred.");
        bleGamepad.release(BUTTON_5);
        bleGamepad.releaseStart();
        bleGamepad.setHat1(HAT_CENTERED);
        bleGamepad.setAxes(0, 0, 0, 0, 0, 0, 0, 0);
        delay(500);

    // bleGamepad.pressStart();        delay(500);

    // bleGamepad.releaseStart();        delay(500);

    // bleGamepad.pressSelect();        delay(500);

    // bleGamepad.releaseSelect();        delay(500);

    // bleGamepad.pressMenu();        delay(500);

    // bleGamepad.releaseMenu();        delay(500);

    // bleGamepad.pressHome();        delay(500);

    // bleGamepad.releaseHome();        delay(500);

    // bleGamepad.pressBack();        delay(500);

    // bleGamepad.releaseBack();        delay(500);

    // bleGamepad.pressVolumeInc();        delay(500);

    // bleGamepad.releaseVolumeInc();        delay(500);

    // bleGamepad.pressVolumeDec();        delay(500);

    // bleGamepad.releaseVolumeDec();        delay(500);

    // bleGamepad.pressVolumeMute();        delay(500);

    // bleGamepad.releaseVolumeMute();        delay(500);



    }
}
