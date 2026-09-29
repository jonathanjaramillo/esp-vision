#include "JoystickHandle.h"

JoystickHandle myJoystickHandle(JOYSTICK_I2C_ADDR);

void setup() {
  Serial.begin(115200);
}

void loop() {
  if (myJoystickHandle.Get_Button_Status(BUTTON_A) == PRESS_DOWN) {  // Determine if button A is pressed
    Serial.println("Button A Pressed");
  }
  if (myJoystickHandle.Get_Button_Status(BUTTON_B) == PRESS_DOWN) {  // Determine if button B is pressed
    Serial.println("Button B Pressed");
  }
  if (myJoystickHandle.Get_Button_Status(BUTTON_C) == PRESS_DOWN) {  // Determine if button C is pressed
    Serial.println("Button C Pressed");
  }
  if (myJoystickHandle.Get_Button_Status(BUTTON_D) == PRESS_DOWN) {  // Determine if button D is pressed
    Serial.println("Button D Pressed");
  }
  if (myJoystickHandle.Get_Button_Status(BUTTON_OK) == PRESS_DOWN) {  // Detect if the joystick button is pressed
    Serial.println("Button OK Pressed");
  }
  Serial.print("Value_X: ");
  Serial.println(myJoystickHandle.AnalogRead_X());  // Read and print the analog value of the joystick X-axis
  Serial.print("Value_Y: ");
  Serial.println(myJoystickHandle.AnalogRead_Y());  // Read and print the analog value of the joystick Y-axis
                                                    // Serial.println(myJoystickHandle.Get_Button_Status(BUTTON_A));    // Read button A state
  delay(50);
}
