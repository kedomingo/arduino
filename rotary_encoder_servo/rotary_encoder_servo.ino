#include <Servo.h>

// Rotary Encoder Inputs
#define ROTARY_CLK 2
#define ROTARY_DT 3
#define ROTARY_SW 4
#define SERVO 9

Servo servo;

int counter = 0;
int currentStateCLK;
int lastStateCLK;
String currentDir ="";
unsigned long lastButtonPress = 0;

void setup() {
  servo.attach(SERVO);

  // Set encoder pins as inputs
  pinMode(ROTARY_CLK, INPUT);
  pinMode(ROTARY_DT, INPUT);
  pinMode(ROTARY_SW, INPUT_PULLUP);

  // Setup Serial Monitor
  Serial.begin(9600);

  // Read the initial state of ROTARY_CLK
  lastStateCLK = digitalRead(ROTARY_CLK);
}

void loop() {
  int rotation = getRotaryRotation();

  // Rotation counter should be bound between 0 and 180
  // There are 5 clicks to cover 90 degrees so multiply 
  // rotation value by 90/5 = 18
  if (rotation != 0) {
    counter = min(180, max(0, counter + (rotation * 18)));
    Serial.print("Counter: ");
    Serial.println(counter);
    servo.write(counter);
  }
  
  if (isRotaryButtonPressed()) {
      Serial.println("Button pressed!");
  }

  // Put in a slight delay to help debounce the reading
  delay(1);
}

/**
 * Get the rotation value of the rotary encoder.
 * Returns -1 when rotated counter clockwise
 * Returns 1 when clockwise
 * Returns 0 when stationary
 */
int getRotaryRotation()
{
  // Read the current state of ROTARY_CLK
  currentStateCLK = digitalRead(ROTARY_CLK);
  int result = 0;
  
  // If last and current state of ROTARY_CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){

    // If the ROTARY_DT state is different than the ROTARY_CLK state then
    // the encoder is rotating CCW
    if (digitalRead(ROTARY_DT) != currentStateCLK) {
      result = 1;
    } else {
      result = -1;
    }
  }

  // Remember ROTARY_CLK state
  lastStateCLK = currentStateCLK;

  return result;
}

boolean isRotaryButtonPressed()
{
  // Read the button state
  int btnState = digitalRead(ROTARY_SW);
  boolean result = false;
  
  //If we detect LOW signal, button is pressed
  if (btnState == LOW) {
    //if 50ms have passed since last LOW pulse, it means that the
    //button has been pressed, released and pressed again
    if (millis() - lastButtonPress > 50) {
      result = true;
    }

    // Remember last button press event
    lastButtonPress = millis();
  }

  return result;
}
