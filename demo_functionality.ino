#include <ESP32Servo.h>

// A4988 Stepper
#define ROTATING_PLATE_DIR 12
#define ROTATING_PLATE_STEP 13
#define STEPS 200
#define GEAR_RATIO1 200
#define GEAR_RATIO2 24
#define MICRO_STEPS 4
// FIT0521 DC
#define DC_MOTOR_PIN1 26
#define DC_MOTOR_PIN2 27
#define DC_MOTOR_ENABLE 14
// Servo
#define LATCH_SERVO_PIN 4

#define CLOCKWISE HIGH
#define ANTICLOCKWISE LOW

uint8_t speed = 50;
uint8_t freq = 2;

Servo arm_extension;
Servo latch;

bool latchOpen = false;

void setup() {
  // put your setup code here, to run once:
  // Set Stepper outputs
  pinMode(ROTATING_PLATE_DIR, OUTPUT);
  pinMode(ROTATING_PLATE_STEP, OUTPUT);

  // Set DC motor outputs
  pinMode(DC_MOTOR_PIN1, OUTPUT);
  pinMode(DC_MOTOR_PIN2, OUTPUT);
  pinMode(DC_MOTOR_ENABLE, OUTPUT);

  latch.attach(LATCH_SERVO_PIN);
  arm_extension.attach(ARM_EXTENSION_PIN);

  Serial.begin(9600);

  // Demonstrate latch and stepper works
  delay(500);
  latch.write(120);
  latchOpen = false;
  delay(500);
  latch.write(0);
  latchOpen = true;

  delay(500);
  armRotateBy(45);
  delay(500);
  armRotateBy(-45);

  Serial.println("Started listening");
}

int16_t degToCycles(float degrees) {
  /*
   * degToCycles()
   * =============
   * Converts a (float) number of degrees into
   * the number of cycles needed to rotate that angle.
   * 
   * cycles = angle/360 * GEAR_RATIO * MICRO_STEPS * STEPS
   * At our chosen values 360 degrees is 6667 points.
   */
  float n = (degrees * MICRO_STEPS * STEPS * GEAR_RATIO1)/(360 * GEAR_RATIO2);
  return (n + 0.5f);
}

void armRotateBy(float degrees) {
  /*
   * armRotateBy()
   * =============
   * Rotates the rotating plate by the given number of degrees.
   * Positive values result in clockwise motion,
   * negative values result in anticlockwise motion.
   */
  int16_t cycles = degToCycles(degrees);
  // Choose the direction
  uint8_t direction = (cycles > 0) ? CLOCKWISE : ANTICLOCKWISE;
  digitalWrite(ROTATING_PLATE_DIR, direction);
  for (uint16_t x = 0; x <= abs(cycles); x++) {
    digitalWrite(ROTATING_PLATE_STEP, HIGH);
    delay(freq);
    digitalWrite(ROTATING_PLATE_STEP, LOW);
    // currentRotation = (currentRotation+direction) % STEPS;
    delay(freq);
  }
}

void openLatch(uint8_t angle = 0) {
  /*
   * openLatch()
   * ===========
   * Opens the collection latch.
   * If an angle is specified it is opened to that angle.
   */
  latch.write(angle);
  latchOpen = true;
}

void closeLatch(uint8_t angle = 120) {
  /*
   * closeLatch()
   * ===========
   * Closes the collection latch.
   * If an angle is specified it is opened to that angle.
   */
  latch.write(angle);
  latchOpen = false;
}

uint8_t mode = 0;
uint16_t value = 0;

void loop() {
  // put your main code here, to run repeatedly:

  // Wait for some input.
  while (!Serial.available()) {
    delay(10);
  }
  int8_t dir = 1;
  Serial.println("Heard!");
  while (Serial.available()) {
    char ch = Serial.read();
    // If it is a mode selection character then set mode.
    if (ch == 'L') {
      Serial.println("Latch");
      mode = 5;
    } else if (ch == 'T') {
      Serial.println("Rotation");
      mode = 1;
    } else if (ch == 'X') {
      Serial.println("Extension");
      mode = 2;
    } else if (ch == 'V') {
      Serial.println("Elevation");
      mode = 3;
    } else if (ch == 'D') {
      Serial.println("Drive");
      mode = 4;
    } else if (ch == 'S') {
      Serial.println("Speed");
      mode = 11;
    } else if (ch == 'F') {
      Serial.println("Frequency");
      mode = 12;
    // If it includes '-' then set dir to negative.
    } else if (ch == '-') {
      Serial.println("Backwards!");
      dir = -1;
    // If it is a number then accumulate the value.
    } else if (ch >= '0' && ch <= '9') {
      value = (value * 10) + (ch - '0');  // Accumulate the value
    // If it is the string terminator then perform the action based on the mode.
    } else if (ch == 10) {
      Serial.println(value * dir);
      if (mode == 1) {
        Serial.println("Rotation");
        armRotateBy(value * dir);

      } else if (mode == 2) {
        Serial.println("Extension");
        arm_extension.write(90 + speed*dir);
        delay(value);
        arm_extension.write(90);

      } else if (mode == 3) {
        Serial.println("Elevation");
        // No elevation functionality.

      } else if (mode == 4) {
        Serial.println("Drive");
        if (dir == 1) {
          digitalWrite(DC_MOTOR_PIN1, HIGH);
          digitalWrite(DC_MOTOR_PIN2, LOW);
        } else {
          digitalWrite(DC_MOTOR_PIN1, LOW);
          digitalWrite(DC_MOTOR_PIN2, HIGH);
        }
        digitalWrite(DC_MOTOR_ENABLE, HIGH);
        delay(value);
        digitalWrite(DC_MOTOR_ENABLE, LOW);

      } else if (mode == 5) {
        Serial.println("Latch");
        if (latchOpen) {
          Serial.println("Close");
          closeLatch();
        } else {
          Serial.println("Open");
          openLatch();
        }

      } else if (mode == 11) {
        Serial.println("Speed");
        speed = value;
        Serial.println(speed);
      } else if (mode == 12) {
        Serial.println("Frequency");
        freq = value;
        Serial.println(freq);
      }

      value = 0;  // reset val to 0 ready for the next sequence of digits
      Serial.println("Finished");
    }
  }
}

/*
 * L - Opens latch
 * T angle - Stepper
 * X ms - Extension
 * V ms - Elevator
 * D ms - Drive
 * F int - Rotating Frequency
 * S int - Servo Speed
 *
 * Important examples:
 * L
 * T90
 * T-90
 * X1000
 * X-1000
 * F2
 * S50
 */
