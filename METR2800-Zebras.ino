#include <ESP32Servo.h>
/*
    \\/),
   ,'.' /,
  (_)- / /,       mrf
     /\_/ |__..--,  *
    (\___/\ \ \ / ).'
     \____/ / (_ //
      \\_ ,'--'\_(
      )_)_/ )_/ )_)
     (_(_.'(_.'(_.'
 ====================
   METR2800 ZEBRAS
 Alistair, Oliver, Bailey, Wilbur, Gideon, Jestin
 Modified 2024-04-15
 By Gideon McKinlay
 */

/*
 * 1 - Trot
 * 2 - Canter
 * 3 - Gallop
 */
enum Speed {
  trot = 4,
  canter = 2,
  gallop = 1
};

Speed speed = trot;

// FT5116M Servos
#define ARM_EXTENSION_PIN 23
#define ELEVATOR_PIN 11
// Stepper
#define ROTATING_PLATE_DIR 12
#define ROTATING_PLATE_STEP 14
#define STEPS 200
// FIT0521 DC
#define DC_MOTOR_PINA 26
#define DC_MOTOR_PINB 27
// Servo
#define LATCH_SERVO_PIN 4
#define TILT_SERVO_PIN 5
// Limit Switch
#define LIMIT_SWITCH_PIN 13
#define IR_SENSOR_PIN 18

Servo arm_extension;
Servo elevator;
// Servo rotating_plate;
Servo latch;
Servo tilt;

uint16_t currentExtension = 0;
uint16_t currentHeight = 0;
uint8_t currentRotation = 0;

bool latchOpen = false;

void setup() {
  arm_extension.attach(ARM_EXTENSION_PIN);
  elevator.attach(ELEVATOR_PIN);

  pinMode(ROTATING_PLATE_DIR, OUTPUT);
  pinMode(ROTATING_PLATE_STEP, OUTPUT);

  currentExtension = 0;
  currentHeight = 0;
  currentRotation = 0;
}

void armExtendTo(uint16_t distance) {
  arm_extension.write(distance);
  currentExtension = distance;
}

void changeHeightTo(uint16_t height) {
  elevator.write(height);
  currentHeight = height;
}

void armRotateTo(uint8_t point, uint8_t direction) {
  switch(direction) {
    case 1:
      digitalWrite(ROTATING_PLATE_DIR, HIGH); // Clockwise
      break;
    case -1:
      digitalWrite(ROTATING_PLATE_DIR, LOW); // Anti-clockwise
      break;
    default:
      break;
  }
  while (currentRotation != point) {
    digitalWrite(ROTATING_PLATE_STEP, HIGH); // Move one step
    currentRotation += direction;
    delay(50);
  }
}

void openLatch() {
  latch.write(0);
  latchOpen = true;
}
void closeLatch() {
  latch.write(180);
  latchOpen = false;
}

void loop() {
  /*
  START​
  Starts on the left side closest to the tallest tree pod.​
  Arm is in retracted state.​

  Arm pivots towards the first ground pod.​
  Arm extends to the required length.​

  Arm scoops up the first ground pod.​

  Arm pivots to the raised tree pod.​
  Elevator raises the arm to the required height for the tree pod.​
  Arm retracts to the required length.​

  Arm scoops up the tree pod.​

  Arm pivots to the other ground pod.​
  Elevator lowers the arm to the required height for the ground pod.​

  Arm scoops up the other ground pod.​

  Robot drives towards the hole until an IR sensor detects the incinerator hole.​

  Latch opens to deposit all three collected pods into the incinerator hole.​

  Robot drives towards the uncollected pods until the contact switch detects the raised section.​
  Arm pivots towards the first ground pod on the new side.​

  Steps 2 to 8 are repeated for the second side.
  */
}
