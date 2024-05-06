#include <ESP32Servo.h>
#include <Stepper.h>
#include <thread>
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
 Modified 2024-04-26
 By Gideon McKinlay
 */

enum Speed {
  walk = 100,
  trot = 10,
  canter = 5,
  gallop = 1
};

Speed speed = trot;

// FT5116M Servos
#define ARM_EXTENSION_PIN 23
#define ELEVATOR_PIN 11
// A4988 Stepper
#define ROTATING_PLATE_DIR 12
#define ROTATING_PLATE_STEP 14
#define STEPS 200
// FIT0521 DC
#define DC_MOTOR_PIN1 26
#define DC_MOTOR_PIN2 27
#define DC_MOTOR_ENABLE 13
// Servo
#define LATCH_SERVO_PIN 4
#define TILT_SERVO_PIN 5
// Limit Switch
#define LIMIT_SWITCH_PIN 35
#define IR_SENSOR_PIN 18

#define CLOCKWISE HIGH
#define ANTICLOCKWISE LOW

Servo arm_extension;
Servo elevator;
Servo latch;
Servo tilt;

uint16_t currentExtension = 0;
uint16_t currentHeight = 0;
uint8_t currentRotation = 0;

bool latchOpen = false;

int sign(int x) {
  /*
   * Returns the sign of a number.
   * x > 0: 1
   * x = 0: 0
   * x < 0: -1
   */
  if (x > 0) {
    return 1;
  }
  if (x < 0) {
    return -1;
  }
  return 0;
}

void setup() {
  // Attach servos to pins
  arm_extension.attach(ARM_EXTENSION_PIN);
  elevator.attach(ELEVATOR_PIN);

  pinMode(ROTATING_PLATE_DIR, OUTPUT);
  pinMode(ROTATING_PLATE_STEP, OUTPUT);
  
  pinMode(DC_MOTOR_PIN1, OUTPUT);
  pinMode(DC_MOTOR_PIN2, OUTPUT);
  pinMode(DC_MOTOR_ENABLE, OUTPUT);

  currentExtension = 0;
  currentHeight = 0;
  currentRotation = 0;

  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLDOWN);
}

void armExtendTo(uint16_t distance) {
  for (int i = currentExtension; i != distance; i += sign(distance-currentExtension)) {
    arm_extension.write(i);
    currentExtension = i;
    delay(speed);
  }
  currentExtension = distance;
}

void changeHeightTo(uint16_t height) {
  for (int i = currentHeight; i != height; i += sign(height-currentHeight)) {
    elevator.write(i);
    currentHeight = i;
    delay(speed);
  }
  currentHeight = height;
}

void armRotateTo(uint8_t point, uint8_t direction) {
  // Choose the direction
  digitalWrite(ROTATING_PLATE_DIR, direction);
  // TODO: Use >= or <= for comparison
  while (currentRotation != point) {
    digitalWrite(ROTATING_PLATE_STEP, HIGH); // Move one step
    delay(speed);
    digitalWrite(ROTATING_PLATE_STEP, LOW); // Move one step
    currentRotation = (currentRotation+direction) % STEPS;
    delay(speed);
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

void driveToSide() {
  digitalWrite(DC_MOTOR_PINA, HIGH);
  digitalWrite(DC_MOTOR_PINB, LOW);

  // Set motor to drive
  digitalWrite(DC_MOTOR_ENABLE, HIGH);

  // Wait until correct switch is pressed
  waitForLimitPress();
  
  // Set motor to stop
  digitalWrite(DC_MOTOR_ENABLE, LOW);
}

void waitForLimitPress() {
  while (!digitalRead(LIMIT_SWITCH_PIN)) {
    delay(50);
    Serial.println("Lim: " + digitalRead(LIMIT_SWITCH_PIN));
  }
  Serial.println("Lim: PRESSED");
}

void waitForIRSense() {
  while (digitalRead(IR_SENSOR_PIN)) {
    delay(50);
    Serial.println(" IR: " + !digitalRead(IR_SENSOR_PIN));
  }
  Serial.println(" IR: DETECTED");
}

void moveToPosition(uint8_t rotation, uint8_t direction, uint8_t extension, uint8_t elevation) {
  std::thread t1(armRotateTo, rotation, direction);
  std::thread t2(armExtendTo, extension);
  std::thread t3(changeHeightTo, elevation);
  t1.join();
  t2.join();
  t3.join();
}

void loop() {

  // Wait until operation is started
  while (!digitalRead(LIMIT_SWITCH_PIN)) {
    delay(50);
  }

  // Move to first target
  moveToPosition(0, CLOCKWISE, 180, 180);
  closeLatch();
  // Move to hole
  moveToPosition(90, ANTICLOCKWISE, 90, 0);
  openLatch();
  // Move to second target
  moveToPosition(0, CLOCKWISE, 180, 180);
  closeLatch();
  // Move to hole
  moveToPosition(90, ANTICLOCKWISE, 90, 0);
  openLatch();
  // Move to third target
  moveToPosition(0, CLOCKWISE, 180, 180);
  closeLatch();
  // Move to hole
  moveToPosition(90, ANTICLOCKWISE, 90, 0);
  openLatch();

  // Drive to opposite side
  std::thread t1(driveToSide);
  std::thread t2(armRotateTo, 0, CLOCKWISE);
  std::thread t3(armExtendTo, 180);
  std::thread t4(changeHeightTo, 180);
  t1.join();
  t2.join();
  t3.join();
  t4.join();

  // Move to first target
  moveToPosition(0, CLOCKWISE, 180, 180);
  closeLatch();
  // Move to hole
  moveToPosition(90, ANTICLOCKWISE, 90, 0);
  openLatch();
  // Move to second target
  moveToPosition(0, CLOCKWISE, 180, 180);
  closeLatch();
  // Move to hole
  moveToPosition(90, ANTICLOCKWISE, 90, 0);
  openLatch();
  // Move to third target
  moveToPosition(0, CLOCKWISE, 180, 180);
  closeLatch();
  // Move to hole
  moveToPosition(90, ANTICLOCKWISE, 90, 0);
  openLatch();

  while (true) {
    delay(1000);
  }
}
