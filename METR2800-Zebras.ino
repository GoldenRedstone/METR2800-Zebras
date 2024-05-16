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
 Modified 2024-05-15
 By Gideon McKinlay
 */

/*
 * 
 */
enum Speed {
  walk = 10,
  trot = 5,
  canter = 2,
  gallop = 1
};

Speed speed = trot;

// FT5116M Servos
#define ARM_EXTENSION_PIN 23
#define ELEVATOR_PIN1 11
#define ELEVATOR_PIN2 32
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

int16_t degToCycles(float degrees) {
  /*
   * Converts a (float) number of degrees into
   * the number of cycles needed to rotate that angle.
   */
  float n = (degrees * MICRO_STEPS * STEPS * GEAR_RATIO1)/(360 * GEAR_RATIO2);
  return (n + 0.5f);
}

void setup() {
  // Attach servos to pins
  arm_extension.attach(ARM_EXTENSION_PIN);
  elevator.attach(ELEVATOR_PIN);
  latch.attach(LATCH_SERVO_PIN);
  // Set stepper outputs
  pinMode(ROTATING_PLATE_DIR, OUTPUT);
  pinMode(ROTATING_PLATE_STEP, OUTPUT);
  // Set DC motor outputs
  pinMode(DC_MOTOR_PIN1, OUTPUT);
  pinMode(DC_MOTOR_PIN2, OUTPUT);
  pinMode(DC_MOTOR_ENABLE, OUTPUT);
  // Set limit switch to input
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLDOWN);
  // Reset extension and height variables
  currentExtension = 0;
  currentHeight = 0;
}

void armExtendTo(uint16_t distance) {
  /*
   * armExtendTo()
   * =============
   * Extends the arm radially to the given distance.
   * distance is an integer number in arbitrary units.
   */
  uint16_t movementDuration = (distance-currentExtension)/speed;
  uint16_t movementMagnitude = speed*sign(distance-currentExtension);
  arm_extension.write(90 + movementMagnitude);
  delay(movementDuration);
  arm_extension.write(90);
  currentExtension = distance;
}

void changeHeightTo(uint16_t height) {
  /*
   * changeHeightTo()
   * ================
   * Raises the arm vertically to the given height.
   * height is an integer number in arbitrary units.
   */
  uint16_t movementDuration = speed*(height-currentHeight);
  uint16_t movementMagnitude = speed*sign(height-currentHeight);
  elevator.write(90 + movementMagnitude);
  delay(movementDuration);
  elevator.write(90);
  currentHeight = height;
}

void armRotateBy(int16_t cycles) {
  /*
   * armRotateBy()
   * =============
   * Rotates the rotating plate by the given number of cycles.
   * Positive values result in clockwise motion,
   * and negative values result in anticlockwise motion.
   *
   * cycles = angle * GEAR_RATIO * MICRO_STEPS * STEPS * 1/360
   * 360 degrees is 6667 points.
   */
  // int16_t cycles = degToCycles(degrees);
  // Choose the direction
  uint8_t direction = (cycles > 0) ? CLOCKWISE : ANTICLOCKWISE;
  digitalWrite(ROTATING_PLATE_DIR, direction);
  // Repeat for the given number of cycles.
  for(uint16_t x = 0; x <= abs(cycles); x++) {
    digitalWrite(ROTATING_PLATE_STEP, HIGH);
    delay(speed);
    digitalWrite(ROTATING_PLATE_STEP, LOW);
    // currentRotation = (currentRotation+direction) % STEPS;
    delay(speed);
  }
}

void openLatch(uint_8 angle = 0) {
  /*
   * openLatch()
   * ===========
   * Opens the collection latch.
   * If an angle is specified it is opened to 
   */
  latch.write(angle);
  latchOpen = true;
}

void closeLatch() {
  /*
   * closeLatch()
   * ===========
   * Closes the collection latch.
   */
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

void moveToPosition(uint8_t rotation, uint8_t extension, uint8_t elevation) {
  /*
    moveToPosition()
    ================
    Moves the rotating plate to the given rotation, extends the arm to the given extension,
    and raises the arm to the given elevation.
   */
  // Uses three threads to do the three motions so that the resultant motion occurs in parallel.
  std::thread t1(armRotateBy, rotation);
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
  moveToPosition(90, 100, 0);
  closeLatch();
  // Move to hole
  moveToPosition(90, 90, 0);
  openLatch();
  // Move to second target
  moveToPosition(135, CLOCKWISE, 180, 180);
  closeLatch();
  // Move to hole
  moveToPosition(-135, ANTICLOCKWISE, 90, 0);
  openLatch();
  // Move to third target
  moveToPosition(180, CLOCKWISE, 180, 180);
  closeLatch();
  // Move to hole
  moveToPosition(-180, ANTICLOCKWISE, 90, 0);
  openLatch();

  // Drive to opposite side
  std::thread t1(driveToSide);
  std::thread t2(armRotateBy, 0, CLOCKWISE);
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
