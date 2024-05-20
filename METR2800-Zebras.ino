#include <ESP32Servo.h>
#include <Stepper.h>
#include <thread>
/*
    \\/),
   ,'.' /,
  (_)- / /,
     /\_/ |__..--,  *
    (\___/\ \ \ / ).'
     \____/ / (_ //
      \\_ ,'--'\_(
      )_)_/ )_/ )_)
mrf  (_(_.'(_.'(_.'
 ====================
   METR2800 ZEBRAS
 Alistair, Oliver, Bailey, Wilbur, Gideon, Jestin
 Modified 2024-05-20
 By Gideon McKinlay
 */

enum Speed { walk = 10, trot = 5, canter = 2, gallop = 1 };

Speed speed = trot;

// FT5116M Servos
#define ARM_EXTENSION_PIN 23
#define ELEVATOR1_PIN 33
#define ELEVATOR2_PIN 32
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
// SG90 Servo
#define LATCH_SERVO_PIN 4
#define TILT_SERVO_PIN 5
// Limit Switch
#define LIMIT_SWITCH_PIN 35
#define IR_SENSOR_PIN 18

#define CLOCKWISE HIGH
#define ANTICLOCKWISE LOW

Servo arm_extension;
Servo elevator1;
Servo elevator2;
Servo latch;

int8_t ex_speed = 50;
int8_t el_speed = 20;
int8_t freq = 1;

float grav_compensation = 1.3;

int16_t currentExtension;
int16_t currentHeight;
int16_t currentRotation;
bool latchOpen;

int8_t sign(int16_t x) {
   /*
    * sign()
    * ======
    * Returns the sign of a number.
    * x > 0: 1
    * x = 0: 0
    * x < 0: -1
    */
    if (x > 0) { return 1; }
    if (x < 0) { return -1; }
    return 0;
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
    float n = (degrees * MICRO_STEPS * STEPS * GEAR_RATIO1) / (360 * GEAR_RATIO2);
    return (n + 0.5f);
}

void armExtendBy(int16_t duration) {
    /*
     * armExtendBy()
     * =============
     * Extends the arm radially for the given duration.
     * duration is an integer number in ms.
     */
    int8_t direction = sign(duration);
    int16_t magnitude = (50 * abs(duration)) / el_speed;
    arm_extension.write(90 + ex_speed * direction);
    delay(magnitude);
    arm_extension.write(90);
}

void changeHeightBy(int16_t duration) {
    /*
     * changeHeightBy()
     * ================
     * Extends the arm vertically for the given duration.
     * duration is an integer number in ms.
     * Positive is upwards, negative is downwards.
     */
    int8_t direction = sign(duration);
    int16_t magnitude = (20 * abs(duration)) / el_speed;
    elevator1.write(90 - el_speed * direction);
    elevator2.write(90 - el_speed * direction);
    delay(magnitude);
    elevator1.write(90);
    elevator2.write(90);
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
    int8_t direction = (cycles > 0) ? ANTICLOCKWISE : CLOCKWISE;
    digitalWrite(ROTATING_PLATE_DIR, direction);
    for (int16_t x = 0; x <= abs(cycles); x++) {
        digitalWrite(ROTATING_PLATE_STEP, HIGH);
        delay(freq);
        digitalWrite(ROTATING_PLATE_STEP, LOW);
        delay(freq);
    }
    currentRotation = currentRotation + degrees;
}

void openLatch(int16_t angle = 0) {
    /*
     * openLatch()
     * ===========
     * Opens the collection latch.
     * If an angle is specified it is opened to that angle.
     */
    latch.write(angle);
    latchOpen = true;
}

void closeLatch(int16_t angle = 120) {
    /*
     * closeLatch()
     * ===========
     * Closes the collection latch.
     * If an angle is specified it is closed to that angle.
     */
    latch.write(angle);
    latchOpen = false;
}

void driveToSide() {
    /*
     * driveToSide()
     * =============
     * Engages the DC motors until the contact switch is pressed.
     */
    digitalWrite(DC_MOTOR_PIN1, HIGH);
    digitalWrite(DC_MOTOR_PIN2, LOW);
    // Set motor to drive
    digitalWrite(DC_MOTOR_ENABLE, HIGH);
    // Wait until correct switch is pressed
    waitForLimitPress();
    // Set motor to stop
    digitalWrite(DC_MOTOR_ENABLE, LOW);
}

void waitForLimitPress() {
    /*
   * waitForLimitPress()
   * ===================
   * Enter a loop until the limit switch is pressed.
   */
    while (!digitalRead(LIMIT_SWITCH_PIN)) { delay(10); }
}

void waitForIRSense() {
    /*
   * waitForLimitPress()
   * ===================
   * Enter a loop until the IR sensor gives a reading.
   */
    while (digitalRead(IR_SENSOR_PIN)) { delay(10); }
}

void moveToPosition(float rotation, int16_t elevation, int16_t extension) {
    /*
     * moveToPosition()
     * ================
     * Moves the rotating plate to the given rotation, extends the arm to the given extension,
     * and raises the arm to the given elevation.
     */
    // Uses three threads to do the three motions so that the resultant motion occurs in parallel.
    std::thread t1(armRotateBy, rotation);
    std::thread t2(armExtendBy, extension);
    std::thread t3(changeHeightBy, elevation);
    t1.join();
    t2.join();
    t3.join();
}

void setup() {
    // Attach servos to pins
    arm_extension.attach(ARM_EXTENSION_PIN);
    elevator1.attach(ELEVATOR1_PIN);
    elevator2.attach(ELEVATOR2_PIN);
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
    currentRotation = 0;
    latchOpen = false;

    COLLECT();
}

void COLLECT() {
    // Wait for signal to start
    while (!digitalRead(LIMIT_SWITCH_PIN)) { delay(10); }

    // Move to first target
    armRotateBy(120);

    delay(500);

    changeHeightBy(1300);
    // elevator1.write(90 + el_speed);
    // elevator2.write(90 + el_speed);
    // delay(1300);
    // elevator1.write(90);
    // elevator2.write(90);
    // openLatch();

    armRotateBy(60);

    arm_extension.write(90 - ex_speed);
    delay(2000);
    arm_extension.write(90);

    closeLatch();
    delay(500);
    // Move to hole
    armRotateBy(-60);

    delay(500);

    changeHeightBy(-1000);
    // elevator1.write(90 - el_speed);
    // elevator2.write(90 - el_speed);
    // delay(1000);
    // elevator1.write(90);
    // elevator2.write(90);

    armRotateBy(-120);

    openLatch();

    delay(1000);

    arm_extension.write(90 + ex_speed);
    delay(2000);
    arm_extension.write(90);


    // Second target time!
    armRotateBy(-45);

    delay(500);

    changeHeightBy(1300);
    // elevator1.write(90 + el_speed);
    // elevator2.write(90 + el_speed);
    // delay(1300);
    // elevator1.write(90);
    // elevator2.write(90);

    armRotateBy(-60);

    arm_extension.write(90 - ex_speed);
    delay(2000);
    arm_extension.write(90);

    closeLatch();
    delay(500);
    // Move to hole
    armRotateBy(60);

    delay(500);

    changeHeightBy(-1000);
    // elevator1.write(90 - el_speed);
    // elevator2.write(90 - el_speed);
    // delay(1000);
    // elevator1.write(90);
    // elevator2.write(90);

    armRotateBy(45);

    openLatch();

    delay(1000);

    arm_extension.write(90 + ex_speed);
    delay(2000);
    arm_extension.write(90);
}

void loop() {
    /*
          _,,        
          "-.\=       
            \\=   _.~ 
            _|/||||)_ 
    ejm97  \        \    
    */
}