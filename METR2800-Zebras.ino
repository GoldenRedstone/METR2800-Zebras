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
#define DC_MOTOR_PIN3 22
#define DC_MOTOR_PIN4 25
#define DC_MOTOR1_ENABLE 14
#define DC_MOTOR2_ENABLE 34
// SG90 Servo
#define LATCH_SERVO1_PIN 4
#define LATCH_SERVO2_PIN 5
// Limit Switch
#define LIMIT_SWITCH_PIN 35
#define IR_SENSOR_PIN 18

#define CLOCKWISE HIGH
#define ANTICLOCKWISE LOW

// Servos
Servo arm_extension;
Servo elevator1;
Servo elevator2;
Servo latch1;
Servo latch2;

// Set speeds
int8_t ex_speed = 50;   // Speed of extending arm. Larger is faster. 50 is almost max.
int8_t el_speed = 20;   // Speed of elevator. Larger is faster. 50 is almost max.
int8_t freq = 1;        // Frequency of rotating plate. Smaller is faster.

float grav_comp = 1.3;  // Compensate for elevator faster going down.

int16_t currentExtension;
int16_t currentHeight;
int16_t currentRotation;
bool latchOpen;


// Utility functions.
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


// Motion is broken into multiple functions.

void armExtendBy(int16_t duration) {
    /*
     * armExtendBy()
     * =============
     * Extends the arm radially for the given duration.
     * duration is an integer number in ms.
     * Negative extends, and positive retracts.
     */
    int8_t direction = sign(duration);
    int16_t magnitude = (50 * abs(duration)) / el_speed;
    arm_extension.write(90 + ex_speed * direction);
    delay(magnitude);
    arm_extension.write(90);
    currentExtension += duration;
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
    currentHeight += duration;
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
    latch1.write(angle);
    latch2.write(angle);
    latchOpen = true;
}

void closeLatch(int16_t angle = 90) {
    /*
     * closeLatch()
     * ===========
     * Closes the collection latch.
     * If an angle is specified it is closed to that angle.
     */
    for (uint16_t x = 30; x > 0; x--) {
      latch1.write(angle - (angle*x)/30);
      latch2.write(angle - (angle*x)/30);
      delay(freq);
    }
    latch1.write(angle);
    latch2.write(angle);
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
    digitalWrite(DC_MOTOR_PIN3, HIGH);
    digitalWrite(DC_MOTOR_PIN4, LOW);
    // Set motor to drive
    digitalWrite(DC_MOTOR1_ENABLE, HIGH);
    digitalWrite(DC_MOTOR2_ENABLE, HIGH);
    // Wait until switch is pressed
    waitForLimitPress();
    // Set motor to stop
    digitalWrite(DC_MOTOR1_ENABLE, LOW);
    digitalWrite(DC_MOTOR2_ENABLE, LOW);
}


// Functions used to wait for conditions.
void waitForLimitPress(int32_t timeout = 0) {
    /*
     * waitForLimitPress()
     * ===================
     * Enter a loop until the limit switch is pressed.
     * If a timeout is provided then the function will also return after that amount of time in ms.
     */
    int32_t startTime = millis();
    while (!digitalRead(LIMIT_SWITCH_PIN)) {
        if (timeout && millis() >= startTime+timeout) {
            return;
        }
        delay(10);
    }
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
    // Wait for all three to be done.
    t1.join();
    t2.join();
    t3.join();
}

void setup() {
    // Attach servos to pins
    arm_extension.attach(ARM_EXTENSION_PIN);
    elevator1.attach(ELEVATOR1_PIN);
    elevator2.attach(ELEVATOR2_PIN);
    latch1.attach(LATCH_SERVO1_PIN);
    latch2.attach(LATCH_SERVO2_PIN);
    // Set stepper outputs
    pinMode(ROTATING_PLATE_DIR, OUTPUT);
    pinMode(ROTATING_PLATE_STEP, OUTPUT);
    // Set DC motor outputs
    pinMode(DC_MOTOR_PIN1, OUTPUT);
    pinMode(DC_MOTOR_PIN2, OUTPUT);
    pinMode(DC_MOTOR_PIN3, OUTPUT);
    pinMode(DC_MOTOR_PIN4, OUTPUT);
    pinMode(DC_MOTOR1_ENABLE, OUTPUT);
    pinMode(DC_MOTOR2_ENABLE, OUTPUT);
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
     * Prancing in the great paddock in the sky.
     *     _,,        
     *    "-.\=       
     *      \\=   _.~ 
     *      _|/||||)_ 
     *ejm97 \        \    
      */
}