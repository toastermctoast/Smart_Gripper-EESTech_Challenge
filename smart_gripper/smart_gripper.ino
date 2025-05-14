/**
 * Torque control example of adaptive gripper. Based on SimpleFOC library.
 *
 * 1. After flashing this code to the XMC4700 Relax Kit, angle alignment will be
 *    applied. The gripper must be opened to its fullest extent to release the gear
 *    and minimize load for alignment.
 *
 * 2. After angle alignment, attach the gears (manually close the gripper a bit
 *    to align the gears) and you can start controlling the gripper's opening and
 *    closing. Pressing button 1 on the board will close the gripper, and pressing
 *    button 2 will open it. Note: There is no upper limit set for opening, so it
 *    is possible that the gears may detach if the maximum is exceeded.
 *
 * 3. Open the serial monitor/serial plotter to view data from the magnetic
 *    sensors placed under the TPU material on top of the clip. When the gripping
 *    clip grabs an object and generates pressure, the data changes.
 *
 * This is a basic example; you can be creative to improve this gripper!
 */
#include "TLE5012Sensor.h"
#include "TLx493D_inc.hpp"
#include "config.h"
#include <SimpleFOC.h>

<<<<<<< Updated upstream
// define SPI pins for TLE5012 sensor
#define PIN_SPI1_SS0 94  // Chip Select (CS) pin
#define PIN_SPI1_MOSI 69 // MOSI pin
#define PIN_SPI1_MISO 95 // MISO pin
#define PIN_SPI1_SCK 68  // SCK pin

enum ObjectType {
  NO_OBJECT,
  SOFT_OBJECT,
  MEDIUM_OBJECT,
  HARD_OBJECT
};

=======
>>>>>>> Stashed changes
// create an instance of SPIClass3W for 3-wire SPI communication
tle5012::SPIClass3W tle5012::SPI3W1(2);
// create an instance of TLE5012Sensor
TLE5012Sensor tle5012Sensor(&SPI3W1, PIN_SPI1_SS0, PIN_SPI1_MISO, PIN_SPI1_MOSI,
                            PIN_SPI1_SCK);

// BLDC motor instance BLDCMotor (polepairs, motor phase resistance, motor KV
// rating, motor phase inductance)
BLDCMotor motor = BLDCMotor(
    7, 0.24, 360,
    0.000133); // 7 pole pairs, 0.24 Ohm phase resistance, 360 KV and 0.000133H
// you can find more data of motor in the doc

// BLDC driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(U, V, W, EN_U, EN_V, EN_W);

<<<<<<< Updated upstream
// voltage set point variable
float target_voltage = -1;
double B_abs;

=======
>>>>>>> Stashed changes
float target_angle = 2.0;          // Angle target in radians
const float angle_step = 1;        // Change per button press
ObjectType object;

#if ENABLE_MAGNETIC_SENSOR
// create a instance of 3D magnetic sensor
using namespace ifx::tlx493d;
TLx493D_A2B6 dut(Wire1, TLx493D_IIC_ADDR_A0_e);
// define the number of calibration samples
const int CALIBRATION_SAMPLES = 20;
// offsets for calibration
double xOffset = 0, yOffset = 0, zOffset = 0;
#endif

#if ENABLE_COMMANDER
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char *cmd) { command.scalar(&target_voltage, cmd); }
#endif

void setup() {
  // use monitoring with serial
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  // initialise magnetic sensor hardware
  tle5012Sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&tle5012Sensor);

  // power supply voltage
  driver.voltage_power_supply = 12;
  // limit the maximal dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  driver.voltage_limit = 6;
  if (!driver.init()) {
    Serial.println("Driver init failed!");
    return;
  }
  // link the motor and the driver
  motor.linkDriver(&driver);

  // aligning voltage
  motor.voltage_sensor_align = 2;
  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // set motion control loop to be used
  //motor.controller = MotionControlType::torque;

  motor.controller = MotionControlType::angle;

  // velocity PID controller parameters
  // default P=0.5 I = 10 D =0
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0.001;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best. 
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01;

  // setting the limits
  // either voltage
  motor.voltage_limit = 10; // Volts - default driver.voltage_limit
  // of current 
  motor.current_limit = 2; // Amps - default 0.2Amps

  // angle PID controller 
  // default P=20
  motor.P_angle.P = 20; 
  motor.P_angle.I = 0;  // usually only P controller is enough 
  motor.P_angle.D = 0;  // usually only P controller is enough 
  // acceleration control using output ramp
  // this variable is in rad/s^2 and sets the limit of acceleration
  motor.P_angle.output_ramp = 10000; // default 1e6 rad/s^2

  // angle low pass filtering
  // default 0 - disabled  
  // use only for very noisy position sensors - try to avoid and keep the values very small
  motor.LPF_angle.Tf = 0; // default 0

  // setting the limits
  //  maximal velocity of the position control
  motor.velocity_limit = 4; // rad/s - default 20


  // comment out if not needed
  // motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();
  Serial.println(F("Motor ready."));

  #if ENABLE_MAGNETIC_SENSOR
  // start 3D magnetic sensor
  dut.begin();
  // calibrate 3D magnetic sensor to get the offsets
  calibrateSensor();
  Serial.println("3D magnetic sensor Calibration completed.");

  // set the pin modes for buttons
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
  #endif

  Serial.print("setup done.\n");
  _delay(1000);
}

void loop() {

  // -- Gripper Control with Buttons --
  if (digitalRead(BUTTON1) == LOW) {
    target_angle += angle_step;
    //if (target_angle > -1) target_angle = -1;
    delay(150); // debounce
  }
   
  else if (digitalRead(BUTTON2) == LOW) {
    target_angle -= angle_step;
    //if (target_angle < -18) target_angle = -18;
    delay(150); // debounce
  }

  double x, y, z, B_abs;
  getB(&x, &y, &z);

  Serial.print("Target angle: ");
  Serial.print(target_angle);
  Serial.print(" | Shaft angle: ");
  Serial.println(motor.shaftAngle());

  B_abs = sqrt(x*x + y*y + z*z);

<<<<<<< Updated upstream
    // print the magnetic field data
    Serial.print("Magnetic Field: ");
    Serial.print(x);
    Serial.print(",");

    Serial.print(y);
    Serial.print(",");

    Serial.print(z);
    Serial.println("");

    B_abs = sqrt(x*x + y*y + z*z);

    object = is_there_object(B_abs);

  #endif
=======
  object = is_there_object(B_abs);
>>>>>>> Stashed changes

  // update angle sensor data
  tle5012Sensor.update();

  #if ENABLE_READ_ANGLE
    Serial.print(tle5012Sensor.getSensorAngle());
    Serial.println("");
  #endif

  //--------MOVE MOTOR----------               

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();    

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_angle);         // PID angle control
    
  #if ENABLE_COMMANDER
    // user communication
    command.run();
  #endif

  Serial.println();
}

#if ENABLE_MAGNETIC_SENSOR
  /**
  * @brief Calibrates the magnetic field sensor by calculating the average
  * offsets for the X, Y, and Z axes over a series of samples.
  */
void calibrateSensor() {
  double sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < CALIBRATION_SAMPLES; ++i) {
    double temp;
    double valX, valY, valZ;

    dut.getMagneticFieldAndTemperature(&valX, &valY, &valZ, &temp);
    sumX += valX;
    sumY += valY;
    sumZ += valZ;

    delay(10); // Adjust delay as needed
  }

  // Calculate average offsets
  xOffset = sumX / CALIBRATION_SAMPLES;
  yOffset = sumY / CALIBRATION_SAMPLES;
  zOffset = sumZ / CALIBRATION_SAMPLES;

}

<<<<<<< Updated upstream

=======
void getB(double* x, double* y, double* z){
  dut.getMagneticField(x, y, z);
  *x -= xOffset;
  *y -= yOffset;
  *z -= zOffset;

  Serial.print("Magnetic Field: ");
  Serial.print(*x);
  Serial.print(",");
  Serial.print(*y);
  Serial.print(",");
  Serial.println(*z);
}
#endif
>>>>>>> Stashed changes

ObjectType is_there_object(double B) {

  static double last_B = 0;

  double d_B = fabs(B - last_B);
  last_B = B;

  if (d_B > 0.15) {   // Object detected
    Serial.println("🟥 Hard object detected");
    return HARD_OBJECT;

  } else if (d_B > 0.05) {
    Serial.println("🟨 Medium-soft object detected");
    return MEDIUM_OBJECT;

  } else if (d_B > 0.01) {
    Serial.println("🟩 Soft object detected");
    return SOFT_OBJECT;
  }
  return NO_OBJECT;

}



