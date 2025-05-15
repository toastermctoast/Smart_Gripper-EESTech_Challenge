#include "TLE5012Sensor.h"
#include "TLx493D_inc.hpp"
#include "config.h"
#include <SimpleFOC.h>

// Create an instance of SPIClass3W for 3-wire SPI communication
tle5012::SPIClass3W tle5012::SPI3W1(2);
// Create an instance of TLE5012Sensor
TLE5012Sensor tle5012Sensor(&SPI3W1, PIN_SPI1_SS0, PIN_SPI1_MISO, PIN_SPI1_MOSI,
                            PIN_SPI1_SCK);

BLDCMotor motor = BLDCMotor(
    7, 0.24, 360,
    0.000133); // 7 pole pairs, 0.24 Ohm phase resistance, 360 KV and 0.000133H

// BLDC driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(U, V, W, EN_U, EN_V, EN_W);

float target_angle;
const float angle_step = 0.5;        // Change per button press
int flag=0;
bool has_object = false;

#if ENABLE_MAGNETIC_SENSOR
// create a instance of 3D magnetic sensor
using namespace ifx::tlx493d;
TLx493D_A2B6 dut(Wire1, TLx493D_IIC_ADDR_A0_e);

const int CALIBRATION_SAMPLES = 20;    // define the number of calibration samples
double xVals[CALIBRATION_SAMPLES], yVals[CALIBRATION_SAMPLES], zVals[CALIBRATION_SAMPLES]; 
double xOffset = 0, yOffset = 0, zOffset = 0;      // offsets for calibration

double filteredX = 0, filteredY = 0, filteredZ = 0;
const double alpha = 0.5;   // higher alpha = more sensitive
ObjectType object;

#endif

#if ENABLE_COMMANDER
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char *cmd) { command.scalar(&target_voltage, cmd); }
#endif

void setup() {

  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  // initialise magnetic sensor hardware
  tle5012Sensor.init();

  // link the motor to the sensor
  motor.linkSensor(&tle5012Sensor);
  // power supply voltage
  driver.voltage_power_supply = 12;
  // limit the maximal dc voltage the driver can set
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
  motor.controller = MotionControlType::angle;

  // velocity PID controller parameters
  motor.PID_velocity.P = 0.5;
  motor.PID_velocity.I = 10;
  motor.PID_velocity.D = 0;

  // jerk control using voltage voltage ramp
  motor.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering (the lower the less filtered)
  motor.LPF_velocity.Tf = 0.01;

  // setting the limits of volts and amps
  motor.voltage_limit = 10; 
  motor.current_limit = 2; 

  // angle PID controller 
  motor.P_angle.P = 20; 
  motor.P_angle.I = 0;  
  motor.P_angle.D = 0;  

  // acceleration control using output ramp
  motor.P_angle.output_ramp = 10000; 

  // angle low pass filtering
  motor.LPF_angle.Tf = 0; 

  // setting the limits
  motor.velocity_limit = 4; 

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();
  Serial.println(F("Motor ready."));

  target_angle= motor.shaftAngle();

  #if ENABLE_MAGNETIC_SENSOR
  // start 3D magnetic sensor
  dut.begin();
  // calibrate 3D magnetic sensor to get the offsets
  calibrateSensor();
  Serial.println("3D magnetic sensor Calibration completed.");

  float tunedP = 0;

  // set the pin modes for buttons
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
  #endif

  Serial.print("setup done.\n");
  _delay(1000);
}


void loop() {

  double x, y, z, B_abs;
  
  if (digitalRead(BUTTON1) == LOW){
    flag=1;
    target_angle= motor.shaftAngle();
  }

  if(flag==1){
    if(object == NO_OBJECT && has_object==false){
        //Gripper Control with Buttons
      if (digitalRead(BUTTON1) == LOW) { 
        target_angle += angle_step;
        delay(150); 
      }
      
      else if (digitalRead(BUTTON2) == LOW) {
        target_angle -= angle_step;
        delay(150); 
      }
    }
    if(object == HARD_OBJECT || object == MEDIUM_OBJECT || object == SOFT_OBJECT){
      Serial.println("ETSOY CA");
      has_object=true;
    }

    if(has_object==true){
      if(digitalRead(BUTTON2)==LOW){
        target_angle += 1;
        delay(150); // debounce
      }
      if(digitalRead(BUTTON1)== LOW){
        target_angle -= 0.05;
        delay(150);
      }
    }

      //MOVE MOTOR               

      motor.loopFOC();    

      motor.move(target_angle);// PID angle control

      getB(&x, &y, &z);   // gets magnetic field
      B_abs = sqrt(x*x + y*y + z*z);    // absolute value of magnetic field
      object = is_there_object(B_abs);   // checks if the change in the magnetic field is big enough to consider it's hit an object
  
  }
 
  tle5012Sensor.update();

  #if ENABLE_COMMANDER
    // user communication
    command.run();
  #endif

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

    delay(10); 
  }

  // Calculate average offsets
  xOffset = sumX / CALIBRATION_SAMPLES;
  yOffset = sumY / CALIBRATION_SAMPLES;
  zOffset = sumZ / CALIBRATION_SAMPLES;
}

void getB(double* x, double* y, double* z){
  dut.getMagneticField(x, y, z);
  *x -= xOffset;
  *y -= yOffset;
  *z -= zOffset;
}
#endif

ObjectType is_there_object(double B) {

  double d_B = B;

  if (d_B > 2) {   
    Serial.println("🟥 Hard object detected");
    return HARD_OBJECT;

  } else if (d_B > 1) {  
    Serial.println("🟨 Medium-hard object detected");
    return MEDIUM_OBJECT;

  } else if (d_B > 0.5) { 
    Serial.println("🟩 Soft object detected");
    return SOFT_OBJECT;
  }
  return NO_OBJECT;
}

   


