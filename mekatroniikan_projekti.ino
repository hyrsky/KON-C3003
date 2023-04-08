#include "SparkFun_TB6612.h"
#include "LSM6DSOX.h"
#include "PID.h"

// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
// the default pins listed are the ones used on the Redbot (ROB-12097) with
// the exception of STBY which the Redbot controls with a physical switch
#define AIN1 33
#define BIN1 26
#define AIN2 25
#define BIN2 27
#define PWMA 14
#define PWMB 12
#define STBY 32

// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// setting PWM properties
const int freq = 5000;
const int ledChannelA = 0;
const int ledChannelB = 1;
const int resolution = 8;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
Motor motor1 = Motor(AIN1, AIN2, ledChannelA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, ledChannelB, offsetB, STBY);

#define I2C_SDA 22
#define I2C_SCL 23
#define I2C_FREQ 400000 // 400 kHz

TwoWire I2CIMU = TwoWire(0);
LSM6DSOX IMU = LSM6DSOX(I2CIMU, I2C_SDA, I2C_SCL, I2C_FREQ, LSM6DSOX_ADDRESS);

TaskHandle_t imu_task_handle;
QueueHandle_t imu_mailbox;

#define ACCL_WEIGHT 0.05
#define GYRO_WEIGHT (1.0 - ACCL_WEIGHT)
#define IMU_SAMPLING_RATE 20

/**
 * Angles in radians
 */
struct Angles {
  float phi;
  float theta;
};

// Define variables for the PID controller
PIDController pid;

void print_angles(const Angles& angles) {
  static char buff[128];
  //sprintf(buff, "p:%.3f,t:%.3f", angles.phi * RAD_TO_DEG, angles.theta * RAD_TO_DEG);
  //Serial.print(buff);
  sprintf(buff, "p:%.3f", angles.phi * RAD_TO_DEG);
  Serial.print(buff);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // configure LED PWM functionalitites
  ledcSetup(ledChannelA, freq, resolution);
  ledcSetup(ledChannelB, freq, resolution);

  ledcAttachPin(PWMA, ledChannelA);
  ledcAttachPin(PWMB, ledChannelB);

  PID_init(&pid);
  pid.Kp = 2;
  pid.Ki = 0.5;
  pid.Kd = 0.001;
  pid.tau = 0.01;
  pid.limit_max = 255;
  pid.limit_min = -255;

  IMU.begin();

  imu_mailbox = xQueueCreate(1, sizeof(Angles));

  xTaskCreate(
    IMU_task, /* Task function. */
    "IMU-task", /* name of task. */
    10000, /* Stack size of task */
    NULL, /* parameter of the task */
    5, /* priority of the task */
    &imu_task_handle /* Task handle to keep track of created task */
  );
}

void IMU_task(void *args) {
  LSM6DSOX_out sensor;

  // Estimated euler angles.
  Angles estimated;
  estimated.theta = 0;
  estimated.phi = 0;

  while(1)
  {
    Angles gyro; // Gyro estimate
    Angles accl; // Accelerometer estimate;

    // Why would this not work?
    IMU.readSensor(sensor);

    // Elapsed time in seconds - todo: verify timings.
    float dt = IMU_SAMPLING_RATE / 1000.0;

    // Calculate roll and pitch from accelerometer values.
    accl.phi = atanf(sensor.y / sensor.z);
    accl.theta = asinf(sensor.x / ACCL_DUE_TO_GRAVITY_MPS2);

    // Transform gyro rates to euler rates.
    float phidot_radps   = sensor.p + tanf(estimated.phi) * (sinf(estimated.phi) * sensor.q + cosf(estimated.phi) * sensor.r);
    float thetadot_radps =                                  (cosf(estimated.phi) * sensor.q - sinf(estimated.phi) * sensor.r);

    // Integrate euler rates to estimate euler angles.
    gyro.phi   = estimated.phi   + phidot_radps   * dt;
    gyro.theta = estimated.theta + thetadot_radps * dt;

    // Complementary filter
    estimated.phi   = ACCL_WEIGHT * accl.phi   + GYRO_WEIGHT * gyro.phi;
    estimated.theta = ACCL_WEIGHT * accl.theta + GYRO_WEIGHT * gyro.theta;

    // Push estimated angles to queue - best effort.
    xQueueSendToBack(imu_mailbox, &estimated, 0);

    delay(IMU_SAMPLING_RATE);
  }
}

unsigned long prev = 0;
float setpoint = 0.0f;

void loop() {
  Angles angles;

  // Receive from IMU task.
  if (xQueueReceive(imu_mailbox, &angles, 1) == pdTRUE) {
    unsigned long next = millis();
    float dt = (next - prev) / 1000.0f;
    prev = next;

    float out = PID_update(&pid, setpoint, angles.phi, dt);
    motor1.drive(out);

    print_angles(angles);
    Serial.print(",PWM:");
    Serial.print(out);
  }

  if (Serial.available() > 0) {
    // Read the incoming data and store it as a string
    String data = Serial.readStringUntil('\n');

    int startPos = 0;
    float values[4];

    // Read four values
    for (int i = 0; i < 4; ++i) {
      // Find the position of the comma in the string
      int commaPos = data.indexOf(',', startPos);

      String substr;

      // If a comma was found, extract the two values
      if (commaPos >= 0) {
        substr = data.substring(startPos, commaPos);
        startPos = commaPos + 1;
      } else {
        substr = data.substring(startPos); // Last entry
      }

      // Convert the values to floats
      values[i] = substr.toFloat();
    }
    
    pid.Kp  = values[0];
    pid.Ki  = values[1];
    pid.Kd  = values[2];
    pid.tau = values[3];

    Serial.print("Kp:");
    Serial.print(pid.Kp);
    Serial.print(",Ki:");
    Serial.print(pid.Ki);
    Serial.print(",Kd:");
    Serial.print(pid.Kd);
    Serial.print(",tau:");
    Serial.println(pid.tau);
  }
}
