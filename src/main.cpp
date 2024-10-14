/*
 * SRAD Avionics Flight Software
 *
 * O. Rangel Morales
 * (GitHub: OrlandoR4)
 * MIT license, all text above must be included in any redistribution
 */

// Include necessary libraries
#include <Wire.h>          // I2C communication
#include <SPI.h>           // SPI communication
#include <SD.h>            // SD card operations

#include <Orientation.h>   // Custom orientation library

// Adafruit sensor libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LSM6DSO32.h>

// Timing variables
int32_t mstime, previoustime, deltatime;

int32_t previoustime_state, deltatime_state = 50;

int32_t deltatime_logslow = 200, deltatime_logfast = 20;
int32_t previoustime_log, deltatime_log = deltatime_logslow;

int32_t previoustime_radio, deltatime_radio = 500;

// Pin definitions for various components
const uint32_t PIN_BUZZER = 33;
const uint32_t PIN_LED = 29;
const uint32_t LSM_CS = 40;    // Chip Select for LSM6DSO32
const uint32_t BMP_CS = 41;    // Chip Select for BMP390
const uint32_t ADXL_CS = 39;   // Chip Select for ADXL375

// Sensor objects
Adafruit_LSM6DSO32 LSM;        // 6-DoF IMU
Adafruit_BMP3XX BMP;           // Barometric pressure sensor
Adafruit_ADXL375 ADXL(ADXL_CS, &SPI, 12345);  // High-G accelerometer
Adafruit_BNO055 BNO(55, 0x28, &Wire);         // 9-DoF IMU

Adafruit_GPS GPS(&Serial2);    // GPS module

// Sensor raw data storage
Vector3 lsm_gyro, lsm_acc;     // LSM6DSO32 gyroscope and accelerometer data
Vector3 adxl_acc;              // ADXL375 accelerometer data

Vector3 bno_gyro, bno_acc, bno_mag;  // BNO055 gyroscope, accelerometer, and magnetometer data
Vector3 bno_euler;                   // BNO055 Euler angles
Quaternion bno_orientation;          // BNO055 orientation as quaternion

// Processed inertial data
Vector3 angular_rate;
Vector3 acceleration_body, acceleration_inertial;
Vector3 magnetic_flux;

Vector3 euler_angles;
Quaternion orientation;

// Temperature readings from various sensors
float lsm_temp, adxl_temp, bno_temp, bmp_temp;

// Barometer data
float bmp_press, bmp_alt;
float off_alt, prev_alt, v_vel;

// Calibration flag and offset for angular rates
bool rates_offset_set = false;
Vector3 rates_offset;

// Flight states enum
enum class States{
  IdleNoCal,  // Idle state before calibration
  Idle,       // Idle state after calibration
  Flight,     // In-flight state
  Landed      // Landed state
} flight_state;

// Thresholds for state detection
#define accel_liftoff_threshold                    30  // METERS PER SECOND^2
#define accel_liftoff_time_threshold              250  // MILLISECONDS
#define land_time_threshold                     30000  // MILLISECONDS
#define land_altitude_threshold                    50  // METERS

// Function to detect liftoff based on acceleration
bool detect_liftoff(uint32_t dt, float meas_accel){
  static uint32_t liftoff_timer;

  // Check if measured acceleration exceeds the liftoff threshold
  if(meas_accel > accel_liftoff_threshold){
    liftoff_timer += dt;  // Accumulate time above threshold

    // If acceleration has been above threshold for sufficient time, declare liftoff
    if(liftoff_timer > accel_liftoff_time_threshold){
      return true;
    }
  }else{
    liftoff_timer = 0;  // Reset timer if acceleration drops below threshold
  }

  return false;  // Liftoff not detected
}

// Function to detect landing based on altitude
bool detect_landing(uint32_t dt, float meas_alt){
  static uint32_t landing_timer;

  // Check if measured altitude is below the landing threshold
  if(meas_alt < land_altitude_threshold){
    landing_timer += dt;  // Accumulate time below threshold
  }else{
    landing_timer = 0;  // Reset timer if altitude goes above threshold
  }

  // If altitude has been below threshold for sufficient time, declare landing
  if(landing_timer > land_time_threshold){
    return true;
  }

  return false;  // Landing not detected
}

// Function to calibrate angular rate offsets
bool rates_offset_calibration(uint32_t dt, uint32_t sample_runtime){
  static uint32_t sampling_time = 0;
  static Vector3 pre_rates_offset;

  // Continue sampling if within the specified runtime and not yet calibrated
  if((sampling_time <= sample_runtime) && !rates_offset_set){
    sampling_time += dt;
    rates_offset_set = false;

    // Only accumulate offset if angular rate is below a threshold (to avoid including actual motion)
    if(angular_rate.get_magnitude() < 5.f * DEG_TO_RAD)
      pre_rates_offset += angular_rate * dt;
  }else{
    rates_offset_set = true;
  }

  // If calibration is complete, calculate the final offset
  if(rates_offset_set){
    rates_offset = pre_rates_offset/float(sample_runtime);
    return true;
  }else{
    return false;
  }
}

// Logging variables and settings
bool log_enable = true;
File csvfile;

// CSV header for logged data
const String data_header = 
"time,lat,lon,"

"sat,spd,cou,g_alt,"

"state,"

"eul_x,eul_y,eul_z,"
"q_w,q_x,q_y,q_z,"
"q_wn,q_xn,q_yn,q_zn,"
"acc_i_x,acc_i_y,acc_i_z,"

"rate_x,rate_y,rate_z,"
"acc_b_x,acc_b_y,acc_b_z,"

"rate_xn,rate_yn,rate_zn,"
"acc_b_xn,acc_b_yn,acc_b_zn,"

"acc_b_xh,acc_b_yh,acc_b_zh,"

"press,alt,vel_z,"
"t_lsm,t_axl,t_bno,t_bmp"
;

// Function to control buzzer and LED pulsing
void set_buzzer_LED_pulse(uint32_t dt, uint32_t on_time, uint32_t off_time, uint32_t frequency = 2000){
  static uint32_t pulse_timer;

  pulse_timer += dt;  // Accumulate time

  // Check if a full cycle (on + off time) has passed
  if(pulse_timer >= (on_time + off_time)){
    pulse_timer = 0;  // Reset timer
    tone(PIN_BUZZER, frequency, on_time);  // Activate buzzer
  }
  else if(pulse_timer < on_time)
  {
    digitalWrite(PIN_LED, 1);  // Turn LED on during 'on' time
  }
  else
  {
    digitalWrite(PIN_LED, 0);  // Turn LED off during 'off' time
  }
}

// Function to read data from LSM6DSO32 sensor
bool read_LSM()
{
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  // Attempt to read sensor data
  if(!LSM.getEvent(&accel, &gyro, &temp))
  {
    return false;  // Return false if read fails
  }

  // Store gyroscope data
  lsm_gyro.x = gyro.gyro.x;
  lsm_gyro.y = gyro.gyro.y;
  lsm_gyro.z = gyro.gyro.z;

  // Store accelerometer data
  lsm_acc.x = accel.acceleration.x;
  lsm_acc.y = accel.acceleration.y;
  lsm_acc.z = accel.acceleration.z;

  // Store temperature data
  lsm_temp = float(temp.temperature);

  return true;  // Return true if read succeeds
}

// Function to read data from BMP390 sensor
bool read_BMP()
{
  // Attempt to perform a reading
  if(!BMP.performReading())
  {
    return false;  // Return false if read fails
  }

  // Store temperature, pressure, and altitude data
  bmp_temp = BMP.temperature;
  bmp_press = BMP.pressure;
  bmp_alt = BMP.readAltitude(1013.25) - off_alt;  // Calculate altitude relative to baseline
  
  return true;  // Return true if read succeeds
}

// Function to read data from ADXL375 sensor
bool read_ADXL()
{
  sensors_event_t event;
  // Attempt to get sensor event
  if(!ADXL.getEvent(&event))
  {
    return false;  // Return false if read fails
  }

  // Store accelerometer data
  adxl_acc.x = event.acceleration.x;
  adxl_acc.y = event.acceleration.y;
  adxl_acc.z = event.acceleration.z;

  // Store temperature data
  adxl_temp = float(event.temperature);

  return true;  // Return true if read succeeds
}

// Function to read data from BNO055 sensor
bool read_BNO()
{
  sensors_event_t orientationData , angVelocityData, magnetometerData, accelerometerData;

  // Attempt to read various sensor data types
  if(!BNO.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER))
  {
    return false;
  }
  if(!BNO.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE))
  {
    return false;
  }
  if(!BNO.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER))
  {
    return false;
  }
  if(!BNO.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER))
  {
    return false;
  }

  // Get and store quaternion orientation
  imu::Quaternion quat = BNO.getQuat();
  bno_orientation.w = quat.w();
  bno_orientation.x = quat.x();
  bno_orientation.y = quat.y();
  bno_orientation.z = quat.z();
  //bno_orientation *= Quaternion().euler_to_quaternion(Vector3(0, 0, (90 + 20) * DEG_TO_RAD));

  // Store Euler angles
  bno_euler.x = -orientationData.orientation.heading;
  bno_euler.y = -orientationData.orientation.pitch;
  bno_euler.z = -orientationData.orientation.roll + 180.0;

  // Store gyroscope data
  bno_gyro.x = angVelocityData.gyro.x;
  bno_gyro.y = angVelocityData.gyro.y;
  bno_gyro.z = angVelocityData.gyro.z;

  // Store accelerometer data
  bno_acc.x = accelerometerData.acceleration.x;
  bno_acc.y = accelerometerData.acceleration.y;
  bno_acc.z = accelerometerData.acceleration.z;

  // Store magnetometer data
  bno_mag.x = magnetometerData.magnetic.x;
  bno_mag.y = magnetometerData.magnetic.y;
  bno_mag.z = magnetometerData.magnetic.z;

  // Store temperature data
  bno_temp = float(BNO.getTemp());

  return true;  // Return true if all reads succeed
}

// Setup function to initialize sensors and serial communication
void setup() 
{
  // Configure pin modes for output devices
  pinMode(PIN_BUZZER, OUTPUT);  // Set buzzer pin as output
  pinMode(PIN_LED, OUTPUT);     // Set LED pin as output

  // Initialize serial communication ports
  Serial.begin(115200);   // USB Serial Port at 115200 baud
  Serial1.begin(9600);    // Radio Serial Port at 9600 baud
  Serial2.begin(115200);  // GPS Serial Port at 115200 baud

  // Initialize LSM6DSO32 (Accelerometer and Gyroscope)
  while(!LSM.begin_SPI(LSM_CS, &SPI))
  {
    Serial.println(F("LSM6DSO32 not found..."));  // Print error message if sensor not found
    delay(1000);  // Wait for 1 second before retrying
  }
  Serial.println(F("LSM6DSO32 initialized"));  // Confirm successful initialization

  // Initialize BMP390 (Barometric Pressure Sensor)
  while(!BMP.begin_SPI(BMP_CS, &SPI))
  {
    Serial.println(F("BMP390 not found..."));  // Print error message if sensor not found
    delay(1000);  // Wait for 1 second before retrying
  }
  Serial.println(F("BMP390 initialized"));  // Confirm successful initialization

  // Initialize ADXL375 (High-G Accelerometer)
  while(!ADXL.begin())
  {
    Serial.println(F("ADXL375 not found..."));  // Print error message if sensor not found
    delay(1000);  // Wait for 1 second before retrying
  }
  Serial.println(F("ADXL375 initialized"));  // Confirm successful initialization

  // Initialize BNO055 (9-DOF Inertial Measurement Unit)
  while(!BNO.begin())
  {
    Serial.println(F("BNO055 not found..."));  // Print error message if sensor not found
    delay(1000);  // Wait for 1 second before retrying
  }
  Serial.println(F("BNO055 initialized"));  // Confirm successful initialization

  // Initialize SD card
  while(!SD.begin(BUILTIN_SDCARD)) {
    Serial.println(F("SD not found..."));  // Print error message if SD card not found
    delay(1000);  // Wait for 1 second before retrying
  }
  Serial.println(F("SD initialized"));  // Confirm successful initialization

  // Configure LSM6DSO32 sensor settings
  LSM.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G);  // Set accelerometer range to ±32g
  LSM.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);   // Set gyroscope range to ±1000 degrees per second
  LSM.setAccelDataRate(LSM6DS_RATE_416_HZ);       // Set accelerometer data rate to 416 Hz
  LSM.setGyroDataRate(LSM6DS_RATE_416_HZ);        // Set gyroscope data rate to 416 Hz

  // Configure BMP390 sensor settings
  BMP.setTemperatureOversampling(BMP3_OVERSAMPLING_16X);  // Set temperature oversampling to 16x
  BMP.setPressureOversampling(BMP3_OVERSAMPLING_16X);     // Set pressure oversampling to 16x
  BMP.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);         // Set IIR filter coefficient to 3
  BMP.setOutputDataRate(BMP3_ODR_200_HZ);                 // Set output data rate to 200 Hz

  // Configure ADXL375 sensor settings
  ADXL.setDataRate(ADXL343_DATARATE_200_HZ);  // Set data rate to 200 Hz

  // Configure BNO055 sensor settings
  BNO.setMode(OPERATION_MODE_CONFIG);                   // Set to configuration mode
  BNO.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P1);   // Remap axes configuration
  BNO.setMode(OPERATION_MODE_NDOF);                     // Set to NDOF (Nine Degrees of Freedom) fusion mode

  // GPS Initialization and Configuration
  GPS.begin(115200);  // Initialize GPS at 115200 baud
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);  // Request all available NMEA data
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);     // Set update rate to 10 Hz

  // Datalogging initialization

  // Create datalogging file with a unique name
  char csvfilename[17] = "FL0.csv";
  for(uint32_t i = 0; SD.exists(csvfilename); i++){
    sprintf(csvfilename, "FL%d.csv", i);  // Increment filename if it already exists
  }

  // Open file for writing
  csvfile = SD.open(csvfilename, FILE_WRITE);

  // Print data header to the CSV file
  csvfile.println(data_header);
  csvfile.flush();  // Ensure data is written to the file
}

//#define SERIALDEBUG

void loop()
{
  // Update timing variables
  previoustime = mstime;
  mstime = millis();
  deltatime = mstime - previoustime;
  float deltatime_s = float(deltatime) * 0.001;  // Convert milliseconds to seconds

  // Read data from all sensors
  read_ADXL();  // Read data from ADXL375 accelerometer
  read_BMP();   // Read data from BMP390 barometer
  read_BNO();   // Read data from BNO055 IMU
  read_LSM();   // Read data from LSM6DSO32 IMU

  // Update GPS data
  while(GPS.available())
  {
    GPS.read();  // Read available GPS data
  }

  if(GPS.newNMEAreceived())
  {
    GPS.parse(GPS.lastNMEA());  // Parse the latest NMEA sentence
  }
  
  // Update orientation calculations
  angular_rate = lsm_gyro - rates_offset;  // Calculate angular rate by subtracting offset
  acceleration_body = lsm_acc;  // Set body acceleration from LSM6DSO32 readings
  magnetic_flux = bno_mag;  // Set magnetic flux from BNO055 readings

  acceleration_inertial = orientation.rotate_vector(acceleration_body);  // Transform acceleration to inertial frame

  orientation.update_with_rates(deltatime_s, angular_rate);  // Update orientation based on angular rates
  
  // Additional orientation updates for non-flight states
  if (flight_state < States::Flight)
  {
    // Update orientation using accelerometer data
    orientation.update_with_accel(acceleration_inertial, Vector3(0, 0, 1), 0.1);
    // Update orientation using magnetometer data
    orientation.update_with_mag(bno_mag, lsm_acc, Vector3(0, -1, 0), Quaternion().euler_to_quaternion(Vector3(0, 0, 14 * DEG_TO_RAD)), 0.1);
  }

  acceleration_inertial.z -= 9.8066f;  // Subtract gravity from vertical acceleration
  euler_angles = orientation.quaternion_to_euler().rad_to_deg();  // Convert orientation to Euler angles in degrees

  // Calculate vertical velocity
  if (deltatime_s > 0)
  {
    v_vel = (bmp_alt - prev_alt) / deltatime_s;  // Vertical velocity from altitude change
  }
  prev_alt = bmp_alt;  // Update previous altitude
  
  // State machine update
  if (mstime - previoustime_state >= deltatime_state)
  {
    previoustime_state = mstime;

    switch (flight_state)
    {
      case (States::IdleNoCal):  // Uncalibrated idle state
      {
        // Set buzzer and LED to pulse for indication
        set_buzzer_LED_pulse(deltatime, 250, 2000);

        // Check for liftoff
        if (detect_liftoff(deltatime_state, acceleration_body.z))
        {
          deltatime_log = deltatime_logfast;  // Increase logging rate
          flight_state = States::Flight;  // Transition to flight state
          off_alt = bmp_alt;  // Record liftoff altitude

          break;
        }

        // Perform IMU calibration
        rates_offset_calibration(deltatime_state, 5000);
        if(rates_offset_set){
          flight_state = States::Idle;  // Transition to calibrated idle state
        }

        break;
      }
      case (States::Idle):  // Calibrated idle state
      {
        // Set buzzer and LED to pulse for indication
        set_buzzer_LED_pulse(deltatime, 1000, 3000);

        // Check for liftoff
        if (detect_liftoff(deltatime_state, acceleration_body.z))
        { 
          deltatime_log = deltatime_logfast;  // Increase logging rate
          flight_state = States::Flight;  // Transition to flight state
          off_alt = bmp_alt;  // Record liftoff altitude
          tone(PIN_BUZZER, 2000, 2000);  // Sound buzzer to indicate liftoff

          break;
        }

        break;
      }
      case (States::Flight):  // Flight state
      {
        // Check for landing
        if(detect_landing(deltatime_state, bmp_alt)){ 
          flight_state = States::Landed;  // Transition to landed state
        }
        break;
      }
      case (States::Landed):  // Landed state
      {
        log_enable = false;  // Disable logging
        set_buzzer_LED_pulse(deltatime, 1000, 500);  // Set buzzer and LED pulse to indicate landed state
        break;
      }

      default: 
        break;
    }
  }

  // Radio telemetry update
  if (mstime - previoustime_radio >= deltatime_radio)
  {
    previoustime_radio = mstime;

    // Send altitude data
    Serial1.print(F("ALT: ")); 
    Serial1.print(bmp_alt, 1); 
    Serial1.print(F("\t"));
    
    // Send vertical velocity data
    Serial1.print(F("VEL: ")); 
    Serial1.print(v_vel, 1);   
    Serial1.print(F("\t"));

    // Send latitude data
    Serial1.print(F("LAT: "));
    Serial1.print(GPS.latitudeDegrees, 6); 
    Serial1.print(GPS.lat); 
    Serial1.print(F("\t"));

    // Send longitude data
    Serial1.print(F("LON: "));
    Serial1.print(GPS.longitudeDegrees, 6); 
    Serial1.print(GPS.lon); 
    Serial1.print(F("\t"));

    // Send number of GPS satellites
    Serial1.print(F("STN: "));
    Serial1.print((int)GPS.satellites); 
    Serial1.print(F("\t"));

    // Send flight state
    Serial1.print(F("STA: "));            
    Serial1.print(F("\t"));
    Serial1.print(uint32_t(flight_state));
    Serial1.print(F("\n"));
  }

  // Data logging update
  if (mstime - previoustime_log >= deltatime_log)
  {
    previoustime_log = mstime;

    if (log_enable)
    {
      // Log timestamp
      csvfile.print(mstime);                  csvfile.print(",");

      // Log GPS data
      csvfile.print(GPS.latitudeDegrees, 6); 
      csvfile.print(GPS.lat);                 csvfile.print(",");
      csvfile.print(GPS.longitudeDegrees, 6); 
      csvfile.print(GPS.lon);                 csvfile.print(",");
      csvfile.print((int32_t)GPS.satellites); csvfile.print(",");
      csvfile.print(GPS.speed, 3);            csvfile.print(",");
      csvfile.print(GPS.angle, 3);            csvfile.print(",");
      csvfile.print(GPS.altitude, 3);         csvfile.print(",");

      csvfile.flush();  // Ensure GPS data is written to file

      // Log flight state
      csvfile.print(int32_t(flight_state));   csvfile.print(",");

      // Log orientation data
      csvfile.print(euler_angles.x, 2);       csvfile.print(",");
      csvfile.print(euler_angles.y, 2);       csvfile.print(",");
      csvfile.print(euler_angles.z, 2);       csvfile.print(",");

      csvfile.print(orientation.w, 5);        csvfile.print(",");
      csvfile.print(orientation.x, 5);        csvfile.print(",");
      csvfile.print(orientation.y, 5);        csvfile.print(",");
      csvfile.print(orientation.z, 5);        csvfile.print(",");

      csvfile.print(bno_orientation.w, 5);    csvfile.print(",");
      csvfile.print(bno_orientation.x, 5);    csvfile.print(",");
      csvfile.print(bno_orientation.y, 5);    csvfile.print(",");
      csvfile.print(bno_orientation.z, 5);    csvfile.print(",");

      csvfile.flush();  // Ensure orientation data is written to file

      // Log acceleration and angular rate data
      csvfile.print(acceleration_inertial.x, 4);       csvfile.print(",");
      csvfile.print(acceleration_inertial.y, 4);       csvfile.print(",");
      csvfile.print(acceleration_inertial.z, 4);       csvfile.print(",");

      csvfile.print(angular_rate.x, 4);       csvfile.print(",");
      csvfile.print(angular_rate.y, 4);       csvfile.print(",");
      csvfile.print(angular_rate.z, 4);       csvfile.print(",");

      csvfile.print(acceleration_body.x, 4);       csvfile.print(",");
      csvfile.print(acceleration_body.y, 4);       csvfile.print(",");
      csvfile.print(acceleration_body.z, 4);       csvfile.print(",");

      csvfile.print(bno_gyro.x, 4);       csvfile.print(",");
      csvfile.print(bno_gyro.y, 4);       csvfile.print(",");
      csvfile.print(bno_gyro.z, 4);       csvfile.print(",");

      csvfile.print(bno_acc.x, 4);       csvfile.print(",");
      csvfile.print(bno_acc.y, 4);       csvfile.print(",");
      csvfile.print(bno_acc.z, 4);       csvfile.print(",");

      csvfile.print(adxl_acc.x, 2);       csvfile.print(",");
      csvfile.print(adxl_acc.y, 2);       csvfile.print(",");
      csvfile.print(adxl_acc.z, 2);       csvfile.print(",");

      csvfile.flush();  // Ensure acceleration and angular rate data is written to file

      // Log pressure, altitude, and velocity data
      csvfile.print(bmp_press, 6);    csvfile.print(",");
      csvfile.print(bmp_alt, 4);      csvfile.print(",");
      csvfile.print(v_vel, 4);        csvfile.print(",");

      // Log temperature data from various sensors
      csvfile.print(lsm_temp, 2);       csvfile.print(",");
      csvfile.print(adxl_temp, 2);      csvfile.print(",");
      csvfile.print(bno_temp, 2);       csvfile.print(",");
      csvfile.print(bmp_temp, 2);       csvfile.print(",");

      csvfile.println();  // End the log entry
      csvfile.flush();  // Ensure all data is written to file
    }

    // Debug output to serial monitor if SERIALDEBUG is defined
    #ifdef SERIALDEBUG
      // Print orientation angles
      Serial.print(F("XYZ: "));
      Serial.print(euler_angles.x, 1); Serial.print(F("\t"));
      Serial.print(euler_angles.y, 1); Serial.print(F("\t"));
      Serial.print(euler_angles.z, 1); Serial.print(F("\t"));

      // Print altitude
      Serial.print(F("ALT: "));
      Serial.print(bmp_alt, 1);   Serial.print(F("\t"));

      // Print vertical velocity
      Serial.print(F("VEL: "));
      Serial.print(v_vel, 1);   Serial.print(F("\t"));

      // Print GPS latitude
      Serial.print(F("LAT: "));
      Serial.print(GPS.latitudeDegrees, 6); 
      Serial.print(GPS.lat); 
      Serial.print(F("\t"));

      // Print GPS longitude
      Serial.print(F("LON: "));
      Serial.print(GPS.longitudeDegrees, 6); 
      Serial.print(GPS.lon); 
      Serial.print(F("\t"));

      // Print number of GPS satellites
      Serial.print(F("SAT: "));
      Serial.print((int)GPS.satellites); Serial.print(F("\t"));

      // Print flight state
      Serial.print(F("STATE: "));                 Serial.print(F("\t"));
      Serial.print(uint32_t(flight_state));       Serial.print(F("\n"));
    #endif

    // Print loop rate (iterations per second)
    Serial.print(F("RATE: "));              Serial.print(F("\t"));
    Serial.print(1000.0/float(deltatime));  Serial.print(F("\t"));
  }
}