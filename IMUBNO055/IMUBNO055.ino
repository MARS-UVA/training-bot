#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);

void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.println(mag, DEC);
}

void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}


void setup(void) 
{
  Serial.begin(9600);
  
  // 1. Initialize in IMU Mode (0x08)
  if (!bno.begin((adafruit_bno055_opmode_t)0x08)) 
  {
    Serial.print("BNO055 not detected");
    while (1);
  }

  delay(1000);

  adafruit_bno055_offsets_t fixedOffsets = {
    .accel_offset_x = -22,
    .accel_offset_y = 26,
    .accel_offset_z = -31,
    .mag_offset_x = -132,
    .mag_offset_y = 325,
    .mag_offset_z = 539,
    .gyro_offset_x = -2,
    .gyro_offset_y = 2,
    .gyro_offset_z = 2,
    .accel_radius = 1000,
    .mag_radius = 847
  };

  // 2. Load the offsets (This internally switches to CONFIG mode)
  Serial.println("Loading offsets...");
  bno.setSensorOffsets(fixedOffsets);
  
  // 3. FORCE back to IMU Mode (0x08) right after setting offsets
  // This ensures the sensor leaves CONFIG mode and starts calculating again
  bno.setMode((adafruit_bno055_opmode_t)0x08); 
  delay(100);

  bno.setExtCrystalUse(true);
  delay(500); 
}


struct __attribute__((packed)) IMUPacket {
  uint8_t startByte;       // Index 0: 0xFF
  uint8_t reserved;        // Index 1: 0x00
  float accelX;            // Index 2-5
  float accelY;            // Index 6-9
  float accelZ;            // Index 10-13
  float orientX;           // Index 14-17 (Roll)
  float orientY;           // Index 18-21 (Pitch)
  float orientZ;           // Index 22-25 (Yaw)
  float gyroX;             // Index 26-29
  float gyroY;             // Index 30-33
  float gyroZ;             // Index 34-37
  double gravityVector;    // Index 38-45 (Magnitude)
};



void loop(void) 
{
  adafruit_bno055_offsets_t myCalibData;
  bno.getSensorOffsets(myCalibData);


  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  
  /* Display the floating point data */
  IMUPacket packet;
  
  // Initialize header
  packet.startByte = 0xFF;
  packet.reserved = 0x00;

  // Fetch Data from BNO055
  sensors_event_t orientData, accelData, gyroData;
  bno.getEvent(&orientData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&accelData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  
  // Populate the Struct
  packet.accelX = accelData.acceleration.x;
  packet.accelY = accelData.acceleration.y;
  packet.accelZ = accelData.acceleration.z;
  
  packet.orientX = orientData.orientation.x;
  packet.orientY = orientData.orientation.y;
  packet.orientZ = orientData.orientation.z;
  
  packet.gyroX = gyroData.gyro.x;
  packet.gyroY = gyroData.gyro.y;
  packet.gyroZ = gyroData.gyro.z;

  // Calculate Gravity Vector Magnitude: sqrt(x^2 + y^2 + z^2)
  imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  packet.gravityVector = sqrt(sq(gravity.x()) + sq(gravity.y()) + sq(gravity.z()));

  // Send the raw binary data
  Serial.println(); 
  Serial.println("[PACKET]"); 
  Serial.write((uint8_t*)&packet, sizeof(packet));

  delay(50); // Increased frequency; binary is much faster than text

  
  Serial.println(""); 
  Serial.println("[CALIBRATION STATUS]");
  displayCalStatus(); 
  delay(50); 
  Serial.println("[CALIBRATION DATA]");
  displaySensorOffsets(myCalibData); 
  Serial.println(); 

  
  delay(100);
}
