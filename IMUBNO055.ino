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

void loop(void) 
{
  adafruit_bno055_offsets_t myCalibData;
  bno.getSensorOffsets(myCalibData);


  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  
  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");
  delay(500); 
  Serial.println("[CALIBRATION STATUS]");
  displayCalStatus(); 
  delay(500); 
  Serial.println("[CALIBRATION DATA]");
  displaySensorOffsets(myCalibData); 
  
  
  delay(100);
}
