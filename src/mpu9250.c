#include "hal_millis.h"

#include "mpu9250.h"

#include "uart_debug.h"
//#include "stm32f4xx_hal_i2c.h"
#include "hal_i2c.h"

#include <stdio.h>
#include <stdlib.h> //free
#include <math.h>

#define Sasprintf(write_to, ...) { \
		char *tmp_string_for_extend = (write_to); \
		asprintf(&(write_to), __VA_ARGS__); \
		free(tmp_string_for_extend); \
}
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

// current partial number
//int currentValue;


// current state-machine state

//======Sensor and servo init start
/* change this with pwm code to drive a servo */
//Servo myservo; //download the servo library and inspect how it works and use it in C code later
const int ultrasonic_sensor_address[SENSORNUM] = {34, 24}; // TODO: XXX Define What I want for the receivers address

//===================================================================================================================
//====== Set of helper function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

int sampleIMU(I2C_TypeDef * I2Cx, ImuState_t *imu_state) {
  if (readByte(I2Cx, MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt from mpu9250, check if data ready interrupt
    readAccelData(I2Cx, imu_state->accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the acceleration value into actual g's
    imu_state->ax = (float)imu_state->accelCount[0] * imu_state->aRes - imu_state->accelBias[0]/2.f;  // get actual g value, this depends on scale being set
    imu_state->ay = (float)imu_state->accelCount[1] * imu_state->aRes - imu_state->accelBias[1]/2.f;
    imu_state->az = (float)imu_state->accelCount[2] * imu_state->aRes - imu_state->accelBias[2]/2.f;

    readGyroData(I2Cx, imu_state->gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    imu_state->gx = (float)imu_state->gyroCount[0] * imu_state->gRes - imu_state->gyroBias[0]/2.f; // get actual gyro value, this depends on scale being set
    imu_state->gy = (float)imu_state->gyroCount[1] * imu_state->gRes - imu_state->gyroBias[1]/2.f;
    imu_state->gz = (float)imu_state->gyroCount[2] * imu_state->gRes - imu_state->gyroBias[2]/2.f;

    readMagData(I2Cx, imu_state->magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    imu_state->mx = (float)imu_state->magCount[0] * imu_state->mRes * imu_state->magCalibration[0] - imu_state->magBias[0]; // get actual magnetometer value, this depends on scale being set
    imu_state->my = (float)imu_state->magCount[1] * imu_state->mRes * imu_state->magCalibration[1] - imu_state->magBias[1];
    imu_state->mz = (float)imu_state->magCount[2] * imu_state->mRes * imu_state->magCalibration[2] - imu_state->magBias[2];

    imu_state->mx *= imu_state->magScale[0];
    imu_state->my *= imu_state->magScale[1];
    imu_state->mz *= imu_state->magScale[2];
  }
  else{
	  return 0;
  }

  imu_state->Now = micros();
	//TODO: micro() from imer
  imu_state->deltat = ((imu_state->Now - imu_state->lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  imu_state->lastUpdate = imu_state->Now;

  imu_state->sum += imu_state->deltat; // sum for averaging filter update rate
  imu_state->sumCount++;

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
  // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!
  // Pass gyro rate as rad/s
  if (AHRS) {
  //MadgwickQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f,  my,  mx, mz);
	  MahonyQuaternionUpdate(imu_state ,imu_state->ax, imu_state->ay, imu_state->az, imu_state->gx * PI / 180.0f, imu_state->gy * PI / 180.0f, imu_state->gz * PI / 180.0f, imu_state->my, imu_state->mx, imu_state->mz);
  }
  return 1;
}

int sampleIMUtoSensor(I2C_TypeDef * I2Cx, ImuState_t *imu_state , SensorData * sensorData) {

  if (sampleIMU(I2Cx, imu_state)==1){

	  sensorData->timeStamp = millis();
	  imu_state->delt_t = millis() - imu_state->prevMeasure;
	  /* if the internal timer more than set time TIMGAP then sample it show print them and update with the sensorData stuct */
	  if (imu_state->delt_t > TIMEGAP) {
		  if (!AHRS) { //if AHRS false
			  if (SerialDebug) {
				char *temp = NULL;
				// Print acceleration values in milligs!
				Sasprintf(temp,"X-acceleration:  %f2.3 mg ",1000 * imu_state->ax);
				Sasprintf(temp,"Y-acceleration: %f2.3 mg ",1000 * imu_state->ay);
				Sasprintf(temp,"Z-acceleration: %f2.3 mg \n",1000 * imu_state->az);

				// Print gyro values in degree/sec
				Sasprintf(temp,"X-gyro rate: %f2.3 degrees/sec ",imu_state->gx);
				Sasprintf(temp,"Y-gyro rate: %f2.3 degrees/sec ",imu_state->gy);
				Sasprintf(temp,"Z-gyro rate: %f2.3 degrees/sec \n",imu_state->gz);

				// Print mag values in degree/sec
				Sasprintf(temp,"X-mag field: %f2.3 mG ",imu_state->mx);
				Sasprintf(temp,"Y-mag field: %f2.3 mG ",imu_state->my);
				Sasprintf(temp,"Z-mag field: %f2.3 mG \n",imu_state->mz);

				uart_printf(temp);
				free(temp);

				imu_state->tempCount = readTempData(I2Cx);  // Read the adc values
				imu_state->temperature = ((float) imu_state->tempCount) / 333.87f + 21.0f; // Temperature in degrees Centigrade TODO need to change it to the
				sensorData->gyroValue.temperature = (uint16_t)(imu_state->temperature);
				// Print temperature in degrees Centigrade
				//uart_printf("Temperature is ");  uart_printf(temperature, 1);  uart_printf(" degrees C\n"); // Print T values to tenths of s degree C

			  }
			  imu_state->prevMeasure = millis();
		  }
		  else { //if AHRS true
			// Serial print and/or display at 0.5 s rate independent of data rates
			  if (SerialDebug) {
				char *temp = NULL;

				Sasprintf(temp,"ax = %f2.3",(int)1000 * imu_state->ax);
				Sasprintf(temp," ay = %f2.3",(int)1000 * imu_state->ay);
				Sasprintf(temp," az = %f2.3 mg\n",(int)1000 * imu_state->az);
				Sasprintf(temp,"gx = %f2.3", imu_state->gx);
				Sasprintf(temp," gy = %f2.3", imu_state->gy);
				Sasprintf(temp," gz = %f2.3 deg/s\n", imu_state->gz);
				Sasprintf(temp,"mx = %d", (int)imu_state->mx );
				Sasprintf(temp," my = %d", (int)imu_state->my );
				Sasprintf(temp," mz = %d mG\n", (int)imu_state->mz );

				Sasprintf(temp,"q0 = %f2.3",imu_state->q[0]);
				Sasprintf(temp," qx = %f2.3",imu_state->q[1]);
				Sasprintf(temp," qy = %f2.3",imu_state->q[2]);
				Sasprintf(temp," qz = %f2.3\n",imu_state->q[3]);

				uart_printf(temp);
				free(temp);
			  }

			  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
			  // In this coordinate system, the positive z-axis is down toward Earth.
			  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
			  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
			  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
			  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
			  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
			  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
			  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
			  imu_state->yaw   = atan2(2.0f * (imu_state->q[1] * imu_state->q[2] + imu_state->q[0] * imu_state->q[3]), imu_state->q[0] * imu_state->q[0] + imu_state->q[1] * imu_state->q[1] - imu_state->q[2] * imu_state->q[2] - imu_state->q[3] * imu_state->q[3]);
			  imu_state->pitch = -asin(2.0f * (imu_state->q[1] * imu_state->q[3] - imu_state->q[0] * imu_state->q[2]));
			  imu_state->roll  = atan2(2.0f * (imu_state->q[0] * imu_state->q[1] + imu_state->q[2] * imu_state->q[3]), imu_state->q[0] * imu_state->q[0] - imu_state->q[1] * imu_state->q[1] - imu_state->q[2] * imu_state->q[2] + imu_state->q[3] * imu_state->q[3]);
			  imu_state->pitch *= 180.0f / PI;
			  imu_state->yaw   *= 180.0f / PI;
			  imu_state->yaw   -= 10.9f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
			  imu_state->roll  *= 180.0f / PI;

			  sensorData->gyroValue.yaw = (uint16_t)(imu_state->yaw);
			  sensorData->gyroValue.pitch = (uint16_t)(imu_state->pitch);
			  sensorData->gyroValue.roll = (uint16_t)(imu_state->roll);
			  sensorData->gyroValue.freq = (float)(imu_state->sumCount / imu_state->sum);

			  //printing all results
			  if (SerialDebug) {
						char *temp = NULL;
						Sasprintf(temp, "Yaw, Pitch, Roll: %f2.3, %f2.3, %f2.3\nrate =  %f2.3Hz\n", imu_state->yaw, imu_state->pitch, imu_state->roll, (float)imu_state->sumCount / imu_state->sum);
						uart_printf(temp);
						free(temp);
			  }
			  // With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and
			  // >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
			  // The filter update rate is determined mostly by the mathematical steps in the respective algorithms,
			  // the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
			  // an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
			  // filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively.
			  // This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
			  // This filter update rate should be fast enough to maintain accurate platform orientation for
			  // stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
			  // produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
			  // The 3.3 V 8 MHz Pro Mini is doing pretty well!
			  //TODO: interrupt make to led light gpio
					//led_toggle(GPIOD,LED_RED);
			  imu_state->prevMeasure = millis();
			  imu_state->sumCount = 0.f;
			  imu_state->sum = 0.f;
		  }

		  /* timed SensorData struct update*/
		  sensorData->gyroValue.xa = 1000.f * imu_state->ax;
		  sensorData->gyroValue.ya = 1000.f * imu_state->ay;
		  sensorData->gyroValue.za = 1000.f * imu_state->az;

		  sensorData->gyroValue.xg = imu_state->gx;
		  sensorData->gyroValue.yg = imu_state->gy;
		  sensorData->gyroValue.zg = imu_state->gz;

		  sensorData->gyroValue.xm = imu_state->mx;
		  sensorData->gyroValue.ym = imu_state->my;
		  sensorData->gyroValue.zm = imu_state->mz;
	  }
  }
  else{
	  return 0;
  }
	  return 1;
}

void getMres(ImuState_t *imu_state) {
  switch (imu_state->mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
      imu_state->mRes = 10.*4912.f / 8190.f; // Proper scale to return milliGauss
      break;
    case MFS_16BITS:
      imu_state->mRes = 10.*4912.f / 32760.0f; // Proper scale to return milliGauss
      break;
  }
}

void getGres(ImuState_t *imu_state) {
  switch (imu_state->gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
      imu_state->gRes = 250.0f / 32768.0f;
      break;
    case GFS_500DPS:
      imu_state->gRes = 500.0f / 32768.0f;
      break;
    case GFS_1000DPS:
      imu_state->gRes = 1000.0f / 32768.0f;
      break;
    case GFS_2000DPS:
      imu_state->gRes = 2000.0f / 32768.0f;
      break;
  }
}

void getAres(ImuState_t *imu_state) {
  switch (imu_state->ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
      imu_state->aRes = 2.0f / 32768.0f;
      break;
    case AFS_4G:
      imu_state->aRes = 4.0f / 32768.0f;
      break;
    case AFS_8G:
      imu_state->aRes = 8.0f / 32768.0f;
      break;
    case AFS_16G:
      imu_state->aRes = 16.0f / 32768.0f;
      break;
  }
}


void readAccelData(I2C_TypeDef * I2Cx, int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(I2Cx,MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}


void readGyroData(I2C_TypeDef * I2Cx, int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(I2Cx,MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void readMagData(I2C_TypeDef * I2Cx, int16_t * destination)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if (readByte(I2Cx, AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
    readBytes(I2Cx, AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
    uint8_t c = rawData[6]; // End data read by reading ST2 register
    if (!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
      destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
      destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
      destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
    }
  }
}

int16_t readTempData(I2C_TypeDef * I2Cx)
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(I2Cx,MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

void initAK8963(I2C_TypeDef * I2Cx, ImuState_t *imu_state, float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(I2Cx, AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  writeByte(I2Cx, AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(I2Cx, AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128) / 256.f + 1.f; // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128) / 256.f + 1.f;
  destination[2] =  (float)(rawData[2] - 128) / 256.f + 1.f;
  writeByte(I2Cx, AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(I2Cx, AK8963_ADDRESS, AK8963_CNTL, imu_state->mscale << 4 | imu_state->mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
}


void initMPU9250(I2C_TypeDef * I2Cx, ImuState_t *imu_state)
{
  // wake up device
  writeByte(I2Cx, MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  delay(100); // Wait for all registers to reset

  // get stable time source
  writeByte(I2Cx, MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200);

  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
  // be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(I2Cx, MPU9250_ADDRESS, MPU9250CONFIG, 0x03);

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(I2Cx, MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
  // determined inset in MPU9250CONFIG above

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(I2Cx, MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x02; // Clear Fchoice bits [1:0]
  c = c & ~0x18; // Clear AFS bits [4:3]
  c = c | imu_state->gscale << 3; // Set full scale range for the gyro
  // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeByte(I2Cx, MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

  // Set accelerometer full-scale range configuration
  c = readByte(I2Cx, MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | imu_state->ascale << 3; // Set full scale range for the accelerometer
  writeByte(I2Cx, MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(I2Cx, MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(I2Cx, MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  writeByte(I2Cx, MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
  writeByte(I2Cx, MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
  delay(100);
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9250(I2C_TypeDef * I2Cx, float * dest1, float * dest2)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  // reset device
  writeByte(I2Cx, MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);

  // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
  // else use the internal oscillator, bits 2:0 = 001
  writeByte(I2Cx, MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
  writeByte(I2Cx, MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
  delay(200);

  // Configure device for bias calculation
  writeByte(I2Cx, MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(I2Cx, MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(I2Cx, MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(I2Cx, MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(I2Cx, MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(I2Cx, MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(I2Cx, MPU9250_ADDRESS, MPU9250CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(I2Cx, MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(I2Cx, MPU9250_ADDRESS, GYRO_CONFIG, 0x10);  // Set gyro full-scale to 1000 degrees per second, maximum sensitivity
  writeByte(I2Cx, MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(I2Cx, MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
  writeByte(I2Cx, MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  writeByte(I2Cx, MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(I2Cx, MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(I2Cx, MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];

  }
  accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;

  if (accel_bias[2] > 0L) {
    accel_bias[2] -= (int32_t) accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
  }
  else {
    accel_bias[2] += (int32_t) accelsensitivity;
  }

  // Construct the gyro biases to push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1] / 4)       & 0xFF;
  data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2] / 4)       & 0xFF;

  // Push gyro biases to hardware registers
  writeByte(I2Cx, MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(I2Cx, MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(I2Cx, MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(I2Cx, MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(I2Cx, MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(I2Cx, MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

  // Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity;
  dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(I2Cx, MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(I2Cx, MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(I2Cx, MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for (ii = 0; ii < 3; ii++) {
    if ((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1] / 8);
  accel_bias_reg[2] -= (accel_bias[2] / 8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

  // Apparently this is not working for the acceleration biases in the MPU-9250
  // Are we handling the temperature correction bit properly?
  // Push accelerometer biases to hardware registers
  writeByte(I2Cx, MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(I2Cx, MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(I2Cx, MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(I2Cx, MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(I2Cx, MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(I2Cx, MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

  // Output scaled accelerometer biases for display in the main program
  dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
  dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
  dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(I2C_TypeDef * I2Cx, float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t selfTest[6];
  int32_t  aAvg[3] = {0}, aSTAvg[3] = {0}, gAvg[3] = {0}, gSTAvg[3] = {0};
  float factoryTrim[6];
  uint8_t FS = 0;

  writeByte(I2Cx, MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  writeByte(I2Cx, MPU9250_ADDRESS, MPU9250CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(I2Cx, MPU9250_ADDRESS, GYRO_CONFIG, 1 << FS); // Set full scale range for the gyro to 250 dps
  writeByte(I2Cx, MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(I2Cx, MPU9250_ADDRESS, ACCEL_CONFIG, 1 << FS); // Set full scale range for the accelerometer to 2 g

  for ( int ii = 0; ii < 200; ii++) { // get average current values of gyro and acclerometer

    readBytes(I2Cx, MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(I2Cx, MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average current readings
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
  }

  // Configure the accelerometer for self-test
  writeByte(I2Cx, MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
  writeByte(I2Cx, MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  delay(25);  // Delay a while to let the device stabilize

  for ( int ii = 0; ii < 200; ii++) { // get average self-test values of gyro and acclerometer

    readBytes(I2Cx, MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(I2Cx, MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average self-test readings
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
  }

  // Configure the gyro and accelerometer for normal operation
  writeByte(I2Cx, MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
  writeByte(I2Cx, MPU9250_ADDRESS, GYRO_CONFIG,  0x00);
  delay(25);  // Delay a while to let the device stabilize

  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  selfTest[0] = readByte(I2Cx, MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
  selfTest[1] = readByte(I2Cx, MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
  selfTest[2] = readByte(I2Cx, MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
  selfTest[3] = readByte(I2Cx, MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
  selfTest[4] = readByte(I2Cx, MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
  selfTest[5] = readByte(I2Cx, MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
  factoryTrim[0] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[0] - 1.0f) )); // FT[Xa] factory trim calculation
  factoryTrim[1] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[1] - 1.0f) )); // FT[Ya] factory trim calculation
  factoryTrim[2] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[2] - 1.0f) )); // FT[Za] factory trim calculation
  factoryTrim[3] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[3] - 1.0f) )); // FT[Xg] factory trim calculation
  factoryTrim[4] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[4] - 1.0f) )); // FT[Yg] factory trim calculation
  factoryTrim[5] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[5] - 1.0f) )); // FT[Zg] factory trim calculation

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  // To get percent, must multiply by 100
  for (int i = 0; i < 3; i++) {
    destination[i]   = 100.0f * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.f; // Report percent differences
    destination[i + 3] = 100.0f * ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3] - 100.f; // Report percent differences
  }
}

void magcalMPU9250(I2C_TypeDef * I2Cx, ImuState_t *imu_state, float * dest1, float * dest2)
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = { -32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

  uart_printf("Mag Calibration: Wave device in a figure eight until done!\n");
  delay(1000);

  // shoot for ~fifteen seconds of mag data
  if (imu_state->mmode == 0x02) sample_count = 128; // at 8 Hz ODR, new mag data is available every 125 ms
  if (imu_state->mmode == 0x06) sample_count = 1500; // at 100 Hz ODR, new mag data is available every 10 ms
  for (ii = 0; ii < sample_count; ii++) {
    readMagData(I2Cx, mag_temp);  // Read the mag data
    for (int jj = 0; jj < 3; jj++) {
      if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    if (imu_state->mmode == 0x02) delay(135); // at 8 Hz ODR, new mag data is available every 125 ms
    if (imu_state->mmode == 0x06) delay(12); // at 100 Hz ODR, new mag data is available every 10 ms
  }
	char *temp = NULL;
	Sasprintf(temp, "avg mag x   y   z:\n %d %d %d\n", (mag_max[0] + mag_min[0] ) / 2,(mag_max[1] + mag_min[1] ) / 2,(mag_max[2] + mag_min[2] ) / 2);
	uart_printf(temp);
	free(temp);

  // Get hard iron correction
  mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

  dest1[0] = (float) mag_bias[0] * imu_state->mRes * imu_state->magCalibration[0]; // save mag biases in G for main program
  dest1[1] = (float) mag_bias[1] * imu_state->mRes * imu_state->magCalibration[1];
  dest1[2] = (float) mag_bias[2] * imu_state->mRes * imu_state->magCalibration[2];

	Sasprintf(temp, "avg magBias x   y   z:\n %3.3f %3.3f %3.3f\n", dest1[0],dest1[1],dest1[2]);
	uart_printf(temp);
	free(temp);

  // Get soft iron correction estimate
  mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2; // get average x axis max chord length in counts
  mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2; // get average y axis max chord length in counts
  mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2; // get average z axis max chord length in counts


  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  avg_rad /= 3.0f;

  dest2[0] = avg_rad / ((float)mag_scale[0]);
  dest2[1] = avg_rad / ((float)mag_scale[1]);
  dest2[2] = avg_rad / ((float)mag_scale[2]);

	Sasprintf(temp, "avg magScale x   y   z:\n %3.3f %3.3f %3.3f\n", dest2[0],dest2[1],dest2[2]);
	uart_printf(temp);
	free(temp);

  uart_printf("Mag Calibration done!\n");
}

void writeByte(I2C_TypeDef * I2Cx, uint8_t address, uint8_t subAddress, uint8_t data)
{
	uint32_t numData = 1;

	//Wire.beginTransmission(address);  // Initialize the Tx buffer
	//Wire.write(subAddress);           // Put slave register address in Tx buffer
	//Wire.write(data);                 // Put data in Tx buffer
	Status result = Success;

	result &= I2C_Write (I2Cx , (const uint8_t*)&subAddress , numData , (uint8_t)address );
	result &= I2C_Write (I2Cx , (const uint8_t*)&numData , numData , (uint8_t)address );
	result &= I2C_Write (I2Cx , (const uint8_t*)&data , numData , (uint8_t)address );

	//hal_i2c_master_tx(I2Cx, (uint8_t)address, (uint8_t*)&subAddress,1);
	//while(I2Cx->State != HAL_I2C_STATE_READY){continue;};
	//I2C1->
	//hal_i2c_master_tx(I2Cx, (uint8_t)address, (uint8_t*)&numData,1);
	//while(I2Cx->State != HAL_I2C_STATE_READY){continue;};

	//hal_i2c_master_tx(I2Cx, (uint8_t)address, (uint8_t*)&data,1);
	//while(I2Cx->State != HAL_I2C_STATE_READY){continue;};

	//Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(I2C_TypeDef * I2Cx, uint8_t address, uint8_t subAddress)
{

	//wait while busy bit is set
	//while(I2Cx->SR2 & I2C_FLAG_BUSY);

	//not busy now
	//Set slave address
	//I2Cx->SR1 |= (0x01 << 3);
	const uint8_t data [1] = {0};
  	//uint8_t data=0; // `data` will store the register data
	const uint32_t numData = 1;
	Status result = Success;
	//Wire.beginTransmission(address);         // Initialize the Tx buffer
	//Wire.write(subAddress);                  // Put slave register address in Tx buffer
	result &= I2C_Write (I2Cx , (const uint8_t*)&subAddress , numData , (uint8_t)address );
	result &= I2C_Write (I2Cx , (const uint8_t*)&numData , numData , (uint8_t)address );
	result &= I2C_Read (I2Cx , (uint8_t*)&data , numData , (uint8_t)address );

  	//hal_i2c_master_tx(I2Cx, (uint8_t)address, (uint8_t*)&subAddress,(uint32_t)numData);
	//while(I2C_GetFlagStatus (I2Cx , I2C_FLAG_BUSY ));

	//hal_i2c_master_tx(I2Cx, (uint8_t)address, (uint8_t*)&numData,(uint32_t)numData);
	//while(I2Cx->State != HAL_I2C_STATE_READY){
	//}

	//hal_i2c_master_rx(I2Cx, (uint8_t)address, (uint8_t*)&data,(uint32_t)numData);
	//while(I2Cx->State != HAL_I2C_STATE_READY){
	//}

	//Wire.endTransmission();        // Send the Tx buffer, but send a restart to keep connection alive
  //Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  //  Wire.requestFrom(address, 1);  // Read one byte from slave register address
  //Wire.requestFrom(address, (size_t) 1);  // Read one byte from slave register address
  //data = Wire.read();                      // Fill Rx buffer with result



  return *data;                             // Return data read from slave register
}

void readBytes(I2C_TypeDef * I2Cx, uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
  //Wire.beginTransmission(address);   // Initialize the Tx buffer
  //Wire.write(subAddress);            // Put slave register address in Tx buffer
  //Wire.endTransmission();  // Send the Tx buffer, but send a restart to keep connection alive
  //Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  //uint8_t i = 0;
  //        Wire.requestFrom(address, count);  // Read bytes from slave register address
  //Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address
  //while (Wire.available()) {
  //  dest[i++] = Wire.read();
  //}         // Put read results in the Rx buffer
	Status result = Success;

	result &= I2C_Write (I2Cx , (const uint8_t*)&subAddress , (uint32_t)1 , (uint8_t)address );
	result &= I2C_Write (I2Cx , (const uint8_t*)&count , (uint32_t)1 , (uint8_t)address );
	result &= I2C_Read (I2Cx , (uint8_t*)&dest , (uint32_t)count , (uint8_t)address );

	//hal_i2c_master_tx(I2Cx, (uint8_t)address, (uint8_t*)&subAddress,1);
	//while(I2Cx->State != HAL_I2C_STATE_READY){continue;};

	//hal_i2c_master_tx(I2Cx, (uint8_t)address, (uint8_t*)&count,1);
	//while(I2Cx->State != HAL_I2C_STATE_READY){continue;};

	//hal_i2c_master_rx(I2Cx, (uint8_t)address, dest, count);
	//while(I2Cx->State != HAL_I2C_STATE_READY){continue;};
}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
uint8_t imu_dataReady(I2C_TypeDef * I2Cx, ImuState_t *imu_state)
{
	uint8_t data;  // x/y/z accel register data stored here
	data = readByte(I2Cx, MPU9250_ADDRESS, INT_STATUS);  // Read the six raw data registers into data array
	if (data & 0x01)
	{
		return 1;
	}
	return 0;
}
void MadgwickQuaternionUpdate(ImuState_t *imu_state, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
	float q1 = imu_state->q[0], q2 = imu_state->q[1], q3 = imu_state->q[2], q4 = imu_state->q[3];   // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrtf(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // I2Cx NaN
	norm = 1.0f / norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrtf(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // I2Cx NaN
	norm = 1.0f / norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = sqrtf(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// Gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	norm = 1.0f / norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - imu_state->beta * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - imu_state->beta * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - imu_state->beta * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - imu_state->beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * imu_state->deltat;
	q2 += qDot2 * imu_state->deltat;
	q3 += qDot3 * imu_state->deltat;
	q4 += qDot4 * imu_state->deltat;
	norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f / norm;
	imu_state->q[0] = q1 * norm;
	imu_state->q[1] = q2 * norm;
	imu_state->q[2] = q3 * norm;
	imu_state->q[3] = q4 * norm;

}



// Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
// measured ones.
void MahonyQuaternionUpdate(ImuState_t *imu_state, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
	float q1 = imu_state->q[0], q2 = imu_state->q[1], q3 = imu_state->q[2], q4 = imu_state->q[3];   // short name local variable for readability
	float norm;
	float hx, hy, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	float pa, pb, pc;

	// Auxiliary variables to avoid repeated arithmetic
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrtf(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // I2Cx NaN
	norm = 1.0f / norm;        // use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrtf(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // I2Cx NaN
	norm = 1.0f / norm;        // use reciprocal for division
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
	hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
	bx = sqrtf((hx * hx) + (hy * hy));
	bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

	// Estimated direction of gravity and magnetic field
	vx = 2.0f * (q2q4 - q1q3);
	vy = 2.0f * (q1q2 + q3q4);
	vz = q1q1 - q2q2 - q3q3 + q4q4;
	wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
	wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
	wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

	// Error is cross product between estimated direction and measured direction of gravity
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
	if (Ki > 0.0f)
	{
	imu_state->eInt[0] += ex;      // accumulate integral error
	imu_state->eInt[1] += ey;
	imu_state->eInt[2] += ez;
	}
	else
	{
	imu_state->eInt[0] = 0.0f;     // prevent integral wind up
	imu_state->eInt[1] = 0.0f;
	imu_state->eInt[2] = 0.0f;
	}
	// Apply feedback terms
	gx = gx + Kp * ex + Ki * imu_state->eInt[0];
	gy = gy + Kp * ey + Ki * imu_state->eInt[1];
	gz = gz + Kp * ez + Ki * imu_state->eInt[2];

	// Integrate rate of change of quaternion
	pa = q2;
	pb = q3;
	pc = q4;
	q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * imu_state->deltat);
	q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * imu_state->deltat);
	q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * imu_state->deltat);
	q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * imu_state->deltat);

	// Normalise quaternion
	norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	norm = 1.0f / norm;
	imu_state->q[0] = q1 * norm;
	imu_state->q[1] = q2 * norm;
	imu_state->q[2] = q3 * norm;
	imu_state->q[3] = q4 * norm;

}
