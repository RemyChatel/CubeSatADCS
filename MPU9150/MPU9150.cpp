#include "MPU9150.h"

MPU9150::MPU9150(PinName sda, PinName scl) {
    i2c_ = new I2C(sda, scl);
    i2c_->frequency(400000);
}

MPU9150::MPU9150(I2C *i2c):i2c_(i2c){}
    
void MPU9150::writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
    char data_write[2];
    data_write[0] = subAddress;
    data_write[1] = data;
    i2c_->write(address, data_write, 2, 0);
}

char MPU9150::readByte(uint8_t address, uint8_t subAddress) {
    char data[1]; // `data` will store the register data     
    char data_write[1];
    data_write[0] = subAddress;
    i2c_->write(address, data_write, 1, 1); // no stop
    i2c_->read(address, data, 1, 0); 
    return data[0]; 
}

void MPU9150::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {     
    char data[14];
    char data_write[1];
    data_write[0] = subAddress;
    i2c_->write(address, data_write, 1, 1); // no stop
    i2c_->read(address, data, count, 0); 
    for(int ii = 0; ii < count; ii++) {
        dest[ii] = data[ii];
    }
} 

void MPU9150::getGres() {
    switch (gscale) {
        // Possible gyro scales (and their register bit settings) are:
        // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
        case GFS_250DPS:
            gRes_ = 250.0/32768.0;
            break;
        case GFS_500DPS:
            gRes_ = 500.0/32768.0;
            break;
        case GFS_1000DPS:
            gRes_ = 1000.0/32768.0;
            break;
        case GFS_2000DPS:
            gRes_ = 2000.0/32768.0;
            break;
    }
}

void MPU9150::setGres(uint8_t scale){
    gscale = scale;
    switch (scale) {
        // Possible gyro scales (and their register bit settings) are:
        // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
        case GFS_250DPS:
            gRes_ = 250.0/32768.0;
            break;
        case GFS_500DPS:
            gRes_ = 500.0/32768.0;
            break;
        case GFS_1000DPS:
            gRes_ = 1000.0/32768.0;
            break;
        case GFS_2000DPS:
            gRes_ = 2000.0/32768.0;
            break;
    }
}

void MPU9150::getAres() {
  switch (ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes_ = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes_ = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes_ = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes_ = 16.0/32768.0;
          break;
  }
}

void MPU9150::setAres(uint8_t scale){
    ascale = scale;
    switch (scale)
    {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
        aRes_ = 2.0/32768.0;
        break;
    case AFS_4G:
        aRes_ = 4.0/32768.0;
        break;
    case AFS_8G:
        aRes_ = 8.0/32768.0;
        break;
    case AFS_16G:
        aRes_ = 16.0/32768.0;
        break;
  }
}

void MPU9150::readAccelData(int16_t * destination) {
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MPU9150_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
}

void MPU9150::getAccel(float acc[3]){
    int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
    readAccelData(accelCount);  // Read the x/y/z adc values   
    // Now we'll calculate the accleration value into actual g's
    acc[0] = (float)accelCount[0]*aRes_;// - accelBias[0];  // get actual g value, this depends on scale being set
    acc[1] = (float)accelCount[1]*aRes_;// - accelBias[1];   
    acc[2] = (float)accelCount[2]*aRes_;// - accelBias[2]; 
}

void MPU9150::readGyroData(int16_t * destination) {
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU9150_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
}

void MPU9150::getGyro(float gyr[3]){
    int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
    readGyroData(gyroCount);  // Read the x/y/z adc values
    // Calculate the gyro value into actual degrees per second
    gyr[0] = (float)gyroCount[0]*gRes_ - gyroBias[0];  // get actual gyro value, this depends on scale being set
    gyr[1] = (float)gyroCount[1]*gRes_ - gyroBias[1];  
    gyr[2] = (float)gyroCount[2]*gRes_ - gyroBias[2]; 
}

void MPU9150::readMagData(int16_t * destination) {
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  writeByte(AK8975A_ADDRESS, AK8975A_CNTL, 0x01); // toggle enable data read from magnetometer, no continuous read mode!
  wait(0.01);
  // Only accept a new magnetometer data read if the data ready bit is set and 
  // if there are no sensor overflow or data read errors
  if(readByte(AK8975A_ADDRESS, AK8975A_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
  readBytes(AK8975A_ADDRESS, AK8975A_XOUT_L, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
  }
}

void MPU9150::getMag(float mag[3]){
    int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
    readMagData(magCount);  // Read the x/y/z adc values
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    mag[0] = (float)magCount[0]*mRes_*magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
    mag[1] = (float)magCount[1]*mRes_*magCalibration[1] - magBias[1];  
    mag[2] = (float)magCount[2]*mRes_*magCalibration[2] - magBias[2]; 
}

void MPU9150::initAK8975A() {
    uint8_t rawData[3];  // x/y/z gyro register data stored here
    writeByte(AK8975A_ADDRESS, AK8975A_CNTL, 0x00); // Power down
    wait(0.01);
    writeByte(AK8975A_ADDRESS, AK8975A_CNTL, 0x0F); // Enter Fuse ROM access mode
    wait(0.01);
    readBytes(AK8975A_ADDRESS, AK8975A_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
    magCalibration[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f; // Return x-axis sensitivity adjustment values
    magCalibration[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;  
    magCalibration[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f; 
}

int16_t MPU9150::readTempData() {
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(MPU9150_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
  return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

float MPU9150::getTemp(){
    uint16_t tempCount = readTempData();
    return ((float) tempCount) / 340.0f + 36.53f;
}

uint8_t MPU9150::initIMU(uint8_t acc_scale, uint8_t gyr_scale){
    uint8_t whoami = readByte(MPU9150_ADDRESS, WHO_AM_I_MPU9150);
    if (whoami == 0x73 || whoami == 0x68) {   // WHO_AM_I should be 0x68 or 0x73
        wait(1);
        MPU9150SelfTest();
        wait(0.5);
        resetMPU9150(); // Reset registers to default in preparation for device calibration
        calibrateMPU9150(); // Calibrate gyro and accelerometers, load biases in bias registers  
        wait(0.5);
        initMPU9150(acc_scale, gyr_scale); // Initialize device for active mode read of accelerometer, gyroscope, and temperature
        initAK8975A(); // Initialize device for active mode read of magnetometer
        return whoami;
    }
    else {
        return 0;
    }
}

void MPU9150::resetMPU9150() {
    // reset device
    writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    wait(0.1);
}

void MPU9150::initMPU9150(uint8_t acc_scale, uint8_t gyr_scale) {  
    // Initialize MPU9150 device
    // wake up device
    writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
    wait(0.1); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  

    // get stable time source
    writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

    // Configure Gyro and Accelerometer
    // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively; 
    // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
    // Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
    writeByte(MPU9150_ADDRESS, CONFIG, 0x03);  

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    writeByte(MPU9150_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above

    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    uint8_t c =  readByte(MPU9150_ADDRESS, GYRO_CONFIG);
    writeByte(MPU9150_ADDRESS, GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
    writeByte(MPU9150_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
    writeByte(MPU9150_ADDRESS, GYRO_CONFIG, c | gyr_scale << 3); // Set full scale range for the gyro

    // Set accelerometer configuration
    c =  readByte(MPU9150_ADDRESS, ACCEL_CONFIG);
    writeByte(MPU9150_ADDRESS, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
    writeByte(MPU9150_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
    writeByte(MPU9150_ADDRESS, ACCEL_CONFIG, c | acc_scale << 3); // Set full scale range for the accelerometer 

    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
    // can join the I2C bus and all can be controlled by the Arduino as master
    writeByte(MPU9150_ADDRESS, INT_PIN_CFG, 0x22);    
    writeByte(MPU9150_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt

    setAres(acc_scale); // Get accelerometer sensitivity
    setGres(gyr_scale); // Get gyro sensitivity
    mRes_ = 1229./4096.; // Conversion to 1229 microTesla full scale (4096)
    // So far, magnetometer bias is calculated and subtracted here manually, should construct an algorithm to do it automatically
    // like the gyro and accelerometer biases
    magBias[0] = -.5;   // User environmental x-axis correction in uT
    magBias[1] = -9.5;  // User environmental y-axis correction in uT
    magBias[2] = -26.0; // User environmental z-axis correction in uT
}

void MPU9150::calibrateMPU9150() {  
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
    
    // reset device, reset all registers, clear gyro and accelerometer bias registers
    writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    wait(0.1);  
    
    // get stable time source
    // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x01);  
    writeByte(MPU9150_ADDRESS, PWR_MGMT_2, 0x00); 
    wait(0.2);
    
    // Configure device for bias calculation
    writeByte(MPU9150_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
    writeByte(MPU9150_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
    writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
    writeByte(MPU9150_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
    writeByte(MPU9150_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    writeByte(MPU9150_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
    wait(0.015);
    
    // Configure MPU9150 gyro and accelerometer for bias calculation
    writeByte(MPU9150_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
    writeByte(MPU9150_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    writeByte(MPU9150_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    writeByte(MPU9150_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
    
    uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    writeByte(MPU9150_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO  
    writeByte(MPU9150_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO (max size 1024 bytes in MPU9150)
    wait(0.08); // accumulate 80 samples in 80 milliseconds = 960 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    writeByte(MPU9150_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
    readBytes(MPU9150_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        readBytes(MPU9150_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
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
        
    if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
    else {accel_bias[2] += (int32_t) accelsensitivity;}
    
    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;

    /// Push gyro biases to hardware registers
    writeByte(MPU9150_ADDRESS, XG_OFFS_USRH, data[0]); 
    writeByte(MPU9150_ADDRESS, XG_OFFS_USRL, data[1]);
    writeByte(MPU9150_ADDRESS, YG_OFFS_USRH, data[2]);
    writeByte(MPU9150_ADDRESS, YG_OFFS_USRL, data[3]);
    writeByte(MPU9150_ADDRESS, ZG_OFFS_USRH, data[4]);
    writeByte(MPU9150_ADDRESS, ZG_OFFS_USRL, data[5]);

    gyroBias[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
    gyroBias[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
    gyroBias[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    readBytes(MPU9150_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
    readBytes(MPU9150_ADDRESS, YA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
    readBytes(MPU9150_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];
    
    uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
    
    for(ii = 0; ii < 3; ii++) {
        if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }

    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1]/8);
    accel_bias_reg[2] -= (accel_bias[2]/8);
    
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
    writeByte(MPU9150_ADDRESS, XA_OFFSET_H, data[0]);  
    writeByte(MPU9150_ADDRESS, XA_OFFSET_L_TC, data[1]);
    writeByte(MPU9150_ADDRESS, YA_OFFSET_H, data[2]);
    writeByte(MPU9150_ADDRESS, YA_OFFSET_L_TC, data[3]);
    writeByte(MPU9150_ADDRESS, ZA_OFFSET_H, data[4]);
    writeByte(MPU9150_ADDRESS, ZA_OFFSET_L_TC, data[5]);

    // Output scaled accelerometer biases for manual subtraction in the main program
    accelBias[0] = (float)accel_bias[0]/(float)accelsensitivity; 
    accelBias[1] = (float)accel_bias[1]/(float)accelsensitivity;
    accelBias[2] = (float)accel_bias[2]/(float)accelsensitivity;
}


void MPU9150::MPU9150SelfTest() {
    uint8_t rawData[4] = {0, 0, 0, 0};
    uint8_t selfTest[6];
    float factoryTrim[6];
    
    // Configure the accelerometer for self-test
    writeByte(MPU9150_ADDRESS, ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
    writeByte(MPU9150_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    wait(0.25);  // Delay a while to let the device execute the self-test
    rawData[0] = readByte(MPU9150_ADDRESS, SELF_TEST_X); // X-axis self-test results
    rawData[1] = readByte(MPU9150_ADDRESS, SELF_TEST_Y); // Y-axis self-test results
    rawData[2] = readByte(MPU9150_ADDRESS, SELF_TEST_Z); // Z-axis self-test results
    rawData[3] = readByte(MPU9150_ADDRESS, SELF_TEST_A); // Mixed-axis self-test results
    // Extract the acceleration test results first
    selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
    selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 4 ; // YA_TEST result is a five-bit unsigned integer
    selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 4 ; // ZA_TEST result is a five-bit unsigned integer
    // Extract the gyration test results first
    selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
    selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
    selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer   
    // Process results to allow final comparison with factory set values
    factoryTrim[0] = (4096.0f*0.34f)*(pow( (0.92f/0.34f) , ((selfTest[0] - 1.0f)/30.0f))); // FT[Xa] factory trim calculation
    factoryTrim[1] = (4096.0f*0.34f)*(pow( (0.92f/0.34f) , ((selfTest[1] - 1.0f)/30.0f))); // FT[Ya] factory trim calculation
    factoryTrim[2] = (4096.0f*0.34f)*(pow( (0.92f/0.34f) , ((selfTest[2] - 1.0f)/30.0f))); // FT[Za] factory trim calculation
    factoryTrim[3] =  ( 25.0f*131.0f)*(pow( 1.046f , (selfTest[3] - 1.0f) ));             // FT[Xg] factory trim calculation
    factoryTrim[4] =  (-25.0f*131.0f)*(pow( 1.046f , (selfTest[4] - 1.0f) ));             // FT[Yg] factory trim calculation
    factoryTrim[5] =  ( 25.0f*131.0f)*(pow( 1.046f , (selfTest[5] - 1.0f) ));             // FT[Zg] factory trim calculation
    // To get to percent, must multiply by 100 and subtract result from 100
    for (int i = 0; i < 6; i++) {
        SelfTest_[i] = 100.0f + 100.0f*(selfTest[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
    }
}

void MPU9150::MadgwickQuaternionUpdate(float quat[4], float acc[3], float gyr[3], float mag[3], float dt) {
    float q1 = quat[0], q2 = quat[1], q3 = quat[2], q4 = quat[3];   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    float GyroMeasError = pi * (60.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
    float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta

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
    norm = sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    acc[0] *= norm;
    acc[1] *= norm;
    acc[2] *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    mag[0] *= norm;
    mag[1] *= norm;
    mag[2] *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mag[0];
    _2q1my = 2.0f * q1 * mag[1];
    _2q1mz = 2.0f * q1 * mag[2];
    _2q2mx = 2.0f * q2 * mag[0];
    hx = mag[0] * q1q1 - _2q1my * q4 + _2q1mz * q3 + mag[0] * q2q2 + _2q2 * mag[1] * q3 + _2q2 * mag[2] * q4 - mag[0] * q3q3 - mag[0] * q4q4;
    hy = _2q1mx * q4 + mag[1] * q1q1 - _2q1mz * q2 + _2q2mx * q3 - mag[1] * q2q2 + mag[1] * q3q3 + _2q3 * mag[2] * q4 - mag[1] * q4q4;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mag[2] * q1q1 + _2q2mx * q4 - mag[2] * q2q2 + _2q3 * mag[1] * q4 - mag[2] * q3q3 + mag[2] * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - acc[0]) + _2q2 * (2.0f * q1q2 + _2q3q4 - acc[1]) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mag[0]) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - mag[1]) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mag[2]);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - acc[0]) + _2q1 * (2.0f * q1q2 + _2q3q4 - acc[1]) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - acc[2]) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mag[0]) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - mag[1]) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mag[2]);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - acc[0]) + _2q4 * (2.0f * q1q2 + _2q3q4 - acc[1]) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - acc[2]) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mag[0]) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - mag[1]) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mag[2]);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - acc[0]) + _2q3 * (2.0f * q1q2 + _2q3q4 - acc[1]) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mag[0]) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - mag[1]) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mag[2]);
    norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gyr[0] - q3 * gyr[1] - q4 * gyr[2]) - beta * s1;
    qDot2 = 0.5f * (q1 * gyr[0] + q3 * gyr[2] - q4 * gyr[1]) - beta * s2;
    qDot3 = 0.5f * (q1 * gyr[1] - q2 * gyr[2] + q4 * gyr[0]) - beta * s3;
    qDot4 = 0.5f * (q1 * gyr[2] + q2 * gyr[1] - q3 * gyr[0]) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;
    q4 += qDot4 * dt;
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    quat[0] = q1 * norm;
    quat[1] = q2 * norm;
    quat[2] = q3 * norm;
    quat[3] = q4 * norm;

}

void MPU9150::MahonyQuaternionUpdate(float quat[4],float acc0[3], float gyr0[3], float mag0[3], float dt) {
    float q1 = quat[0], q2 = quat[1], q3 = quat[2], q4 = quat[3];   // short name local variable for readability
    
    float acc[3], gyr[3], mag[3];


    
    float norm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float pa, pb, pc;

    float eInt[3] = {0.0f, 0.0f, 0.0f};              // vector to hold integral error for Mahony method
    float GyroMeasDrift = pi * (1.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
    float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
    #define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
    #define Ki 0.0f

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
    norm = sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;        // use reciprocal for division
    acc[0] = acc0[0] * norm;
    acc[1] = acc0[1] * norm;
    acc[2] = acc0[2] * norm;

    // Normalise magnetometer measurement
    norm = sqrt(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;        // use reciprocal for division
    mag[0] = mag0[0] * norm;
    mag[1] = mag0[0] * norm;
    mag[2] = mag0[0] * norm;

    // Reference direction of Earth's magnetic field
    hx = 2.0f * mag[0] * (0.5f - q3q3 - q4q4) + 2.0f * mag[1] * (q2q3 - q1q4) + 2.0f * mag[2] * (q2q4 + q1q3);
    hy = 2.0f * mag[0] * (q2q3 + q1q4) + 2.0f * mag[1] * (0.5f - q2q2 - q4q4) + 2.0f * mag[2] * (q3q4 - q1q2);
    bx = sqrt((hx * hx) + (hy * hy));
    bz = 2.0f * mag[0] * (q2q4 - q1q3) + 2.0f * mag[1] * (q3q4 + q1q2) + 2.0f * mag[2] * (0.5f - q2q2 - q3q3);

    // Estimated direction of gravity and magnetic field
    vx = 2.0f * (q2q4 - q1q3);
    vy = 2.0f * (q1q2 + q3q4);
    vz = q1q1 - q2q2 - q3q3 + q4q4;
    wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
    wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
    wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);  

    // Error is cross product between estimated direction and measured direction of gravity
    ex = (acc[1] * vz - acc[2] * vy) + (mag[1] * wz - mag[2] * wy);
    ey = (acc[2] * vx - acc[0] * vz) + (mag[2] * wx - mag[0] * wz);
    ez = (acc[0] * vy - acc[1] * vx) + (mag[0] * wy - mag[1] * wx);
    if (Ki > 0.0f)
    {
        eInt[0] += ex;      // accumulate integral error
        eInt[1] += ey;
        eInt[2] += ez;
    }
    else
    {
        eInt[0] = 0.0f;     // prevent integral wind up
        eInt[1] = 0.0f;
        eInt[2] = 0.0f;
    }

    // Apply feedback terms
    gyr[0] = gyr0[0] + Kp * ex + Ki * eInt[0];
    gyr[1] = gyr0[1] + Kp * ey + Ki * eInt[1];
    gyr[2] = gyr0[2] + Kp * ez + Ki * eInt[2];

    // Integrate rate of change of quaternion
    pa = q2;
    pb = q3;
    pc = q4;
    q1 = q1 + (-q2 * gyr[0] - q3 * gyr[1] - q4 * gyr[2]) * (0.5f * dt);
    q2 = pa + (q1 * gyr[0] + pb * gyr[2] - pc * gyr[1]) * (0.5f * dt);
    q3 = pb + (q1 * gyr[1] - pa * gyr[2] + pc * gyr[0]) * (0.5f * dt);
    q4 = pc + (q1 * gyr[2] + pa * gyr[1] - pb * gyr[0]) * (0.5f * dt);

    // Normalise quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    quat[0] = q1 * norm;
    quat[1] = q2 * norm;
    quat[2] = q3 * norm;
    quat[3] = q4 * norm;

}