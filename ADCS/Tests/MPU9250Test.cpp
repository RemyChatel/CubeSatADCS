#include "Test.h"
#include "MPU9250.h"


int MPU9250Test(Serial *pc, I2C *i2c, Timer *t) {

    MPU9250 MPU9250(i2c);
    /*************************** INIT *****************************/
    i2c->frequency(400000);  // use fast (400 kHz) I2C 
    t->start();
  
    pc->printf("Connection ok\r\n");       
  
    char buffer[14];

    float acc[3], gyr[3], mag[3];
    float q[4];
    float roll = 0, pitch = 0, yaw = 0;
    
    // Read the WHO_AM_I register, this is a good test of communication
    uint8_t whoami = MPU9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9150
    pc->printf("I AM 0x%x\n\r", whoami); pc->printf("I SHOULD BE 0x68 or 0x73\n\r");
  
    if (whoami == 0x73) { // WHO_AM_I should be 0x68  
        pc->printf("MPU9250 WHO_AM_I is 0x%x\n\r", whoami);
        pc->printf("MPU9250 is online...\n\r");

        wait(.5);
    
        MPU9250.MPU9250SelfTest(MPU9250.SelfTest);
            pc->printf("x-axis self test: acceleration trim within %f  of factory value\n\r", MPU9250.SelfTest[0]);
            pc->printf("y-axis self test: acceleration trim within %f  of factory value\n\r", MPU9250.SelfTest[1]);
            pc->printf("z-axis self test: acceleration trim within %f  of factory value\n\r", MPU9250.SelfTest[2]);
            pc->printf("x-axis self test: gyration trim within %f  of factory value\n\r", MPU9250.SelfTest[3]);
            pc->printf("y-axis self test: gyration trim within %f  of factory value\n\r", MPU9250.SelfTest[4]);
            pc->printf("z-axis self test: gyration trim within %f  of factory value\n\r", MPU9250.SelfTest[5]);

        wait(.5);

        MPU9250.resetMPU9250(); // Reset registers to default in preparation for device calibration
        MPU9250.calibrateMPU9250(); // Calibrate gyro and accelerometers, load biases in bias registers  
            pc->printf("x gyro bias = %f\n\r", MPU9250.gyroBias[0]);
            pc->printf("y gyro bias = %f\n\r", MPU9250.gyroBias[1]);
            pc->printf("z gyro bias = %f\n\r", MPU9250.gyroBias[2]);
            pc->printf("x accel bias = %f\n\r", MPU9250.accelBias[0]);
            pc->printf("y accel bias = %f\n\r", MPU9250.accelBias[1]);
            pc->printf("z accel bias = %f\n\r", MPU9250.accelBias[2]);

        wait(.5);
        MPU9250.initMPU9250(); 
        pc->printf("MPU9250 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
        MPU9250.initAK8963(MPU9250.magCalibration);
        pc->printf("AK8963 initialized for active data mode....\n\r"); // Initialize device for active mode read of magnetometer
    }
    else {
        pc->printf("Could not connect to MPU9250: \n\r");
        pc->printf("%#x \n",  whoami);
 
            while(1) ; // Loop forever if communication doesn't happen
    }

    uint8_t MagRate = 100; // set magnetometer read rate in Hz; 10 to 100 (max) Hz are reasonable values
    MPU9250.getAres(); // Get accelerometer sensitivity
    MPU9250.getGres(); // Get gyro sensitivity
    MPU9250.mRes = 1229./4096.; // Conversion to 1229 microTesla full scale (4096)
    // So far, magnetometer bias is calculated and subtracted here manually, should construct an algorithm to do it automatically
    // like the gyro and accelerometer biases
    /*
    MPU9250.magbias[0] = -50.;   // User environmental x-axis correction in milliGauss
    MPU9250.magbias[1] = -950.;  // User environmental y-axis correction in milliGauss
    MPU9250.magbias[2] = -2600.; // User environmental z-axis correction in milliGauss
    */
    MPU9250.magbias[0] = -0.;   // User environmental x-axis correction in milliGauss
    MPU9250.magbias[1] = -0.;  // User environmental y-axis correction in milliGauss
    MPU9250.magbias[2] = -0.; // User environmental z-axis correction in milliGauss


    /*************************** LOOP *****************************/
 
    int count;
    count = t->read_ms();
    uint32_t  mcount = 0;
    float deltat = 0;
    int lastUpdate = 0;
    int Now = 0;
    int delt_t = 0;
    int convtime = 0;

    while(1) {

    Now = t->read_us();

    // If intPin goes high, all data registers have new data
    if(MPU9250.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
        MPU9250.getAccel(acc);
        MPU9250.getGyro(gyr);
        MPU9250.getMag(mag);
    }
   
    deltat = (float)((Now - lastUpdate)) ; // set integration time by time elapsed since last filter update
    lastUpdate = Now;
    
    gyr[0] *= PI/180.0f;
    gyr[1] *= PI/180.0f;
    gyr[2] *= PI/180.0f;

    /*
    MPU9250.MahonyQuaternionUpdate(acc, gyr, mag, deltat, q);
  
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    roll  *= 180.0f / PI;
    yaw   -= -2.48f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    */
    convtime = t->read_us() - Now;
    /*
    pc->printf("ax = %f", 1000*acc[0]); 
    pc->printf(" ay = %f", 1000*acc[1]); 
    pc->printf(" az = %f  mg\n\r", 1000*acc[2]); 

    pc->printf("gx = %f", gyr[0]); 
    pc->printf(" gy = %f", gyr[1]); 
    pc->printf(" gz = %f  deg/s\n\r", gyr[2]); 
    */
    pc->printf("gx = %e", mag[0]); 
    pc->printf(" gy = %e", mag[1]); 
    pc->printf(" gz = %e  uT\n\r", mag[2]);

    int16_t magc[3];
    MPU9250.readMagData(magc);
    pc->printf("gxcount = %d"     , magc[0]); 
    pc->printf(" gycount = %d"    , magc[1]); 
    pc->printf(" gzcount = %d\n\r", magc[2]);

    pc->printf("gxCal = %f"     , MPU9250.magCalibration[0]); 
    pc->printf(" gyCal = %f"    , MPU9250.magCalibration[1]); 
    pc->printf(" gzCal = %f\n\r", MPU9250.magCalibration[2]);

    pc->printf("mRes: %f\n\r", MPU9250.mRes);

    /*    
    pc->printf("q0 = %f\n\r", q[0]);
    pc->printf("q1 = %f\n\r", q[1]);
    pc->printf("q2 = %f\n\r", q[2]);
    pc->printf("q3 = %f\n\r", q[3]);      
    */
    // pc->printf("Yaw, Pitch, Roll: %3.1f %3.1f %3.1f\n\r", yaw, pitch, roll);

    // pc->printf("Conversion time %d us | Frequency %4.0f\n\r", convtime, 1000000.0f/convtime);

    wait(5);
    }

    return 1;
}