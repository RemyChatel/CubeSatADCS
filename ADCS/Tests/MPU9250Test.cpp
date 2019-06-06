#include "Test.h"
#include "MPU9150.h"

#define DEG2RAD 3.14159265358979323846f / 180.0f
#define RAD2DEG 180.0f / 3.14159265358979323846f
#define PI 3.14159265358979323846f

int MPU9250Test(Serial *pc, I2C *i2c, Timer *t) {
    
    MPU9150 MPU9150(i2c);
    float sum = 0;
    uint32_t sumCount = 0, mcount = 0;
    int delt_t = 0; // used to control display output rate
    int count = t->read_ms();
    int count2 = 0;  // used to control display output rate
    float deltat = 0.0f;                             // integration interval for both filter schemes

    int lastUpdate = 0, firstUpdate = 0, Now = 0;    // used to calculate integration interval 
    char buffer[14];

    float pitch, yaw, roll;
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};           // vector to hold quaternion

    i2c->frequency(400000);  // use fast (400 kHz) I2C   
    t->start();              // starting timer

    uint8_t MagRate = 100; // set magnetometer read rate in Hz; 10 to 100 (max) Hz are reasonable values
    float val_acc[3], val_gyr[3], val_mag[3];
    uint8_t acc_scale = AFS_2G;
    uint8_t gyr_scale = GFS_250DPS;
    
    // Read the WHO_AM_I register, this is a good test of communication
    uint8_t whoami = MPU9150.readByte(MPU9150_ADDRESS, WHO_AM_I_MPU9150);  // Read WHO_AM_I register for MPU-9150
    pc->printf("I AM 0x%x\n\r", whoami); pc->printf("I SHOULD BE 0x68 or 0x73\n\r");
  
    if (!MPU9150.initIMU(acc_scale, gyr_scale)) {   // WHO_AM_I should be 0x68
        pc->printf("Could not connect to MPU9150: \n\r");
        while(1) ; // Loop forever if communication doesn't happen
    }


    wait(2);
 
    while(1) {
        int length_timer = t->read_us();
        // If intPin goes high, all data registers have new data
        if(MPU9150.readByte(MPU9150_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
            MPU9150.getAccel(val_acc);  // Read the x/y/z adc values
        
            MPU9150.getGyro(val_gyr);  // Read the x/y/z adc values
        
            mcount++;
            if (mcount > 200/MagRate) {  // this is a poor man's way of setting the magnetometer read rate (see below) 
                MPU9150.getMag(val_mag);  // Read the x/y/z adc values
                mcount = 0;
            }
        }
   
        Now = t->read_us();
        deltat = (float)((Now - lastUpdate)/1000000.0f) ; // set integration time by time elapsed since last filter update
        lastUpdate = Now;

        // Pass gyro rate as rad/s   
        val_gyr[0] *= DEG2RAD;
        val_gyr[1] *= DEG2RAD;
        val_gyr[2] *= DEG2RAD;
        MPU9150.MahonyQuaternionUpdate(q, val_acc, val_gyr, val_mag, deltat);  
        val_gyr[0] *= RAD2DEG;
        val_gyr[1] *= RAD2DEG;
        val_gyr[2] *= RAD2DEG;
        
        roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);   
        pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
        yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
        roll  *= 180.0f / PI;
        pitch *= 180.0f / PI;
        yaw   *= 180.0f / PI;
        yaw   -= -2.48f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
        
        int convtime = length_timer - t->read_us();
        
        // Serial print and/or display at 0.5 s rate independent of data rates
        delt_t = t->read_ms() - count;
        if (delt_t > 100) { // update LCD once per half-second independent of read rate initial 500

            pc->printf("Conversion time %d us \n\r", -convtime);
            
            pc->printf("ax = % 4.2f", 1000*val_acc[0]); 
            pc->printf(" ay = % 4.2f", 1000*val_acc[1]); 
            pc->printf(" az = % 4.2f  mg\n\r", 1000*val_acc[2]); 

            pc->printf("gx = %+3.1f", val_gyr[0]); 
            pc->printf(" gy = %+3.1f", val_gyr[1]); 
            pc->printf(" gz = %+3.1f  deg/s\n\r", val_gyr[2]); 
            
            pc->printf("mx = %+3.1f", val_mag[0]); 
            pc->printf(" my = %+3.1f", val_mag[1]); 
            pc->printf(" mz = %+3.1f  uT\n\r", val_mag[2]);

            // pc->printf("Yaw, Pitch, Roll: %3.2f %3.2f %3.2f\n\r", yaw, pitch, roll);
            count = t->read_ms(); 

            if(count > 1<<21) {
                t->start(); // start the timer over again if ~30 minutes has passed
                count = 0;
                deltat= 0;
                lastUpdate = t->read_us();
            }
        }
    } // while(1)
} // main()