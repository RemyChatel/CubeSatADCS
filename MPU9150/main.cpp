#include "mbed.h"
#include "MPU9150.h"

#define DEG2RAD 3.14159265358979323846f / 180.0f
#define RAD2DEG 180.0f / 3.14159265358979323846f
#define PI 3.14159265358979323846f

Serial pc(USBTX, USBRX, 115200);
I2C i2c(I2C_SDA, I2C_SCL);
Timer t;
MPU9150 MPU9150(&i2c);

int main()
{
    float sum = 0;
    uint32_t sumCount = 0, mcount = 0;
    int delt_t = 0; // used to control display output rate
    int count = t.read_ms();
    int count2 = 0;  // used to control display output rate
    float deltat = 0.0f;                             // integration interval for both filter schemes

    int lastUpdate = 0, firstUpdate = 0, Now = 0;    // used to calculate integration interval 
    char buffer[14];

    float pitch, yaw, roll;
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};           // vector to hold quaternion

    i2c.frequency(400000);  // use fast (400 kHz) I2C   
    t.start();              // starting timer

    uint8_t MagRate = 100; // set magnetometer read rate in Hz; 10 to 100 (max) Hz are reasonable values
    float val_acc[3], val_gyr[3], val_mag[3];
    
    // Read the WHO_AM_I register, this is a good test of communication
    uint8_t whoami = MPU9150.readByte(MPU9150_ADDRESS, WHO_AM_I_MPU9150);  // Read WHO_AM_I register for MPU-9150
    pc.printf("I AM 0x%x\n\r", whoami); pc.printf("I SHOULD BE 0x68 or 0x73\n\r");
  
    if (whoami == 0x73) {   // WHO_AM_I should be 0x68
        pc.printf("MPU9150 WHO_AM_I is 0x%x\n\r", whoami);
        pc.printf("MPU9150 is online...\n\r");
        wait(1);
        
        MPU9150.MPU9150SelfTest();
        wait(1);
        MPU9150.resetMPU9150(); // Reset registers to default in preparation for device calibration
        MPU9150.calibrateMPU9150(); // Calibrate gyro and accelerometers, load biases in bias registers  
        wait(1);
        MPU9150.initMPU9150(); 
        pc.printf("MPU9150 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
        MPU9150.initAK8975A();
        pc.printf("AK8975 initialized for active data mode....\n\r"); // Initialize device for active mode read of magnetometer
    }
    else {
        pc.printf("Could not connect to MPU9150: \n\r");
        pc.printf("%#x \n",  whoami);
        while(1) ; // Loop forever if communication doesn't happen
    }
 
    while(1) {
        int length_timer = t.read_us();
        // If intPin goes high, all data registers have new data
        if(MPU9150.readByte(MPU9150_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
            MPU9150.getAccel(val_acc);  // Read the x/y/z adc values
        
            MPU9150.getGyro(val_gyr);  // Read the x/y/z adc values   
            val_gyr[0] *=DEG2RAD;
            val_gyr[1] *=DEG2RAD;
            val_gyr[2] *=DEG2RAD;
        
            mcount++;
            if (mcount > 200/MagRate) {  // this is a poor man's way of setting the magnetometer read rate (see below) 
            MPU9150.getMag(val_mag);  // Read the x/y/z adc values
            mcount = 0;
            }
        }
   
        Now = t.read_us();
        deltat = (float)((Now - lastUpdate)/1000000.0f) ; // set integration time by time elapsed since last filter update
        lastUpdate = Now;
        
        sum += deltat;
        sumCount++;
    
        //    if(lastUpdate - firstUpdate > 10000000.0f) {
        //        beta = 0.04;  // decrease filter gain after stabilized
        //        zeta = 0.015; // increasey bias drift gain after stabilized
        //    }
        
        // Pass gyro rate as rad/s
        // MPU9150.MadgwickQuaternionUpdate(q, val_acc, val_gyr, val_mag, deltat);
        MPU9150.MahonyQuaternionUpdate(q, val_acc, val_gyr, val_mag, deltat);
    
        // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
        // In this coordinate system, the positive z-axis is down toward Earth. 
        // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
        // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
        // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
        // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
        // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
        // applied in the correct order which for this configuration is yaw, pitch, and then roll.
        // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
        yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
        pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
        roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
        pitch *= 180.0f / PI;
        yaw   *= 180.0f / PI; 
        yaw   -= -2.48f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
        roll  *= 180.0f / PI;
        
        int convtime = length_timer - t.read_us();
    
        
        // Serial print and/or display at 0.5 s rate independent of data rates
        delt_t = t.read_ms() - count;
        if (delt_t > 500) { // update LCD once per half-second independent of read rate initial 500
            pc.printf("Conversion time %d us \n\r", convtime);
            
            pc.printf("ax = %f", 1000*val_acc[0]); 
            pc.printf(" ay = %f", 1000*val_acc[1]); 
            pc.printf(" az = %f  mg\n\r", 1000*val_acc[2]); 

            pc.printf("gx = %f", val_gyr[0]); 
            pc.printf(" gy = %f", val_gyr[1]); 
            pc.printf(" gz = %f  deg/s\n\r", val_gyr[2]); 
            
            pc.printf("gx = %3.1f", val_mag[0]); 
            pc.printf(" gy = %3.1f", val_mag[1]); 
            pc.printf(" gz = %3.1f  uT\n\r", val_mag[2]);

            //tempCount = MPU9150.readTempData();  // Read the adc values
            //temperature = ((float) tempCount) / 340.0f + 36.53f; // Temperature in degrees Centigrade
            //pc.printf(" temperature = %f  C\n\r", temperature); 
            /*    
            pc.printf("q0 = %f\n\r", q[0]);
            pc.printf("q1 = %f\n\r", q[1]);
            pc.printf("q2 = %f\n\r", q[2]);
            pc.printf("q3 = %f\n\r", q[3]);      
            */


            pc.printf("Yaw, Pitch, Roll: %f %f %f\n\r", yaw, pitch, roll);
            //pc.printf("average rate = %f\n\r", (float) sumCount/sum);
            count = t.read_ms(); 

            if(count > 1<<21) {
                t.start(); // start the timer over again if ~30 minutes has passed
                count = 0;
                deltat= 0;
                lastUpdate = t.read_us();
            }
            sum = 0;
            sumCount = 0; 
        }
    } // while(1)
} // main()