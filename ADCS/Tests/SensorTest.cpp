#include "Test.h"
#include "SunSensor.h"
#include "MPU9150.h"

int SensorTest(Serial *pc, I2C *i2c, Timer *t){
    SunSensor sun(A0,A1,A2);
    MPU9150 imu(i2c);

    int ellapsed = 0;
    int last_update = 0;
    int print_update = t->read_ms();

    float rsun_b[3];
    float rsun_e[3];
    float rmag_b[3];
    float rmag_e[3] = {17.5, 0.5, 47};

    int mcount = 0; // Frequency divider for the magnetometer
    uint8_t MagRate = 50; // set magnetometer read rate in Hz; 10 to 100 (max) Hz are reasonable values
    uint8_t acc_scale = AFS_2G;
    uint8_t gyr_scale = GFS_250DPS;
    float val_acc[3], val_gyr[3], val_mag[3];

    //--------------------- INIT ---------------------//
    pc->printf("\n\r\n\r\n\r\n\r\n\r\n\r--------------------------------------\n\r");
    pc->printf("Connection ok\n\r");
    uint8_t whoami = imu.readByte(MPU9150_ADDRESS, WHO_AM_I_MPU9150);  // Read WHO_AM_I register for MPU-9150
    pc->printf("I AM 0x%x\n\r", whoami); pc->printf("I SHOULD BE 0x68 or 0x73\n\r");
    if (!imu.initIMU(acc_scale, gyr_scale)) {   // WHO_AM_I should be 0x68
        pc->printf("Could not connect to MPU9150: \n\r");
        while(1) ; // Loop forever if communication doesn't happen
    }
    imu.recalibrateIMU(1000, 100);
    float null_avg[3] = {0,0,0};
    imu.setAvgAcc(null_avg);
    imu.setAvgMag(null_avg);
    pc->printf("IMU ok\n\r");

    Matrix dq(4,1);
    Matrix gyr(3,1);
    Matrix quat = Matrix::zeros(4,1);
    quat(2) = 1;
    float dt2, w_norm;
    int time = t->read_us();

    while(1){
        last_update = t->read_us();

        //--------------------- LOOP ---------------------//
        if(imu.readByte(MPU9150_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
            imu.getAccel(val_acc);  // Read the x/y/z adc values
            imu.getGyro(val_gyr);  // Read the x/y/z adc values
            sun.getSunVector(rsun_b);

            mcount++;
            if (mcount > 200/MagRate) {  // this is a poor man's way of setting the magnetometer read rate (see below) 
                imu.getMag(val_mag);  // Read the x/y/z adc values
                mcount = 0;
            }
        
            gyr = Matrix(3,1, val_gyr);
            gyr *= 3.1415926535f/180.0f;
            w_norm = gyr.norm();
            dt2 = 0.5 * ( t->read_us() - time );
            time = t->read_us();
            dq(1) = cos(w_norm*dt2);
            gyr *= sin(w_norm*dt2) / w_norm;
            dq(2) = gyr(1);
            dq(3) = gyr(2);
            dq(4) = gyr(3);
            quat = Matrix::quatmul(dq, quat);
            quat *= 1/quat.norm();
        }
        //------------------- END LOOP -------------------//

        ellapsed = t->read_us() - last_update;

        //-------------------- PRINT ---------------------//



        // pc->printf("Print update: %d\n\r", t->read_ms());
        if(t->read_ms() - print_update > 500){
            print_update = t->read_ms();
            pc->printf("\n\r\n\rLoop time %d us | Frequency %4.0f Hz\n\r", ellapsed, 1000000.0f/ellapsed);
            pc->printf("Acc (mg):  ");
            pc->printf("{% 4.2f, % 4.2f, % 4.2f}\n\r", val_acc[0]*1000, val_acc[1]*1000, val_acc[2]*1000);
            pc->printf("Gyr (°/s): ");
            pc->printf("{% 4.2f, % 4.2f, % 4.2f}\n\r", val_gyr[0], val_gyr[1], val_gyr[2]);
            pc->printf("Mag (uT):  ");
            pc->printf("{% 4.2f, % 4.2f, % 4.2f}\n\r", val_mag[0], val_mag[1], val_mag[2]);
            pc->printf("Sun:       ");
            pc->printf("{% 4.2f, % 4.2f, % 4.2f}\n\r", rsun_b[0], rsun_b[1], rsun_b[2]);
            pc->printf("Quaternion\n\r");
            printMat(dq, pc);
        }

        if(t->read_ms() > 1<<21) {
            t->start(); // start the timer over again if ~30 minutes has passed
            last_update = t->read_us();
        }
        wait_ms(20);
    }

    return 1;
}