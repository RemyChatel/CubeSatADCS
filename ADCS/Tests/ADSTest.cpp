#include "Test.h"
#include "Matrix.h"
#include "MPU9150.h"
#include "SunSensor.h"
#include "AstroLib.h"
#include "Estimators.h"
#include "Filters.h"
#include "ADSCore.h"

#define DEG2RAD 3.1415926535f/180.0f
#define RAD2DEG 180.0f/3.1415926535f
#define ADS_NSENSOR 2
#define ADS_PRINTS

int ADSTest(Serial *pc, I2C *i2c, Timer *t){
// Initialisation                               //
    int year        = 2019;//2006;
    int month       = 06;//04;
    int day         = 27;//02;
    int hours       = 15;//00; // /!\ Use GMT+00 !!!!
    int minutes     = 00;//00;
    float seconds   = 00;
    int t_print = t->read_ms();

    // Serial over Bluetooth for logging with the Python Ground station
    Serial bluetooth(D1,D0, 115200);
    
    pc->printf("\r\n\r\n\r\n\r\n\r\n\r\n");
    pc->printf("----------------------------------------\r\n");
    pc->printf("Connection ok\r\n");

// - Global Variables                           //
    t->start();                               // Starting the internal timer
    float dt = 0.010;                           // In s
    Matrix q(4,1), w(3,1), gyrb(3,1), vbod[2], veci[2];
    
    float sigma_gyr = 0.5, sigma_mag = 0.1;     // Covariance of the IMU sensors
    float sigma_sun = 0.5;                      // Covariance of the sun sensor
    // Ground setting     [lattitude, longitude, altitude,  mag_N  ,  mag_E ,   mg_D  ]
    float parameters[6] = {55.86515 , -4.25763 ,   0.0f  , 17.3186f, -.6779f, 46.8663f};
    int date[6] = {year, month, day, hours, minutes, (int)seconds};
    float sigma_eta = 0.1, sigma_epsilon = 0.1; // Covariance of the quest process

    Matrix I_sat(3,3);
    // Inertia matrix set up
    I_sat << 27 <<  0 <<  0
          <<  0 << 17 <<  0
          <<  0 <<  0 << 25;
    Matrix w_init = Matrix::zeros(3,1);
    Matrix q_init = Matrix::zeros(4,1);
    q_init(1) = 1.0f;

    ADSCore ads(i2c, A0, A1, A2);
    ads.initSensors();
    ads.initOrbit(parameters, date);
    ads.initQuest(sigma_mag, sigma_sun);
    ads.initKalman(sigma_eta, sigma_epsilon, sigma_gyr, dt, I_sat, q_init, w_init);

// - Timing measurement                         //
    int last_iteration = t->read_us();        // In us
    int loop_time = 0;                          // In us

// Loop                                         //
    pc->printf("Program initialized\r\nStarting loop:\n\r");
    while(1){
        loop_time = t->read_us();

        ads.update();

        loop_time = t->read_us() - loop_time; // Measure the execution time of the process
        last_iteration = t->read_us();        // Save the date of last update
        seconds+=(t->read_us()-last_iteration)/1000000.0f; // Update the global time
        if(seconds >= 60.0f){
            minutes++;
            seconds -= 60.0f;
        }

// - Prints                                     //
        if(t->read_ms() - t_print > 1000){
            t_print = t->read_ms();
            q = ads.getQ();
            w = ads.getW();
            gyrb = ads.getGyrb();
            vbod[0] = ads.getSensorBody(0);
            vbod[1] = ads.getSensorBody(1);
            veci[0] = ads.getSensorECI(0);
            veci[1] = ads.getSensorECI(1);

            // To Python Ground Station program
            bluetooth.printf(" %f %f %f %f %f %f %f %f \r\n", q(1), q(2), q(3), q(4), w(1), w(2), w(3), (float)loop_time);

            // To debug serial output
            pc->printf("Date: %4d-%02d-%02d-%02d:%02d:%02.2f\r\n", year, month, day, hours, minutes, seconds);
            pc->printf("Execution time (ms): %f\r\n", (float)loop_time/1000.0f);
            pc->printf("\r\n");
            pc->printf("Quaternion:\r\n");
            printMat(q.Transpose(), pc);
            pc->printf("Euler angles ZYX:\r\n");
            printMat(Matrix::quat2euler(q).Transpose()*RAD2DEG, pc);
            pc->printf("Angular rates measured then predicted\r\n");
            printMat(gyrb.Transpose(), pc);
            printMat(w.Transpose(), pc);
            pc->printf("Mag in body then ECI\r\n");
            printMat(vbod[0].Transpose(), pc);
            printMat(veci[0].Transpose(), pc);
            pc->printf("Sun in body then ECI\r\n");
            printMat(vbod[1].Transpose(), pc);
            printMat(veci[1].Transpose(), pc);

            pc->printf("\r\n");
        }
        wait_ms(10);

// End of Loop                                  //
    }
    return 1;
}