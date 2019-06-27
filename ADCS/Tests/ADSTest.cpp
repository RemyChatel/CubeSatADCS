#include "Test.h"
#include "Matrix.h"
#include "MPU9150.h"
#include "SunSensor.h"
#include "AstroLib.h"
#include "Estimators.h"
#include "Filters.h"
#define DEG2RAD 3.1415926535f/180.0f
#define RAD2DEG 180.0f/3.1415926535f
#define ADS_NSENSOR 2

int ADSTest(Serial *pc, I2C *i2c, Timer *t){
// Initialisation                               //
    int year        = 2019;//2006;
    int month       = 06;//04;
    int day         = 27;//02;
    int hours       = 15;//00; // /!\ Use GMT+00 !!!!
    int minutes     = 00;//00;
    float seconds   = 00;
// - Global Variables                           //
    Timer time;                                 // Internal timer
    time.start();                               // Starting the internal timer
    float dt = 0.010;                           // In s
    float gyr_f[3], mag_f[3], sun_f[3];         // Temporary table to store sensor output
    Matrix gyr_b(3,1);                          // Angular rates (in rad/s)
    Matrix vbod[ADS_NSENSOR];                   // Body frame vectors [mag, sun]
    Matrix veci[ADS_NSENSOR];                   // ECI frame vectors  [mag, sun]
    for(int i = 0; i < ADS_NSENSOR; i++){       // Initializing the vector to zero
        vbod[i] = Matrix::zeros(3,1);
        veci[i] = Matrix::zeros(3,1);
    }
    Matrix q(4,1);                              // Attitude quaternion between body and ECI frame
    AstroLib::JulianDate date(year,month,day,hours,minutes,seconds);
// - Sensors                                    //
// -- IMU                                      //
    float sigma_gyr = 0.5, sigma_mag = 0.1;     // Covariance of the IMU sensors
    MPU9150 imu(i2c);
    uint8_t whoami = imu.readByte(MPU9150_ADDRESS, WHO_AM_I_MPU9150);  // Read WHO_AM_I register for MPU-9150
    pc->printf("I AM 0x%x\n\r", whoami); pc->printf("I SHOULD BE 0x68 or 0x73\n\r");
    if (!imu.initIMU(AFS_2G, GFS_250DPS)) {   // WHO_AM_I should be 0x68
        pc->printf("Could not connect to MPU9150: \n\r");
        while(1) ; // Loop forever if communication doesn't happen
    }
    imu.recalibrateIMU(1000, 100);
    float null_avg[3] = {0,0,0};
    imu.setAvgAcc(null_avg);
    imu.setAvgMag(null_avg);
    pc->printf("IMU ok\n\r");
// -- Sun                                      //
    float sigma_sun = 0.5;                      // Covariance of the sun sensor
    SunSensor sun(A0,A1,A2);
// -- Orbit model                              //
    AstroLib::Ground orbit;
    orbit.setJulianDate(AstroLib::JulianDate(year,month,day,hours,minutes,seconds));
    // Ground setting [lattitude, longitude, altitude, mag_N, mag_E, mg_D]
    orbit.setOrbit(55.86515, -4.25763, 0.0f, 17.3186f, -.6779f, 46.8663f);
// - Processes                                  //
// -- QUEST                                    //
    #define ADS_TOLERANCE 1e-5
    float omega[ADS_NSENSOR] = {1/sigma_mag, 1/sigma_sun};
    float sigma_eta = 0.1, sigma_epsilon = 0.1; // Covariance of the quest process
// -- Kalman                                   //
    Matrix I_sat(3,3);
    Matrix P_init   = Matrix::eye(7);
    Matrix Kalman_Q = Matrix::eye(7);
    Matrix Kalman_R = Matrix::eye(7);
    Matrix q_init   = Matrix::zeros(4,1);

    I_sat << 27 <<  0 <<  0
          <<  0 << 17 <<  0
          <<  0 <<  0 << 25;

    P_init *= 0.01;
    P_init(5,5) = (0.1*DEG2RAD) * (0.1*DEG2RAD);
    P_init(6,6) = (0.1*DEG2RAD) * (0.1*DEG2RAD);
    P_init(7,7) = (0.1*DEG2RAD) * (0.1*DEG2RAD);

    Kalman_Q(1,1) *= 1e-4 * dt * dt * dt / (12 * I_sat(1,1) * I_sat(1,1));
    Kalman_Q(2,2) *= 1e-4 * dt * dt * dt / (12 * I_sat(2,2) * I_sat(2,2));
    Kalman_Q(3,3) *= 1e-4 * dt * dt * dt / (12 * I_sat(3,3) * I_sat(3,3));
    Kalman_Q(4,4) *= 1e-4 * dt * dt * dt / 12;
    Kalman_Q(4,4) *=  (q_init(1)*I_sat(1,1))*(q_init(1)*I_sat(1,1))
                    + (q_init(2)*I_sat(2,2))*(q_init(2)*I_sat(2,2))
                    + (q_init(3)*I_sat(3,3))*(q_init(3)*I_sat(3,3));
    Kalman_Q(5,5) *= 1e-4 * dt * dt * dt / (     I_sat(1,1) * I_sat(1,1));
    Kalman_Q(6,6) *= 1e-4 * dt * dt * dt / (     I_sat(2,2) * I_sat(2,2));    
    Kalman_Q(7,7) *= 1e-4 * dt * dt * dt / (     I_sat(3,3) * I_sat(3,3));

    float sigma_r[7] = {sigma_eta, sigma_epsilon, sigma_epsilon, sigma_epsilon, sigma_gyr, sigma_gyr, sigma_gyr};
    for(int i = 0; i < 7; i++){ Kalman_R(i+1,i+1) *= sigma_r[i]*sigma_r[i];}
    
    q_init(2) = 1;

    Filters::KalmanFilter kalman(I_sat, Matrix::zeros(3,3), P_init, Kalman_Q, Kalman_R, q_init, Matrix::zeros(3,1),pc);

// - Timing measurement                         //
    int last_iteration = time.read_us();          // In ms
    int loop_time = 0;                          // In us

// Loop                                         //
    while(1){
        loop_time = time.read_us();
// - Processing                                 //
// -- Fetching measurements                    //
    // Earth Centered Inertial frame model
    orbit.update(time.read_us() - last_iteration);
    orbit.getSunVector(sun_f);
    orbit.getMagVector(mag_f);
    veci[1] << sun_f[0] << sun_f[1] << sun_f[2];
    veci[0] << mag_f[0] << mag_f[1] << mag_f[2];

    // Sun Sensor
    sun.getSunVector(sun_f);
    vbod[1] << sun_f[0] << sun_f[1] << sun_f[2];

    // IMU
    if(imu.readByte(MPU9150_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
        imu.getGyro(gyr_f);  // Read the x/y/z adc values
        gyr_b << gyr_f[0] << gyr_f[1] << gyr_f[2];
        imu.getMag(mag_f);  // Read the x/y/z adc values
        vbod[0] << mag_f[0] << mag_f[1] << mag_f[2];
    }

    // Normalizing measurements and converting angular rates to rad/s
    veci[1] *= 1/veci[1].norm();
    vbod[1] *= 1/vbod[1].norm();
    veci[0] *= 1/veci[0].norm();
    vbod[0] *= 1/vbod[0].norm();
    gyr_b *= DEG2RAD;

// -- QUEST Processing                         //
    Estimators::QUEST(q, ADS_NSENSOR, veci, vbod, omega, ADS_TOLERANCE);
// -- Kalman filtering                         //
    q = kalman.filter(q, gyr_b, time.read_us() - last_iteration, Matrix::zeros(3,1), Matrix::zeros(3,1), Matrix::zeros(3,1));
// - End of processing                          //
        last_iteration = time.read_us();          // Save the date of last update
        loop_time = time.read_us() - loop_time;   // Measure the execution time of the process
// - Prints                                     //
// End of Loop                                  //
    wait_ms(5);
    }
    return 1;
}