#include "Test.h"
#include "Filters.h"
#define DEG2RAD 3.1415926535f/180.0f
#define RAD2DEG 180.0f/3.1415926535f

#define LOOP_TIME 5

int FilterTest(Serial *pc, I2C *i2c, Timer *t){
    int lastUpdate = 0;
    int ellapsed = 0;
    int seconds = 0;;
    int minutes = 0;
    t->start();
    pc->printf("\n\r\n\r------------------------------\n\r");
    pc->printf("Connection OK\n\r");

    /**************** INIT ****************/
    using namespace Filters;
    Matrix p_init(7,7);
    Matrix kalman_q(7,7);
    Matrix kalman_r(7,7);
    Matrix I_sat = Matrix::eye(3);
    Matrix I_wheel = Matrix::zeros(0,0);
    KalmanFilter kalman(I_sat, I_wheel, p_init, kalman_q, kalman_r);

    Matrix q_predicted(4,1);
    Matrix q_measured(4,1);
    Matrix w_measured(3,1);
    float dt;

    /************** END INIT **************/

    while(1){
    lastUpdate = t->read_us();

    /**************** LOOP ****************/

    q_predicted = kalman.filter(q_measured, w_measured, dt, Matrix::zeros(3,1), Matrix::zeros(3,1), Matrix::zeros(3,1));
    
    /************** LOOP END **************/

    ellapsed = t->read_us()-lastUpdate;
    pc->printf("\n\r\n\rLoop time %d us | Frequency %4.0f Hz\n\r", ellapsed, 1000000.0f/ellapsed);

    /*************** PRINTS ***************/
    pc->printf("Measured quaternion\n\r");
    printMat(q_measured, pc);
    pc->printf("Predicted quaternion\n\r");
    printMat(q_predicted, pc);

    /************* PRINTS END **************/

    seconds+=LOOP_TIME;
    if(seconds>=60){
        seconds -= 60;
        minutes++;
    }
    wait(LOOP_TIME);
    }
    return 1;
}