#include "Test.h"
#include "Estimators.h"
#include "AstroLib.h"
#define DEG2RAD 3.1415926535f/180.0f
#define RAD2DEG 180.0f/3.1415926535f

int QuestTest(Serial *pc, I2C *i2c, Timer *t){
    
    int lastUpdate = 0;
    int ellapsed = 0;
    int seconds = 0;;
    int minutes = 0;
    t->start();
    pc->baud(115200);
    pc->printf("\n\r\n\r------------------------------\n\r");
    pc->printf("Connection OK\n\r");

    Matrix quat(4,1);
    //#define NOISE
    #ifdef NOISE
    float coef_th[9] = {0.4153, 0.4472, 0.7921, -0.7652, 0.6537, 0.0274, -0.5056, -0.6104, 0.6097};
    Matrix mat_th = Matrix(3,3, coef_th);
    
    float sa1n[3] = {0.0f, 0.447214f, 0.894427f};
    float sa2n[3] = {0.316228f, 0.948683f, 0.0f};
    float sa3n[3] = {-0.980581f, 0.0f, 0.196116f};
    float sa4n[3] = {0.235702f, -0.235702f, 0.942809f};
    float sa5n[3] = {0.57735f, 0.57735f, 0.57735f};

    float sb1n[3] = { 0.9082f, 0.3185f, 0.2715f };
    float sb2n[3] = { 0.5670f, 0.3732f, -0.7343f };
    float sb3n[3] = { -0.2821f, 0.7163f, 0.6382 };
    float sb4n[3] = { 0.7510f, -0.3303f, 0.5718};
    float sb5n[3] = { 0.9261f, -0.2053, -0.3166};
    
    float *san[5] = {sa1n, sa2n, sa3n, sa4n, sa5n};
    float *sbn[5] = {sb1n, sb2n, sb3n, sb4n, sb5n};
    
    #else
    // To test without noise
    Matrix mat_th = Matrix::Rot321(45 * DEG2RAD, -30*DEG2RAD, 60*DEG2RAD);
    float sa1n[3],sa2n[3],sa3n[3],sa4n[3],sa5n[3];
    float sb1n[3],sb2n[3],sb3n[3],sb4n[3],sb5n[3];
    float *san[5] = {sa1n, sa2n, sa3n, sa4n, sa5n};
    float *sbn[5] = {sb1n, sb2n, sb3n, sb4n, sb5n};
    
    float sa1[3] = {0, 1, 2};
    float sa2[3] = {1, 3, 0};
    float sa3[3] = {-5, 0, 1};
    float sa4[3] = {1, -1, 4};
    float sa5[3] = {1, 1, 1};

    Matrix sa[5];
    Matrix sb[5];
    sa[0] = Matrix(3,1,sa1);
    sa[1] = Matrix(3,1,sa2);
    sa[2] = Matrix(3,1,sa3);
    sa[3] = Matrix(3,1,sa4);
    sa[4] = Matrix(3,1,sa5);
    sb[0] = Matrix(3,1);
    sb[1] = Matrix(3,1);
    sb[2] = Matrix(3,1);
    sb[3] = Matrix(3,1);
    sb[4] = Matrix(3,1);

    for(int i = 0; i < 5; i++){
        sa[i] *= 1/sa[i].norm();
        sb[i] = mat_th*sa[i];
        sb[i] *= 1/sb[i].norm();
        sa[i].getCoef(san[i]);
        sb[i].getCoef(sbn[i]);
    }
    #endif

    float om[5] = {0.0100f, 0.0325f, 0.0550f, 0.0775, 0.1000};

    float q[4];

    while(1){
    lastUpdate = t->read_us();

    /**************** LOOP ****************/

    Estimators::QUEST(q, 5, sbn, san, om, 1e-5);
    
    /************** LOOP END **************/

    ellapsed = t->read_us()-lastUpdate;
    pc->printf("\n\r\n\rLoop time %d us | Frequency %4.0f Hz\n\r", ellapsed, 1000000.0f/ellapsed);

    /*************** PRINTS ***************/
    quat = Matrix(4,1, q);
    Matrix mat_rot(3,3);
    mat_rot = Matrix::quat2rot(quat);
    mat_rot = mat_rot.Transpose(); // Taking into account the 1-2-3 rot matrix
    pc->printf("Computed Rotation matrix\n\r");
    printMat(mat_rot, pc);
    pc->printf("Expected Rotation matrix\n\r");
    printMat(mat_th, pc);

    Matrix mat_error = mat_rot * mat_th.Transpose();
    float cos_error = 0.5 * (mat_error.trace() - 1);
    float error = acos((cos_error>1)?2-cos_error:cos_error);

    pc->printf("Error matrix\n\r");
    printMat(mat_error, pc);
    pc->printf("Cos Angular error %f\n\r", cos_error);
    pc->printf("Angular error %f deg\n\r", error*180.0f/3.141592f);
    pc->printf("Difference matrix in percents\n\r");
    printMat((mat_th - mat_rot)*100, pc);

    pc->printf("Expected quaternion\n\r");
    printMat(Matrix::rot2quat(mat_th), pc);
    pc->printf("Computed quaternion\n\r");
    printMat(quat, pc);

    pc->printf("Expected Euler\n\r");
    printMat(RAD2DEG*Matrix::quat2euler(Matrix::rot2quat(mat_th)), pc);
    pc->printf("Computed Euler\n\r");
    printMat(RAD2DEG*Matrix::quat2euler(quat), pc);

    /************* PRINTS END **************/

    seconds+=5;
    if(seconds==60){
        seconds = 0;
        minutes++;
    }
    wait(5);
    }

    return 1;
}