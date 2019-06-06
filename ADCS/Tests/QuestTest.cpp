#include "Test.h"
#include "Estimators.h"
#include "AstroLib.h"
#define DEG2RAD 3.1415926535f/180.0f

int QuestTest(Serial *pc, I2C *i2c, Timer *t){
    
    int lastUpdate = 0;
    int ellapsed = 0;
    int seconds = 0;;
    int minutes = 0;
    t->start();
    pc->baud(115200);
    pc->printf("\n\r\n\r------------------------------\n\r");
    pc->printf("Connection OK\n\r");

    Matrix quat;
    float coef_th[9] = {0.4153, 0.4472, 0.7921, -0.7652, 0.6537, 0.0274, -0.5056, -0.6104, 0.6097};
    Matrix mat_th = Matrix::Rot321(45 * DEG2RAD, -30*DEG2RAD, 60*DEG2RAD);

    float sa1n[3],sa2n[3],sa3n[3],sa4n[3],sa5n[3];
    float sb1n[3],sb2n[3],sb3n[3],sb4n[3],sb5n[3];
    /*
    sa1n[3] = {0.0f, 0.447214f, 0.894427f};
    sa2n[3] = {0.316228f, 0.948683f, 0.0f};
    sa3n[3] = {-0.980581f, 0.0f, 0.196116f};
    sa4n[3] = {0.235702f, -0.235702f, 0.942809f};
    sa5n[3] = {0.57735f, 0.57735f, 0.57735f};

    sb1n[3] = { 0.9082f, 0.3185f, 0.2715f };
    sb2n[3] = { 0.5670f, 0.3732f, -0.7343f };
    sb3n[3] = { -0.2821f, 0.7163f, 0.6382 };
    sb4n[3] = { 0.7510f, -0.3303f, 0.5718};
    sb5n[3] = { 0.9261f, -0.2053, -0.3166};
    */
    float *san[5] = {sa1n, sa2n, sa3n, sa4n, sa5n};
    float *sbn[5] = {sb1n, sb2n, sb3n, sb4n, sb5n};
    
    // To test without noise
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
    

    float om[5] = {0.0100f, 0.0325f, 0.0550f, 0.0775, 0.1000};

    float q[4];

    while(1){
    lastUpdate = t->read_us();

    /**************** LOOP ****************/

    Estimators::QUEST(q, 5, sbn, san, om, 1e-5);
    
    /************** LOOP END **************/

    ellapsed = t->read_us()-lastUpdate;
    pc->printf("\n\r\n\rLoop time %d us | Frequency %4f Hz\n\r", ellapsed, 1000000.0f/ellapsed);

    /*************** PRINTS ***************/
    pc->printf("Quaternion: %f | %f | %f | %f\n\r", q[0], q[1], q[2], q[3]);
    float rot[9];
    Matrix mat_rot(3,3);
    AstroLib::Orbit::quat2rot(q, rot);
    mat_rot = Matrix(3,3, rot);
    quat = Matrix(4,1, q);
    mat_rot = Matrix::quat2rot(quat);
    //mat_rot = mat_rot.Transpose();
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
    /*
    pc->printf("san[0][0]: %f\n\r", san[2][0]);
    pc->printf("san[0][1]: %f\n\r", san[2][1]);
    pc->printf("san[0][2]: %f\n\r", san[2][2]);
    */
    printMat(sb[0],pc);
    printMat(sb[1],pc);
    printMat(sb[2],pc);
    printMat(sb[3],pc);
    printMat(sb[4],pc);

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