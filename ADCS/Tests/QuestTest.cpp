#include "Test.h"
#include "Estimators.h"
#include "AstroLib.h"

int QuestTest(Serial *pc, I2C *i2c, Timer *t){
    
    int lastUpdate = 0;
    int ellapsed = 0;
    int seconds = 0;;
    int minutes = 0;
    t->start();
    pc->baud(115200);
    pc->printf("\n\r\n\r------------------------------\n\r");
    pc->printf("Connection OK\n\r");

    float sa1[3] = {0.0f, 0.447214f, 0.894427f};
    float sa2[3] = {0.316228f, 0.948683f, 0.0f};
    float sa3[3] = {-0.980581f, 0.0f, 0.196116f};
    float sa4[3] = {0.235702f, -0.235702f, 0.942809f};
    float sa5[3] = {0.57735f, 0.57735f, 0.57735f};
    float *sa[5] = {sa1, sa2, sa3, sa4, sa5};

    float sb1[3] = { 0.9082f, 0.3185f, 0.2715f };
    float sb2[3] = { 0.5670f, 0.3732f, -0.7343f };
    float sb3[3] = { -0.2821f, 0.7163f, 0.6382 };
    float sb4[3] = { 0.7510f, -0.3303f, 0.5718};
    float sb5[3] = { 0.9261f, -0.2053, -0.3166};
    float *sb[5] = {sb1, sb2, sb3, sb4, sb5};

    float om[5] = {0.0100f, 0.0325f, 0.0550f, 0.0775, 0.1000};

    float q[4];

    float coef_th[9] = {0.4153, 0.4472, 0.7921, -0.7652, 0.6537, 0.0274, -0.5056, -0.6104, 0.6097};
    SimpleMatrix::Matrix mat_th(coef_th);
    SimpleMatrix::Matrix mat_rot;

    while(1){
    lastUpdate = t->read_us();
    /************* LOOP **************/

    Estimators::QUEST(q, 5, sb, sa, om, 1e-5);
    
    /************* END ***************/
    ellapsed = t->read_us()-lastUpdate;
    pc->printf("Loop time %d | Frequency %4f\n\r", ellapsed, 1.0f/ellapsed);

    /************* PRINTS **************/
    pc->printf("Quaternion: %f | %f | %f | %f\n\r", q[0], q[1], q[2], q[3]);
    float rot[9];
    AstroLib::Orbit::quat2rot(q, rot);
    mat_rot.setCoef(rot);
    mat_rot = mat_rot.transpose();
    pc->printf("Computed Rotation matrix\n\r");
    printMat(mat_rot, pc);
    pc->printf("Expected Rotation matrix\n\r");
    printMat(mat_th, pc);

    SimpleMatrix::Matrix mat_error = mat_rot * mat_th.transpose();
    float cos_error = 0.5 * (mat_error.tr() - 1);
    float error = acos(cos_error);

    pc->printf("Error matrix\n\r");
    printMat(mat_error, pc);
    pc->printf("Cos Angular error %f deg\n\r", cos_error);
    pc->printf("Angular error %f deg\n\r", error);
    pc->printf("Difference matrix in percents\n\r");
    printMat((mat_th - mat_rot)*100, pc);


    /************* END **************/

    seconds+=5;
    if(seconds==60){
        seconds = 0;
        minutes++;
    }
    wait(5);
    }

    return 1;
}