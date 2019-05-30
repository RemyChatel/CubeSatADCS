#ifndef TEST_H
#define TEST_H
#include "mbed.h"
#include "MatrixMath.h"
#include "SimpleMatrix.h"

int QuestTest(Serial *pc, I2C *i2c, Timer *t);
int MatrixTest(Serial *pc, I2C *i2c, Timer *t);
int SimpleMatrixTest(Serial *pc, I2C *i2c, Timer *t);
int MPU9250Test(Serial *pc, I2C *i2c, Timer *t);
int AstroLibTest(Serial *pc, I2C *i2c, Timer *t);

void printVec(SimpleMatrix::Vector a, Serial *pc);
void printMat(SimpleMatrix::Matrix a, Serial *pc);
void printMat(Matrix a, Serial *pc);

#endif // TEST_H