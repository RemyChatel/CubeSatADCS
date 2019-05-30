#include "mbed.h"
#include "Tests/Test.h"

Serial pc(USBTX, USBRX, 115200);
I2C i2c(I2C_SDA, I2C_SCL);
Timer t;

int main(){
   return MatrixTest(&pc, &i2c, &t);
}