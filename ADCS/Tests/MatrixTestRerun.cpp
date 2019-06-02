#include "Test.h"
#include <vector>

int MatrixTestRerun(Serial *pc, I2C *i2c, Timer *t){
    pc->printf("\n\r\n\r\n\r\n\r\n\r\n\r");
    pc->printf("-------------------------------------\n\r-");
    pc->printf("\n\r\n\rTest of std::vector library\n\r");

    int size = 3;
    std::vector < std::vector < float > > vec;
    vec.resize(3);
    
    for(int i = 0; i < size; i++){
        vec[i].resize(size);
    }

    for(int i = 0; i < size; i++){
        for(int j = 0; j < size; j++){
            vec[i][j] = 3*i+j;
        }
    }
    pc->printf("{");
    for(int i = 0; i < size; i++){
        if(i!=0){pc->printf(" ");}
        pc->printf("{");
        for(int j = 0; j < size; j++){
            pc->printf("%f, ", vec[i][j]);
        }
        pc->printf("}");
        if(i == size){pc->printf("}");}
        else{pc->printf(",\n\r");}
    }

    while(1);
}
