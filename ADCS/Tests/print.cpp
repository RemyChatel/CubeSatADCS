#include "Test.h"

void printVec(SimpleMatrix::Vector a, Serial *pc){
    float pcoef[3];
    a.getCoef(pcoef);
    pc->printf("[ %f, %f, %f ]\n\r", pcoef[0], pcoef[1], pcoef[2]);
}

void printMat(SimpleMatrix::Matrix a, Serial *pc){
    float pcoef[9];
    a.getCoef(pcoef);
    pc->printf("[[ %f, %f, %f ]\n\r" , pcoef[0], pcoef[1], pcoef[2]);
    pc->printf(" [ %f, %f, %f ]\n\r" , pcoef[3], pcoef[4], pcoef[5]);
    pc->printf(" [ %f, %f, %f ]]\n\r", pcoef[6], pcoef[7], pcoef[8]);
}

void printMat(Matrix a, Serial *pc){
    int col = a.getCols();
    int row = a.getRows();

    pc->printf("[[");

    for(int i = 0; i < row; i++){
        for(int j = 0; j < col; j++){
            pc->printf("%f", a.getNumber(i, j));
            if(j!=col-1){
                pc->printf(", ");
            }
        }
        if(i==row-1){
                pc->printf("]]\n\r");
            }
        else{
            pc->printf("],\n\r");
        }
    }
}