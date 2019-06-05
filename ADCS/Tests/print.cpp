#include "Test.h"

void printMat(Matrix a, Serial *pc){
    int col = a.getCols();
    int row = a.getRows();

    pc->printf("{{");

    for(int i = 0; i < row; i++){
        if(i != 0){
            pc->printf(" {");
        }
        for(int j = 0; j < col; j++){
            pc->printf("% 7f", a(i, j));
            if(j!=col-1){
                pc->printf(", ");
            }
        }
        if(i==row-1){
                pc->printf("}}\n\r");
            }
        else{
            pc->printf("},\n\r");
        }
    }
}