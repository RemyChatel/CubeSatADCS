#include "Test.h"

void printMat(Matrix a, Serial *pc){
    int col = a.getCols();
    int row = a.getRows();

    printf("{{");

    for(int i = 0; i < row; i++){
        if(i != 0){
            printf(" {");
        }
        for(int j = 0; j < col; j++){
            printf("% 7f", a.getNumber(i, j));
            if(j!=col-1){
                printf(", ");
            }
        }
        if(i==row-1){
                printf("}}\r\n");
            }
        else{
            printf("},\r\n");
        }
    }
}