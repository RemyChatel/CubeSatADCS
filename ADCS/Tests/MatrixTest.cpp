#include "Test.h"
#include "Matrix.h"

int MatrixTest(Serial *pc, I2C *i2c, Timer *t){
    
    pc->printf("\n\r\n\r\n\r\n\r\n\r\n\r");
    pc->printf("--------------------------------------\n\r\n\r");
    
    // SimpleMatrix Vector Test
    float coef1[3] = {3.8f,5.2f,8.4f};
    float coef2[3] = {9.1f,2.7f,0.2f};
    float  coef[3] = {4.0f,5.0f,3.0f};
    Matrix vec1(3,1, coef1);
    pc->printf("Vector 1\n\r");
    printMat(vec1, pc);
    
    Matrix vec2(3,1, coef2);
    pc->printf("Vector 2\n\r");
    printMat(vec2, pc);
    
    pc->printf("Assignement c = vec2 and testing modification of b\n\r");
    Matrix vec3(3,1);
    vec3 = vec2;
    printMat(vec3, pc);
    vec2(1,0) = 10.0f;
    pc->printf("b: ");
    printMat(vec2, pc);
    pc->printf("c: ");
    printMat(vec3, pc);
    
    vec2 = Matrix(3, 1, coef2);

    pc->printf("\n\rAddition (vec1+vec2) (expected {12.9, 7.9, 8.6})\n\r");
    printMat(vec1+vec2, pc);
    pc->printf("vec2 :\n\r");
    printMat(vec2, pc);
    pc->printf("Addition (vec2 += vec1)\n\r");
    vec2 += vec1;
    printMat(vec2, pc);
    vec2 = Matrix(3, 1, coef2);
    printMat(vec2, pc);

    pc->printf("Substraction (vec2-vec1) (expected {5.3, -2.5, -8.2})\n\r");
    printMat(vec2-vec1, pc);
    pc->printf("Substraction (vec2 -= vec1)\n\r");
    vec2 -= vec1;
    printMat(vec2, pc);
    vec2 = Matrix(3, 1, coef2);

    pc->printf("\n\rScalar multiplication (2*vec2) then (vec2*2) then b*=2\n\r");
    printMat(2.0f * vec2, pc);
    printMat(vec2 * 2.0f, pc);
    vec2 *= 2.0f;
    printMat(vec2, pc);
    vec2 = Matrix(3, 1, coef2);

    pc->printf("\n\rNorm of vec1 (expected 10.5849)\n\r");
    pc->printf("%f\n\r", vec1.norm());

    // pc->printf("Normalize a (expected {0.359002, 0.491266, 0.793584}\n\r");
    // Matrix d = Matrix(vec1);
    // d.normalize();
    // printVec(d, pc);

    pc->printf("Dot product transpose(vec1) * vec2 or vec1.vec2 (expected 50.3)\n\r");
    pc->printf("%f\n\r", Matrix::dot(vec1, vec2));

    pc->printf("Cross product vec1 x vec2 {-21.64, 75.68, -37.06}\n\r");
    printMat(Matrix::cross(vec1, vec2), pc);

    pc->printf("Vector to matrix product vec1 * transpose(vec2) (expected \n\r");
    pc->printf("{{34.58, 10.26, 0.76}, {47.32, 14.04, 1.04}, {76.44, 22.68, 1.68}})\n\r");
    printMat(vec1 * vec2.Transpose(), pc);

    // SimpleMatrix Matrix Test

    float coefA[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    float coefB[9] = {10, 12, 13, 14, 15, 16, 17, 18, 19};
    Matrix A(3,3, coefA), B(3,3, coefB);

    pc->printf("\n\r\n\rMatrix A\n\r");
    printMat(A, pc);
    pc->printf("Matrix B\n\r");
    printMat(B, pc);
    
    pc->printf("Assignement C = B\n\r");
    Matrix c;
    c = B;
    printMat(c, pc);
    B(0,0) = 0.0f;
    pc->printf("B :\n\r");
    printMat(B, pc);
    pc->printf("C :\n\r");
    printMat(c, pc);
    B = Matrix(3,3, coefB);

    pc->printf("\n\rAddition A+B\n\r");
    printMat(A+B, pc);

    pc->printf("\n\rAddition A+=B\n\r");
    A+=B;
    printMat(A, pc);
    A = Matrix(3,3, coefA);

    pc->printf("Substraction A-B\n\r");
    printMat(A-B, pc);

    pc->printf("Substraction A-=B\n\r");
    A-=B;
    printMat(A, pc);
    A = Matrix(3,3, coefA);

    pc->printf("\n\rMultiplication A*2\n\r");
    printMat(A*2, pc);

    pc->printf("Multiplication 2*A\n\r");
    printMat(2*A, pc);

    pc->printf("Multiplication A*=2\n\r");
    A*=2;
    printMat(A, pc);
    A = Matrix(3,3, coefA);
    
    pc->printf("Multiplication A*B {{89, 96, 102}, {212, 231, 246}, {335, 366, 390}}\n\r");
    printMat(A*B, pc);

    pc->printf("Multiplication A*=B {{89, 96, 102}, {212, 231, 246}, {335, 366, 390}}\n\r");
    A*=B;
    printMat(A, pc);

    A = Matrix(3,3, coefA);

    pc->printf("Multiplication B * A {{149, 184, 219}, {186, 231, 276}, {222, 276, 330}}\n\r");
    printMat(B*A, pc);

    pc->printf("Multiplication A*vec1 {39.4, 91.6, 143.8}\n\r");
    printMat(A*vec1, pc);

    pc->printf("Multiplication vec1*A {83.4, 100.8, 118.2}\n\r");
    printMat(vec1.Transpose()*A, pc);
    
    pc->printf("\n\rDeterminant det(A) (expected 0)\n\r");
    pc->printf("%f\n\r", A.det());

    pc->printf("Determinant det(B) (expected 3)\n\r");
    pc->printf("%f\n\r", B.det());

    pc->printf("Trace tr(A) (15)\n\r");
    pc->printf("%f\n\r", A.trace());

    pc->printf("Trace tr(B) (44)\n\r");
    pc->printf("%f\n\r", B.trace());

    pc->printf("Transpose transpose(B)\n\r");
    printMat(B.Transpose(), pc);

    pc->printf("Inverse of B matrix inv(B) {{-1, 2, -1}, {2, -10.33, 7.33}, {-1, 8, -6}} \n\r");
    printMat(B.Inv(), pc);

    return 1;
}