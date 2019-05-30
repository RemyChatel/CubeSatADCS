#include "Test.h"
#include "Matrix.h"
#include "MatrixMath.h"

int MatrixTest(Serial *pc, I2C *i2c, Timer *t){
    pc->printf("\n\r\n\r\n\r\n\r\n\r\n\r");
    pc->printf("--------------------------------------\n\r\n\r");
    
    /* SimpleMatrix Vector Test */
    float coef1[3] = {3.8f,5.2f,8.4f};
    float coef2[3] = {9.1f,2.7f,0.2f};
    float coef[3] = {4.0f, 5.0f , 3.0f} ;
    Matrix vec1(3,1),vec2(3,1);
    vec1 << coef1[0]
         << coef1[1]
         << coef1[2];

    pc->printf("Vector 1\n\r");
    printMat(vec1, pc);
    pc->printf("Vector 2\n\r");
    printMat(vec2, pc);

    pc->printf("Assignement c = vec2 and testing modification of b\n\r");
    Matrix vec3(3,1);
    vec3 = vec2;
    printMat(vec3, pc);
    vec2(2,1) = 0.0f;
    pc->printf("b: ");
    printMat(vec2, pc);
    pc->printf("c: ");
    printMat(vec3, pc);
    vec2 << coef2[0]
         << coef2[1]
         << coef2[2];

    pc->printf("Addition (vec1+vec2)\n\r");
    printMat(vec1+vec2, pc);
    pc->printf("Addition (vec2 += vec1)\n\r");
    vec2 += vec1;
    printMat(vec2, pc);
    vec2 << coef2[0]
         << coef2[1]
         << coef2[2];

    pc->printf("Substraction (vec2-vec1) (expected [ -10.8, -4.96, 9 ]\n\r");
    printMat(vec2-vec1, pc);
    pc->printf("Substraction (vec2 -= vec1)\n\r");
    vec2 -= vec1;
    printMat(vec2, pc);
    vec2 << coef2[0]
         << coef2[1]
         << coef2[2];

    pc->printf("Scalar multiplication (2*vec2) then (vec2*2) then b*=2\n\r");
    printMat(2.0f * vec2, pc);
    printMat(vec2 * 2.0f, pc);
    vec2 *= 2.0f;
    printMat(vec2, pc);
    vec2 << coef2[0]
         << coef2[1]
         << coef2[2];

    // pc->printf("Norm of vec1 (expected 7.07107)\n\r");
    // pc->printf("%f\n\r", vec1.norm());

    // pc->printf("Normalize a (expected [ 0.565685, 0.707107, 0.424264 ]\n\r");
    // Matrix d = Matrix(vec1);
    // d.normalize();
    // printVec(d, pc);

    pc->printf("Dot product transpose(vec1) * vec2 or vec1.vec2 (expected 9.0)\n\r");
    pc->printf("%f\n\r", MatrixMath::dot(vec1, vec2));

    // pc->printf("Cross product vec1 x vec2 [ 59.88, -68.4, 34.16 ]\n\r");
    // printVec(vec1.cross(vec2), pc);

    /* SimpleMatrix Matrix Test */

    float coefA[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    float coefB[9] = {10, 12, 13, 14, 15, 16, 17, 18, 19};
    Matrix A, B;
    A << coefA[0] << coefA[1] << coefA[2]
      << coefA[3] << coefA[4] << coefA[5]
      << coefA[6] << coefA[7] << coefA[8];
    B << coefB[0] << coefB[1] << coefB[2]
      << coefB[3] << coefB[4] << coefB[5]
      << coefB[6] << coefB[7] << coefB[8];

    pc->printf("Matrix A\n\r");
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
    B << coefB[0] << coefB[1] << coefB[2]
      << coefB[3] << coefB[4] << coefB[5]
      << coefB[6] << coefB[7] << coefB[8];

    pc->printf("Addition A+B\n\r");
    printMat(A+B, pc);

    pc->printf("Addition A+=B\n\r");
    A+=B;
    printMat(A, pc);
    A << coefA[0] << coefA[1] << coefA[2]
      << coefA[3] << coefA[4] << coefA[5]
      << coefA[6] << coefA[7] << coefA[8];

    pc->printf("Substraction A-B\n\r");
    printMat(A-B, pc);

    pc->printf("Substraction A-=B\n\r");
    A-=B;
    printMat(A, pc);
    A << coefA[0] << coefA[1] << coefA[2]
      << coefA[3] << coefA[4] << coefA[5]
      << coefA[6] << coefA[7] << coefA[8];

    pc->printf("Multiplication A*2\n\r");
    printMat(A*2, pc);

    pc->printf("Multiplication 2*A\n\r");
    printMat(2*A, pc);

    pc->printf("Multiplication A*=2\n\r");
    A*=2;
    printMat(A, pc);
    A << coefA[0] << coefA[1] << coefA[2]
      << coefA[3] << coefA[4] << coefA[5]
      << coefA[6] << coefA[7] << coefA[8];
    
    pc->printf("Multiplication A*B {{89, 96, 102}, {212, 231, 246}, {335, 366, 390}}\n\r");
    printMat(A*B, pc);

    pc->printf("Multiplication A*=B {{89, 96, 102}, {212, 231, 246}, {335, 366, 390}}\n\r");
    A*=B;
    printMat(A, pc);

    pc->printf("Multiplication A*vec1 {39.4, 91.6, 143.8}\n\r");
    printMat(A*vec1, pc);

    pc->printf("Multiplication vec1*A {83.4, 100.8, 118.2}\n\r");
    printMat(vec1*A, pc);

    pc->printf("Determinant det(A) (expected 0)\n\r");
    pc->printf("%f\n\r", MatrixMath::det(A));

    pc->printf("Determinant det(B) (expected 3)\n\r");
    pc->printf("%f\n\r", MatrixMath::det(B));

    pc->printf("Trace tr(A) (15)\n\r");
    pc->printf("%f\n\r", MatrixMath::trace(A));

    pc->printf("Trace tr(B) (44)\n\r");
    pc->printf("%f\n\r", MatrixMath::trace(B));

    pc->printf("Transpose transpose(A)\n\r");
    printMat(MatrixMath::Transpose(B), pc);

    // pc->printf("Classical adjoint adj(A) {{-3, 6, -3}, {6, -12, 6}, {-3, 6, -3}}\n\r");
    // printMat(B.adj(), pc);
    // A << coefA[0] << coefA[1] << coefA[2]
    //   << coefA[3] << coefA[4] << coefA[5]
    //   << coefA[6] << coefA[7] << coefA[8];
    // pc->printf("Inverse of B matrix inv(B) {{-1, 2, -1}, {2, -10.33, 7.33}, {-1, 8, -6}} \n\r");
    // printMat(MatrixMath::Inv(B), pc);

    return 1;
}