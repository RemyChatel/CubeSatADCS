#include "Test.h"
#include "SimpleMatrix.h"

using namespace SimpleMatrix;

int SimpleMatrixTest(Serial *pc, I2C *i2c, Timer *t){
    pc->printf("\n\r\n\r\n\r\n\r\n\r\n\r");
    pc->printf("--------------------------------------\n\r\n\r");
    
    /* SimpleMatrix Vector Test */
    float coef1[3] = {3.8f,5.2f,8.4f};
    float coef2[3] = {9.1f,2.7f,0.2f};
    float coef[3] = {4.0f, 5.0f , 3.0f} ;
    Vector vec1(coef1),vec2(coef2);

    pc->printf("Vector 1\n\r");
    printVec(vec1, pc);
    pc->printf("Vector 2\n\r");
    printVec(vec2, pc);

    pc->printf("Assignement c = vec2 and testing modification of b\n\r");
    Vector vec3;
    vec3 = vec2;
    printVec(vec3, pc);
    vec2.setCoef(2, 0.0);
    pc->printf("b: ");
    printVec(vec2, pc);
    pc->printf("c: ");
    printVec(vec3, pc);
    vec2.setCoef(coef);

    pc->printf("Addition (vec1+vec2)\n\r");
    printVec(vec1+vec2, pc);
    pc->printf("Addition (vec2 += vec1)\n\r");
    vec2 += vec1;
    printVec(vec2, pc);
    vec2 = Vector(coef2); // reseting b to original

    pc->printf("Substraction (vec2-vec1) (expected [ -10.8, -4.96, 9 ]\n\r");
    printVec(vec2-vec1, pc);
    pc->printf("Substraction (vec2 -= vec1)\n\r");
    vec2 -= vec1;
    printVec(vec2, pc);
    vec2 = Vector(coef2); // reseting b to original

    pc->printf("Scalar multiplication (2*vec2) then (vec2*2) then b*=2\n\r");
    printVec(2.0f * vec2, pc);
    printVec(vec2 * 2.0f, pc);
    vec2 *= 2.0f;
    printVec(vec2, pc);
    vec2 = Vector(coef);

    pc->printf("Scalar division (b/2) then b/=2\n\r");
    printVec(vec2 / 2.0f, pc);
    vec2 /= 2.0f;
    printVec(vec2, pc);
    vec2 = Vector(coef);

    pc->printf("Norm of vec1 (expected 7.07107)\n\r");
    pc->printf("%f\n\r", vec1.norm());

    pc->printf("Normalize a (expected [ 0.565685, 0.707107, 0.424264 ]\n\r");
    Vector d = Vector(vec1);
    d.normalize();
    printVec(d, pc);

    pc->printf("Dot product transpose(vec1) * vec2 or vec1.vec2 (expected 9.0)\n\r");
    pc->printf("%f\n\r", vec2.dot(vec1));

    pc->printf("Cross product vec1 x vec2 [ 59.88, -68.4, 34.16 ]\n\r");
    printVec(vec1.cross(vec2), pc);

    /* SimpleMatrix Matrix Test */

    float coefA[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    float coefB[9] = {10, 12, 13, 14, 15, 16, 17, 18, 19};
    SimpleMatrix::Matrix a(coefA),b(coefB);

    pc->printf("Matrix A\n\r");
    printMat(a, pc);
    pc->printf("Matrix B\n\r");
    printMat(b, pc);
    
    pc->printf("Assignement C = B\n\r");
    SimpleMatrix::Matrix c;
    c = b;
    printMat(c, pc);
    b.setCoef(0, 0.0f);
    pc->printf("B :\n\r");
    printMat(b, pc);
    pc->printf("C :\n\r");
    printMat(c, pc);
    b.setCoef(coefB);

    pc->printf("Addition A+B\n\r");
    printMat(a+b, pc);

    pc->printf("Addition A+=B\n\r");
    a+=b;
    printMat(a, pc);
    a = SimpleMatrix::Matrix(coefA);

    pc->printf("Substraction A-B\n\r");
    printMat(a-b, pc);

    pc->printf("Substraction A-=B\n\r");
    a-=b;
    printMat(a, pc);
    a = SimpleMatrix::Matrix(coefA);

    pc->printf("Multiplication A*2\n\r");
    printMat(a*2, pc);

    pc->printf("Multiplication 2*A\n\r");
    printMat(2*a, pc);

    pc->printf("Multiplication A*=2\n\r");
    a*=2;
    printMat(a, pc);
    a = SimpleMatrix::Matrix(coefA);
    
    pc->printf("Multiplication A*B {{89, 96, 102}, {212, 231, 246}, {335, 366, 390}}\n\r");
    printMat(a*b, pc);

    pc->printf("Multiplication A*=B {{89, 96, 102}, {212, 231, 246}, {335, 366, 390}}\n\r");
    a*=b;
    printMat(a, pc);

    pc->printf("Multiplication A*vec1 {39.4, 91.6, 143.8}\n\r");
    printVec(a*vec1, pc);

    pc->printf("Multiplication vec1*A {83.4, 100.8, 118.2}\n\r");
    printVec(vec1*a, pc);

    pc->printf("Scalar division A / 2\n\r");
    printMat(a/2, pc);

    pc->printf("Scalar division A /= 2\n\r");
    a /= 2;
    printMat(a, pc);
    a = SimpleMatrix::Matrix(coefA);

    pc->printf("Determinant det(A) (expected 0)\n\r");
    pc->printf("%f\n\r", a.det());

    pc->printf("Determinant det(B) (expected 3)\n\r");
    pc->printf("%f\n\r", b.det());

    pc->printf("Trace tr(A) (15)\n\r");
    pc->printf("%f\n\r", a.tr());

    pc->printf("Trace tr(B) (44)\n\r");
    pc->printf("%f\n\r", b.tr());

    pc->printf("Transpose transpose(A)\n\r");
    printMat(a.transpose(), pc);

    pc->printf("Classical adjoint adj(A) {{-3, 6, -3}, {6, -12, 6}, {-3, 6, -3}}\n\r");
    printMat(a.adj(), pc);
    a = SimpleMatrix::Matrix(coefA);

    // pc->printf("Inverse of B matrix inv(B) {{-1, 2, -1}, {2, -10.33, 7.33}, {-1, 8, -6}} \n\r");
    // printMat(b.inv(), pc);

    return 1;
}