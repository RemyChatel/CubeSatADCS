/**
 * @file Matrix.test.cpp
 * @version 1.0
 * @date 2019
 * @author Remy CHATEL
 * @copyright GNU Public License v3.0
 * 
 * @brief
 * Source code for Matrix.test.h
 * 
 * @see Matrix.test.h
 * 
 * # License
 * <b>(C) Copyright 2019 Remy CHATEL</b>
 * 
 * Licensed Under  GPL v3.0 License
 * http://www.gnu.org/licenses/gpl-3.0.html
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "Matrix.test.h"

int MatrixTest(){
    
    printf("\n\r\n\r\n\r\n\r\n\r\n\r");
    printf("--------------------------------------\n\r\n\r");
    
    // SimpleMatrix Vector Test
    float coef1[3] = {3.8f,5.2f,8.4f};
    float coef2[3] = {9.1f,2.7f,0.2f};
    float  coef[3] = {4.0f,5.0f,3.0f};
    Matrix vec1(3,1, coef1);
    printf("Vector 1\n\r");
    vec1.print();
    
    Matrix vec2(3,1, coef2);
    printf("Vector 2\n\r");
    vec2.print();
    
    printf("Assignement c = vec2 and testing modification of b\n\r");
    Matrix vec3(3,1);
    vec3 = vec2;
    vec3.print();
    vec2(2) = 10.0f;
    printf("b: ");
    vec2.print();
    printf("c: ");
    vec3.print();
    
    vec2 = Matrix(3, 1, coef2);

    printf("\n\rAddition (vec1+vec2) (expected {12.9, 7.9, 8.6})\n\r");
    (vec1+vec2).print();
    printf("vec2 :\n\r");
    vec2.print();
    printf("Addition (vec2 += vec1)\n\r");
    vec2 += vec1;
    vec2.print();
    vec2 = Matrix(3, 1, coef2);
    vec2.print();

    printf("Substraction (vec2-vec1) (expected {5.3, -2.5, -8.2})\n\r");
    (vec2-vec1).print();
    printf("Substraction (vec2 -= vec1)\n\r");
    vec2 -= vec1;
    vec2.print();
    vec2 = Matrix(3, 1, coef2);

    printf("\n\rScalar multiplication (2*vec2) then (vec2*2) then b*=2\n\r");
    (2.0f * vec2).print();
    (vec2 * 2.0f).print();
    vec2 *= 2.0f;
    vec2.print();
    vec2 = Matrix(3, 1, coef2);

    printf("\n\rNorm of vec1 (expected 10.5849)\n\r");
    printf("%f\n\r", vec1.norm());

    // printf("Normalize a (expected {0.359002, 0.491266, 0.793584}\n\r");
    // Matrix d = Matrix(vec1);
    // d.normalize();
    // printVec(d;

    printf("Dot product transpose(vec1) * vec2 or vec1.vec2 (expected 50.3)\n\r");
    printf("%f\n\r", Matrix::dot(vec1, vec2));

    printf("Cross product vec1 x vec2 {-21.64, 75.68, -37.06}\n\r");
    Matrix::cross(vec1, vec2).print();

    printf("Vector to matrix product vec1 * transpose(vec2) (expected \n\r");
    printf("{{34.58, 10.26, 0.76}, {47.32, 14.04, 1.04}, {76.44, 22.68, 1.68}})\n\r");
    (vec1 * vec2.Transpose()).print();

    // SimpleMatrix Matrix Test

    float coefA[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    float coefB[9] = {10, 12, 13, 14, 15, 16, 17, 18, 19};
    Matrix A(3,3, coefA), B(3,3, coefB);

    printf("\n\r\n\rMatrix A\n\r");
    A.print();
    printf("Matrix B\n\r");
    B.print();
    
    printf("Assignement C = B\n\r");
    Matrix c;
    c = B;
    c.print();
    B(1,1) = 0.0f;
    printf("B :\n\r");
    B.print();
    printf("C :\n\r");
    c.print();
    B = Matrix(3,3, coefB);

    printf("\n\rAddition A+B\n\r");
    (A+B).print();

    printf("\n\rAddition A+=B\n\r");
    A+=B;
    A.print();
    A = Matrix(3,3, coefA);

    printf("Substraction A-B\n\r");
    (A-B).print();

    printf("Substraction A-=B\n\r");
    A-=B;
    A.print();
    A = Matrix(3,3, coefA);

    printf("\n\rMultiplication A*2\n\r");
    (A*2).print();

    printf("Multiplication 2*A\n\r");
    (2*A).print();

    printf("Multiplication A*=2\n\r");
    A*=2;
    A.print();
    A = Matrix(3,3, coefA);
    
    printf("Multiplication A*B {{89, 96, 102}, {212, 231, 246}, {335, 366, 390}}\n\r");
    (A*B).print();

    printf("Multiplication A*=B {{89, 96, 102}, {212, 231, 246}, {335, 366, 390}}\n\r");
    A*=B;
    A.print();

    A = Matrix(3,3, coefA);

    printf("Multiplication B * A {{149, 184, 219}, {186, 231, 276}, {222, 276, 330}}\n\r");
    (B*A).print();

    printf("Multiplication A*vec1 {39.4, 91.6, 143.8}\n\r");
    (A*vec1).print();

    printf("Multiplication vec1*A {83.4, 100.8, 118.2}\n\r");
    (vec1.Transpose()*A).print();
    
    printf("\n\rDeterminant det(A) (expected 0)\n\r");
    printf("%f\n\r", A.det());

    printf("Determinant det(B) (expected 3)\n\r");
    printf("%f\n\r", B.det());

    printf("Trace tr(A) (15)\n\r");
    printf("%f\n\r", A.trace());

    printf("Trace tr(B) (44)\n\r");
    printf("%f\n\r", B.trace());

    printf("Transpose transpose(B)\n\r");
    B.Transpose().print();

    printf("Inverse of B matrix inv(B) {{-1, 2, -1}, {2, -10.33, 7.33}, {-1, 8, -6}} \n\r");
    B.Inv().print();
    
    printf("Test of vector packing\n\r");
    Matrix::ToPackedVector(A).print();

    printf("Test of getCoef\n\r");
    float array[9];
    A.getCoef(array);
    printf("{");
    for(int i = 0; i < 9; i++){
        printf("%f, ", array[i]);
    }
    printf("}\n\r");

    printf("\n\r\n\rAssignation test\n\r");
    A.print();
    A(2) = 25;
    A(1,2) = 30;
    A.print();
    A = Matrix(3,3, coefA);

    printf("\n\r << test\n\r");
    A << 19 << 18 << 17
      << 16 << 15 << 14
      << 13 << 12 << 11;
    A.print();

    printf("<< test on vector\r\n");
    Matrix vec4 = Matrix::zeros(3,1);
    vec4.print();
    vec4 << 12 << 18 << 30;
    vec4.print();

    printf("\n\r\n\rKinematic methods test\n\r");
    float phi = 45*3.1415926535f/180.0f;        // Rotation around the new X axis
    float theta = -30*3.1415926535f/180.0f;     // Rotation around the new Y axis
    float psi = 60*3.1415926535f/180.0f;        // Rotation around teh original Z axis
    float eul_coef[3] = {phi, theta, psi};
    Matrix eul(3,1, eul_coef);

    printf("Rotation around X of 60°\n\r");
    Matrix::RotX(psi).print();
    printf("Rotation around Y of 60°\n\r");
    Matrix::RotY(psi).print();
    printf("Rotation around Z of 60°\n\r");
    Matrix::RotZ(psi).print();

    printf("\n\rEuler angles (in deg):\n\r");
    eul*(180.0f/3.1415926535f);

    printf("Euler to Quaternion\n\r");
    Matrix::euler2quat(eul).print();

    printf("Euler to 1-2-3 Rotation matrix\n\r");
    Matrix::euler2rot123(eul).print();

    printf("Euler to 3-2-1 Rotation matrix\n\r");
    Matrix::euler2rot(eul).print();

    printf("Quaternion to Euler\n\r");
    (Matrix::quat2euler(Matrix::euler2quat(eul))*(180.0f/3.1415926535f)).print();

    printf("Quaternion to Rotation matrix\n\r");
    Matrix::quat2rot(Matrix::euler2quat(eul)).Transpose().print();

    printf("Rotation matrix to Euler\n\r");
    (Matrix::rot2euler(Matrix::euler2rot(eul))*(180.0f/3.1415926535f)).print();

    printf("Rotation matrix to quaternion\n\r");
    Matrix::rot2quat(Matrix::euler2rot(eul)).print();

    printf("Rotation matrix 321\n\r");
    Matrix::Rot321(eul).print();

    printf("Diagonal matrix:\r\n");
    float d_coef[3] = {5,6,7};
    Matrix diag = Matrix::diag(3, d_coef);
    diag.print();
    Matrix test = diag * A;
    test.print();
    
    printf("\n\r\n\rQuaternion\n\r");
    Matrix q1(4,1);
    q1 << +0.079324 << +0.560843 << -0.290980 << +0.753516;
    Matrix q2(4,1);
    q2 << -0.302377 << -0.460457 << +0.295403 << -0.799420;
    printf("q1: \n\r");
    q1.print();
    printf("q2: \n\r");
    q2.print();
    printf("q1 x q2 (expected {0.92259,-0.19609,0.21281,-0.25957}) \n\r");
    Matrix::quatmul(q1, q2).print();
    printf("norm of q1 %f\n\r", q1.norm());

    Timer t2;
    t2.start();
    int time;
    float p_coef[49] = { 0.012005,  0.000338,  0.000539,  0.000908,  0.000792, -0.000029, -0.000384,
                         0.000338,  0.011768,  0.000031, -0.000350,  0.000830, -0.000451, -0.000155,
                         0.000539,  0.000031,  0.013249,  0.002569, -0.000004, -0.000221,  0.001432,
                         0.000908, -0.000350,  0.002569,  0.013035, -0.000104, -0.000511,  0.000451,
                         0.000792,  0.000830, -0.000004, -0.000104,  0.252785, -0.000750,  0.000257,
                        -0.000029, -0.000451, -0.000221, -0.000511, -0.000750,  0.252275, -0.001431,
                        -0.000384, -0.000155,  0.001432,  0.000451,  0.000257, -0.001431,  0.256326};
    float p_th[49] = {  83.910756, -2.546317, -2.366192, -5.454592, -0.256994, -0.007956,  0.147195,
                        -2.546317, 85.153444, -0.599366,  2.584252, -0.270148,  0.156115,  0.047621,
                        -2.366192, -0.599366, 78.592075, -15.32480,  0.004844,  0.034119, -0.415823,
                        -5.454592,  2.584252,-15.324805,  80.19461,  0.041871,  0.152784, -0.061284,
                        -0.256994, -0.270148,  0.004844,  0.041871,  3.957678,  0.011316, -0.004554,
                        -0.007956,  0.156115,  0.034119,  0.152784,  0.011316,  3.964702,  0.021745,
                         0.147195,  0.047621, -0.415823, -0.061284, -0.004554,  0.021745,  3.904088};

    Matrix P(7,7, p_coef);
    Matrix Pth(7,7,p_th);

    printf("\r\nFull Inverse of B matrix inv(B) \n\r");
    time = t2.read_us();
    Matrix P2 = P.Inv();
    time = t2.read_us() - time;
    P2.print();
    printf("Execution time: %f ms\n\r", (float)time/1000);

    printf("\r\nApproximate Inverse of B matrix inv(B) \n\r");
    time = t2.read_us();
    Matrix Pinv = P.TaylorInv(4);
    time = t2.read_us() - time;
    Pinv.print();
    printf("Execution time: %f ms\n\r", (float)time/1000);

    printf("Relative error matrix for TaylorInv (in percent)\n\r");
    Pinv -= Pth;
    for(int i=1; i<=7; i++){
        for(int j=1; j<=7; j++){
            Pinv(i,j) = Pinv(i,j) / Pth(i,j);
        }
    }
    (Pinv*100).print();

    printf("Relative error matrix for Inv (in percent)\n\r");
    P2 -= Pth;
    for(int i=1; i<=7; i++){
        for(int j=1; j<=7; j++){
            P2(i,j) = P2(i,j) / Pth(i,j);
        }
    }
    (P2*100).print();
    
    return 1;
}