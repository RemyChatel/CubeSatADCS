/***************************************************************************//**
 * @file quest.cpp
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2019 </b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/
 
#include "Estimators.h"

#include "mbed.h"
Serial pc2(USBTX, USBRX, 115200);
void printMat2(Matrix a);

void Estimators::QUEST(float quat[4], int N, float **s_eci, float **s_body, float *omega, float tolerance = 1e-5){
    /* Variable to store the solution */
    float lambda;       // The estimated eigen value of the problem
    Matrix x;           // Quaternion's vector (of the eigen vector)
    float gamma;        // Quaternion's rotation (of the eigen vector)
    
    /* Internal computation variables  */
    float lambda0 = 0;  // A reasonable initial value for lambda
    Matrix s_a[N];      // ECI frame vector array
    Matrix s_b[N];      // Body frame vector array
    Matrix k12;
    Matrix B, S, Id;
    float k22;
    float a, b, c, d;
    float alpha, beta;
    float detS;
    float trAdjS;
    float normQ;

    /* Preparation of the Newton optimisation problem to find lambda */
    for(int i = 0; i < N; i++){
        lambda0 += omega[i];
    }

    for(int i = 0; i < N; i++){
        s_a[i] = Matrix(3,1, s_eci[i]);
        s_b[i] = Matrix(3,1, s_body[i]);
    }

    for(int i = 0; i < N; i++){
        B += omega[i] * ( s_a[i] * s_b[i].Transpose() );
    }

    B = B.Transpose();

    pc2.printf("\n\nB matrix\n\r");
    printMat2(B);

    S = B + B.Transpose();
    detS = S.det();

    k22 = B.trace();

    k12(0,0) = B(1,2) - B(2,1);
    k12(1,0) = B(2,0) - B(0,2);
    k12(2,0) = B(0,1) - B(1,0);

    trAdjS  = S(1,1)*S(2,2) - S(2,1)*S(1,2) 
            + S(0,0)*S(2,2) - S(0,2)*S(2,0) 
            + S(0,0)*S(1,1) - S(0,1)*S(1,0);

    a = k22 * k22 - trAdjS;
    b = k22 * k22 + Matrix::dot(k12, k12);
    c = detS + Matrix::dot(k12, S * k12);
    d = Matrix::dot(k12, S * S * k12);

    /* Newton's optimization method to find lambda */
    int iteration = 0;
    lambda = lambda0;
    float lambda_prev = 0;
    while(fabs(lambda-lambda_prev) > tolerance && iteration < 10000){
        // pc2.printf("In loop\n\r");
        lambda_prev = lambda;
        lambda -= (lambda*lambda*lambda*lambda - (a + b) * lambda * lambda - c *lambda + (a * b + c * k22 - d)) / (4 * lambda * lambda * lambda - 2 * (a + b) * lambda - c);
        iteration++;
    }

    /* Then find the eigen vector associated with the eigen value lambda */
    Id = Matrix::eye(3);

    alpha = lambda * lambda - a;
    beta = lambda - k22;
    gamma = (lambda + k22) * alpha - detS; 
    x = (alpha * Id + beta * S + S * S) * k12;

    normQ = sqrt(gamma * gamma + Matrix::dot(x,x));
    x *= 1/normQ;
    gamma /= normQ;

    /* Returning the quaternion */
    quat[0] = x(0,0);
    quat[1] = x(1,0);
    quat[2] = x(2,0);
    quat[3] = gamma;
}

void printMat2(Matrix a){
    int col = a.getCols();
    int row = a.getRows();

    pc2.printf("{{");

    for(int i = 0; i < row; i++){
        if(i != 0){
            pc2.printf(" {");
        }
        for(int j = 0; j < col; j++){
            pc2.printf("%f", a(i, j));
            if(j!=col-1){
                pc2.printf(", ");
            }
        }
        if(i==row-1){
                pc2.printf("}}\n\r");
            }
        else{
            pc2.printf("},\n\r");
        }
    }
}