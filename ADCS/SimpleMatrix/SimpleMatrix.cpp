/***************************************************************************//**
 * @file SimpleMatrix.cpp
 * @version 1.0
 * @date 2019
 * @author Remy CHATEL
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2019 Remy CHATEL</b>
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
 
#include "SimpleMatrix.h"

using namespace SimpleMatrix;

Matrix::Matrix(){
    for(int i = 0; i < 9; i++){
        coef[i] = 0;
    }
}

Matrix::Matrix(float coeffs[9]){
    for(int i = 0; i < 9; i++){
        coef[i] = coeffs[i];
    }
}

Matrix::Matrix(Matrix *a){
    a->getCoef(coef);
}

Matrix::~Matrix(void){}

void Matrix::setCoef(int id, float a){
    coef[id] = a;
}

void Matrix::setCoef(int line, int col, float a){
    coef[3*line+col] = a;
}

void Matrix::setCoef(float *coeffs){
    for(int i = 0; i < 9; i++){
        coef[i] = coeffs[i];
    }
}

Matrix& Matrix::operator+=(const Matrix& b){
    for(int i = 0; i < 9; i++){
        coef[i] += b.coef[i];
    }
    return *this;
}

Matrix& Matrix::operator-=(const Matrix& b){
    for(int i = 0; i < 9; i++){
        coef[i] -= b.coef[i];
    }
    return *this;
}

Matrix& Matrix::operator*=(Matrix b){
    float temp_coef[9] = {0,0,0,0,0,0,0,0,0};
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            for(int k = 0; k < 3; k++){
                temp_coef[3*i+j] += coef[3*i+k] * b.coef[3*k+j];
            }
        }
    }
    for(int i = 0; i < 9; i++){
        coef[i] = temp_coef[i];
    }
    return *this;
}

Matrix& Matrix::operator*=(const float& b){
    for(int i = 0; i < 9; i++){
        coef[i] *= b;
    }
    return *this;
}

Vector Matrix::operator*(Vector b){
    Vector tmp;
    float vec_coef[3];
    b.getCoef(vec_coef);
    for(int i = 0; i < 3; i++){
        tmp.setCoef(i, coef[3*i] * vec_coef[0] + coef[3*i+1] * vec_coef[1] + coef[3*i+2] * vec_coef[2]);
    }
    return tmp;
}

Matrix Matrix::vec_mul(Vector lhs, Vector rhs){
    float coef[9];
    float veca[3];
    float vecb[3];
    lhs.getCoef(veca);
    rhs.getCoef(vecb);
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            coef[3*i+j] = veca[j]*vecb[i];
        }
    }
    Matrix tmp(coef);
    return tmp;
}

Matrix& Matrix::operator/=(const float& b){
    for(int i = 0; i < 9; i++){
        coef[i] /= b;
    }
    return *this;
}

Matrix Matrix::transpose(){
    Matrix tmp(coef);
    tmp.coef[1] = coef[3];
    tmp.coef[2] = coef[6];
    tmp.coef[3] = coef[1];
    tmp.coef[5] = coef[7];
    tmp.coef[6] = coef[2];
    tmp.coef[7] = coef[5];
    return tmp;
}

Matrix Matrix::adj(){
    Matrix tmp;

    tmp.coef[0] =  (coef[4]*coef[8] - coef[5]*coef[7]);
    tmp.coef[1] = -(coef[1]*coef[8] - coef[2]*coef[7]);
    tmp.coef[2] =  (coef[1]*coef[5] - coef[2]*coef[4]);
    tmp.coef[3] = -(coef[3]*coef[8] - coef[5]*coef[6]);
    tmp.coef[4] =  (coef[0]*coef[8] - coef[2]*coef[6]);
    tmp.coef[5] = -(coef[0]*coef[5] - coef[2]*coef[3]);
    tmp.coef[6] =  (coef[3]*coef[7] - coef[4]*coef[6]);
    tmp.coef[7] = -(coef[0]*coef[7] - coef[1]*coef[6]);
    tmp.coef[8] =  (coef[0]*coef[4] - coef[1]*coef[3]);

    return tmp;
}

float Matrix::tr(){
    float sum = 0;
    sum += coef[0];
    sum += coef[4];
    sum += coef[8];
    return sum;
}

float Matrix::det(){
    float sum = 0;
    sum += coef[0] * coef[4] * coef[8];
    sum += coef[1] * coef[5] * coef[6];
    sum += coef[2] * coef[3] * coef[7];
    sum -= coef[2] * coef[4] * coef[6];
    sum -= coef[0] * coef[5] * coef[7];
    sum -= coef[1] * coef[3] * coef[8];
    return sum;
}


Matrix Matrix::inv(){
    Matrix tmp;
    /*
    Matrix Id, thisSquare;
    float thisTrace = this->tr();
    Id.setCoef(0, 0, 1.0f);     // Creating the identity matrix
    Id.setCoef(1, 1, 1.0f);
    Id.setCoef(2, 2, 1.0f);
    thisSquare = *this * *this;
    tmp += thisSquare;
    tmp += *this * thisTrace;
    tmp += (.5f * ( thisTrace  * thisTrace  - thisSquare.tr()) * Id);
    tmp /= this->det();
    */
    return tmp;
}

void Matrix::getCoef(float coeffs[9]){
    for(int i = 0; i < 9; i++){
        coeffs[i] = coef[i];
    }
}

float Matrix::getCoef(int id){
    return coef[id];
}

float Matrix::getCoef(int line, int col){
    return coef[3 * line + col];
}

Vector::Vector(){
    coef[0] = 0;
    coef[1] = 0;
    coef[2] = 0;
}

Vector::Vector(float coeffs[3]){
    coef[0] = coeffs[0];
    coef[1] = coeffs[1];
    coef[2] = coeffs[2];
}

Vector::Vector(Vector *a){
    a->getCoef(coef);    
}

Vector::~Vector(void){}

float Vector::getCoef(int id){
    return coef[id];
}

void Vector::getCoef(float coeffs[3]){
    coeffs[0] = coef[0];
    coeffs[1] = coef[1];
    coeffs[2] = coef[2];
}

void Vector::setCoef(float *coeff){
    coef[0] = coeff[0];
    coef[1] = coeff[1];
    coef[2] = coeff[2];
}

void Vector::setCoef(int id, float coeff){
    coef[id] = coeff;
}

void Vector::operator+=(Vector b){
    coef[0] += b.getCoef(0);
    coef[1] += b.getCoef(1);
    coef[2] += b.getCoef(2);
}

void Vector::operator-=(Vector b){
    coef[0] -= b.getCoef(0);
    coef[1] -= b.getCoef(1);
    coef[2] -= b.getCoef(2);
}

void Vector::operator*=(float a){
    coef[0] *= a;
    coef[1] *= a;
    coef[2] *= a;
}

Vector& Vector::operator/=(const float& a){
    coef[0] /= a;
    coef[1] /= a;
    coef[2] /= a;
    return *this;
}

float Vector::dot(Vector b){
    float coeffs[3];
    b.getCoef(coeffs);
    return coef[0]*coeffs[0] + coef[1]*coeffs[1] + coef[2]*coeffs[2];
}

Vector Vector::cross(Vector b){
    Vector tmp;
    tmp.coef[0] = coef[1]*b.getCoef(2) - coef[2]*b.getCoef(1);
    tmp.coef[1] = coef[2]*b.getCoef(0) - coef[0]*b.getCoef(2);
    tmp.coef[2] = coef[0]*b.getCoef(1) - coef[1]*b.getCoef(0);
    return tmp;
}

float Vector::norm(){
    return sqrt(coef[0]*coef[0] + coef[1]*coef[1] + coef[2]*coef[2]);
}

void Vector::normalize(){
    float norm = this->norm();
    coef[0] /= norm;
    coef[1] /= norm;
    coef[2] /= norm;
}