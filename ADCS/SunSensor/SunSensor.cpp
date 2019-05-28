/***************************************************************************//**
 * @file SunSensor.cpp
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
 
#include "SunSensor.h"

SunSensor::SunSensor(){
    
}

SunSensor::SunSensor(AnalogIn* analogX, AnalogIn* analogY, AnalogIn* analogZ){
    faceX_ = analogX;
    faceY_ = analogY;
    faceZ_ = analogZ;
}

SunSensor::~SunSensor(void){
    
}

void SunSensor::getSunVector(float rsun[3]){
    rsun[0] = (*faceX_);
    rsun[1] = (*faceY_);
    rsun[2] = (*faceZ_);
}

float SunSensor::getXface(){
    return (*faceX_).read();
}

float SunSensor::getYface(){
    return (*faceY_).read();
}

float SunSensor::getZface(){
    return (*faceZ_).read();
}