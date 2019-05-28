/***************************************************************************//**
 * @file SunSensor.h
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
 
 #ifndef SUNSENSOR_H
 #define SUNSENSOR_H
 #include "mbed.h"
 
 /** SunSensor Class
 * Read 3 analog photodiodes an provide a Sun Vector
 * Developed for MBed5+
 * 
 * Example:
 * @code
 * 
 * @endcode 
 */

class SunSensor{
public:
    /* Public functions */
    
    SunSensor();
    
    /**
     * SunSensor Constructor
     * @param analogX The pin of the X face sensor
     * @param analogY The pin of the Y face sensor 
     * @param analogZ The pin of the Z face sensor
     */
    SunSensor(AnalogIn* analogX, AnalogIn* analogY, AnalogIn* analogZ);
    
    /**
     * SunSensor Destructor
     */
    ~SunSensor(void);
    
    /**
     * Provides the sun vector in the body frame
     * @param rsun A table to hold the sun vector
     */
    void getSunVector(float rsun[3]);
    
    /**
     * Provides the X face light reading (normalised to 1.0)
     */
    float getXface();
    
    /**
     * Provides the Y face light reading (normalised to 1.0)
     */
    float getYface();
    
    /**
     * Provides the Z face light reading (normalised to 1.0)
     */
    float getZface();
    
private:
    /* Private variables */
    AnalogIn* faceX_;
    AnalogIn* faceY_;
    AnalogIn* faceZ_;
};
#endif