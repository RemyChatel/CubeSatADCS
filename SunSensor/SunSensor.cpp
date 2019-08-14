/**
 * @file   SunSensor.cpp
 * @version 1.0
 * @date 2019
 * @author Remy CHATEL
 * @copyright GNU Public License v3.0
 * @brief  Source code for the SunSensor Library
 * 
 * @details
 * # Description
 * This library provide an interface to read the analog output of
 * a set of three photodiodes that make a Sun sensor
 * 
 * @see SunSensor
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
 
#include "SunSensor.h"

SunSensor::SunSensor():faceX_(AnalogIn(A0)),faceY_(AnalogIn(A1)),faceZ_(AnalogIn(A2)), dummy_(AnalogIn(A5)){}

SunSensor::SunSensor(PinName pinX, PinName pinY, PinName pinZ): faceX_(AnalogIn(pinX)),
                                                                faceY_(AnalogIn(pinY)),
                                                                faceZ_(AnalogIn(pinZ)), dummy_(AnalogIn(A5)){}

SunSensor::~SunSensor(void){}

void SunSensor::getSunVector(float rsun[3]){
    float test = dummy_; // added to properly wake up the adc before using it
    rsun[0] = faceX_.read();
    rsun[1] = faceY_.read();
    rsun[2] = faceZ_.read();
}

float SunSensor::getXface(){
    return (faceX_).read();
}

float SunSensor::getYface(){
    return (faceY_).read();
}

float SunSensor::getZface(){
    return (faceZ_).read();
}