/**
 * @file ADSCore.test.h
 * @version 1.0
 * @date 2019
 * @author Remy CHATEL
 * @copyright GNU Public License v3.0
 * 
 * @brief
 * Test framework library for the ADScore module
 * 
 * @details
 * # Description
 * This library provides a test function for the ADSCore module and also
 * serves as an example program for the module.
 * 
 * @see ADSCore.h
 * 
 * # Dependencies and data type
 * This library depends on:
 * - the Mbed framework (https://www.mbed.com/en/)
 * - <std::cmath> in order to perform cos, sin and sqrt operations.
 * - <std::vector> for the matrix algebra
 * - "Matrix.h" for matrices implementation
 * - "Estimators.h" for the QuEst algorithm
 * - "Filters.h" for the Kalman Filter
 * - "AstroLib.h" for the reference vector models
 * - "SunSensor.h" to access the Sun sensor
 * - "MPU9150.h" to access the IMU
 * 
 * @attention This library uses float (32-bits) and not double (64-bits)
 * to make best use of the Floating Point Unit of 32-bits microcontrollers.
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
#ifndef ADSCORE_TEST_H
#define ADSCORE_TEST_H
#include "mbed.h"
#include "ADSCore.h"

/**
 * @brief
 * Testing function for the ADSCore module
 * 
 * This method test the ADSCore module by fetching data from an IMU
 * (through I2C) and a Sun sensor (3 analog channel), then it generates
 * reference vector from models and estimate the attitude using the
 * Quaternion Estimator algorithm. Finally, the output is filtered through
 * a Kalman Filter to improve accuracy.
 * 
 * Runs in an infinite loop an print the results on the Serial port.
 * 
 * @return 1 if successful, 0 otherwise
 */
int ADSTest();
#endif