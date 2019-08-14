/**
 * @file SunSensor.test.h
 * @version 1.0
 * @date 2019
 * @author Remy CHATEL
 * @copyright GNU Public License v3.0
 * 
 * @brief
 * Test framework library for the SunSensor module
 * 
 * @details
 * # Description
 * This library provides a test function for the SunSensor module and also
 * serves as an example program for the module.
 * 
 * To use it, simply connect three photodiodes to the Analog pins of the 
 * board.
 * 
 * @see SunSensor.h
 * 
 * # Dependencies and data type
 * This library depends on:
 * - the Mbed framework (https://www.mbed.com/en/)
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
#ifndef SUNSENSOR_TEST_H
#define SUNSENSOR_TEST_H
#include "mbed.h"
#include "SunSensor.h"
#include "Matrix.h"

/**
 * @brief
 * Test of the Sun Sensor and other sensors
 * return 1 if successful, 0 otherwise
 */
int SunSensorTest();
#endif