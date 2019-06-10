/**
 * @file Test.h
 * @version 1.0
 * @date 2019
 * @author Remy CHATEL
 * @copyright GNU Public License v3.0
 * @brief Provides a test framework for the ADCS project
 * 
 * @details
 * # Description
 * This file provide a framework to test different part of the program separatly
 * 
 * # Dependencies
 * This framework requires Mbed
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
#ifndef TEST_H
#define TEST_H
#include "mbed.h"
#include "Matrix.h"

/**
 * @brief
 * Test of the Estimator Library
 * @param pc A reference to the Serial bus
 * @param i2c A reference to the I2C bus
 * @param t A refence to a Timer instance
 * return 1 if successful, 0 otherwise
 */
int QuestTest(Serial *pc, I2C *i2c, Timer *t);

/**
 * @brief
 * Test of the Matrix library
 * @param pc A reference to the Serial bus
 * @param i2c A reference to the I2C bus
 * @param t A refence to a Timer instance
 * return 1 if successful, 0 otherwise
 */
int MatrixTest(Serial *pc, I2C *i2c, Timer *t);

/**
 * @brief
 * Test of the MPU9150 driver library
 * @param pc A reference to the Serial bus
 * @param i2c A reference to the I2C bus
 * @param t A refence to a Timer instance
 * return 1 if successful, 0 otherwise
 */
int MPU9250Test(Serial *pc, I2C *i2c, Timer *t);

/**
 * @brief
 * Test of the AstroLib library
 * @param pc A reference to the Serial bus
 * @param i2c A reference to the I2C bus
 * @param t A refence to a Timer instance
 * return 1 if successful, 0 otherwise
 */
int AstroLibTest(Serial *pc, I2C *i2c, Timer *t);

/**
 * @brief
 * Test of the Sun Sensor and other sensors
 * @param pc A reference to the Serial bus
 * @param i2c A reference to the I2C bus
 * @param t A refence to a Timer instance
 * return 1 if successful, 0 otherwise
 */
int SensorTest(Serial *pc, I2C *i2c, Timer *t);

/**
 * @brief
 * Test of the Filters Library
 * @param pc A reference to the Serial bus
 * @param i2c A reference to the I2C bus
 * @param t A refence to a Timer instance
 * return 1 if successful, 0 otherwise
 */
int FilterTest(Serial *pc, I2C *i2c, Timer *t);

/**
 * @brief
 * Print a Matrix in a formated way
 * @param a The matrix to print
 * @param pc A reference to the Serial bus
 */
void printMat(Matrix a, Serial *pc);

#endif // TEST_H