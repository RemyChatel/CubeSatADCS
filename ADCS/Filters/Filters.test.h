/**
 * @file Filters.test.h
 * @version 1.0
 * @date 2019
 * @author Remy CHATEL
 * @copyright GNU Public License v3.0
 * 
 * @brief
 * Test framework library for the Filters module
 * 
 * @details
 * # Description
 * This library provides a test function for the Filters module and also
 * serves as an example program for the module.
 * 
 * To generate the test vectors, use the Python script provided:
 * - Execute the testèvector_generator.py to create the vectors
 * - Remove the unwanted commas at the begining and end of the generated files
 * - Compile and upload the program in the board
 * - Set the correct COM port in test_serial_output.py
 * - Hold the reset button on the board
 * - Execute FilterTestSerial.py then release the reset button
 * 
 * @see Filters.h
 * 
 * # Dependencies and data type
 * This library depends on:
 * - the Mbed framework (https://www.mbed.com/en/)
 * - <std::cmath> in order to perform cos, sin and sqrt operations.
 * - <std::vector> for the matrix algebra
 * - "Matrix.h" for matrices implementation
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
#ifndef FILTERS_TEST_H
#define FILTERS_TEST_H
#include "mbed.h"
#include "Filters.h"

/**
 * @brief
 * Test of the Filters module
 * 
 * Tests the Filters module by importing attitude data and angular rates generated
 * by a Python script, and processing them. The output is sent back for comparison.
 * 
 * return 1 if successful, 0 otherwise
 */
int KalmanFilterTest();
#endif