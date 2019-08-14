/**
 * @file Matrix.test.h
 * @version 1.0
 * @date 2019
 * @author Remy CHATEL
 * @copyright GNU Public License v3.0
 * 
 * @brief
 * Test framework library for the Matrix module
 * 
 * @details
 * # Description
 * This library provides a test function for the Matrix module and also
 * serves as an example program for the module.
 * 
 * 
 * @see Matrix.h
 * 
 * # Dependencies and data type
 * This library depends on:
 * - the Mbed framework (https://www.mbed.com/en/)
 * - <std::cmath> in order to perform cos, sin and sqrt operations.
 * - <std::vector> for the matrix algebra
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
#ifndef MATRIX_TEST_H
#define MATRIX_TEST_H
#include "mbed.h"
#include "Matrix.h"

/**
 * @brief
 * Test of the Matrix library
 * return 1 if successful, 0 otherwise
 */
int MatrixTest();
#endif