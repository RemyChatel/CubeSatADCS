/**
 * @file AstroLib.test.h
 * @version 1.0
 * @date 2019
 * @author Remy CHATEL
 * @copyright GNU Public License v3.0
 * 
 * @brief
 * Test framework library for the AstroLib module
 * 
 * @details
 * # Description
 * This library provides a test function for the AstroLib module and also
 * serves as an example program for the module.
 * 
 * @see AstroLib.h
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
#ifndef ASTROLIB_TEST_H
#define ASTROLIB_TEST_H
#include "mbed.h"
#include "AstroLib.h"

/**
 * @brief
 * Test of the AstroLib module
 * 
 * Tests the AstroLib module by calling most of the functions and comparing
 * the output with the expected values.
 * 
 * return 1 if successful, 0 otherwise
 */
int AstroLibTest();
#endif