/**
 * @file Estimators.test.h
 * @version 1.0
 * @date 2019
 * @author Remy CHATEL
 * @copyright GNU Public License v3.0
 * 
 * @brief
 * Test framework library for the Estimators module
 * 
 * @details
 * # Description
 * This library provides a test function for the Estimators module and also
 * serves as an example program for the module.
 * 
 * Tests the Estimators module by generating a set of five model vectors and
 * a theoretical rotation matrix. The rotation matrix is applied to the vectors
 * in order to generate fake measurements. The model-measurement vector pair
 * are then fed to the QuEst algorithm and the output error is computed.
 * 
 * @see Estimators.h
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
#ifndef ESTIMATORS_TEST_H
#define ESTIMATORS_TEST_H
#include "mbed.h"
#include "Estimators.h"

/**
 * @brief
 * Test of the Estimators module
 * 
 * Tests the Estimators module by generating a set of five model vectors and
 * a theoretical rotation matrix. The rotation matrix is applied to the vectors
 * in order to generate fake measurements. The model-measurement vector pair
 * are then fed to the QuEst algorithm and the output error is computed.
 * 
 * return 1 if successful, 0 otherwise
 */
int QuestTest();
#endif