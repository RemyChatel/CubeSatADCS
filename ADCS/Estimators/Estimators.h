/**
 * @file   Estimators.h
 * @defgroup EstimatorsGr Estimators
 * @version 1.0
 * @date 2019
 * @author Remy CHATEL
 * @copyright GNU Public License v3.0
 * 
 * @brief
 * A library for attitude estimators given a set of atitude measurements
 * 
 * @details
 * # Description
 * In order to determine the attitude of a satellite, it is useful
 * to fuse the output of multiple sensors to increase accuracy.
 *
 * This library provides estimators algorithm that returns the rotation
 * quaternion using multiple sensors.
 * 
 * The QuEst (Quaternion Estimator) algorithm is a way of solving Wahba's problem:
 * 
 * If we have a set of measurements in the body frame and a set of the same measurememts
 * in an inertial frame, then we can try to find the optimal rotation that links best
 * each pair of body-inertial measurements.
 * 
 * The QuEst algorithm works by finding an optimal eigen value for the combined
 * measurement matrix, using a Newton solver.
 * 
 * @see Estimators::QUEST
 * 
 * # Example code
 * 
 * @see Estimators.test.cpp
 * 
 * # References
 * [1] "Spacecraft Dynamics and Control An Introduction" by
 * de Ruiter, Damaren and Forbes
 * 
 * # Dependencies
 * This library depends on the "Matrix" library that can be found
 * here: https://github.com/RemyChatel/
 * In turn this library relies on <std::cmath> and <std::vector>
 * 
 * @attention This library uses float only (NOT double) and therefore
 * expect 6 to 7 significant figures
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
 
#ifndef ESTIMATORS_H
#define ESTIMATORS_H
#include "Matrix.h"

/**
 * @brief
 * A library for attitude estimators given a set of 
 * atitude measurements
 * 
 * @namespace Estimators
 */
namespace Estimators{

/**
 * @ingroup EstimatorsGr
 * @brief
 * Return the quaternion estimate from the QuEst Algorithm
 * @details
 * Algorithm from "Spacecraft Dynamics and Control An Introduction" by
 * de Ruiter, Damaren and Forbes
 * 
 * This method compute the quaternion of the rotation between the spacecraft body
 * frame and the Earth Centered Inertial frame using a set of N sensors using the
 * QuEst algorithm.
 * 
 * Each sensor should provide the measurement in the spacecraft body frame, while
 * models/data should provide the measurement in the ECI frame. Each sensor is
 * weighted according to the quality of its output (usually, omega = 1 / (sigma^2), 
 * with sigma^2 the variance of the output.
 * 
 * The quaternion return follow the format [x, y, z, eta] with eta the scalar part
 * and x, y, z the vector components.
 * 
 * @param quat The quaternion to update [x, y, z, eta]
 * @param N The number of measurements
 * @param s_eci  A pointer to N 3-element array (the vectors) of the normalized models in the ECI frame [x, y, z]
 * @param s_body A pointer to N 3-element array (the vectors) of the normalized measurements in the satellite body frame [x, y, z]
 * @param omega An N-element array containing the weight of each measurement
 * @param tolerance The tolerance of Newton's optimization problem (default is 1e-5)
 */
void QUEST(float quat[4], int N, float **s_eci, float **s_body, float *omega, float tolerance);

/**
 * @ingroup EstimatorsGr
 * @brief
 * Return the quaternion estimate from the QuEst Algorithm
 * @details
 * Algorithm from "Spacecraft Dynamics and Control An Introduction" by
 * de Ruiter, Damaren and Forbes
 * 
 * This method compute the quaternion of the rotation between the spacecraft body
 * frame and the Earth Centered Inertial frame using a set of N sensors using the
 * QuEst algorithm.
 * 
 * Each sensor should provide the measurement in the spacecraft body frame, while
 * models/data should provide the measurement in the ECI frame. Each sensor is
 * weighted according to the quality of its output (usually, omega = 1 / (sigma^2), 
 * with sigma^2 the variance of the output.
 * 
 * The quaternion return follow the format [x, y, z, eta] with eta the scalar part
 * and x, y, z the vector components.
 * 
 * @param quat The quaternion to update [x, y, z, eta]
 * @param N The number of measurements
 * @param s_eci  An array of N normalized models in the ECI frame [x, y, z]
 * @param s_body An array of N normalized measurements in the satellite body frame [x, y, z]
 * @param omega An N-element array containing the weight of each measurement
 * @param tolerance The tolerance of Newton's optimization problem (default is 1e-5)
 */
void QUEST(Matrix *quat, int N, Matrix *s_eci, Matrix *s_body, float *omega, float tolerance);

} // namespace Estimators
#endif // ESTIMATORS_H