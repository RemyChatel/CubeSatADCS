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
 * @code
 * #include "Estimators.h"
 * 
 * #define ADS_TOLERANCE 1e-5                  // Tolerance for the Newton solver
 * float coef_th[9] = {0.4153, 0.4472, 0.7921, -0.7652, 0.6537, 0.0274, -0.5056, -0.6104, 0.6097};
 * Matrix mat_th = Matrix(3,3, coef_th);
 * 
 * // measurements in the inertial frame
 * float sa1n[3] = {0.0f, 0.447214f, 0.894427f};
 * float sa2n[3] = {0.316228f, 0.948683f, 0.0f};
 * float sa3n[3] = {-0.980581f, 0.0f, 0.196116f};
 * float sa4n[3] = {0.235702f, -0.235702f, 0.942809f};
 * float sa5n[3] = {0.57735f, 0.57735f, 0.57735f};
 * 
 * // noisy measurements in the body frame
 * float sb1n[3] = { 0.9082f, 0.3185f, 0.2715f };
 * float sb2n[3] = { 0.5670f, 0.3732f, -0.7343f };
 * float sb3n[3] = { -0.2821f, 0.7163f, 0.6382 };
 * float sb4n[3] = { 0.7510f, -0.3303f, 0.5718};
 * float sb5n[3] = { 0.9261f, -0.2053, -0.3166};
 * 
 * float *san[5] = {sa1n, sa2n, sa3n, sa4n, sa5n};
 * float *sbn[5] = {sb1n, sb2n, sb3n, sb4n, sb5n};
 * 
 * // Weight of the sensors
 * float om[5] = {0.0100f, 0.0325f, 0.0550f, 0.0775, 0.1000};
 * 
 * float q[4]; // Output quaternion
 * 
 * Estimators::QUEST(q, 5, sbn, san, om, TOLERANCE);
 * @endcode
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