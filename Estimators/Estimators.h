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
 * ## Quick overview
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
 * ## Detailed description
 * Now that both body frame measurements and ECI frame theoretical vectors are available,
 * the rotation that links each pair can be estimated:
 * 
 * @f{equation}{
 *     \hat{\mathbf{s}}_{b k}=\mathbf{C}_{b a} \hat{\mathbf{s}}_{a k}
 * @f}
 * 
 * where @f$\hat{\mathbf{s}}_{b k}@f$ is the measurement in the body frame, @f$\hat{\mathbf{s}}_{a k}@f$
 * is the model in the ECI frame and @f$\mathbf{C}_{b a}@f$ is the rotation matrix between the two.
 * 
 * The @f$\mathbf{C}_{b a}@f$ would supposedly unique if the measurement were noiseless and accurate
 * as well as if the model is perfect. The goal is therefore to find the rotation that would fit
 * best all the vector pairs.
 * 
 * This problem can be looked at as the following optimization problem, called _Whaba's problem_
 * 
 * @f{equation}{
 *     J\left(\mathbf{C}_{b a}\right) = \frac{1}{2} \sum_{k=1}^{N} w_{k}\left(\hat{\mathbf{s}}_{b k}-\mathbf{C}_{b a} \hat{\mathbf{s}}_{a k}\right)^{T}\left(\hat{\mathbf{s}}_{b k}-\mathbf{C}_{b a} \hat{\mathbf{s}}_{a k}\right)
 * @f}
 * 
 * and in term of quaternion, with @f$ B = \left(\sum_{k=1}^{N} w_{k} \hat{\mathbf{s}}_{a k} \hat{\mathbf{s}}_{b k}^{T}\right)^T @f$:
 * 
 * @f{equation}{
 *     \hat{J}(\mathbf{q})=\left[\begin{array}{cc}{\eta} & {\epsilon^{T}}\end{array}\right] \underbrace{\left[\begin{array}{cc}{\mathbf{K}_{11}} & {\mathbf{k}_{12}} \\ {\mathbf{k}_{12}^{T}} & {k_{22}}\end{array}\right]}_{\mathbf{K}} \underbrace{\left[\begin{array}{l} {\eta} \\ {\epsilon} \end{array}\right]}_{\mathbf{q}}
 * @f}
 * 
 * where @f$ k_{22} = tr( \mathbf{B} ) @f$ , 
 *       @f$ \mathbf{K}_{11} = \mathbf{B} + \mathbf{B}^T - k_{22}\mathbf{1} @f$ and 
 *       @f$ \mathbf{k}_{12}=\left[\left(B_{23}-B_{32}\right)\left(B_{31}-B_{13}\right)\left(B_{12}-B_{21}\right)\right]^{T} @f$ .
 * 
 * After some derivation, the QuEst (Quaternion Estimator)
 * algorithm can be found. This algorithm solves the optimization function by finding the maximum
 * eigenvalue of @f$ \mathbf{K}@f$ . The resulting eigenvector will be the best quaternion of the
 * optimization problem.
 * 
 * In order to avoid solving explicitly for the eigenvalue, which is a computationally-intensive
 * process, the QuEst algorithm uses Newton's method to compute a reasonable approximation of the
 * eigenvalue, then calculate the eigenvector. For that, it needs an initial value to converge in
 * the right direction.
 * 
 * See _Spacecraft dynamics and control: An introduction_ for the full derivation of the implemented algorithm.
 * 
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