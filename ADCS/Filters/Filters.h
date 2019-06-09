/**
 * @file   Filters.h
 * @version 1.0
 * @date 2019
 * @author Remy CHATEL
 * @copyright GNU Public License v3.0
 * @defgroup FiltersGr Filters
 * 
 * @brief
 * A library for filtering attitude quaternion and other signals
 * 
 * @details
 * # Description
 * A set of algorithm to filter the attitude quaternion of a spacecraft
 *
 * 
 * @see Kalman
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
#ifndef FILTERS_H
#define FILTERS_H
#include "Matrix.h"

/**
 * @ingroup FiltersGr
 * @{
 * @brief 
 * 
 * 
 * @namespace Filters
 * 
 * @details
 * # Description
 * 
 * 
 * @see Filters.h
 * 
 * # Dependencies
 * This library depends on the "Matrix" library that can be found
 * here: https://github.com/RemyChatel/
 * 
 * In turn this library relies on <std::cmath> and <std::vector>
 * 
 * @attention This library uses float only (NOT double) and therefore
 * expect 6 to 7 significant figures
 * @}
 */
namespace Filters{

/**
 * @brief
 * 
 * 
 * @class Filters::KalmanFilter
 * 
 * @details
 * 
 * 
 * @see Filters
 */
class KalmanFilter{
public:
// Constructors
    /**
     * @brief
     * Default constructor for the Kalman filter class
     */
    KalmanFilter();

    /**
     * @brief
     * Initialize the Kalman filter with the spacecraft and filter parameterss
     * @param I_sat_init    The inertia matrix of the satellite (3x3) Matrix
     * @param I_wheel_init  The inertia matrix of the reation wheels (3x3) Matrix
     * @param p_init        The initial covariance matrix (7x7) Matrix
     * @param kalman_q      The process noise covariance for the Kalman filter
     * @param kalman_r      The sensor noise covariance for the Kalman filter
     */
    KalmanFilter(Matrix I_sat_init, Matrix I_wheel_init, Matrix p_init, Matrix kalman_q, Matrix kalman_r);

    /**
     * @brief
     * Default destructor for the Kalman filter class
     */
    ~KalmanFilter(void);

// Getters and Setters
    /**
     * @brief
     * Fetched the predicted quaternion
     * @return The predicted quaternion
     */
    Matrix getQuaternion();

    /**
     * @brief
     * Fetched the predicted Angular Rate
     * @return The predicted Angular Rate
     */
    Matrix getAngularRate();

    /**
     * @brief
     * Fetched the predicted Covariance
     * @return The predicted Covariance
     */
    Matrix getCovariance();

// Filters
    /**
     * @brief
     * Filter the measured quaternion using a Kalman filter
     * @param q_measured      (@ step k)      Attitude quaternion measured from sensors (4x1) Matrix
     * @param w_measured      (@ step k)      Angular velocities in bf measured from sensors [rad/s] (3x1) Matrix
     * @param w_rw_prev       (@ step k-1)    Reaction wheel angular velocity [rad/s] at the previous time step (3x1) Matrix
     * @param T_bf_prev       (@ step k-1)    Total torque commanded to satellite in bf by external environment and magnetorquers (not reaction wheels!) [Nm] (3x1) Matrix  
     * @param T_rw_prev     (@ step k-1)    Torque commanded to satellite in bf by only reaction wheels [Nm] (3x1) Matrix
     * @param dt              (@ step k)      Simulation time step [sec].
     * @return The new predicted quaternion
     */
    Matrix filter(Matrix q_measured, Matrix w_measured, float dt, Matrix w_rw_prev, Matrix T_bf_prev, Matrix T_rw_prev);

private:
    Matrix I_sat;       /**< Inertia matrix of the spacecraft (3x3) Matrix */
    Matrix I_wheel;     /**< Inertia matrix of the reaction wheels (3x3) Matrix */

    Matrix q_predict;   /**< The predicted quaternion at step k (4x1) Matrix */
    Matrix w_predict;   /**< The predicted angular rates at step k (3x1) Matrix */
    Matrix p_predict;   /**<  The predicted covariance matrix at step k (7x7) Matrix */

    Matrix _kalman_q;   /**< Process noise covariance */
    Matrix _kalman_r;   /**< Sensor noise covariance */

}; // class KalmanFilter
}; // namespace Filters
#endif // FILTERS_H