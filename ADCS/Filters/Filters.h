/**
 * @file   Filters.h
 * @defgroup FiltersGr Filters
 * @version 1.0
 * @date 2019
 * @author Remy CHATEL
 * @copyright GNU Public License v3.0
 * 
 * @brief
 * A library for filtering attitude quaternion and other signals
 * 
 * @details
 * # Description
 * A set of algorithm to filter the attitude quaternion of a spacecraft. It implements
 * a 7 state (quaternion and angular rates) Extended Kalman Filter.
 * 
 * It can be use to filter out noise in space applications for instance in Atitude
 * Determination and Control Systems. 
 * 
 * @see Filters::KalmanFilter
 * 
 * # Example code without atitude control
 * 
 * @code
 * #include "Filters.h"
 * // Set ups where taken from
 * // "Kalman Filtering and the Attitude Determination and Control Task"
 * // by Hale, Vergez, Meerman and Hashida
 * 
 * // Inputs
 * Matrix q = Matrix::zeros(4,1);       // The output quaternion
 * Matrix gyr_b = Matrix::zeros(3,1);   // The measured angular rates (in rad/s)
 * float dt = 0.010;                    // The integration step
 * float sigma_gyr = 0.5;               // The gyroscope variance
 * float sigma_eta = 0.1;               // The quaternion scalar part variance
 * float sigma_epsilon = 0.1;           // The quaternion vector part variance
 * Matrix q_init = Matrix::zeros(4,1)   // The initial quaterion
 * Matrix I_sat(3,3);                   // Satellite inertia matrix
 * I_sat << 27 <<  0 <<  0
 *       <<  0 << 17 <<  0
 *       <<  0 <<  0 << 25;
 * 
 * // Set up
 * Matrix P_init   = Matrix::eye(7);
 * Matrix Kalman_Q = Matrix::eye(7);
 * Matrix Kalman_R = Matrix::eye(7);
 * Matrix q_init   = Matrix::zeros(4,1);
 * 
 * // Initial Covariance matrix set up
 * P_init *= 0.01;
 * P_init(5,5) = (sigma_gyr*DEG2RAD) * (sigma_gyr*DEG2RAD);
 * P_init(6,6) = (sigma_gyr*DEG2RAD) * (sigma_gyr*DEG2RAD);
 * P_init(7,7) = (sigma_gyr*DEG2RAD) * (sigma_gyr*DEG2RAD);
 * 
 * // Kalman process noice matrix set up
 * Kalman_Q(1,1) *= 1e-4 * dt * dt * dt / (12 * I_sat(1,1) * I_sat(1,1));
 * Kalman_Q(2,2) *= 1e-4 * dt * dt * dt / (12 * I_sat(2,2) * I_sat(2,2));
 * Kalman_Q(3,3) *= 1e-4 * dt * dt * dt / (12 * I_sat(3,3) * I_sat(3,3));
 * Kalman_Q(4,4) *= 1e-4 * dt * dt * dt / 12;
 * Kalman_Q(4,4) *=  (q_init(1)*I_sat(1,1))*(q_init(1)*I_sat(1,1))
 *                 + (q_init(2)*I_sat(2,2))*(q_init(2)*I_sat(2,2))
 *                 + (q_init(3)*I_sat(3,3))*(q_init(3)*I_sat(3,3));
 * Kalman_Q(5,5) *= 1e-4 * dt * dt * dt / (     I_sat(1,1) * I_sat(1,1));
 * Kalman_Q(6,6) *= 1e-4 * dt * dt * dt / (     I_sat(2,2) * I_sat(2,2));    
 * Kalman_Q(7,7) *= 1e-4 * dt * dt * dt / (     I_sat(3,3) * I_sat(3,3));
 *
 * // Kalman measurement noise matrix set up
 * float sigma_r[7] = {sigma_eta, sigma_epsilon, sigma_epsilon, sigma_epsilon, sigma_gyr, sigma_gyr, sigma_gyr};
 * for(int i = 0; i < 7; i++){ Kalman_R(i+1,i+1) *= sigma_r[i]*sigma_r[i];}
 * 
 * // Initial quaternion set up
 * q_init(2) = 1;
 * 
 * Filters::KalmanFilter kalman(I_sat, Matrix::zeros(3,3), P_init, Kalman_Q, Kalman_R, q_init, Matrix::zeros(3,1));
 * 
 * while(1){
 *      q = kalman.filter(q, gyr_b, dt, Matrix::zeros(3,1), Matrix::zeros(3,1), Matrix::zeros(3,1));
 *      wait_ms(dt);
 * }
 * 
 * @endcode
 *
 * # Dependencies
 * This library depends on the "Matrix" library that can be found
 * here: https://github.com/RemyChatel/
 * 
 * In turn this library relies on <std::cmath> and <std::vector>
 * 
 * @attention This library uses float only (NOT double) and therefore
 * expect 6 to 7 significant figures
 * 
 * # References
 * - "Kalman Filtering and the Attitude Determination and Control Task",
 * by Hale, Vergez, Meerman and Hashida
 * - "Spacecraft Dynamic and Control: An introduction",
 * by A. de Ruiter, C. Damaren and J Forbes
 * - "Fundamentals of Astrodynamics and Applications", by D. Vallado
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
 * @brief 
 * A namespace to hold the filtering processes
 * 
 * @namespace Filters
 * 
 * @see Filters
 */
namespace Filters{

/**
 * @ingroup FiltersGr
 * @brief
 * This class implements a 7-state (quaternion and angular rates)
 * Extended Kalman Filter for spacecraft atitude filtering.
 * 
 * @class Filters::KalmanFilter
 * 
 * @details
 * # Description
 * This class implements a 7-state (quaternion and angular rates)
 * Extended Kalman Filter for spacecraft atitude filtering.
 *
 * The initial inputs are the following:
 * - The inertia matrix of the satellite (3x3)
 * - The initial covariance matrix P (7x7)
 * - The process noise covariance matrix Q (7x7)
 * - The measurement noise covariance matrix R (7x7)
 * - The initial quaternion and angular rates
 * - The inertia matrix of the reaction wheels
 *
 * Once set up, the Kalman filter will predict the next quaternion using
 * an integration of how the system should evolve and the following inputs:
 * - The new measured quaternion
 * - The new measured angular rates
 * - The time ellapsed since last filter update
 * - The torques and angular rates of the control system of the spacecraft
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
     * @param I_sat_init    The inertia matrix of the satellite (in kg.m2) (3x3) Matrix
     * @param I_wheel_init  The inertia matrix of the reation wheels (in kg.m2) (3x3) Matrix
     * @param p_init        The initial covariance matrix (7x7) Matrix
     * @param kalman_q      The process noise covariance for the Kalman filter (7x7) Matrix
     * @param kalman_r      The sensor noise covariance for the Kalman filter (7x7) Matrix
     * @param q_init        The initial quaternion
     * @param w_init        The initial angular rates
     */
    KalmanFilter(Matrix I_sat_init, Matrix I_wheel_init, Matrix p_init, Matrix kalman_q, Matrix kalman_r, Matrix q_init, Matrix w_init);

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
     * @param T_rw_prev       (@ step k-1)    Torque commanded to satellite in bf by only reaction wheels [Nm] (3x1) Matrix
     * @param dt              (@ step k)      Simulation time step [sec].
     * @return The new predicted quaternion
     */
    Matrix filter(Matrix q_measured, Matrix w_measured, float dt, Matrix w_rw_prev, Matrix T_bf_prev, Matrix T_rw_prev);

private:
    Matrix I_sat;       /**< Inertia matrix of the spacecraft (in kg.m2) (3x3) Matrix */
    Matrix I_sat_inv;
    Matrix I_wheel;     /**< Inertia matrix of the reaction wheels (in kg.m2) (3x3) Matrix */

    Matrix q_predict;   /**< The predicted quaternion at step k (4x1) Matrix */
    Matrix w_predict;   /**< The predicted angular rates at step k (3x1) Matrix */
    Matrix p_predict;   /**<  The predicted covariance matrix at step k (7x7) Matrix */

    Matrix _kalman_q;   /**< Process noise covariance */
    Matrix _kalman_r;   /**< Sensor noise covariance */

}; // class KalmanFilter
}; // namespace Filters
#endif // FILTERS_H