/**
 * @file   Filters.cpp
 * @version 1.0
 * @date 2019
 * @author Remy CHATEL
 * @copyright GNU Public License v3.0
 * 
 * @brief
 * Source code of Filters.h
 * 
 * @see Filters.h
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

#include "Filters.h"
#ifdef FILTERS_USE_PRINTF
#include "mbed.h"
#endif

using namespace Filters;

// Constructors
    KalmanFilter::KalmanFilter(){
        I_sat = Matrix::zeros(3, 3);
        I_wheel = Matrix::zeros(3, 3);
        I_sat_inv = Matrix::zeros(3, 3);

        q_predict = Matrix::zeros(4, 1);
        w_predict = Matrix::zeros(3, 1);
        p_predict = Matrix::zeros(7, 7);

        _kalman_q = Matrix::zeros(7, 7);
        _kalman_r = Matrix::zeros(7, 7);
    }

    KalmanFilter::KalmanFilter(Matrix I_sat_init, Matrix I_wheel_init, Matrix p_init, Matrix kalman_q, Matrix kalman_r, Matrix q_init, Matrix w_init){
        I_sat = I_sat_init;
        I_wheel = I_wheel_init;
        I_sat_inv = I_sat.Inv();

        q_predict = Matrix::zeros(4, 1);
        w_predict = Matrix::zeros(3, 1);
        p_predict = p_init;

        _kalman_q = kalman_q;
        _kalman_r = kalman_r;

        q_predict = q_init;
        w_predict = w_init;
    }

    KalmanFilter::~KalmanFilter(void){}

// Getters and Setters
    Matrix KalmanFilter::getQuaternion()  const {return q_predict;}
    Matrix KalmanFilter::getAngularRate() const {return w_predict;}
    Matrix KalmanFilter::getCovariance()  const {return p_predict;}

// Filters
    Matrix KalmanFilter::filter(Matrix q_measured, Matrix w_measured, float dt, Matrix w_rw_prev, Matrix T_bf_prev, Matrix T_rw_prev){
        // (0) Shift data to "previous state"
        Matrix q_predict_prev = Matrix(q_predict);
        Matrix w_predict_prev = Matrix(w_predict);
        Matrix p_predict_prev = Matrix(p_predict);

        // (1) Propagate the covariance
        Matrix h_rw_prev   = I_wheel * w_rw_prev;
        float f56 = (1/I_sat(1,1))*(-I_sat(2,2)*w_predict_prev(3)+I_sat(3,3)*w_predict_prev(3) - h_rw_prev(3));
        float f57 = (1/I_sat(1,1))*(-I_sat(2,2)*w_predict_prev(2)+I_sat(3,3)*w_predict_prev(2) + h_rw_prev(2));
        float f65 = (1/I_sat(2,2))*(-I_sat(3,3)*w_predict_prev(3)+I_sat(1,1)*w_predict_prev(3) + h_rw_prev(3));
        float f67 = (1/I_sat(2,2))*(-I_sat(3,3)*w_predict_prev(1)+I_sat(1,1)*w_predict_prev(1) - h_rw_prev(1));
        float f75 = (1/I_sat(3,3))*(-I_sat(1,1)*w_predict_prev(2)+I_sat(2,2)*w_predict_prev(2) - h_rw_prev(2));
        float f76 = (1/I_sat(3,3))*(-I_sat(1,1)*w_predict_prev(1)+I_sat(2,2)*w_predict_prev(1) + h_rw_prev(1));
        
        float f_coef[49] = {0                   ,-w_predict_prev(1) ,-w_predict_prev(2) ,-w_predict_prev(3) ,-q_predict_prev(2) ,-q_predict_prev(3) ,-q_predict_prev(4) ,
                            w_predict_prev(1)   ,0                  ,w_predict_prev(3)  ,-w_predict_prev(2) ,q_predict_prev(1)  ,-q_predict_prev(4) ,q_predict_prev(3)  ,
                            w_predict_prev(2)   ,-w_predict_prev(3) ,0,w_predict_prev(1),q_predict_prev(4)  ,q_predict_prev(4)  ,-q_predict_prev(2) ,
                            w_predict_prev(2)   ,w_predict_prev(3)  ,w_predict_prev(1)  ,0,q_predict_prev(3),q_predict_prev(2)  ,q_predict_prev(1)  ,
                            0                   ,0                  ,0                  ,0                  ,0                  ,f56                ,f57                ,
                            0                   ,0                  ,0                  ,0                  ,f65                ,0                  ,f67                ,
                            0                   ,0                  ,0                  ,0                  ,f75                ,f76                ,0                  };
        Matrix f(7,7, f_coef);
        
        f *= dt;
        Matrix p_propagate = (Matrix::eye(7) + f) * p_predict_prev * (Matrix::eye(7) + f).Transpose() + _kalman_q;
        
        // (2) Predict the state ahead
        float temp_coef[16] = { 0                   ,-w_predict_prev(1) ,-w_predict_prev(2) ,-w_predict_prev(3) ,
                                w_predict_prev(1)   ,0                  ,w_predict_prev(3)  ,-w_predict_prev(2) ,
                                w_predict_prev(2)   ,-w_predict_prev(3) ,0                  , w_predict_prev(1) ,
                                w_predict_prev(3)   ,w_predict_prev(2)  ,-w_predict_prev(1) , 0                 };
                                
        Matrix tmp(4,4, temp_coef);

        Matrix q_propagate = q_predict_prev + (0.5 * tmp * q_predict_prev) * dt;     

        q_propagate = q_propagate * (1 / q_propagate.norm());
                        
        Matrix w_x_hr = Matrix::cross(w_predict_prev, h_rw_prev);

        Matrix w_x_Iw = Matrix::cross(w_predict_prev,I_sat * w_predict_prev);   // cross product of satellite angular velocity in bf [rad/s], and the product of the satellite inertia [kgm2] and the satellite angular velocity in bf [rad/s].                              

        Matrix w_propagate = w_predict_prev + (I_sat_inv * (T_bf_prev - w_x_Iw - w_x_hr - T_rw_prev)) * dt; // state propagated to next time step using the satellite dynamics model.

        Matrix x_propagate(7,1);
        x_propagate(1) = q_propagate(1);
        x_propagate(2) = q_propagate(2);
        x_propagate(3) = q_propagate(3);
        x_propagate(4) = q_propagate(4);
        x_propagate(5) = w_propagate(1);
        x_propagate(6) = w_propagate(2);
        x_propagate(7) = w_propagate(3);

        // (3) Calculate the Kalman Gain
        Matrix kalman = p_propagate * ( p_propagate + _kalman_r ).TaylorInv(3);
    
        // (5) Update the state
        Matrix z(7,1);
        z(1) = q_measured(1);
        z(2) = q_measured(2);
        z(3) = q_measured(3);
        z(4) = q_measured(4);
        z(5) = w_measured(1);
        z(6) = w_measured(2);
        z(7) = w_measured(3);

        Matrix x_predict     = x_propagate + kalman * (z - x_propagate); // predict the current state

        // output the current predicted quaternion // Problem here
        q_predict(1) = x_predict(1);
        q_predict(2) = x_predict(2);
        q_predict(3) = x_predict(3);
        q_predict(4) = x_predict(4);
        // output the current predicted angular velocity of satellite in bf
        w_predict(1) = x_predict(5);
        w_predict(2) = x_predict(6);
        w_predict(3) = x_predict(7);

        // Renormalize vectors
        q_predict /= q_predict.norm();
        w_predict /= w_predict.norm();

        // (6) Precict the next covariance
        p_predict = ( Matrix::eye(7) - kalman ) * p_propagate;

        return q_predict;
    }