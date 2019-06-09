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
 * @details
 * # Description
 * 
 * @see Filters
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

using namespace Filters;

// Constructors
    KalmanFilter::KalmanFilter(){
        I_sat = Matrix::zeros(3, 3);
        I_wheel = Matrix::zeros(3, 3);

        q_predict = Matrix::zeros(4, 1);
        w_predict = Matrix::zeros(3, 1);
        p_predict = Matrix::zeros(7, 7);
    }

    KalmanFilter::KalmanFilter(Matrix I_sat_init, Matrix I_wheel_init, Matrix p_init, Matrix kalman_q, Matrix kalman_r){
        I_sat = I_sat_init;
        I_wheel = I_wheel_init;

        q_predict = Matrix::zeros(4, 1);
        w_predict = Matrix::zeros(3, 1);
        p_predict = p_init;

        _kalman_q = kalman_q;
        _kalman_r = kalman_r;
    }
    KalmanFilter::~KalmanFilter(void){}

// Getters and Setters
    Matrix KalmanFilter::getQuaternion(){return q_predict;}
    Matrix KalmanFilter::getAngularRate(){return w_predict;}
    Matrix KalmanFilter::getCovariance(){return p_predict;}

// Filters
    Matrix KalmanFilter::filter(Matrix q_measured, Matrix w_measured, float dt, Matrix w_rw_prev, Matrix T_bf_prev, Matrix T_rw_prev){
        // (1) Propagate the covariance 
        Matrix q_predict_prev = q_predict;
        Matrix w_predict_prev = w_predict;
        Matrix p_predict_prev = p_predict;

        Matrix h_rw_prev   = I_wheel * w_rw_prev;
        float f56 = (1/I_sat(0,0))*(-I_sat(1,1)*w_predict_prev(2,0)+I_sat(2,2)*w_predict_prev(2,0) - h_rw_prev(2,0));
        float f57 = (1/I_sat(0,0))*(-I_sat(1,1)*w_predict_prev(1,0)+I_sat(2,2)*w_predict_prev(1,0) + h_rw_prev(1,0));
        float f65 = (1/I_sat(1,1))*(-I_sat(2,2)*w_predict_prev(2,0)+I_sat(0,0)*w_predict_prev(2,0) + h_rw_prev(2,0));
        float f67 = (1/I_sat(1,1))*(-I_sat(2,2)*w_predict_prev(0,0)+I_sat(0,0)*w_predict_prev(0,0) - h_rw_prev(0,0));
        float f75 = (1/I_sat(2,2))*(-I_sat(0,0)*w_predict_prev(1,0)+I_sat(1,1)*w_predict_prev(1,0) - h_rw_prev(1,0));
        float f76 = (1/I_sat(2,2))*(-I_sat(0,0)*w_predict_prev(0,0)+I_sat(1,1)*w_predict_prev(0,0) + h_rw_prev(0,0));
        float f_coef[49] = {0                   ,-w_predict_prev(0,0) ,-w_predict_prev(1,0) ,-w_predict_prev(2,0) ,-q_predict_prev(1,0) ,-q_predict_prev(2,0) ,-q_predict_prev(3,0),
                            w_predict_prev(0,0) , 0                   , w_predict_prev(2,0) ,-w_predict_prev(1,0) , q_predict_prev(0,0) ,-q_predict_prev(3,0) , q_predict_prev(2,0),
                            w_predict_prev(1,0) ,-w_predict_prev(2,0) , 0                   , w_predict_prev(0,0) , q_predict_prev(3,0) , q_predict_prev(3,0) ,-q_predict_prev(1,0),
                            w_predict_prev(1,0) , w_predict_prev(2,0) , w_predict_prev(0,0) , 0                   , q_predict_prev(2,0) , q_predict_prev(1,0) , q_predict_prev(0,0),
                            0                   , 0                   , 0                   , 0                   , 0                   , f56                 , f57                ,
                            0                   , 0                   , 0                   , 0                   , f65                 , 0                   , f67                ,
                            0                   , 0                   , 0                   , 0                   , f75                 , f76                 , 0                  };
        Matrix f(7,7, f_coef);
        Matrix p_propagate = (Matrix::eye(7) + f*dt)*p_predict_prev*(Matrix::eye(7) + f*dt).Transpose() + _kalman_q;  // TO BE DETERMINED

        // (2) Predict the state ahead
        float temp_coef[16] = {0                   ,-w_predict_prev(0,0) ,-w_predict_prev(1,0)  ,-w_predict_prev(2,0),
                               w_predict_prev(0,0) , 0                   , w_predict_prev(2,0)  ,-w_predict_prev(1,0),
                               w_predict_prev(1,0) ,-w_predict_prev(2,0) , 0                    , w_predict_prev(0,0),
                               w_predict_prev(2,0) , w_predict_prev(1,0) ,-w_predict_prev(0,0)  , 0                  };
                                
        Matrix tmp(4,4, temp_coef);

        Matrix q_propagate = q_predict_prev + (0.5*tmp*q_predict_prev)*dt;      

        q_propagate = q_propagate * (1 / q_propagate.norm());                    
                        
        Matrix w_x_hr = Matrix::cross(w_predict_prev, h_rw_prev);

        Matrix w_x_Iw = Matrix::cross(w_predict_prev,I_sat*w_predict_prev);   // cross product of satellite angular velocity in bf [rad/s], and the product of the satellite inertia [kgm2] and the satellite angular velocity in bf [rad/s].                              

        Matrix w_propagate = w_predict_prev + (I_sat.Inv()*(T_bf_prev - w_x_Iw - w_x_hr - T_rw_prev))*dt; // state propagated to next time step using the satellite dynamics model.

        Matrix x_propagate(7,1);
        x_propagate(0,0) = q_propagate(0,0);
        x_propagate(1,0) = q_propagate(1,0);
        x_propagate(2,0) = q_propagate(2,0);
        x_propagate(3,0) = q_propagate(3,0);
        x_propagate(4,0) = w_propagate(0,0);
        x_propagate(5,0) = w_propagate(1,0);
        x_propagate(6,0) = w_propagate(2,0);


        // (3) Calculate the Kalman Gain
        Matrix kalman = p_propagate * Matrix::eye(7) * ( Matrix::eye(7) * p_propagate * Matrix::eye (7) + _kalman_r ).Inv();
    
        // (5) Update the state
        Matrix z(7,1);
        z(0,0) = q_measured(0,0);
        z(1,0) = q_measured(1,0);
        z(2,0) = q_measured(2,0);
        z(3,0) = q_measured(3,0);
        z(4,0) = w_measured(0,0);
        z(5,0) = w_measured(1,0);
        z(6,0) = w_measured(2,0);

        Matrix x_predict     = x_propagate + kalman*(z - Matrix::eye(7)*x_propagate); // predict the current state 

        // output the current predicted quaternion // Problem here
        q_predict(0,0) = x_predict(0,0);
        q_predict(1,0) = x_predict(1,0);
        q_predict(2,0) = x_predict(2,0);
        q_predict(3,0) = x_predict(3,0);
        // output the current predicted angular velocity of satellite in bf
        w_predict(0,0) = w_predict(4,0);
        w_predict(1,0) = w_predict(5,0);
        w_predict(2,0) = w_predict(6,0);

        // (6) Precict the next covariance
        p_predict = (Matrix::eye(7) - kalman*Matrix::eye(7))*p_propagate;

        return q_predict;
    }