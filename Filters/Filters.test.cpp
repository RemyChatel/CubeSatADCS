/**
 * @file Filters.test.cpp
 * @version 1.0
 * @date 2019
 * @author Remy CHATEL
 * @copyright GNU Public License v3.0
 * 
 * @brief
 * Source code for Filters.test.h
 * 
 * @see Filters.test.h
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
#include "Filters.test.h"
#define DEG2RAD 3.1415926535f/180.0f    ///< Conversion factor from degrees to radians
#define RAD2DEG 180.0f/3.1415926535f    ///< Conversion factor from radians to degrees

#define LOOP_TIME 1                     ///< Controls the time between loops

int KalmanFilterTest(){
    int size=0;
    float delta = 0;
    int lastUpdate = 0;
    int ellapsed = 0;
    int loop_time = 0;
    float seconds = 0;;
    int minutes = 0;

    Timer t;
    t.start();
    // printf("\n\r\n\r------------------------------\n\r");
    // printf("Connection OK\n\r");

    /**************** INIT ****************/
    using namespace Filters;

    float sigma_p;
    // Kalman Q process noise matrix setup
    float sigma_q;
    // Kalman R measurement noise matrix setup
    float sigma_eta;
    float sigma_epsilon;
    float sigma_omega;
    // Satellite Inertia matrix
    float I_sat_coef[9] = {27, 0, 0, 0, 17, 0, 0, 0, 25};
    
    #include "quat.data"
    #include "omega.data"

    // Squaring to obtain variance
    sigma_p*=sigma_p;
    sigma_q*=sigma_q;
    sigma_eta*=sigma_eta;
    sigma_epsilon*=sigma_epsilon;
    sigma_omega*=sigma_omega;

    Matrix p_init = sigma_p* Matrix::eye(7);
    Matrix kalman_q = sigma_q* Matrix::eye(7);
    //     sigma of:    eta    x    y     z     o_x  o_y  o_z with o_ the angular rate
    float sigma_r[7] = {sigma_eta, sigma_epsilon, sigma_epsilon, sigma_epsilon, sigma_omega, sigma_omega, sigma_omega};
    Matrix kalman_r = Matrix::eye(7);
    for(int i = 0; i < 7; i++){
        kalman_r(i+1,i+1) *= sigma_r[i];
    }

    Matrix I_sat = Matrix(3,3, I_sat_coef);
    Matrix I_wheel = Matrix::zeros(3,3);
    Matrix q_init(4,1, quat);
    Matrix w_init(3,1, omega);
    KalmanFilter kalman(I_sat, I_wheel, p_init, kalman_q, kalman_r, q_init, w_init);

    Matrix q_predicted(4,1);
    Matrix w_predicted(3,1);
    Matrix q_measured(4,1);
    Matrix w_measured(3,1);

    int iter = 0;

    /************** END INIT **************/
    lastUpdate = t.read_us();

    for(int i = 0; i < size; i++){
        for(int j = 0; j < 4; j++){
            q_measured(j+1) = quat[4*i+j];
        }
        for(int k = 0; k < 3; k++){
            w_measured(k+1) = omega[3*i+k];
        }

        loop_time = t.read_us() - lastUpdate;
        lastUpdate = t.read_us();
        /**************** LOOP ****************/

        q_predicted = kalman.filter(q_measured, w_measured, delta, Matrix::zeros(3,1), Matrix::zeros(3,1), Matrix::zeros(3,1));
        q_predicted *= 1/q_predicted.norm();

        /************** LOOP END **************/
        ellapsed = t.read_us()-lastUpdate;
        iter++;
        /*************** PRINTS ***************/
        w_predicted = kalman.getAngularRate();

        printf(" %f %f %f %f ", q_predicted(1), q_predicted(2), q_predicted(3), q_predicted(4));
        printf( "%f %f %f "    , w_predicted(1), w_predicted(2), w_predicted(3));
        printf( "%f %f\n\r", (float)ellapsed, iter/(float)size );

        /************* PRINTS END **************/
        // wait_ms(10);
    }
    return 1;
}