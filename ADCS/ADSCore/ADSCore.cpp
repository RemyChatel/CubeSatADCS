/**
 * @file ADSCore.cpp
 * @version 1.0
 * @date 2019
 * @author Remy CHATEL
 * @copyright GNU Public License v3.0
 * 
 * @brief
 * Source code for the ADSCore library
 * 
 * @see ADSCore.h
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
#include "ADSCore.h"

// Constructors
    ADSCore::ADSCore():
        _i2c(new I2C(I2C_SDA, I2C_SCL)),
        imu(MPU9150(_i2c)),
        sun(SunSensor()) {

        time.start();
        _i2c->frequency(400000);
        last_update = time.read_us();
        q = Matrix::zeros(4,1);
        q(1) = 1.0f;
        w = Matrix::zeros(3,1);
        gyrb = Matrix::zeros(3,1);
        for(int i = 0; i < ADSCore_NSENSOR; i++){       // Initializing the vector to zero
            sbod[i] = Matrix::zeros(3,1);
            seci[i] = Matrix::zeros(3,1);
        }
    }

    ADSCore::ADSCore(I2C* i2c, PinName sunX, PinName sunY, PinName sunZ): 
        _i2c(i2c),
        imu(MPU9150(_i2c)),
        sun(SunSensor(sunX, sunY, sunZ)) {

        time.start();
        last_update = time.read_us();
        q = Matrix::zeros(4,1);
        q(1) = 1.0f;
        w = Matrix::zeros(3,1);
        gyrb = Matrix::zeros(3,1);
        for(int i = 0; i < ADSCore_NSENSOR; i++){       // Initializing the vector to zero
            sbod[i] = Matrix::zeros(3,1);
            seci[i] = Matrix::zeros(3,1);
        }
    }

    ADSCore::ADSCore(PinName sda, PinName scl, PinName sunX, PinName sunY, PinName sunZ):
        _i2c(new I2C(sda, scl)),
        imu(MPU9150(_i2c)),
        sun(SunSensor(sunX, sunY, sunZ)) {

        time.start();
        _i2c->frequency(400000);
        last_update = time.read_us();
        q = Matrix::zeros(4,1);
        q(1) = 1.0f;
        w = Matrix::zeros(3,1);
        gyrb = Matrix::zeros(3,1);
        for(int i = 0; i < ADSCore_NSENSOR; i++){       // Initializing the vector to zero
            sbod[i] = Matrix::zeros(3,1);
            seci[i] = Matrix::zeros(3,1);
        }
    }

// Object initialization
    void ADSCore::initSensors(){
        if (!imu.initIMU(AFS_2G, GFS_250DPS)) { // Try to initialize the IMU
            #ifdef ADSCore_USE_PRINTF
                printf("Could not connect to MPU9150: \r\n");
            #endif
            while(1);                           // Loop forever if communication doesn't happen
        }

        imu.recalibrateIMU(1000, 100);          // Recalibrate the IMU to remove DC noise on gyro
        float null_avg[3] = {0,0,0};
        imu.setAvgAcc(null_avg);                // Discard recalibration on acc
        imu.setAvgMag(null_avg);                // Discard recalibration on mag

        #ifdef ADSCore_USE_PRINTF
            printf("IMU online\r\n");
        #endif
    }

    void ADSCore::initOrbit(float parameters[6], int date[6]){
        orbit.setJulianDate(AstroLib::JulianDate(date[0],date[1],date[2],date[3],date[4],(float)date[5]));
        // Ground setting[lattitude    , longitude    , altitude     ,  mag_N       ,  mag_E       ,   mg_D       ]
        orbit.setOrbit   (parameters[0], parameters[1], parameters[2], parameters[3], parameters[4], parameters[5]);
    }


    void ADSCore::initQuest(float sigma_mag, float sigma_sun){
        omega[0] = sigma_mag;
        omega[1] = sigma_sun;
    }

    void ADSCore::initKalman(float sigma_q_eta, float sigma_q_epsilon, float sigma_gyr, float dt, Matrix I_sat, Matrix q_init, Matrix w_init){
        initKalman(sigma_q_eta, sigma_q_epsilon, sigma_gyr, dt, I_sat, q_init, w_init, Matrix::zeros(3,3));
    }

    void ADSCore::initKalman(float sigma_q_eta, float sigma_q_epsilon, float sigma_gyr, float dt, Matrix I_sat, Matrix q_init, Matrix w_init, Matrix I_wheel_init){
        // Set ups where taken from
        // "Kalman Filtering and the Attitude Determination and Control Task"
        // by Hale, Vergez, Meerman and Hashida
        Matrix P_init   = Matrix::eye(7);
        Matrix Kalman_Q = Matrix::eye(7);
        Matrix Kalman_R = Matrix::eye(7);

        // Initial Covariance matrix set up
        P_init(1,1) = sigma_q_eta * sigma_q_eta;
        P_init(2,2) = sigma_q_epsilon * sigma_q_epsilon;
        P_init(3,3) = sigma_q_epsilon * sigma_q_epsilon;
        P_init(4,4) = sigma_q_epsilon * sigma_q_epsilon;
        P_init(5,5) = (sigma_gyr*DEG2RAD) * (sigma_gyr*DEG2RAD);
        P_init(6,6) = (sigma_gyr*DEG2RAD) * (sigma_gyr*DEG2RAD);
        P_init(7,7) = (sigma_gyr*DEG2RAD) * (sigma_gyr*DEG2RAD);

        // Kalman process noice matrix set up
        Kalman_Q(1,1) *= 1e-4 * dt * dt * dt / (12 * I_sat(1,1) * I_sat(1,1));
        Kalman_Q(2,2) *= 1e-4 * dt * dt * dt / (12 * I_sat(2,2) * I_sat(2,2));
        Kalman_Q(3,3) *= 1e-4 * dt * dt * dt / (12 * I_sat(3,3) * I_sat(3,3));
        Kalman_Q(4,4) *=  (q_init(1)*I_sat(1,1))*(q_init(1)*I_sat(1,1))
                        + (q_init(2)*I_sat(2,2))*(q_init(2)*I_sat(2,2))
                        + (q_init(3)*I_sat(3,3))*(q_init(3)*I_sat(3,3));
        Kalman_Q(4,4) *= 1e-4 * dt * dt * dt / 12;
        Kalman_Q(5,5) *= 1e-4 * dt * dt * dt / (     I_sat(1,1) * I_sat(1,1));
        Kalman_Q(6,6) *= 1e-4 * dt * dt * dt / (     I_sat(2,2) * I_sat(2,2));    
        Kalman_Q(7,7) *= 1e-4 * dt * dt * dt / (     I_sat(3,3) * I_sat(3,3));

        // Kalman measurement noise matrix set up
        float sigma_r[7] = {sigma_q_eta * sigma_q_eta,
                            sigma_q_epsilon * sigma_q_epsilon,
                            sigma_q_epsilon * sigma_q_epsilon,
                            sigma_q_epsilon * sigma_q_epsilon,
                            sigma_gyr * sigma_gyr,
                            sigma_gyr * sigma_gyr,
                            sigma_gyr * sigma_gyr
                            };
        Kalman_R = Matrix::diag(7, sigma_r);
        
        // Initial quaternion set up
        q = q_init;
        w = w_init;
        kalman = Filters::KalmanFilter(I_sat, I_wheel_init, P_init, Kalman_Q, Kalman_R, q_init, Matrix::zeros(3,1));
    }

// Access

    Matrix ADSCore::getQ() const{ return q; }

    Matrix ADSCore::getW() const{ return w; }

    Matrix ADSCore::getGyrb() const{ return gyrb; }

    Matrix ADSCore::getSensorBody(int n){ return sbod[n]; }

    Matrix ADSCore::getSensorECI(int n){ return seci[n]; }

    const Filters::KalmanFilter& ADSCore::getKalman() const{ return kalman; }

    #ifdef ADSCore_USE_GND
    const AstroLib::Ground& ADSCore::getOrbit() const{ return orbit; }
    #else
    AstroLib::Orbit& ADSCore::getOrbit() const{ return orbit; }
    #endif

//Updaters
    Matrix ADSCore::update(){
       return update(Matrix::zeros(3,1),Matrix::zeros(3,1),Matrix::zeros(3,1));
    }

    Matrix ADSCore::update(Matrix w_rw_prev, Matrix T_bf_prev, Matrix T_rw_prev){
        fetchSensors();
        Estimators::QUEST(&q, ADSCore_NSENSOR, seci, sbod, omega, ADSCore_TOLERANCE);
        // kalman.filter(q, gyrb, time.read_us() - last_update, w_rw_prev, T_bf_prev, T_rw_prev);
        // q = kalman.getQuaternion();
        // w = kalman.getAngularRate();
        last_update = time.read_us();
        return q;
    }

    void ADSCore::fetchSensors(){
        // Earth Centered Inertial frame model
        orbit.update((time.read_us() - last_update)/1000000.0f);
        orbit.getMagVector(vecf);
        seci[0] = Matrix(3,1, vecf);
        orbit.getSunVector(vecf);
        seci[1] = Matrix(3,1, vecf);

        // Sun Sensor
        sun.getSunVector(vecf);
        sbod[1] = Matrix(3,1,vecf);

        // IMU
        if(imu.readByte(MPU9150_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
            imu.getGyro(vecf);  // Read the x/y/z adc values
            gyrb = Matrix(3,1,vecf);
            gyrb *= DEG2RAD;
            imu.getMag(vecf);  // Read the x/y/z adc values
            sbod[0] = Matrix(3,1,vecf);
            #ifdef ADSCore_USE_GND
                // Replacing the sun vector by the Earth gravity
                imu.getAccel(vecf);
                sbod[1] = Matrix(3,1,vecf);
            #endif
        }
        #ifdef ADSCore_USE_GND
            // Replacing the sun vector by the Earth gravity
            seci[1] = Matrix::zeros(3, 1);
            seci[1](3) = 1.0f;
        #endif
    }