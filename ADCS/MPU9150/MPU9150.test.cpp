/**
 * @file MPU9150.test.cpp
 * @version 1.0
 * @date 2019
 * @author Remy CHATEL
 * @copyright GNU Public License v3.0
 * 
 * @brief
 * Source code for MPU9150.test.h
 * 
 * @see MPU9150.test.h
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
#include "MPU9150.test.h"

int SensorTest(){
    I2C i2c(I2C_SDA, I2C_SCL);
    i2c.frequency(400000);
    SunSensor sun(A0,A1,A2);
    MPU9150 imu(&i2c);
    Timer t;
    t.start();

    int ellapsed = 0;
    int last_update = 0;
    int print_update = t.read_ms();

    float rsun_b[3];
    float rsun_e[3];
    float rmag_b[3];
    float rmag_e[3] = {17.5, 0.5, 47};

    int mcount = 0; // Frequency divider for the magnetometer
    uint8_t MagRate = 50; // set magnetometer read rate in Hz; 10 to 100 (max) Hz are reasonable values
    uint8_t acc_scale = AFS_2G;
    uint8_t gyr_scale = GFS_250DPS;
    float val_acc[3], val_gyr[3], val_mag[3];

    //--------------------- INIT ---------------------//
    printf("\n\r\n\r\n\r\n\r\n\r\n\r--------------------------------------\n\r");
    printf("Connection ok\n\r");
    uint8_t whoami = imu.readByte(MPU9150_ADDRESS, WHO_AM_I_MPU9150);  // Read WHO_AM_I register for MPU-9150
    printf("I AM 0x%x\n\r", whoami); printf("I SHOULD BE 0x68 or 0x73\n\r");
    if (!imu.initIMU(acc_scale, gyr_scale)) {   // WHO_AM_I should be 0x68
        printf("Could not connect to MPU9150: \n\r");
        while(1) ; // Loop forever if communication doesn't happen
    }
    imu.recalibrateIMU(1000, 100);
    float null_avg[3] = {0,0,0};
    imu.setAvgAcc(null_avg);
    imu.setAvgMag(null_avg);
    printf("IMU ok\n\r");

    Matrix dq(4,1);
    Matrix gyr(3,1);
    Matrix quat = Matrix::zeros(4,1);
    quat(2) = 1;
    float dt2, w_norm;
    int time = t.read_us();

    while(1){
        last_update = t.read_us();

        //--------------------- LOOP ---------------------//
        if(imu.readByte(MPU9150_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
            imu.getAccel(val_acc);  // Read the x/y/z adc values
            imu.getGyro(val_gyr);  // Read the x/y/z adc values
            sun.getSunVector(rsun_b);

            mcount++;
            if (mcount > 200/MagRate) {  // this is a poor man's way of setting the magnetometer read rate (see below) 
                imu.getMag(val_mag);  // Read the x/y/z adc values
                mcount = 0;
            }
        }
        //------------------- END LOOP -------------------//

        ellapsed = t.read_us() - last_update;

        //-------------------- PRINT ---------------------//

        Matrix grav(3,1, val_acc);
        grav /= grav.norm();
        Matrix mag(3,1, val_mag);
        Matrix magNED(3,1);
        magNED << 17.3186f << -.6779f << 46.8663f;
        
        if(t.read_ms() - print_update > 500){
            print_update = t.read_ms();
            printf("\n\r\n\rLoop time %d us | Frequency %4.0f Hz\n\r", ellapsed, 1000000.0f/ellapsed);
            printf("Acc (mg):  ");
            printf("{% 4.2f, % 4.2f, % 4.2f}\n\r", val_acc[0]*1000, val_acc[1]*1000, val_acc[2]*1000);
            printf("Gyr (°/s): ");
            printf("{% 4.2f, % 4.2f, % 4.2f}\n\r", val_gyr[0], val_gyr[1], val_gyr[2]);
            printf("Mag (uT):  ");
            printf("{% 4.2f, % 4.2f, % 4.2f}\n\r", val_mag[0], val_mag[1], val_mag[2]);
            printf("Sun:       ");
            printf("{% 4.2f, % 4.2f, % 4.2f}\n\r", rsun_b[0], rsun_b[1], rsun_b[2]);


            mag.Transpose().print();
            magNED.Transpose().print();
            
            mag /= mag.norm();
            magNED /= magNED.norm();
            mag.Transpose().print();
            magNED.Transpose().print();
        }

        if(t.read_ms() > 1<<21) {
            t.start(); // start the timer over again if ~30 minutes has passed
            last_update = t.read_us();
        }
        wait_ms(20);
    }

    return 1;
}