/**
 * @file AstroLib.test.cpp
 * @version 1.0
 * @date 2019
 * @author Remy CHATEL
 * @copyright GNU Public License v3.0
 * 
 * @brief
 * Source code for AstroLib.test.h
 * 
 * @see AstroLib.test.h
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
#include "AstroLib.test.h"
#define DEG2RAD 3.1415926535f/180.0f    ///< Conversion factor from degrees to radians

int AstroLibTest(){
    using namespace AstroLib;

    #ifdef MBED_H
    printf("\n\r\n\r------------------------------\n\r");
    printf("Connection OK\n\r");
    #endif

    int year        = 2019;
    int month       = 06;
    int day         = 07;
    int hours       = 17;// /!\ Use GMT+00 !!!!
    int minutes     = 15;
    float seconds   = 00;

    JulianDate date;
    Ground orbit;
    orbit.setJulianDate(JulianDate(year,month,day,hours,minutes,seconds));
    // Ground setting[lattitude, longitude, altitude,  mag_N  ,  mag_E ,   mg_D  ] in North East Down frame
    orbit.setOrbit(55.86515, -4.25763, 0.0f, 17.3186f, -.6779f, 46.8663f);
    
    float sun_eci[3];
    float mag_eci[3];
    float sat_eci[3];
    float azel_sun[2];
    float norm = 0;

    int time;

    #ifdef MBED_H
    Timer t;
    t.start();
    printf("Objects created");
    #endif

    while(1){
    
    #ifdef MBED_H
    time = t.read_us();
    #endif

    orbit.update(1.0f);
    orbit.getSunVector(sun_eci);
    orbit.getMagVector(mag_eci);

    // Normalize to obtain the direction vector
    norm = mag_eci[0]*mag_eci[0] + mag_eci[1]*mag_eci[1] + mag_eci[2]*mag_eci[2];
    mag_eci[0] /= norm; mag_eci[1] /= norm; mag_eci[2] /= norm;

    // Normalize to obtain the direction vector 
    norm = sun_eci[0]*sun_eci[0] + sun_eci[1]*sun_eci[1] + sun_eci[2]*sun_eci[2];
    sun_eci[0] /= norm; sun_eci[1] /= norm; sun_eci[2] /= norm;
    
    #ifdef MBED_H
    time = t.read_us();
    #endif
    
    orbit.getPositionVector(sat_eci);
    orbit.getSunAzEl(azel_sun);
    // Normalize to obtain the direction vector 
    norm = sat_eci[0]*sat_eci[0] + sat_eci[1]*sat_eci[1] + sat_eci[2]*sat_eci[2];
    sat_eci[0] /= norm; sat_eci[1] /= norm; sat_eci[2] /= norm;

    #ifdef MBED_H
    printf("\n\rExecution time %d us | frequency %ld Hz\n\r", time, (long)(1000000.0f/time));
    printf("Calendar date %d-%d-%d-%d-%d-%f\n\r", year,month,day,hours,minutes,seconds);
    printf("Julian Date %ld and %f\n\r", orbit.getJulianDate().getDay(), orbit.getJulianDate().getFrac());
    printf("Satelite position [% 1.6e, % 1.6e, % 1.6e]\n\r", sat_eci[0], sat_eci[1], sat_eci[2]);
    printf("Sun vector ephemeride (AU): [% f , % f , % f]\n\r", sun_eci[0], sun_eci[1], sun_eci[2]);
    printf("Mag vector ephemeride: [% f , % f , % f]\n\r", mag_eci[0], mag_eci[1], mag_eci[2]);
    printf("AzEl Sun: [% f, % f]\n\r", azel_sun[0]*180/3.1415926535f, azel_sun[1]*180/3.1415926535f);
    #endif

    seconds+=1;
    if(seconds >= 60){
        seconds = 0;
        minutes++;
    }
    if(minutes >= 60){
        minutes = 0;
        hours++;
    }
    #ifdef MBED_H
    wait_us(1000000);
    #endif
    }
    return 1;
}