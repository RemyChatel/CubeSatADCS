/**
 * @file AstroLib.h
 * @version 1.0
 * @date 2019
 * @author Remy CHATEL
 * @copyright GNU Public License v3.0
 * @defgroup AstroLibGr AstroLib
 * 
 * @brief Provides orbital mechanics tools for spacecrafts
 * 
 * @details
 * # Description
 * This library provides tools for orbital mechanics for embeded C++ aiming
 * at spacecraft orbiting Earth.
 * 
 * Provides an orbit model, handling of Julian date, Sun position and
 * Earth magnetic field model
 * 
 * @see AstroLib
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
 
#ifndef ASTROLIB_H
#define ASTROLIB_H
#include <cmath>


/**
 * @ingroup AstroLibGr 
 * @{
 * @brief Provides orbital mechanics tools for spacecrafts
 * 
 * @namespace AstroLib
 * 
 * @details
 * # Description
 * This library provides tools for orbital mechanics for embeded C++ aiming
 * at spacecraft orbiting Earth.
 * 
 * Features of the library:
 * - Julian date time format
 * - Orbit model based on perifocal parameters
 * - Spacecraft position vector in the ECI (Earth Centered Inertial) frame
 * - Sun vector in the ECI frame
 * - Earth Magnetic Field vector in the ECI frame
 * 
 * @see Orbit
 * @see AstroLib.h
 * 
 * # Dependencies and data type
 * This library depends on <std::cmath> in order to perform 
 * cos, sin and sqrt operations.
 * 
 * @attention This library uses float (32-bits) and not double (64-bits)
 * to make best use of the Floating Point Unit of 32-bits microcontrollers.
 * @}
 */
namespace AstroLib{

/**
 * @brief Provide an orbit model that can return essential parameters
 * about the spacecraft position, the sun position and the magnetic field
 * 
 * @class AstroLib::Orbit
 * 
 * @details
 * Provides methods to provide information on a specified orbit.
 * This class provide an Orbit object that hold the current date and a model
 * of the orbit. It can return the Sun vector and the Earth magnetic field
 * vector in the ECI frame.
 * 
 * @see AstroLib
 */
class Orbit
{
public:
// Constructors
    /**
     * @brief AstroLib constructor
     */
    Orbit();
    
    /**
     * @brief AstroLib destructor
     */
    ~Orbit(void);

// Date management  
    /**
     * @brief Provide the Julian day and Julian day fraction
     * @param day The pointer where to store the day
     * @param frac The pointer where to store the day fraction
     */
    void getJulianDate(long *day, float *frac);
    
    /**
     * @brief Set the Julian date according to a calendar date
     * @param yr The year in XXXX format
     * @param mo The month
     * @param d The day
     * @param h The hours
     * @param mi The minutes
     * @param s The seconds
     */
    void setJulianDate(int yr, int mo, int d, int h, int mi, float s);

    /**
     * @brief Set the Julian date according to a Julian date
     * @param jd The Julian Day
     * @param jfrac The Julian Day Fraction
     */
    void setJulianDate(long jd, float jfrac);
    
    /** Update the Julian Date by adding the provided seconds
     * @param seconds The amount of seconds to add
     */
    void updateJulianDate(float seconds);

// Sun position
    /**
     * @brief Provides the Sun-vector in the ECI frame for a given Julian Date (in AU)
     * @param rsun The array where to store the sun vector
     * @param julianday The Julian day
     * @param julianfrac The Julian Day Fraction
     */
    void getSunVector(float rsun[3], long julianday, float julianfrac);

// Orbit management
    /**
     * @brief Set the obirt parameters
     * @param axis The semi major axis of the orbit (m)
     * @param ecc The eccentricity of the orbit
     * @param inc The orbit plane inclination (rad)
     * @param Omega The right ascension node (rad)
     * @param omega The argument of perigee (rad)
     * @param theta The true anomaly (rad)
     */
    void setOrbit(float axis, float ecc, float inc, float Omega, float omega, float theta);

    /**
     * @brief Update the mean anomaly at the current Julian date
     * @param seconds The time since last update
     * @param tolerance The tolerance for the Newton optimization method
     */
    void updateTrueAnomaly(float seconds, float tolerance);

    /**
     * @brief Provide the current position vector
     * @param r_sat The position vector of the satellite
     */
    void getPositionVector(float r_sat[3]);

// Earth magnetic field
    /**
     * @brief Update the Earth magnetic field vector for a given orbit position in the ECI frame
     * @param mag The magnetic vector of the Earth magnetic field
     * @param r_sat The position vector of the satellite
     * @param jday The Julian day (The object julian day if default or 0)
     * @param jfrac The Julian day fraction (The object julian day fraction if default or 0)
     */
    void mag_vector(float mag[3], float r_sat[3], long jday, float jfrac);

    /**
     * @brief
     * Converts a rotation quaternion to a rotation matrix
     * @param quat The rotation quaternion to convert
     * @param rot_coef The array where to store the rotation matrix
     */
    static void quat2rot(float quat[4], float rot_coef[9]);

    /**
     * @brief
     * Converts the tuple (azimuth, elevation) into a position vector
     * @param azimuth The azimuth of the target
     * @param elevation The elevation of the target
     * @param vec The array where to store the position vector
     */
    static void AzEl2Vec(float azimuth, float elevation, float vec[3]);
       
private:
    /* Private functions */
    /**
     * @brief Returns the norm of a vector
     * @param vec The vector from which to calculate the norm
     * @return The norm of the vector
     */
    float norm(float vec[3]);
    
    /**
     * @brief Performs the scalar product of two vector
     * a.b = transpose(a)*b
     * @param a First vector
     * @param b Second vector
     * @return The result of transpose(a)*b
     */
    float scalar(float *a, float *b);
    
    /* Private variables */
    // Date
    long julianday_;
    float julianfrac_;
    // orbit parameters
    float axis_;    // Semi major axis (m)
    float ecc_;     // Eccentricity
    float inc_;     // Orbital plane inclination (rad)
    float Omega_;   // Right ascension node longitude (rad)
    float omega_;   // Argument perigee (rad)
    float M_;       // Mean anomaly
    float E_;       // Eccentric anomaly (rad)
    float theta_;   // True anomaly (rad)
    // orbit stored computed-once variables 
    float rate;     // Mean angular rate (rad/s)
    float eccRatio; // = sqrt((1+ecc)/(1-ecc))
    float rotECI[9];// Rotation from perifocal to ECI frame

    /* Math and orbital mechanics constants */
    const float pi = 3.1415926535f;
    const float twopi = 6.283185307f;
    const float deg2rad = pi/180.0f;
    const float sec2jFrac = 1 / (60*60*24);
    const double mu = 398600441800000.0; // Earth gravitational constant (m3/s2)
    const float omega_earth = 0.000072921158f; // Earth rotation pulsation periode
    const float r_earth = 6378000.0f;     // Earth radius in m
}; // class Orbit



} // namespace AstroLib

#endif // ASTROLIB_H