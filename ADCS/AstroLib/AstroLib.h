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
 * @see Ground
 * @see JulianDate
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
 * @brief
 * A representation of the Julian Date
 * 
 * @class JulianDate
 * 
 * @details
 * Provides a storage and method to store and manipulate Julian Dates.
 * 
 * The julian day is split in an integer part (stored as a 32-bits 'long' type)
 * and a decimal part (stored as a 32-bits 'float' type) to allow more significant
 * figures.
 * 
 * @see AstroLib
 */
class JulianDate {
public:
// Constructors
    /**
     * @brief
     * Default constructor for JulianDate
     */
    JulianDate();

    /**
     * @brief
     * Constructor for JulianDate using a julian day and a fractional day
     * @param day The integer part of julian day
     * @param frac The decimal part of the Julian day
     */
    JulianDate(long day, float frac);

    /**
     * @brief 
     * Constructor for JulianDate using the calendar date
     * @param yr The year in XXXX format
     * @param mo The month (range [1,12])
     * @param d The day (range [1,31])
     * @param h The hours (range [0,23])
     * @param mi The minutes (range [0,59])
     * @param s The seconds (range [0,59])
     */
    JulianDate(int yr, int mo, int d, int h, int mi, float s);

// Getters and Setters
    /**
     * @brief
     * Gets the integer part of the julian day
     * @return The integer part of the julian day
     */
    long getDay();

    /**
     * @brief
     * Gets the decimal part of the julian day
     * @return The decimal part of the julian day
     */
    float getFrac();

    /**
     * @brief
     * Gets the integer part of the julian day
     * @return The integer part of the julian day
     */
    void setDay(long day);

    /**
     * @brief
     * Gets the decimal part of the julian day
     * @return The decimal part of the julian day
     */
    void setFrac(float frac);

// Specific methods
    /** 
     * @brief
     * Update the Julian Date by adding the provided seconds
     * @param seconds The amount of seconds to add
     */
    void update(float seconds);

private:
    long _day;
    float _frac;
};

/**
 * @brief
 * Provide an orbit model that can return essential parameters
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
class Orbit {
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
     * @brief
     * Gets the Julian Date member of the orbit
     * @return The Julian Date
     */
    JulianDate getJulianDate();

    /**
     * @brief
     * Sets the Julian Date member of the orbit
     * @param date The new date
     */
    void setJulianDate(JulianDate date);

// Sun position
    /**
     * @brief
     * Provides the Sun-vector in the ECI frame for the currently stored Julian date
     * @param rsun The array where to store the sun vector
     */
    void getSunVector(float rsun[3]);

    /**
     * @brief
     * Provides the Sun-vector in the ECI frame for a given Julian Date (in AU)
     * @param rsun The array where to store the sun vector
     * @param date A JulianDate object representing the desired time
     */
    void getSunVector(float rsun[3], JulianDate date);

// Spacecraft position management
    /**
     * @brief
     * Set the orbit parameters
     * @param axis The semi major axis of the orbit (m)
     * @param ecc The eccentricity of the orbit
     * @param inc The orbit plane inclination (rad)
     * @param Omega The right ascension node (rad)
     * @param omega The argument of perigee (rad)
     * @param theta The true anomaly (rad)
     */
    void setOrbit(float axis, float ecc, float inc, float Omega, float omega, float theta);

    /**
     * @brief
     * Set the obirt parameters
     * @param parameters The array with the orbital parameters (units: m and rad)
     * [axis, ecc, inc, right ascension, argument of perigee, true anomaly]
     */
    void setOrbit(float parameters[6]);

    /**
     * @brief
     * Update the mean anomaly at the current Julian date
     * @param seconds The time since last update
     * @param tolerance The tolerance for the Newton optimization method
     */
    void updateTrueAnomaly(float seconds, float tolerance);

    /**
     * @brief
     * Provide the current position vector
     * @param r_sat The position vector of the satellite
     */
    void getPositionVector(float r_sat[3]);

// Earth magnetic field
    /**
     * @brief
     * Update the Earth magnetic field vector for a given orbit position in the ECI frame
     * @param mag The magnetic vector of the Earth magnetic field
     * @param r_sat The position vector of the satellite
     * @param jday The Julian day (The object julian day if default or 0)
     * @param jfrac The Julian day fraction (The object julian day fraction if default or 0)
     */
    void mag_vector(float mag[3], float r_sat[3], long jday, float jfrac);

    /**
     * @brief
     * Provides the Earth magnetic field vector according to the model
     * @param rmag The array where to store the magnetic field
     */
    void getMagVector(float rmag[3]);

// Others
    /**
     * @brief
     * Update the orbit to match the number of seconds ellapsed
     * @param seconds The number of seconds ellapsed since last update
     */
    void update(float seconds);

    /**
     * @brief
     * Converts the tuple (azimuth, elevation) into a position vector
     * in the North-East-Down (NED) reference frame
     * @param azimuth The azimuth of the target measured clockwise from North
     * @param elevation The elevation of the target from the horizon
     * @param vec The array where to store the position vector
     */
    static void AzEl2NED(float azimuth, float elevation, float vec[3]);
       
private:
    /**
     * @brief
     * Returns the norm of a vector
     * @param vec The vector from which to calculate the norm
     * @return The norm of the vector
     */
    float norm(float vec[3]);
    
    /**
     * @brief
     * Performs the scalar product of two vector
     * a.b = transpose(a)*b
     * @param a First vector
     * @param b Second vector
     * @return The result of transpose(a)*b
     */
    float scalar(float *a, float *b);
    
    /* Private variables */
    // Date
    JulianDate _date;
    // orbit parameters
    float axis_;    // Semi major axis (m)
    float ecc_;     // Eccentricity
    float inc_;     // Orbital plane inclination (rad)
    float Omega_;   // Right ascension node longitude (rad)
    float omega_;   // Argument perigee (rad)
    float M_;       // Mean anomaly
    float E_;       // Eccentric anomaly (rad)
    float theta_;   // True anomaly (rad)
    // orbit computed-once variables 
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

/**
 * @brief
 * Provide a ground version of the Orbit class
 * 
 * @class Ground
 * 
 * @details
 * Provides Sun and Earth magnetic field direction vectors in the North-East-Down
 * reference frame to allow ground testing
 * 
 */
class Ground {
public:
// Constructors
    /**
     * Ground default Constructor
     */
    Ground();

    /**
     * Ground default Destructor
     */
    ~Ground(void);

// Date management
    /**
     * @brief
     * Gets the Julian Date member of the orbit
     * @return The Julian Date
     */
    JulianDate getJulianDate();

    /**
     * @brief
     * Sets the Julian Date member of the orbit
     * @param date The new date
     */
    void setJulianDate(JulianDate date);

// Sun position
    /**
     * @brief
     * Provides the azimuth and elevation of the sun
     * @param azel The array where to store [azimuth, elevation]
     */
    void getSunAzEl(float azel[2]);

    /**
     * @brief
     * Provides the Sun-vector in the ECI frame for the currently stored Julian date
     * @param rsun The array where to store the sun vector
     */
    void getSunVector(float rsun[3]);

// Spacecraft position management
    /**
     * @brief
     * Set the orbit parameters
     * @param lat_deg The latitute of the site (in decimal degrees, North is positive)
     * @param lon_deg The longitude of the site (in decimal degrees, East is positive)
     * @param alt The altitude of the site (in m)
     * @param magN The North component of the Earth magnetic field (in uT)
     * @param magE The East component of the Earth magnetic field (in uT)
     * @param magD The Down component of the Earth magnetic field (in uT)
     */
    void setOrbit(float lat_deg, float lon_deg, float alt, float magN, float magE, float magD);

    /**
     * @brief
     * Provide the current position vector
     * @param rsat The position vector of the satellite in m
     */
    void getPositionVector(float rsat[3]);

// Earth magnetic field
    /**
     * @brief
     * Provides the Earth magnetic field vector according to the model
     * @param rmag The array where to store the magnetic field
     */
    void getMagVector(float rmag[3]);

// Others
    /**
     * @brief
     * Update the orbit to match the number of seconds ellapsed
     * @param seconds The number of seconds ellapsed since last update
     */
    void update(float seconds);

    /**
     * @brief
     * Converts the tuple (azimuth, elevation) into a position vector
     * in the North-East-Down (NED) reference frame
     * @param azimuth The azimuth of the target measured clockwise from North (rad)
     * @param elevation The elevation of the target from the horizon (rad)
     * @param vec The array where to store the position vector
     */
    static void AzEl2NED(float azimuth, float elevation, float vec[3]);

private:
    // Private members
    JulianDate _date;
    float _lat, _lon, _alt; // Latitude, longitude and altitude of the ground site (in rad)
    float _rmag[3];

    /* Math and orbital mechanics constants */
    const float pi = 3.1415926535f;
    const float twopi = 6.283185307f;
    const float deg2rad = pi/180.0f;
    const float sec2jFrac = 1 / (60*60*24);
    const double mu = 398600441800000.0; // Earth gravitational constant (m3/s2)
    const float omega_earth = 0.000072921158f; // Earth rotation pulsation periode
    const float r_earth = 6378000.0f;     // Earth radius in m
}; // class Ground

} // namespace AstroLib

#endif // ASTROLIB_H