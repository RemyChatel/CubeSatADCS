/**
 * @file AstroLib.h
 * @defgroup AstroLibGr AstroLib
 * @version 1.0
 * @date 2019
 * @author Remy CHATEL
 * @copyright GNU Public License v3.0
 * 
 * @brief
 * Provides orbital mechanics tools for spacecrafts
 * 
 * @details
 * # Description
 * ## Overview
 * This library provides tools for orbital mechanics for embeded C++ aiming
 * at spacecraft orbiting Earth.
 * 
 * Provides an orbit model, handling of Julian date, Sun position and
 * Earth magnetic field model
 * 
 * This library also provides a "ground" version of the orbital model
 * to allow ground testing with the same front end
 * 
 * Features of the library:
 * - Julian date time format and conversion from common calendar,
 * - Orbit model based on perifocal parameters,
 * - Spacecraft position vector in the ECI (Earth Centered Inertial) frame,
 * - Sun vector in the ECI frame,
 * - Earth Magnetic Field vector in the ECI frame.
 * 
 * @see AstroLib::JulianDate
 * @see AstroLib::Orbit
 * @see AstroLib::Ground
 * 
 * ## Detailed description
 * The AstroLib module provides tools for orbital mechanics. It implements
 * an orbit model, a representation of Julian date, a Sun position model
 * and an Earth magnetic field model.
 * This library also provides a _ground_ version of the orbital model
 * to allow ground testing with the same front end.
 * 
 * It features a Julian date class that allows date manipulation, addition
 * and subtraction as well as conversion from the common calendar date. The
 * conversion is established by the formula given by:
 * 
 * @f{align}{
 *   JD & = 367(y r)-\operatorname{INT}\left\{\frac{7\left(y r+\operatorname{INT}\left(\frac{m o+9}{12}\right)\right\}}{4}\right\}+\operatorname{INT}\left(\frac{275 mo}{9}\right)+d \nonumber \\
 *        & + 1,721,013.5+\frac{1}{24}\left(\frac{s}{3600} + \frac{min}{60} + h \right)
 *   \label{equ:julian}
 * @f}
 * 
 * The Orbit class provide a model of an orbit that can return the spacecraft
 * position, the sun direction vector and the Earth magnetic field vector, for
 * a given orbit set by its perifocal parameters.
 * The formula to compute the Sun vector is given by Vallado's book:
 * 
 * @f{align}{
 *   T_{U T 1} & = \frac{J D_{U T 1}-2,451,545.0}{36,525} \nonumber \\
 *   \lambda_{M_{\odot}} & = 280.460^{\circ}+36,000.771 T_{U T 1} \nonumber \\
 *   M_{\odot} & = 357.5291092^{\circ}+35,999.05034 T_{U T 1} \nonumber \\ \quad \nonumber \\
 *   \lambda_{\text{ecliptic}} & = \lambda_{M_{\odot}}+1.914666471^{\circ} \sin\left( M_{\odot} \right) +0.019994643 \sin \left( 2 M_{\odot} \right) \nonumber \\
 *   r_{\odot} & = 1.000140612-0.016708617 \cos \left(M_{\odot}\right)-0.000139589 \cos \left( 2 M_{\odot} \right) \nonumber \\
 *   \epsilon & = 23.439291^{\circ}-0.0130042 T_{U T 1} \nonumber \\ \quad \nonumber\\
 *   \mathbf{r}_{\odot} & = \left[\begin{array}{c}{r_{\odot} \cos \left(\lambda_{\text {ecliptic}}\right)} \\ {r_{\odot} \cos (\epsilon) \sin\left(\lambda_{\text {ecliptic}}\right)} \\ {r_{\odot} \sin(\epsilon) \sin\left(\lambda_{\text {ecliptic}}\right)}\end{array}\right] \mathrm{AU}
 *   \label{equ:sunvec}
 * @f}
 * 
 * To calculate the position of the satellite, the perifocal parameters are used to
 * determine the Cartesian vector of the spacecraft position in the ECI frame.
 * 
 * To compute the theoretical Earth magnetic field, the following formula
 * can be used:
 * 
 * @f{align}{
 * \alpha_{m} & = \theta_{g 0}+\omega_{\oplus} t+\phi_{m}^{\prime} \nonumber\\
 * \mathbf{d}_{ECI} & =
 * \left[\begin{array}{c}{
 *   \sin \theta_{m}^{\prime} \cos \alpha_{m}} \\
 *   {\sin \theta_{m}^{\prime} \sin \alpha_{m}} \\
 *  {\cos \theta_{m}^{\prime}}
 * \end{array}\right] \nonumber\\
 * \mathbf{m}_{ECI} & =\frac{R_{\oplus}^{3} H_{0}}{r^{3}}\left[3 \mathbf{d}_{i}^{\top} \hat{\mathbf{r}}_{i} \hat{\mathbf{r}}_{i}-\mathbf{d}_{i}\right]
 * \label{equ:magvec}
 * @f}
 * 
 * Where @f$\hat{\mathbf{r}_i}@f$ is the unit vector of the spacecraft position,
 *       @f$\mathbf{d}_i@f$ is the dipole unit direction vector,
 *       @f$ R_{\oplus} = 6378 km @f$,
 *       @f$ H_0 = 30.115 \mu T @f$
 * 
 * The ground version is a drop-in replacement for the Orbit class that facilitates
 * ground testing in a lab.
 * 
 * It can be initialized with the local value of the Earth magnetic field and the
 * site location to provide the sun vector.
 * 
 * All model are clearly indicated in the code in a separate function, and therefore
 * they can be easily changed to accommodate other requirements and be improved.
 * 
 * # Dependencies and data type
 * This library depends on <std::cmath> in order to perform 
 * cos, sin and sqrt operations.
 * 
 * @attention This library uses float (32-bits) and not double (64-bits)
 * to make best use of the Floating Point Unit of 32-bits microcontrollers.
 * 
 * # Example code
 * 
 * @see AstroLib.test.cpp
 * 
 * # References
 * - "Spacecraft Dynamic and Control: An introduction",
 * by A. de Ruiter, C. Damaren and J Forbes
 * - "Fundamentals of Astrodynamics and Applications", by D. Vallado
 * - http://www.instesre.org/ArduinoUnoSolarCalculations.pdf by David Brooks
 * - https://www.ngdc.noaa.gov/geomag/WMM/image.shtml
 * - https://www.esrl.noaa.gov/gmd/grad/solcalc/
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
#include <cmath>                     ///< <std::math> for square root and trigonometric functions

#define PI 3.1415926535f             ///< The number PI
#define TWOPI 6.283185307f           ///< The number 2*PI
#define DEG2RAD 3.1415926535f/180.0f ///< Conversion from degrees to radians
#define RAD2DEG 180.0f/3.1415926535f ///< Conversion from radians to degrees
#define SEC2JFRAC 1 / (60*60*24)     ///< Conversion from seconds to julian fraction
#define MU 398600441800000.0f        ///< Gravitational constant of the Earth
#define OMEGA_EARTH 0.000072921158f  ///< Rotation speed of the Earth
#define R_EARTH 6378000.0f           ///< Radius of the Earth

/**
 * @{
 * @brief Provides orbital mechanics tools for spacecrafts
 * 
 * @namespace AstroLib
 * 
 * @details
 * This namespace holds the classes for Orbit, Ground and JulianDate
 * 
 * @see AstroLib.h
 * 
 * # Dependencies and data type
 * This library depends on <std::cmath> in order to perform 
 * cos, sin and sqrt operations.
 * @}
 */
namespace AstroLib{
/**
 * @ingroup AstroLibGr 
 * @brief
 * A representation of the Julian Date
 * 
 * @class JulianDate
 * 
 * @details
 * Provides a storage and method to store and manipulate Julian Dates.
 * 
 * The Julian day is a calendar that starts at noon (12:00UTC) on January 1st, 4713 BC a date at which 
 * three multi-year cycles started (which are: Indiction, Solar, and Lunar cycles)
 * and which preceded any dates in recorded history.
 * 
 * The julian day is split in an integer part (stored as a 32-bits 'long' type)
 * and a decimal part (stored as a 32-bits 'float' type) to allow more significant
 * figures (precision of 9ms)
 *
 * This class supports addition and substraction of JulianDates with themselves, float, int and long,
 * as well as comparison between JulianDates themselves and between JulianDates and float. Finally this class
 * supports the cast of a JulianDate to int, long and float.
 * 
 * To compare JulianDates with long or int, please cast the value to float.
 * 
 * @see AstroLib
 */
class JulianDate {
public:
///@{
///@name Constructors

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
     * Constructor for JulianDate using the common calendar date
     * @param yr The year in XXXX format
     * @param mo The month (range [1,12])
     * @param d The day (range [1,31])
     * @param h The hours (range [0,23])
     * @param mi The minutes (range [0,59])
     * @param s The seconds (range [0,59.9999])
     */
    JulianDate(int yr, int mo, int d, int h, int mi, float s);


///@name Operators

    /**
     * @brief
     * Overload the += operator for two Julian dates
     * @param rhs The right hand side date
     * @return The sum of the two Julian dates
     */
    void operator+=(const JulianDate& rhs);
    
    /**
     * @brief
     * Overload the += operator for two Julian dates
     * @param rhs The right hand side date in float type
     * @return The sum of the two Julian dates
     */
    void operator+=(float rhs);

    /**
     * @brief
     * Overload the += operator for two Julian dates
     * @param rhs The right hand side date in int type
     * @return The sum of the two Julian dates
     */
    void operator+=(int rhs);

    /**
     * @brief
     * Overload the += operator for two Julian dates
     * @param rhs The right hand side date in long type
     * @return The sum of the two Julian dates
     */
    void operator+=(long rhs);

    /**
     * @brief
     * Overload the + operator for two Julian dates
     * @param lhs The left hand side date
     * @param rhs The right hand side date
     * @return The sum of the two Julian dates
     */
    friend JulianDate operator+(const JulianDate& lhs, const JulianDate& rhs);

    /**
     * @brief
     * Overload the + operator for two Julian dates
     * @param lhs The left hand side date
     * @param rhs The right hand side date in float type to add
     * @return The sum of the two Julian dates
     */
    friend JulianDate operator+(const JulianDate& lhs, float rhs);

    /**
     * @brief
     * Overload the + operator for two Julian dates
     * @param lhs The left hand side date
     * @param rhs The right hand side date in int type to add
     * @return The sum of the two Julian dates
     */
    friend JulianDate operator+(const JulianDate& lhs, int rhs);

    /**
     * @brief
     * Overload the + operator for two Julian dates
     * @param lhs The left hand side date
     * @param rhs The right hand side date in float long to add
     * @return The sum of the two Julian dates
     */
    friend JulianDate operator+(const JulianDate& lhs, long rhs);

    /**
     * @brief
     * Overload the -= operator for two Julian dates
     * @param rhs The right hand side date
     * @return The substraction of the two Julian dates
     */
    void operator-=(const JulianDate& rhs);
    
    /**
     * @brief
     * Overload the -= operator for two Julian dates
     * @param rhs The right hand side date in float type
     * @return The substraction of the two Julian dates
     */
    void operator-=(float rhs);

    /**
     * @brief
     * Overload the += operator for two Julian dates
     * @param rhs The right hand side date in int type
     * @return The substraction of the two Julian dates
     */
    void operator-=(int rhs);

    /**
     * @brief
     * Overload the += operator for two Julian dates
     * @param rhs The right hand side date in long type
     * @return The substraction of the two Julian dates
     */
    void operator-=(long rhs);

    /**
     * @brief
     * Overload the - operator for two Julian dates
     * @param lhs The left hand side date
     * @param rhs The right hand side date
     * @return The sum of the two Julian dates
     */
    friend JulianDate operator-(const JulianDate& lhs, const JulianDate& rhs);

    /**
     * @brief
     * Overload the - operator for two Julian dates
     * @param lhs The left hand side date
     * @param rhs The right hand side date in float type to add
     * @return The sum of the two Julian dates
     */
    friend JulianDate operator-(const JulianDate& lhs, float rhs);

    /**
     * @brief
     * Overload the + operator for two Julian dates
     * @param lhs The left hand side date
     * @param rhs The right hand side date in int type to add
     * @return The sum of the two Julian dates
     */
    friend JulianDate operator-(const JulianDate& lhs, int rhs);

    /**
     * @brief
     * Overload the + operator for two Julian dates
     * @param lhs The left hand side date
     * @param rhs The right hand side date in float long to add
     * @return The sum of the two Julian dates
     */
    friend JulianDate operator-(const JulianDate& lhs, long rhs);

    /**
     * @brief
     * Comparator operator for Julian Dates
     * @param rhs The right hand side date
     */
    int operator==(const JulianDate& rhs) const;

    /**
     * @brief
     * Different operator for Julian Dates
     * @param rhs The right hand side date
     */
    int operator!=(const JulianDate& rhs) const;

    /**
     * @brief
     * Lesser than operator for Julian Dates
     * @param rhs The right hand side date
     */
    int operator<(const JulianDate& rhs) const;

    /**
     * @brief
     * Lesser than or equal operator for Julian Dates
     * @param rhs The right hand side date
     */
    int operator<=(const JulianDate& rhs) const;

    /**
     * @brief
     * Greater than operator for Julian Dates
     * @param rhs The right hand side date
     */
    int operator>(const JulianDate& rhs) const;

    /**
     * @brief
     * Greater than or equal operator for Julian Dates
     * @param rhs The right hand side date
     */
    int operator>=(const JulianDate& rhs) const;

    /**
     * @brief
     * Comparator operator for Julian Dates
     * @param rhs The right hand side date
     */
    int operator==(float rhs) const;

    /**
     * @brief
     * Different operator for Julian Dates
     * @param rhs The right hand side date
     */
    int operator!=(float rhs) const;

    /**
     * @brief
     * Lesser than operator for Julian Dates
     * @param rhs The right hand side date
     */
    int operator<(float rhs) const;

    /**
     * @brief
     * Lesser than or equal operator for Julian Dates
     * @param rhs The right hand side date
     */
    int operator<=(float rhs) const;

    /**
     * @brief
     * Greater than operator for Julian Dates
     * @param rhs The right hand side date
     */
    int operator>(float rhs) const;

    /**
     * @brief
     * Greater than or equal operator for Julian Dates
     * @param rhs The right hand side date
     */
    int operator>=(float rhs) const;

    /**
     * @brief
     * Cast the Julian date to float
     */
    operator float() const;

    /**
     * @brief
     * Cast the Julian date to int
     */
    operator int() const;

    /**
     * @brief
     * Cast the Julian date to long
     */
    operator long() const;


///@name Getters or Setters
    /**
     * @brief
     * Gets the integer part of the julian day
     * @return The integer part of the julian day
     */
    long getDay() const;

    /**
     * @brief
     * Gets the decimal part of the julian day
     * @return The decimal part of the julian day
     */
    float getFrac() const;

    /**
     * @brief
     * Sets the integer part of the julian day
     * @return The integer part of the julian day
     */
    void setDay(long day);

    /**
     * @brief
     * Sets the decimal part of the julian day
     * @return The decimal part of the julian day
     */
    void setFrac(float frac);


///@name Specific methods
    /** 
     * @brief
     * Update the Julian Date by adding the provided seconds
     * @param seconds The amount of seconds to add
     */
    void update(float seconds);
///@}
private:
    /** @brief The integer part of the Julian day */
    long _day;
     /** @brief The decimal part of the Julian day */
    float _frac;
};

/**
 * @ingroup AstroLibGr 
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
///@name Constructors
    /**
     * @brief AstroLib constructor
     */
    Orbit();

///@name Date management
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
    
    /**
     * @brief
     * Update the orbit to match the number of seconds ellapsed
     * @param seconds The number of seconds ellapsed since last update
     */
    void update(float seconds);

///@name Sun position
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

///@name Spacecraft position management
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

///@name Earth magnetic field
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

///@name Static methods
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

    // orbit parameters
    JulianDate _date;   ///< @brief The current Julian Date on the orbit
    float axis_;        ///< Semi major axis (m)
    float ecc_;         ///< Eccentricity
    float inc_;         ///< Orbital plane inclination (rad)
    float Omega_;       ///< Right ascension node longitude (rad)
    float omega_;       ///< Argument perigee (rad)
    float M_;           ///< Mean anomaly
    float E_;           ///< Eccentric anomaly (rad)
    float theta_;       ///< True anomaly (rad)

    // orbit computed-once variables 
    float rate;         ///< Mean angular rate (rad/s)
    float eccRatio;     ///< = sqrt((1+ecc)/(1-ecc))
    float rotECI[9];    ///< Rotation from perifocal to ECI frame
}; // class Orbit

/**
 * @ingroup AstroLibGr 
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
///@name Constructors
    /**
     * Ground default Constructor
     */
    Ground();

///@name Date management
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

    /**
     * @brief
     * Update the orbit to match the number of seconds ellapsed
     * @param seconds The number of seconds ellapsed since last update
     */
    void update(float seconds);

///@name Sun position
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

///@name Spacecraft position management
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
     * Set the orbit parameters
     * @param parameters The array with the orbital parameters (units: m, /!\ deg and uT)
     * [latitude, longitude, altitude, mag_North, mag_East, mag_Down]
     */
    void setOrbit(float parameters[6]);


    /**
     * @brief
     * Provide the current position vector
     * @param rsat The position vector of the satellite in m
     */
    void getPositionVector(float rsat[3]);

///@name Earth magnetic field
    /**
     * @brief
     * Provides the Earth magnetic field vector according to the model
     * @param rmag The array where to store the magnetic field
     */
    void getMagVector(float rmag[3]);

///@name Static methods
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
    JulianDate _date;       ///< The current Julian date of the orbit
    float _lat, _lon, _alt; ///< Latitude, longitude and altitude of the ground site (in rad)
    float _rmag[3];         ///< The magnetic field at the location (in uT)
}; // class Ground

} // namespace AstroLib

#endif // ASTROLIB_H