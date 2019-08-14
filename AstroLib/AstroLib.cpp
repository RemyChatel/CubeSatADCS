/*******************************************************************************
 * @file AstroLib.cpp
 * @version 1.0
 * @date 2019
 * @author Remy CHATEL
 * @copyright GNU Public License v3.0
 * @brief Provides orbital mechanics tools for spacecrafts
 * 
 * @see AstroLib.h
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

#include "AstroLib.h"

using namespace AstroLib;
// ------------------- Julian Date -------------------//
// Constructors
    JulianDate::JulianDate():_day(0),_frac(0){}

    JulianDate::JulianDate(long day, float frac):_day(day),_frac(frac){
        // check that the day and fractional day are correct
        if (fabs(_frac) >= 1.0f) {
            float dtt = floor(_frac);
            _day  += dtt;
            _frac -= dtt;
        }
    }

    JulianDate::JulianDate(int yr, int mo, int d, int h, int mi, float s){
        _day  =   367 * yr -
                        (long)floor((7 * (yr + floor((mo + 9) / 12.0f))) * 0.25f) +
                        (long)floor(275 * mo / 9.0f) +
                        d + 1721013;  // alternatively use - 678987.0 to go to mjd directly
        _frac =   (s + mi * 60.0f + h * 3600.0f) / 86400.0f + 0.5f;
        
        // check that the day and fractional day are correct
        if (fabs(_frac) >= 1.0f) {
            float dtt = floor(_frac);
            _day  += dtt;
            _frac -= dtt;
        }
    }

// Operators
    void JulianDate::operator+=(const JulianDate& rhs){
        this->_day +=  rhs._day;
        this->_frac += rhs._frac;
        if (fabs(this->_frac) >= 1.0f) {
            float dtt = floor(this->_frac);
            this->_day  += dtt;
            this->_frac -= dtt;
        }
    }

    void JulianDate::operator+=(float rhs){
        this->_frac += rhs;
        if (fabs(this->_frac) >= 1.0f) {
            float dtt = floor(this->_frac);
            this->_day  += dtt;
            this->_frac -= dtt;
        }
    }

    void JulianDate::operator+=(int rhs){
        this->_day += rhs;
    }

    void JulianDate::operator+=(long rhs){
        this->_day += rhs;
    }

    JulianDate operator+(const JulianDate& lhs, const JulianDate& rhs){
        JulianDate tmp = lhs;
        tmp += rhs;
        return tmp;
    }

    JulianDate operator+(const JulianDate& lhs, float rhs){
        JulianDate tmp = lhs;
        tmp += rhs;
        return tmp;
    }

    JulianDate operator+(const JulianDate& lhs, int rhs){
        JulianDate tmp = lhs;
        tmp += rhs;
        return tmp;
    }

    JulianDate operator+(const JulianDate& lhs, long rhs){
        JulianDate tmp = lhs;
        tmp += rhs;
        return tmp;
    }

    void JulianDate::operator-=(const JulianDate& rhs){
        this->_day -=  rhs._day;
        this->_frac -= rhs._frac;
        if (fabs(this->_frac) >= 0.0f) {
            this->_frac += 1.0f;
            this->_day  -= 1;
        }
    }

    void JulianDate::operator-=(float rhs){
        this->_frac -= rhs;
        if (fabs(this->_frac) >= 0.0f) {
            this->_frac += 1.0f;
            this->_day  -= 1;
        }
    }

    void JulianDate::operator-=(int rhs){
        this->_day -= rhs;
    }

    void JulianDate::operator-=(long rhs){
        this->_day -= rhs;
    }

    JulianDate operator-(const JulianDate& lhs, const JulianDate& rhs){
        JulianDate tmp = lhs;
        tmp -= rhs;
        return tmp;
    }

    JulianDate operator-(const JulianDate& lhs, float rhs){
        JulianDate tmp = lhs;
        tmp -= rhs;
        return tmp;
    }

    JulianDate operator-(const JulianDate& lhs, int rhs){
        JulianDate tmp = lhs;
        tmp -= rhs;
        return tmp;
    }

    JulianDate operator-(const JulianDate& lhs, long rhs){
        JulianDate tmp = lhs;
        tmp -= rhs;
        return tmp;
    }

    int JulianDate::operator==(const JulianDate& rhs) const{
        if((this->_day == rhs._day) && (this->_frac == rhs._frac)){
            return 1;
        } else {
            return 0;
        }
    }

    int JulianDate::operator!=(const JulianDate& rhs) const{
        return 1 - (*this == rhs);
    }

    int JulianDate::operator<(const JulianDate& rhs) const{
        if (this->_day < rhs._day){
            return 1;
        } else if (this->_day == rhs._day && this->_frac < rhs._frac) {
            return 1;
        } else {
            return 0;
        }
    }

    int JulianDate::operator<=(const JulianDate& rhs) const{
        if (this->_day < rhs._day){
            return 1;
        } else if (this->_day == rhs._day && (this->_frac == rhs._frac)){
            return 1;
        } else if (this->_day == rhs._day && this->_frac < rhs._frac) {
            return 1;
        } else {
            return 0;
        }
    }

    int JulianDate::operator>(const JulianDate& rhs) const{
        if (this->_day > rhs._day){
            return 1;
        } else if (this->_day == rhs._day && this->_frac > rhs._frac) {
            return 1;
        } else {
            return 0;
        }
    }

    int JulianDate::operator>=(const JulianDate& rhs) const{
        if (this->_day > rhs._day){
            return 1;
        } else if (this->_day == rhs._day && (this->_frac == rhs._frac)){
            return 1;
        } else if (this->_day == rhs._day && this->_frac > rhs._frac) {
            return 1;
        } else {
            return 0;
        }
    }

    int JulianDate::operator==(float rhs) const{
        if((this->_day == floor(rhs)) && (this->_frac == rhs - floor(rhs))){
            return 1;
        } else {
            return 0;
        }
    }

    int JulianDate::operator!=(float rhs) const{
        return 1 - (*this == rhs);
    }

    int JulianDate::operator<(float rhs) const{
        if (this->_day < floor(rhs)){
            return 1;
        } else if (this->_day == floor(rhs) && this->_frac < rhs - floor(rhs)) {
            return 1;
        } else {
            return 0;
        }
    }

    int JulianDate::operator<=(float rhs) const{
        if (this->_day < floor(rhs)){
            return 1;
        } else if (this->_day == floor(rhs) && this->_frac == rhs - floor(rhs)){
            return 1;
        } else if (this->_day == floor(rhs) && this->_frac < rhs - floor(rhs)){
            return 1;
        } else {
            return 0;
        }
    }

    int JulianDate::operator>(float rhs) const{
        if (this->_day > floor(rhs)){
            return 1;
        } else if (this->_day == floor(rhs) && this->_frac > rhs - floor(rhs)) {
            return 1;
        } else {
            return 0;
        }
    }

    int JulianDate::operator>=(float rhs) const{
        if (this->_day > floor(rhs)){
            return 1;
        } else if (this->_day == floor(rhs) && this->_frac == rhs - floor(rhs)){
            return 1;
        } else if (this->_day == floor(rhs) && this->_frac > rhs - floor(rhs)){
            return 1;
        } else {
            return 0;
        }
    }
    
    JulianDate::operator float() const{
        return (float)this->_day + this->_frac;
    }

    JulianDate::operator int() const{
        return (int)this->_day + (int)roundf(this->_frac);
    }

    JulianDate::operator long() const{
        return (long)this->_day + (long)roundf(this->_frac);
    }

// Getters and Setters
    long JulianDate::getDay() const {return _day;}

    float JulianDate::getFrac() const {return _frac;}

    void JulianDate::setDay(long day){_day = day;}

    void JulianDate::setFrac(float frac){
        _frac = frac;
        // check that the day and fractional day are correct
        if (fabs(_frac) >= 1.0f) {
            float dtt = floor(_frac);
            _day  += dtt;
            _frac -= dtt;
        }
    }

// Specific methods
    void JulianDate::update(float seconds){
        _frac += seconds / (86400.0f);
        if (fabs(_frac) >= 1.0f) {
            float dtt = floor(_frac);
            _day  += dtt;
            _frac -= dtt;
        }
    }


// ------------------- Orbit -------------------//
// Constructors
    Orbit::Orbit():_date(JulianDate()){}
    
// Date management
    JulianDate Orbit::getJulianDate(){
        return _date;
    }

    void Orbit::setJulianDate(JulianDate date){
        _date = date;
    }

// Sun position
    void Orbit::getSunVector(float rsun[3]){
        getSunVector(rsun, _date);
    }

    void Orbit::getSunVector(float rsun[3], JulianDate date){
        // Algorithm based on Vallado's "Fundamentals of astrodynamics and applications"
        float t_ut1;        // 
        float lambda_m;     // Mean Solar Longitude
        float m_sol;        // Mean Solar Anomaly
        float lambda_e;     // Eclipse Longitude
        float r_sol;        // Sun distance
        float epsilon;      // Obliquity
        
        t_ut1 = ((float)(date.getDay()-2451545) + date.getFrac())/36525.0f;
        
        lambda_m = 280.460f + 36000.771f * t_ut1;       // In degree
        lambda_m = fmod(lambda_m, 360.0f);              // Keeping degree for now
        
        m_sol = 357.5291092f + 35999.05034f * t_ut1;    // In degree
        m_sol = fmod(m_sol*DEG2RAD, TWOPI);             // Converting to rad
        m_sol += (m_sol<0.0f)?TWOPI:0;                  // Keeping in [0,2pi]
        
        lambda_e = lambda_m + 1.914666471f * sin(m_sol) + 0.019994643f * sin(2.0f*m_sol);
                                                        // In degree (with m_sol in rad)
        lambda_e *= DEG2RAD;                            // Converting to rad
        
        epsilon = (23.439291f - 0.0130042f * t_ut1) * DEG2RAD;
                                                        // In rad
        
        lambda_m *= DEG2RAD;                            // Convert to rad
        lambda_m += (lambda_m<0)?TWOPI:0;               // Keeping in [0,2pi]
        
        r_sol = 1.000140612f - 0.016708617f * cos(m_sol) - 0.000139589f * cos(2.0f*m_sol);
                                                        // In AU
        
        rsun[0] = r_sol * cos(lambda_e);
        rsun[1] = r_sol * cos(epsilon) * sin(lambda_e);
        rsun[2] = r_sol * sin(epsilon) * sin(lambda_e);
    }

// Spacecraft position management
    void Orbit::setOrbit(float axis, float ecc, float inc, float Omega, float omega, float theta){
        axis_   = axis;
        ecc_    = ecc;
        inc_    = inc;
        Omega_  = Omega;
        omega_  = omega;
        theta_ = theta;
        
        rate = sqrt(MU/(axis_*axis_*axis_));
        float orbitForm = axis_*(1-ecc_*ecc_);
        eccRatio = sqrt((1+ecc_)/(1-ecc_));

        float cOM = cos(Omega_);
        float sOM = sin(Omega_);
        float co = cos(omega_);
        float so = sin(omega_);
        float ci = cos(inc_);
        float si = sin(inc_);

        rotECI[0] = orbitForm * (cOM*co-sOM*so*ci);
        rotECI[1] = orbitForm * (-cOM*so-sOM*co*ci);
        //rotECI[2] = orbitForm * (sOM*si);           // Can be commented to skip
        rotECI[3] = orbitForm * (sOM*co+cOM*so*ci);
        rotECI[4] = orbitForm * (-sOM*so+cOM*co*ci);
        //rotECI[5] = orbitForm * (-cOM*si);          // Can be commented to skip
        rotECI[6] = orbitForm * (si*so);
        rotECI[7] = orbitForm * (si*co);
        //rotECI[8] = orbitForm * (ci);               // Can be commented to skip
    }

    void Orbit::setOrbit(float parameters[6]){
        setOrbit(parameters[0], parameters[1], parameters[2], parameters[3], parameters[4], parameters[5]);
    }

    void Orbit::updateTrueAnomaly(float seconds, float tolerance = 1e-5){
        // Calculating Mean Anomaly
        M_ += rate * seconds;

        // Updating Eccentric Anomaly (Newton's optimizer)
        float last_E = E_;
        int iter = 1000;
        while(fabs(last_E-E_) > tolerance && iter < 1000){
            E_ = E_ - (E_ - ecc_ * sin(E_) - M_) / (1 - ecc_ * cos(E_));
            iter++;
        }

        // Updating True Anomaly
        theta_ = 2* atan(eccRatio * tan(E_/2));
    }

    void Orbit::getPositionVector(float r_sat[3]){
        float cTheta = cos(theta_);
        float sTheta = sin(theta_);\
        float r0 = cTheta / (1 + ecc_ * cTheta);
        float r1 = sTheta / (1 + ecc_ * cTheta);
        r_sat[0] = rotECI[0] * r0 + rotECI[1] * r1;
        r_sat[1] = rotECI[3] * r0 + rotECI[4] * r1;
        r_sat[2] = rotECI[6] * r0 + rotECI[7] * r1;
    }


// Earth Magnetic field
    void Orbit::mag_vector(float mag[3], float r_sat[3], long jday, float jfrac){
        // Based on Virginia Tech Course AEO4140
        const float H_0 = 0.30115f;             // Earth magnetic constant in Gauss
        float ut1;                              // T UT1
        float theta_g;                          // Greenwich sideral time (rad)
        const float phi_m = 108.43f*DEG2RAD;    // East longitude of the dipole (rad)
        const float theta_m = 196.54f*DEG2RAD;  // Coelevation of the dipole (rad)
        float mag_d[3];                         // Unit dipole direction

        // Greenwich mean sideral time (adapted from Vallado's "Fundamentals of astrodynamics and applications" )
        ut1 = ((jday-2451545) + jfrac) / 36525.0f;
        theta_g = 4.894961212f + 229964.595f * ut1 + (float)(6.7707139449e-6) * ut1 * ut1 - (float)(4.5087672343186846e-10) * ut1 * ut1 * ut1;
        theta_g = fmod(theta_g, TWOPI);
        theta_g += (theta_g<0)?TWOPI:0;
        
        // Magnetic dipole calculation
        mag_d[0] = sin(theta_m)*cos(theta_g + phi_m);
        mag_d[1] = sin(theta_m)*sin(theta_g + phi_m);
        mag_d[2] = cos(theta_m);
        
        
        float r_relative = R_EARTH/Orbit::norm(r_sat);   // Ratio of R_earth on R_sat
        float magr = H_0 * r_relative * r_relative * r_relative;  // Commun multiplier of all lines
        float scal = 3 * scalar(mag_d, r_sat);              //  transpose(mag_d)*r_sat
        
        mag[0] = magr * ( scal * r_sat[0] - mag_d[0] );
        mag[1] = magr * ( scal * r_sat[1] - mag_d[1] );
        mag[2] = magr * ( scal * r_sat[2] - mag_d[2] );
        
    }

    void Orbit::getMagVector(float rmag[3]){
        float rsat[3];
        getPositionVector(rsat);
        mag_vector(rmag, rsat, _date.getDay(), _date.getFrac());
    }

// Private methods and others

    void Orbit::update(float seconds){
        _date.update(seconds);
        updateTrueAnomaly(seconds);
    }

    float Orbit::norm(float vec[3]){
        return (float)sqrt(vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2]);
    }

    float Orbit::scalar(float *a, float *b){
        return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
    }

    void Orbit::AzEl2NED(float azimuth, float elevation, float vec[3]){
        vec[2]      =     - sin(elevation);
        float hyp   =       cos(elevation);
        vec[0]      = hyp * cos(azimuth);
        vec[1]      = hyp * sin(azimuth);
    }

// ------------------- Ground -------------------//
// Constructors
    Ground::Ground():_date(JulianDate()){}

// Date management
    JulianDate Ground::getJulianDate(){
        return _date;
    }

    void Ground::setJulianDate(JulianDate date){
        _date = date;
    }

// Sun position
    void Ground::getSunAzEl(float azel[2]){
        // Adapted from David Brooks, Institute for Earth Science Research and Education
        // Equations from Jean Meeus, Astronomical Algorithms
        
        float JD_frac,L0,M,e,C,L_true,f,R,GrHrAngle,Obl,RA,Decl,HrAngle;
        long JD_whole, T;

        // Separate the date for computation and find the Modified Julian Date
        JD_whole=_date.getDay();
        JD_frac=_date.getFrac();
        T=JD_whole-2451545;
        T=(T+JD_frac)/36525.0f;

        // Compute the direction vector of the sun in the Earth Centered Inertial frame
        L0=DEG2RAD*fmod(280.46645+36000.76983*T,360);
        M=DEG2RAD*fmod(357.5291+35999.0503*T,360);
        e=0.016708617-0.000042037*T;
        C=DEG2RAD*((1.9146-0.004847*T)*sin(M)+(0.019993-0.000101*T)*sin(2*M)+0.00029*sin(3*M));
        f=M+C;
        Obl=DEG2RAD*(23+26/60.+21.448/3600.-46.815/3600*T);

        // Convert sun direction vector to the local frame
        GrHrAngle=280.46061837+(360*T)%360+.98564736629*T+360.98564736629*JD_frac;
        GrHrAngle=fmod(GrHrAngle,360.0f);
        L_true=fmod(C+L0,TWOPI);
        R=1.000001018*(1-e*e)/(1+e*cos(f));
        RA=atan2(sin(L_true)*cos(Obl),cos(L_true));
        Decl=asin(sin(Obl)*sin(L_true));
        HrAngle=DEG2RAD*GrHrAngle+_lon-RA;

        // Convert to Azimuth and Elevation
        azel[0] = PI+atan2(sin(HrAngle),cos(HrAngle)*sin(_lat)-tan(Decl)*cos(_lat));
        azel[1] = asin(sin(_lat)*sin(Decl)+cos(_lat)*(cos(Decl)*cos(HrAngle)));
    }

    void Ground::getSunVector(float rsun[3]){
        float azel[2];
        getSunAzEl(azel);
        AzEl2NED(azel[0], azel[1], rsun);
    }
// Spacecraft position management
    void Ground::setOrbit(float lat_deg, float lon_deg, float alt, float magN, float magE, float magD){
        _rmag[0] = magN;
        _rmag[1] = magE;
        _rmag[2] = magD;

        _lat = lat_deg * DEG2RAD;
        _lon = lon_deg * DEG2RAD;
        _alt = alt;
    }

    void Ground::setOrbit(float parameters[6]){
        setOrbit(parameters[0], parameters[1], parameters[2], parameters[3], parameters[4], parameters[5]);
    }

    void Ground::getPositionVector(float rsat[3]){
        // From Converting between Earth-Centered, Earth Fixed and Geodetic Coordinates, D. Rose 
        float a = 6378137.0; // WGS-84 semi-major axis
        float e2 = 6.6943799901377997e-3; // WGS-84 first eccentricity squared
        float n = a/sqrt( 1 - e2*sin( _lat )*sin( _lat ) );

        rsat[0] = ( n + _alt )*cos( _lat )*cos( _lon );    //ECEF x
        rsat[1] = ( n + _alt )*cos( _lat )*sin( _lon );    //ECEF y
        rsat[2] = ( n*(1 - e2 ) + _alt )*sin( _lat );        //ECEF z
    }

// Earth magnetic field
    void Ground::getMagVector(float rmag[3]){
        rmag[0] = _rmag[0];
        rmag[1] = _rmag[1];
        rmag[2] = _rmag[2];
    }

// Others
    void Ground::update(float seconds){
        _date.update(seconds);
    }

    void Ground::AzEl2NED(float azimuth, float elevation, float vec[3]){
        float hyp   =       cos(elevation);
        vec[0]      = hyp * cos(azimuth);
        vec[1]      = hyp * sin(azimuth);
        vec[2]      =     - sin(elevation);
    }