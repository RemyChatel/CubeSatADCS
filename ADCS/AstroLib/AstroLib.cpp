/***************************************************************************//**
 * @file AstroLib.cpp
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2019 </b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

#include "AstroLib.h"

using namespace AstroLib;

Orbit::Orbit(){
}

Orbit::~Orbit(void){
    
}

void Orbit::getJulianDate(long *day, float *frac){
    *day  = julianday_;
    *frac = julianfrac_;
}
 
void Orbit::setJulianDate(int yr, int mo, int d, int h, int mi, float s){  
    julianday_  =   367 * yr -
                    (long)floor((7 * (yr + floor((mo + 9) / 12.0f))) * 0.25f) +
                    (long)floor(275 * mo / 9.0f) +
                    d + 1721013;  // alternatively use - 678987.0 to go to mjd directly
    julianfrac_ =   (s + mi * 60.0f + h * 3600.0f) / 86400.0f + 0.5f;
    
    // check that the day and fractional day are correct
    if (fabs(julianfrac_) >= 1.0f)
    {
        float dtt = floor(julianfrac_);
        julianday_ = julianday_ + dtt;
        julianfrac_ = julianfrac_ - dtt;
    }
    
}

void Orbit::setJulianDate(long jd, float jfrac){
    julianday_ = jd;
    julianfrac_ = jfrac;
}

void Orbit::updateJulianDate(float seconds){
    julianfrac_ += seconds / (60*60*24);
    if(julianfrac_ >= 1.0f){
        julianday_++;
        julianfrac_ -= 1.0f;
    }
}

void Orbit::getSunVector(float rsun[3], long julianday, float julianfrac){
    // Algorithm based on Vallado's "Fundamentals of astrodynamics and applications"
    float t_ut1;        // 
    float lambda_m;     // Mean Solar Longitude
    float m_sol;        // Mean Solar Anomaly
    float lambda_e;     // Eclipse Longitude
    float r_sol;        // Sun distance
    float epsilon;      // Obliquity
    
    t_ut1 = ((float)(julianday-2451545) + julianfrac)/36525.0f;
    
    lambda_m = 280.460f + 36000.771f * t_ut1;       // In degree
    lambda_m = fmod(lambda_m, 360.0f);              // Keeping degree for now
    
    m_sol = 357.5291092f + 35999.05034f * t_ut1;    // In degree
    m_sol = fmod(m_sol*deg2rad, twopi);             // Converting to rad
    m_sol += (m_sol<0.0f)?twopi:0;                  // Keeping in [0,2pi]
    
    lambda_e = lambda_m + 1.914666471f * sin(m_sol) + 0.019994643f * sin(2.0f*m_sol);
                                                    // In degree (with m_sol in rad)
    lambda_e *= deg2rad;                            // Converting to rad
    
    epsilon = (23.439291f - 0.0130042f * t_ut1) * deg2rad;
                                                    // In rad
    
    lambda_m *= deg2rad;                            // Convert to rad
    lambda_m += (lambda_m<0)?twopi:0;               // Keeping in [0,2pi]
    
    r_sol = 1.000140612f - 0.016708617f * cos(m_sol) - 0.000139589f * cos(2.0f*m_sol);
                                                    // In AU
    
    rsun[0] = r_sol * cos(lambda_e);
    rsun[1] = r_sol * cos(epsilon) * sin(lambda_e);
    rsun[2] = r_sol * sin(epsilon) * sin(lambda_e);
}

void Orbit::setOrbit(float axis, float ecc, float inc, float Omega, float omega, float theta){
    axis_   = axis;
    ecc_    = ecc;
    inc_    = inc;
    Omega_  = Omega;
    omega_  = omega;
    theta_ = theta;
    
    rate = sqrt(mu/(axis_*axis_*axis_));
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

void Orbit::mag_vector(float mag[3], float r_sat[3], long jday = 0, float jfrac = 0){
    // Based on Virginia Tech Course AEO4140
    const float H_0 = 0.30115f;             // Earth magnetic constant in Gauss
    float ut1;                              // T UT1
    float theta_g;                          // Greenwich sideral time (rad)
    const float phi_m = 108.43f*deg2rad;    // East longitude of the dipole (rad)
    const float theta_m = 196.54f*deg2rad;  // Coelevation of the dipole (rad)
    float mag_d[3];                         // Unit dipole direction

    // Greenwich mean sideral time (adapted from Vallado's "Fundamentals of astrodynamics and applications" )
    ut1 = ((((jday==0)?julianday_:jday)-2451545) + ((jfrac==0)?julianfrac_:jfrac)) / 36525.0f;
    theta_g = 4.894961212f + 229964.595f * ut1 + (float)(6.7707139449e-6) * ut1 * ut1 - (float)(4.5087672343186846e-10) * ut1 * ut1 * ut1;
    theta_g = fmod(theta_g, twopi);
    theta_g += (theta_g<0)?twopi:0;
    
    // Magnetic dipole calculation
    mag_d[0] = sin(theta_m)*cos(theta_g + phi_m);
    mag_d[1] = sin(theta_m)*sin(theta_g + phi_m);
    mag_d[2] = cos(theta_m);
    
    
    float r_relative = r_earth/Orbit::norm(r_sat);   // Ratio of R_earth on R_sat
    float magr = H_0 * r_relative * r_relative * r_relative;  // Commun multiplier of all lines
    float scal = 3 * scalar(mag_d, r_sat);              //  transpose(mag_d)*r_sat
    
    mag[0] = magr * ( scal * r_sat[0] - mag_d[0] );
    mag[1] = magr * ( scal * r_sat[1] - mag_d[1] );
    mag[2] = magr * ( scal * r_sat[2] - mag_d[2] );
    
}

float Orbit::norm(float vec[3]){
    return (float)sqrt(vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2]);
}

float Orbit::scalar(float *a, float *b){
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}

void Orbit::quat2rot(float quat[4], float rot_coef[9]){
    float qw = quat[3], qx = quat[0], qy = quat[1], qz = quat[2];
    float sqw = qw*qw;
    float sqx = qx*qx;
    float sqy = qy*qy;
    float sqz = qz*qz;

    // invs (inverse square length) is only required if quaternion is not already normalised
    float invs = 1 / (sqx + sqy + sqz + sqw);
    rot_coef[0] = ( sqx - sqy - sqz + sqw)*invs ; // since sqw + sqx + sqy + sqz =1/invs*invs
    rot_coef[4] = (-sqx + sqy - sqz + sqw)*invs ;
    rot_coef[8] = (-sqx - sqy + sqz + sqw)*invs ;
    
    double tmp1 = qx*qy;
    double tmp2 = qz*qw;
    rot_coef[3] = 2.0 * (tmp1 + tmp2)*invs ;
    rot_coef[1] = 2.0 * (tmp1 - tmp2)*invs ;
    
    tmp1 = qx*qz;
    tmp2 = qy*qw;
    rot_coef[6] = 2.0 * (tmp1 - tmp2)*invs ;
    rot_coef[2] = 2.0 * (tmp1 + tmp2)*invs ;
    tmp1 = qy*qz;
    tmp2 = qx*qw;
    rot_coef[7] = 2.0 * (tmp1 + tmp2)*invs ;
    rot_coef[5] = 2.0 * (tmp1 - tmp2)*invs ;
}

void Orbit::AzEl2Vec(float azimuth, float elevation, float vec[3]){
    vec[0] = sin(elevation);
    float hyp = cos(elevation);
    vec[1] = hyp * cos(azimuth);
    vec[2] = hyp * sin(azimuth);
}