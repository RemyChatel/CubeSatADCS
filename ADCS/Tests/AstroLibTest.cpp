#include "Test.h"
#include "AstroLib.h"
#include "SunSensor.h"
#define DEG2RAD 3.1415926535f/180.0f

void normalize(float vec[3]);

int AstroLibTest(Serial *pc, I2C *i2c, Timer *t){

    using namespace AstroLib;
    
    pc->baud(115200);
    pc->printf("\n\r\n\r------------------------------\n\r");
    pc->printf("Connection OK\n\r");

    int year        = 2019;//2006;
    int month       = 06;//04;
    int day         = 07;//02;
    int hours       = 17;//00; // /!\ Use GMT+00 !!!!
    int minutes     = 15;//00;
    float seconds   = 00;

    JulianDate date;
    Ground orbit;
    orbit.setJulianDate(JulianDate(year,month,day,hours,minutes,seconds));
    // orbit.setOrbit(6378000  + ( 418000.0f / (1 + 0.0007873f) ) , 0.0007873, 51.6420 * DEG2RAD, 49.6761 * DEG2RAD, 19.9151 * DEG2RAD, 340.2306 * DEG2RAD);
    orbit.setOrbit(55.86515, -4.25763, 0.0f, 17.3186f, -.6779f, 46.8663f);
    
    float sun_eci[3];
    float mag_eci[3];
    float sat_eci[3];
    float azel_sun[2];

    // Input here the Earth magnetic field at location  (https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm)
    // mag_ned = {17.3186f, -.6779f, 46.8663f}; //uT in North-East-Down ref
    // Input here the sun Azimuth and Elevation (https://www.esrl.noaa.gov/gmd/grad/solcalc/)
    // Ground::AzEl2NED(117.15f*DEG2RAD, 43.7f*DEG2RAD, sun_ned); // direction in North-East-Down ref

    int time;
    t->start();

    pc->printf("Objects created");
    
    while(1){
    time = t->read_us();

    orbit.update(1.0f);
    orbit.getSunVector(sun_eci);
    orbit.getMagVector(mag_eci);

    normalize(mag_eci); // Normalize to obtain the direction vector
    normalize(sun_eci); // Normalize to obtain the direction vector
    
    time = t->read_us()-time;
    
    orbit.getPositionVector(sat_eci);
    orbit.getSunAzEl(azel_sun);
    normalize(sat_eci); // Normalize to obtain the direction vector

    pc->printf("\n\rExecution time %d us | frequency %ld Hz\n\r", time, (long)(1000000.0f/time));
    pc->printf("Calendar date %d-%d-%d-%d-%d-%f\n\r", year,month,day,hours,minutes,seconds);
    pc->printf("Julian Date %ld and %f\n\r", orbit.getJulianDate().getDay(), orbit.getJulianDate().getFrac());
    pc->printf("Satelite position [% 1.6e, % 1.6e, % 1.6e]\n\r", sat_eci[0], sat_eci[1], sat_eci[2]);
    pc->printf("Sun vector ephemeride (AU): [% f , % f , % f]\n\r", sun_eci[0], sun_eci[1], sun_eci[2]);
    pc->printf("Mag vector ephemeride: [% f , % f , % f]\n\r", mag_eci[0], mag_eci[1], mag_eci[2]);
    pc->printf("AzEl Sun: [% f, % f]\n\r", azel_sun[0]*180/3.1415926535f, azel_sun[1]*180/3.1415926535f);

    seconds+=1;
    if(seconds >= 60){
        seconds = 0;
        minutes++;
    }
    if(minutes >= 60){
        minutes = 0;
        hours++;
    }
    wait(1);
    }
    return 1;
}

void normalize(float vec[3]){
    float norm = sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
    vec[0] /= norm;
    vec[1] /= norm;
    vec[2] /= norm;
}