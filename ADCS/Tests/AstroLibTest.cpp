#include "Test.h"
#include "AstroLib.h"
#include "SunSensor.h"
#define DEG2RAD 3.1415926535f/180.0f

int AstroLibTest(Serial *pc, I2C *i2c, Timer *t){

    using namespace AstroLib;
    
    pc->baud(115200);
    pc->printf("\n\r\n\r------------------------------\n\r");
    pc->printf("Connection OK\n\r");

    int year = 2006;
    int month = 04;
    int day = 02;
    int hours = 00;
    int minutes = 00;
    float seconds = 00;

    JulianDate date;
    Orbit orbit;
    orbit.setJulianDate(JulianDate(year,month,day,hours,minutes,seconds));
    orbit.setOrbit(6378000  + ( 418000.0f / (1 + 0.0007873f) ) , 0.0007873, 51.6420 * DEG2RAD, 49.6761 * DEG2RAD, 19.9151 * DEG2RAD, 340.2306 * DEG2RAD);
    float sun_eci[3];
    float mag_eci[3];
    float sat_eci[3];
    int time;
    t->start();

    pc->printf("Objects created");
    
    while(1){
    time = t->read_us();

    orbit.update(1.0f);
    orbit.getSunVector(sun_eci);
    orbit.getPositionVector(sat_eci);
    orbit.getMagVector(mag_eci);
    
    time = t->read_us()-time;
    
    pc->printf("\n\rExecution time %d us | frequency %ld Hz\n\r", time, (long)(1000000.0f/time));
    pc->printf("Calendar date %d-%d-%d-%d-%d-%f\n\r", year,month,day,hours,minutes,seconds);
    pc->printf("Julian Date %ld and %f\n\r", orbit.getJulianDate().getDay(), orbit.getJulianDate().getFrac());
    pc->printf("Satelite position [% 1.6e, % 1.6e, % 1.6e]\n\r", sat_eci[0], sat_eci[1], sat_eci[2]);
    pc->printf("Sun vector ephemeride: [% f , % f , % f] of Norm %f\n\r", sun_eci[0], sun_eci[1], sun_eci[2], sqrt(sun_eci[0]*sun_eci[0]+sun_eci[1]*sun_eci[1]+sun_eci[2]*sun_eci[2]));
    pc->printf("Mag vector ephemeride: [% f , % f , % f] of Norm %f\n\r", mag_eci[0], mag_eci[1], mag_eci[2], sqrt(mag_eci[0]*mag_eci[0]+mag_eci[1]*mag_eci[1]+mag_eci[2]*mag_eci[2]));
    
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