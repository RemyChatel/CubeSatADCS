#include "Test.h"
#include "AstroLib.h"
#include "SunSensor.h"

#define YEAR 2006
#define MONTH 04
#define DAY 02
#define HOURS 00
#define MINUTES 00
#define SECONDS 00

using namespace SimpleMatrix;

int AstroLibTest(Serial *pc, I2C *i2c, Timer *t){
    
    pc->baud(115200);
    pc->printf("\n\r\n\r------------------------------\n\r");
    pc->printf("Connection OK\n\r");

    //SunSensor Test
    float sun_sat[3];
    AnalogIn faceX(A0);
    AnalogIn faceY(A1);
    AnalogIn faceZ(A3);
    SunSensor sun(&faceX, &faceY, &faceZ);
    // AstroLib test
    AstroLib::Orbit orbit;
    orbit.setJulianDate(YEAR,MONTH,DAY,HOURS,MINUTES,SECONDS);
    long day;
    float frac;
    float seconds;
    float sun_eci[3];
    int time;
    t->start();

    pc->printf("Objects created");
    
    while(1){
    time = t->read_us();
    
    orbit.getJulianDate(&day, &frac);
    orbit.updateJulianDate(1.0f);
    orbit.getJulianDate(&day, &frac);
    orbit.getSunVector(sun_eci, day, frac);
    sun.getSunVector(sun_sat);
    
    time = t->read_us()-time;
    
    pc->printf("\n\rExecution time %d us | frequency %ld Hz\n\r", time, (long)(1000000.0f/time));
    pc->printf("Calendar date %d-%d-%d-%d-%d-%f\n\r", YEAR, MONTH, DAY, HOURS, MINUTES, seconds);
    pc->printf("Julian Date %ld and %f\n\r", day, frac);
    pc->printf("Sun vector ephemeride: [%f , %f , %f] of Norm %f\n\r", sun_eci[0], sun_eci[1], sun_eci[2], sqrt(sun_eci[0]*sun_eci[0]+sun_eci[1]*sun_eci[1]+sun_eci[2]*sun_eci[2]));
    pc->printf("Sun vector satellite : [%f , %f , %f] of Norm %f\n\r", sun_sat[0], sun_sat[1], sun_sat[2], sqrt(sun_sat[0]*sun_sat[0]+sun_sat[1]*sun_sat[1]+sun_sat[2]*sun_sat[2]));
    seconds+=1;
    wait(1);
    }
    return 1;
}