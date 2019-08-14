#include "SunSensor.test.h"

int SunSensorTest(){
    SunSensor sun(A0,A1,A2);
    Timer t;
    t.start();

    float rsun_b[3];
    float rsun_e[3];

    int last_update  = t.read_us();
    int print_update = t.read_ms();
    int ellapsed;

    printf("\n\r\n\r\n\r\n\r\n\r\n\r--------------------------------------\n\r");
    printf("Connection ok\n\r");

    while(1){
        last_update = t.read_us();
        //--------------------- LOOP ---------------------//
        sun.getSunVector(rsun_b);

        //------------------- END LOOP -------------------//
        ellapsed = t.read_us() - last_update;

        //-------------------- PRINT ---------------------//   
        if(t.read_ms() - print_update > 500){
            print_update = t.read_ms();
            printf("\n\r\n\rLoop time %d us | Frequency %4.0f Hz\n\r", ellapsed, 1000000.0f/ellapsed);
            printf("Sun:       ");
            printf("{% 4.2f, % 4.2f, % 4.2f}\n\r", rsun_b[0], rsun_b[1], rsun_b[2]);
        }

        if(t.read_ms() > 1<<21) {
            t.start(); // start the timer over again if ~30 minutes has passed
            last_update = t.read_us();
        }
        wait_us(20000);
    }

    return 1;
}