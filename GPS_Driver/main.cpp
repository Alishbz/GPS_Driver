 /**
 * ALI SAHBAZ
*
* 
*
* Date          : 10.06.2023
* By            : Ali Sahbaz.
* e-mail        : ali_sahbaz@outlook.com
 */
#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include <math.h>
#include <time.h>
#include "GPS_Modul_master.h"

/**

This modul can work like an active object -> GPS_Modul_Class

example for in hardware or real world;

void _1ms_timer(){
  GPS.Timer_Update();
}

void _uart_i2c_usb_or_others_receive_interrup(char byte){
  GPS.Receive_Serial(byte , 1);
}

void main( ){
  GPS.Initiation();

  while(1)
  {
    GPS.Tasks_Manager();    -> SUPER CONTROLLER
  }
}

**/

#define TEST_NMEA_STRING ((char*)"$GPRMC,210230,A,3855.4487,N,09446.0071,W,0.0,076.2,130495,003.8,E*69")

static GPS_Modul_Class GPS;

int main() {



    GPS.Initiation();

    GPS.Receive_Serial( TEST_NMEA_STRING,
                        strlen(TEST_NMEA_STRING));

    printf("GPS NMEA Parse Tester\n");
    printf(" \n");

    printf("Latitude : %f\n" , GPS.Latitude_Degree_Value);
    printf("Longitude : %f\n", GPS.Longitude_Degree_Value);
    printf("Satellites : %d\n", GPS.Satellites_Number);
    printf("Hdop : %f\n", GPS.hdop);

    return 0;
}

