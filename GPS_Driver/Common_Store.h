#pragma once
#ifndef COMMON_STORE_H
#define COMMON_STORE_H

/*************************************************************************
* ALI SAHBAZ
*
*
*
* Date          : 10.06.2023
* By            : Ali Sahbaz.
* e-mail        : ali_sahbaz@outlook.com
*
*************************************************************************/

#define myENABLE   1
#define myDISABLE   0

#define POWER_OFF   0
#define POWER_ON   1

#define CONNECTED       1
#define NOT_CONNECTED   0

#define MAX_TCP_PACKET_SIZE    1500
/*
#define UNLOCK    0
#define LOCK      1
*/

typedef struct
{
    unsigned char next_step;

    unsigned int  timeout;

}Delay_Str;


/** Func Out **/
typedef enum      
{
    mySUCCESS = 0x00U,
    myTRY_AGAIN = 0x01U,
    myBUSY = 0x02U,
    myTIMEOUT = 0x03U,
    myCLEAR = 0x04U,
    myERROR = 0x05U,
    myFAIL = 0x06U,
    myRESET = 0x07U,
    myNO_CARRIER = 0x08U,
    myTRUE_OUT = 0x09U,
    myFALCE_OUT = 0x10U
} funcStatus;




#endif  
