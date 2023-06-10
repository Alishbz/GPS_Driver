#pragma once
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
#ifndef GPS_MODUL_MASTER_H
#define GPS_MODUL_MASTER_H
#include "Common_Store.h"
#include "Polygon_Maping.h"

#define GPS_BUFFER_LENGTH               0x1FF

/** Manage Data Queue Str **/
typedef struct
{
    unsigned char Command_Buffer[GPS_BUFFER_LENGTH];

    unsigned char Module_Buffer[GPS_BUFFER_LENGTH];

    unsigned int  MD_Read_Ptr;

    unsigned int  MD_Write_Ptr;

    unsigned int  Clear_Timer;

    unsigned int  Break_Timer;

    unsigned int Record_Index;

    unsigned char Parse_Step;

}GPS_Queue;

class GPS_Modul_Class
{

public:

    void Initiation(void);

    void Timer_Update(void);

    void Tasks_Manager(void);

    void Receive_Serial(char* str, unsigned int lng);

    /****************************** GPS PARAMS ***********************************/
    /*****************************************************************************/
    double Latitude_Degree_Value;

    double Longitude_Degree_Value;

    unsigned char Satellites_Number;

    double hdop;                      // Horizontal dilution of position
    /*****************************************************************************/

private:

    void debug_manager(void);

    void Timer_Converter_Process(unsigned char* time_row);

    unsigned char crc;

    char power_step;

    GPS_Queue Rcv_Str;

    funcStatus Receive_Command_Check(void);

    PolygonClass Polygon;

    void Clear_Receive_Command_Check_Params(char clear_status);

    unsigned char Calculate_Checksum(char* buffer, unsigned short lng);

    double hexArray_To_Double(char* buffer);

    unsigned char ascii_To_Char(char c);

    unsigned char Latitude_HexBuf[20];

    unsigned char Longitude_HexBuf[20];

    double Latitude_Value;

    double Longitude_Value;

    unsigned char Gravitational_Force;

    char GPS_Parse(void);

    void Fill_Location(double lat, double lon);

    double NMEA_Convert_Degree(double data);

    char NMEA_Parse_With_Location(char type[], unsigned char* parse_Buffer, unsigned short loca, unsigned char* result_Buffer);

    void Power_Control(void);

    unsigned int Power_Control_Timer;

    unsigned int debug_timer;
};
 

#endif // GPS_MODUL_MASTER_H
