/*************************************************************************
* ALI SAHBAZ
*
*
*
* Date          : 10.06.2023
* By            : Ali Sahbaz.
* e-mail        : ali_sahbaz@outlook.com
*
*
*************************************************************************/
#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include <math.h>
#include <time.h>
#include "GPS_Modul_master.h"

#define GNSS_DISABLE  //hardware call interface
#define GNSS_ENABLE   //hardware call interface
#define RTC_UPDATED(time,date)   //hardware call interface, OR Software Timer Feed
 
#define HEADER_STEP                       0
#define START_LINE_DONE                   1
#define RECEIVE_PARAMS_CLEAR_TIMEOUT      2000     // 2000 ms if data not came in this time then clear Rcv_Str
#define BREAK_TIMEOUT                     200
#define CLEAR_ALL                         0
#define CLEAR_FLAGS                       1

char RMC[] = { "RMC" };
char GLL[] = { "GLL" };
char GGA[] = { "GGA" };
char GSV[] = { "GSV" };


/*** INTERFACE ***/
void GPS_Modul_Class::Fill_Location(double lat, double lon) {

    debug_manager();
}

void GPS_Modul_Class::debug_manager(void) {

    if (debug_timer > 5000) {
        debug_timer = 0;
#ifdef GPS_USB_LOG
        char data4[100]; char data3[100];
        snprintf((char*)data4, 100, (char*)"> Watch GPS: LAT: %f , LON: %f %s", Latitude_Degree_Value, Longitude_Degree_Value, (char*)"\r\n");
        USBc.transmit_queue_fill((unsigned char*)data4);
        snprintf((char*)data3, 100, (char*)"> Watch GPS: NumSatellite: %d , Hdop: %f %s", (unsigned short)Satellites_Number, hdop, (char*)"\r\n");
        USBc.transmit_queue_fill((unsigned char*)data3);
#endif
    }
}


void GPS_Modul_Class::Power_Control(void) {

    switch (power_step)
    {
    case 0:
        GNSS_DISABLE;
        power_step = 1;
        Power_Control_Timer = 0;
        break;

    case 1:
        if (Power_Control_Timer > 300)
        {
            GNSS_ENABLE;
            Power_Control_Timer = 0;
            power_step = 2;
        }
        break;


    case 2:
        // ??
        Power_Control_Timer = 0;

        break;

    default:
        power_step = 0;
        Power_Control_Timer = 0;
        break;
    }

}






void GPS_Modul_Class::Tasks_Manager(void) {

    Power_Control();

    Receive_Command_Check();
}

void GPS_Modul_Class::Timer_Update(void) {

    Rcv_Str.Clear_Timer++;

    Rcv_Str.Break_Timer++;

    Power_Control_Timer++;

    debug_timer++;

}

void GPS_Modul_Class::Initiation(void)
{
    memset((unsigned char*)Rcv_Str.Module_Buffer, 0, GPS_BUFFER_LENGTH);
    memset((unsigned char*)Rcv_Str.Command_Buffer, 0, GPS_BUFFER_LENGTH);
    Clear_Receive_Command_Check_Params(CLEAR_ALL);

    Power_Control_Timer = 0;
    memset((unsigned char*)Latitude_HexBuf, 0, 20);
    memset((unsigned char*)Longitude_HexBuf, 0, 20);
    Latitude_Value = 0;
    Longitude_Value = 0;
    Latitude_Degree_Value = 40.821464;
    Longitude_Degree_Value = 29.922830;
    Satellites_Number = 0;
    Gravitational_Force = 0;
    power_step = 0;
    crc = 0;
    hdop = 0.0;
    debug_timer = 0;
}

void GPS_Modul_Class::Receive_Serial(char* str, unsigned int lng) {
    unsigned int cnt = 0;
    while (cnt < lng) {
        Rcv_Str.Module_Buffer[Rcv_Str.MD_Write_Ptr++] = *(str + cnt);
        Rcv_Str.MD_Write_Ptr &= GPS_BUFFER_LENGTH;
        cnt++;
    }
}


funcStatus GPS_Modul_Class::Receive_Command_Check(void)
{
    funcStatus out = myBUSY;

    if (Rcv_Str.Clear_Timer > RECEIVE_PARAMS_CLEAR_TIMEOUT)
    {
        out = myCLEAR;
        memset((char*)Rcv_Str.Command_Buffer, 0, sizeof(Rcv_Str.Command_Buffer));
        Clear_Receive_Command_Check_Params(CLEAR_FLAGS);
        return out;
    }

    Rcv_Str.Break_Timer = 0;

    while (Rcv_Str.MD_Read_Ptr != Rcv_Str.MD_Write_Ptr)
    {
        if (Rcv_Str.Break_Timer > BREAK_TIMEOUT)
        {
            /** loop break and params clear **/
            out = myTIMEOUT;
            Clear_Receive_Command_Check_Params(CLEAR_FLAGS);
            break;
        }
        /** Fill The Command(cmd) Buff and check index size **/
        Rcv_Str.MD_Read_Ptr &= GPS_BUFFER_LENGTH;
        Rcv_Str.Record_Index &= GPS_BUFFER_LENGTH;
        Rcv_Str.Command_Buffer[Rcv_Str.Record_Index++] = Rcv_Str.Module_Buffer[Rcv_Str.MD_Read_Ptr++];
        Rcv_Str.MD_Read_Ptr &= GPS_BUFFER_LENGTH;
        Rcv_Str.Record_Index &= GPS_BUFFER_LENGTH;

        out = myBUSY;

        /*             PARSE             */

        /** Start Line **/
        if (Rcv_Str.Record_Index > 0 &&
            Rcv_Str.Command_Buffer[Rcv_Str.Record_Index - 1] == '$' &&
            Rcv_Str.Parse_Step == HEADER_STEP
            )
        {
            Rcv_Str.Parse_Step = START_LINE_DONE;

            Rcv_Str.Record_Index = 0;

            crc = 0;

            out = myBUSY;
        }
        else if (Rcv_Str.Parse_Step == START_LINE_DONE)
        {
            if (Rcv_Str.Command_Buffer[Rcv_Str.Record_Index - 1] == '*')
            {
                crc = Calculate_Checksum((char*)Rcv_Str.Command_Buffer, Rcv_Str.Record_Index - 2);
            }

            if (Rcv_Str.Command_Buffer[Rcv_Str.Record_Index - 1] == 0x0D && Rcv_Str.Record_Index > 2)
            {
                out = myFAIL;

                if ((unsigned char)(((ascii_To_Char(Rcv_Str.Command_Buffer[Rcv_Str.Record_Index - 3])) << 4) | (ascii_To_Char(Rcv_Str.Command_Buffer[Rcv_Str.Record_Index - 2]))) == crc)
                {
                    if (strstr((const char*)Rcv_Str.Command_Buffer, "GGA"))
                    {
                        out = mySUCCESS;
                    }

                    if (strstr((const char*)Rcv_Str.Command_Buffer, "RMC"))
                    {
                        out = mySUCCESS;
                    }

                    GPS_Parse();

                    out = mySUCCESS;
                }
                else
                {
                    out = myERROR;
                }

                Rcv_Str.Parse_Step = HEADER_STEP;

                Rcv_Str.Record_Index = 0;

                Clear_Receive_Command_Check_Params(CLEAR_FLAGS);

                break;
            }

        }



    }

    return out;
}

unsigned char GPS_Modul_Class::Calculate_Checksum(char* buffer, unsigned short lng)
{
    unsigned char checksum = 0;
    unsigned short i = 0;
    while (*buffer)
    {
        checksum ^= *buffer++;
        if (i == lng)
        {
            break;
        }
        i++;
    }

    return checksum;
}

void GPS_Modul_Class::Clear_Receive_Command_Check_Params(char clear_status)
{
    if (clear_status == CLEAR_ALL) {
        Rcv_Str.MD_Read_Ptr = 0;
        Rcv_Str.MD_Write_Ptr = 0;
    }
    Rcv_Str.Record_Index = 0;
    Rcv_Str.Clear_Timer = 0;
    Rcv_Str.Break_Timer = 0;
    Rcv_Str.Parse_Step = HEADER_STEP;
}

unsigned char GPS_Modul_Class::ascii_To_Char(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;

    return 0;
}


char GPS_Modul_Class::GPS_Parse(void)
{
    unsigned char state, loc_is_came = 0;

    if (NMEA_Parse_With_Location(GLL, Rcv_Str.Command_Buffer, 8, &state))
    {
        if (state == 'A')
        {
            NMEA_Parse_With_Location(GLL, Rcv_Str.Command_Buffer, 2, Latitude_HexBuf);
            NMEA_Parse_With_Location(GLL, Rcv_Str.Command_Buffer, 4, Longitude_HexBuf);

            if (Longitude_HexBuf[0] > 0 && Latitude_HexBuf[0] > 0)
            {
                //Açi hesabi
                Latitude_Value = hexArray_To_Double((char*)Latitude_HexBuf);
                Longitude_Value = hexArray_To_Double((char*)Longitude_HexBuf);
                Latitude_Degree_Value = NMEA_Convert_Degree(Latitude_Value);
                Longitude_Degree_Value = NMEA_Convert_Degree(Longitude_Value);

                //North South - East West kontrolü
                NMEA_Parse_With_Location(GLL, Rcv_Str.Command_Buffer, 3, &state);
                if (state == 'S')  Latitude_Degree_Value = Latitude_Degree_Value * (-1);
                NMEA_Parse_With_Location(GLL, Rcv_Str.Command_Buffer, 5, &state);
                if (state == 'W')  Longitude_Degree_Value = Longitude_Degree_Value * (-1);

                loc_is_came = 1;
            }
        }
    }
    else if (NMEA_Parse_With_Location(RMC, Rcv_Str.Command_Buffer, 3, &state))
    {
        if (state == 'A')
        {
            NMEA_Parse_With_Location(RMC, Rcv_Str.Command_Buffer, 4, Latitude_HexBuf);
            NMEA_Parse_With_Location(RMC, Rcv_Str.Command_Buffer, 6, Longitude_HexBuf);

            if (Longitude_HexBuf[0] > 0 && Latitude_HexBuf[0] > 0)
            {
                //Açi hesabi
                Latitude_Value = hexArray_To_Double((char*)Latitude_HexBuf);
                Longitude_Value = hexArray_To_Double((char*)Longitude_HexBuf);
                Latitude_Degree_Value = NMEA_Convert_Degree(Latitude_Value);
                Longitude_Degree_Value = NMEA_Convert_Degree(Longitude_Value);

                //North South - East West kontrolü
                NMEA_Parse_With_Location(RMC, Rcv_Str.Command_Buffer, 5, &state);
                if (state == 'S')  Latitude_Degree_Value = Latitude_Degree_Value * (-1);
                NMEA_Parse_With_Location(RMC, Rcv_Str.Command_Buffer, 7, &state);
                if (state == 'W')  Longitude_Degree_Value = Longitude_Degree_Value * (-1);

                loc_is_came = 1;
            }
        }
    }
    else if (NMEA_Parse_With_Location(GGA, Rcv_Str.Command_Buffer, 7, &state))
    {
        if (state != '0')
        {
            uint8_t time_row_array[10] = "123519.00";
            unsigned char Numsatellites[2], holder[5];

            /*** SATELLITES NUMBER  ***/

            NMEA_Parse_With_Location(GGA, Rcv_Str.Command_Buffer, 8, Numsatellites);

            Satellites_Number = (unsigned char)hexArray_To_Double((char*)Numsatellites);

            /*** TIME  ***/

            NMEA_Parse_With_Location(GGA, Rcv_Str.Command_Buffer, 2, (unsigned char*)time_row_array);

            Timer_Converter_Process((unsigned char*)time_row_array);

            /*** LAT LON  ***/

            NMEA_Parse_With_Location(GGA, Rcv_Str.Command_Buffer, 3, Latitude_HexBuf);
            NMEA_Parse_With_Location(GGA, Rcv_Str.Command_Buffer, 5, Longitude_HexBuf);

            if (Longitude_HexBuf[0] > 0 && Latitude_HexBuf[0] > 0)
            {
                //Degree
                Latitude_Value = hexArray_To_Double((char*)Latitude_HexBuf);
                Longitude_Value = hexArray_To_Double((char*)Longitude_HexBuf);
                Latitude_Degree_Value = NMEA_Convert_Degree(Latitude_Value);
                Longitude_Degree_Value = NMEA_Convert_Degree(Longitude_Value);

                //North South - East West kontrolü
                NMEA_Parse_With_Location(GGA, Rcv_Str.Command_Buffer, 4, &state);
                if (state == 'S')  Latitude_Degree_Value = Latitude_Degree_Value * (-1);
                NMEA_Parse_With_Location(GGA, Rcv_Str.Command_Buffer, 6, &state);
                if (state == 'W')  Longitude_Degree_Value = Longitude_Degree_Value * (-1);

                loc_is_came = 1;
            }

            /** HTOP **/

            NMEA_Parse_With_Location(GGA, Rcv_Str.Command_Buffer, 9, (unsigned char*)holder);

            hdop = hexArray_To_Double((char*)holder);

        }
    }
    else if (NMEA_Parse_With_Location(GSV, Rcv_Str.Command_Buffer, 2, &state))
    {
        Satellites_Number = (unsigned char)atoi((const char*)&state);

        if (Satellites_Number > 2)        // 2 den fazla uyduya bagli ise
        {
            unsigned char Gra_Force[2];
            NMEA_Parse_With_Location(GSV, Rcv_Str.Command_Buffer, 18, Gra_Force);                         // ÖLÇÜM KALITESI
            Gravitational_Force = (unsigned char)hexArray_To_Double((char*)Gra_Force);
        }
    }

    if (loc_is_came)
    {
        Fill_Location(Latitude_Degree_Value, Longitude_Degree_Value);
    }


    return 0;
}

/*
******************************************************************************************************
" NMEA_Parse_With_Location "
type[] 			-> RMC GLL GGA
parse_Buffer 	-> parse edilecek array
loca 				-> parse edilecek datanin yeri, 2 virgül arasina bakilir
result_Buffer 	-> bulunan data bu arraye atilir
******************************************************************************************************
*/

char GPS_Modul_Class::NMEA_Parse_With_Location(char type[], unsigned char* parse_Buffer, unsigned short loca, unsigned char* result_Buffer)
{
    char rtrn = 0;

    unsigned short location = loca - 1;

    unsigned char i = 0;

    unsigned char comma_Count = 0;

    unsigned char en_flag = 0;

    if (strncmp(type, (char*)(parse_Buffer + 2), 3) == 0)
    {
        while (*(parse_Buffer + i) != 0)
        {
            if (*(parse_Buffer + i) == ',')
            {
                comma_Count++;
            }

            if (comma_Count == location && en_flag == 0)
            {
                unsigned char t = 1;

                while (*(parse_Buffer + i + t) != ',')
                {
                    result_Buffer[t - 1] = *(parse_Buffer + i + t);
                    t++;
                    en_flag = 1;
                    rtrn = 1;
                    if (*(parse_Buffer + i + t) == '*' || t > 15)  break;   //koruma
                }
            }

            i++;
            if (i > 150) break; //koruma
        }
    }

    return rtrn;
}

double GPS_Modul_Class::hexArray_To_Double(char* buffer)
{
    double rtrn = 0;
    char i = 0;
    char comma_state = 0;
    double indx = 0.1;
    int  count = 0;

    while (*(buffer + i))
    {
        if (*(buffer + i) != '.')
        {
            if (comma_state == 0)
            {
                count++;
            }
            else
            {
                rtrn = rtrn + ascii_To_Char(*(buffer + i)) * indx;
                indx = indx / 10;
            }
        }
        else
        {
            comma_state = 1;
        }

        i++;
    }

    indx = 1.0;

    while (count > 0)
    {
        count--;
        rtrn = rtrn + ascii_To_Char(*(buffer + count)) * indx;
        indx = indx * 10;
        if (count > 10 || count < 0) break;                 //koruma
    }

    return rtrn;
}

double GPS_Modul_Class::NMEA_Convert_Degree(double data)
{
    double value = 0.0;

    value = (double)(((data - (double)((int)(data / 100.0) * 100.0)) / 60.0) + (int)(data / 100.0));

    return value;
}


void GPS_Modul_Class::Timer_Converter_Process(unsigned char* time_row)
{
    unsigned char holder[5], time_hour; uint8_t timex[16] = "hh:ms:ss";

    holder[0] = time_row[0]; holder[1] = time_row[1];

    time_hour = (unsigned char)hexArray_To_Double((char*)holder);

    time_hour = time_hour + 3; if (time_hour > 24) { time_hour = time_hour - 24; }

    time_row[1] = (time_hour % 10) + '0';

    time_row[0] = (time_hour / 10) + '0';

    timex[0] = time_row[0];
    timex[1] = time_row[1];
    timex[2] = ':';
    timex[3] = time_row[2];
    timex[4] = time_row[3];
    timex[5] = ':';
    timex[6] = time_row[4];
    timex[7] = time_row[5];

    RTC_UPDATED((uint8_t*)timex, NULL);
}
