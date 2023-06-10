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
#ifndef POLYGON_MAPING_H
#define POLYGON_MAPING_H

typedef enum
{
    NOT_IN_FIELD = 0x00U,
    IN_FIELD = 0x01U,
    MAP_LOAD_ERROR = 0x02U,
    TIMEOUTx = 0x03U,
    NO_LOADED_MAP = 0x04U,
} PolFuncStatus;

class PolygonClass
{

public:

    PolygonClass();

    void Initial_New_Map(double Points[28][2], unsigned int point_number); // point_number max 28 min 3

    PolFuncStatus isCoordinate_inPolgon_Control(double lat, double lon);

private:

    double Map_Field_Coordinates[28][2];   // lat , lon

    unsigned int Map_Field_Coordinates_Number;

};


#endif // POLYGON_MAPING_H
