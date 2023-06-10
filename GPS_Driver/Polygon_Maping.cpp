/*************************************************************************
* ALI SAHBAZ
*
*
*
* Date          : 10.06.2023
* By            : Ali Sahbaz.
* e-mail        : ali_sahbaz@outlook.com
*
* How to use(?) ;

-Load the polygon map
    map[number][0] = latitude
    map[number][1] = longitude
    Initial_New_Map(map,number)

- Find the point is it inside the map or not

   if(isCoordinate_inPolgon_Control(latPoint , lonPoint) == IN_FIELD)
   or
   if(isCoordinate_inPolgon_Control(latPoint , lonPoint) == NOT_IN_FIELD)

*************************************************************************/


#include <stdio.h>
#include <string.h>
#include "Polygon_Maping.h"


PolygonClass::PolygonClass()
{
    Map_Field_Coordinates_Number = 0;
}

void PolygonClass::Initial_New_Map(double Points[28][2], unsigned int point_number)
{
    unsigned int i = 0;

    if (point_number > 2)
    {
        for (i = 0; i < 28; i++)
        {
            if (i < point_number)
            {
                Map_Field_Coordinates[i][0] = Points[i][0];
                Map_Field_Coordinates[i][1] = Points[i][1];
            }
            else
            {
                Map_Field_Coordinates[i][0] = 0;
                Map_Field_Coordinates[i][1] = 0;
            }
        }
    }
    else {
        memset((double*)Map_Field_Coordinates, 0, 28);	 //else the Map_Field_Coordinates
    }

    Map_Field_Coordinates_Number = point_number;

}

PolFuncStatus PolygonClass::isCoordinate_inPolgon_Control(double lat, double lon)
{
    if (Map_Field_Coordinates_Number > 28 || Map_Field_Coordinates_Number < 3)
    {
        return MAP_LOAD_ERROR;
    }

    unsigned int num = Map_Field_Coordinates_Number;

    unsigned int j = num - 1, i, c = 1;

    double x = lat, y = lon, slope;


    for (i = 0; i < num; i++)
    {
        if ((x == Map_Field_Coordinates[i][0]) && (y == Map_Field_Coordinates[i][1])) {
            return IN_FIELD; //NOT_IN_FIELD or IN_FIELD on point valid or not
        }

        if ((Map_Field_Coordinates[i][1] > y) != (Map_Field_Coordinates[j][1] > y))
        {
            slope = (x - Map_Field_Coordinates[i][0]) * (Map_Field_Coordinates[j][1] - Map_Field_Coordinates[i][1]) - (Map_Field_Coordinates[j][0] - Map_Field_Coordinates[i][0]) * (y - Map_Field_Coordinates[i][1]);
            if (slope == 0) {
                return NOT_IN_FIELD;
            }
            if ((slope < 0) != (Map_Field_Coordinates[j][1] < Map_Field_Coordinates[i][1])) {
                c++;
            }
        }

        j = i;
    }

    if (c % 2 == 0)
    {
        return IN_FIELD;
    }

    return NOT_IN_FIELD;
}


