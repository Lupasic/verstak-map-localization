/*
Проекция Меркатора — одна из основных картографических проекций, на атлас.
Земля сферойд
*/

#ifndef MERCATOR_CONVERTIONS_H_
#define MERCATOR_CONVERTIONS_H_
#include <cmath>

#define D_R (M_PI / 180.0)
#define R_D (180.0 / M_PI)
#define R_MAJOR 6378137.0
#define R_MINOR 6356752.3142
#define RATIO (R_MINOR/R_MAJOR)
#define ECCENT (sqrt(1.0 - (RATIO * RATIO)))
#define COM (0.5 * ECCENT)

inline double fmin(double x, double y) { return(x < y ? x : y); }
inline double fmax(double x, double y) { return(x > y ? x : y); }

static inline double deg_rad (double ang) {
        return ang * D_R;
}

inline double merc_x (double lon) {
        return R_MAJOR * deg_rad (lon);
}

inline double merc_y (double lat) {
        lat = fmin (89.5, fmax (lat, -89.5));
        double phi = deg_rad(lat);
        double sinphi = sin(phi);
        double con = ECCENT * sinphi;
        con = pow((1.0 - con) / (1.0 + con), COM);
        double ts = tan(0.5 * (M_PI * 0.5 - phi)) / con;
        return 0 - R_MAJOR * log(ts);
}

inline static double rad_deg (double ang) {
        return ang * R_D;
}
//mercator to yandex tiles points
inline double yampoint_x(double mx)
{
    return 53.5865939582453 * (40075016.685578488 / 2  + mx );
}

inline double yampoint_y(double my)
{
    return 53.5865939582453 * (40075016.685578488 / 2  - my );
}



//mercator from yandex point
inline double merc_x_fromyam(double x)
{
    return x / 53.5865939582453 - 40075016.685578488 / 2 ;
}

inline double merc_y_fromyam(double y)
{
    return 40075016.685578488 / 2 - y / 53.5865939582453 ;
}

//geo coordinates from mercator
inline double lon_x (double x) {
        return rad_deg(x) / R_MAJOR;
}

inline double lat_y (double y) {
        double ts = exp ( -y / R_MAJOR);
        double phi = M_PI_2 - 2 * atan(ts);
        double dphi = 1.0;
        int i;
        for (i = 0; fabs(dphi) > 0.000000001 && i < 15; i++) {
                double con = ECCENT * sin (phi);
                dphi = M_PI_2 - 2 * atan (ts * pow((1.0 - con) / (1.0 + con), COM)) - phi;
                phi += dphi;
        }
        return rad_deg (phi);
}
//length of 1 grad of longitude at given latitude (rad)
inline double metr_from_1grad_lon(double lat)
{
    return M_PI/180 * R_MAJOR*cos(lat) /
                    sqrt( 1 - ECCENT*ECCENT*sin( lat )*sin( lat ) );
}
//length of 1 grad of latitude at given latitude (rad) (including ellipse)
inline double metr_from_1grad_lat(double lat)
{
    return M_PI/180 * R_MAJOR* ( 1 - ECCENT*ECCENT ) /
                        pow( 1-ECCENT*ECCENT*sin(lat)*sin(lat), 1.5);
}

#endif // MERCATOR_CONVERTIONS_H

