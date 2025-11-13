#ifndef INCLUDED_VISUAL_TELEMETRY_MSG_H
#define INCLUDED_VISUAL_TELEMETRY_MSG_H

struct telemetry_msg {
public:
    double timestamp;
    double lat;
    double lon;
    double alt;
    double sog;
    double cog;
    double vx;
    double vy;
    double pitch;
    double roll;
    double yaw;
    int valid;
};

#endif
