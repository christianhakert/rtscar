#ifndef WATCHY_INTERFACE_H
#define WATCHY_INTERFACE_H
struct car2watch {
unsigned long magic; //0x42
unsigned int angle_granularity; // how wide are sections
double udar_map[90]; // per angle section, how far is wall
};

struct watch2car {
unsigned long magic; //0x42
unsigned int speed; // [0,100]
double direction; // angle/s [-90°/s,90] -45° links
int kaputt;
};
#endif