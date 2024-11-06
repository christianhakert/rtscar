#pragma once

#define UDAR_PERIOD 80

#define SCAN_ANGLE_INTV 10

#define UDAR_MAP_SIZE ((90/SCAN_ANGLE_INTV)+1)

void udar_control_task();

double *get_udar_map();