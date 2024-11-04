#pragma once

#define UDAR_PERIOD 1000

#define SCAN_ANGLE_INTV 15

#define UDAR_MAP_SIZE ((90/SCAN_ANGLE_INTV)+1)

void udar_control_task();