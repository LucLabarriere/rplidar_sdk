#pragma once

#ifdef RPLIDAR_BUILD_DLL
#define RPLIDAR_SYMBOL __declspec(dllexport)
#else
#define RPLIDAR_SYMBOL __declspec(dllimport)
#endif