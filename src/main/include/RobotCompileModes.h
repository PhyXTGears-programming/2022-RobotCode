#pragma once
#define ROBOTCMH_COMPETITION_MODE

// #define ROBOTCMH_PID_TUNING_MODE
// #define ROBOTCMH_TESTING_MODE

#ifdef ROBOTCMH_COMPETITION_MODE
#ifdef ROBOTCMH_PID_TUNING_MODE
#error (PID tuning and competition mode are not compatible with each other)
#endif
#ifdef ROBOTCMH_TESTING_MODE
#error (Testing mode not available during competition mode)
#endif
#endif


#ifndef ROBOTCMH_COMPETITION_MODE
#ifdef ROBOTCMH_PID_TUNING_MODE
#warning (PID tuning mode enabled)
#endif
#ifdef ROBOTCMH_TESTING_MODE
#warning (testing mode enabled)
#endif
#endif