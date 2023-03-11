#ifndef TIMER_H
#define TIMER_H

#include <iostream>
#include <chrono>
using namespace std;

class Timer
{
public:
    Timer();
    ~Timer();

private:
    chrono::time_point<chrono::high_resolution_clock>  start_time_point;
    void stop();
};

#endif // TIMER_H
