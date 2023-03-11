#include "Timer.h"

// Pega o tempo em que a atual estrutura Timer foi criada
Timer::Timer()
{
    start_time_point = chrono::high_resolution_clock::now();
}

// Quando ela eh destruida chama a funcao stop
Timer::~Timer()
{
    stop();
}

void Timer::stop()
{
    // pega o tempo atual em que ela eh destruida (sai do escopo)
    auto end_time_point = chrono::high_resolution_clock::now();

    // start e end recebem o valor atual (dado start_time_point e end_time_point) da duração interna do objeto
    auto start = chrono::time_point_cast<chrono::microseconds>(start_time_point).time_since_epoch().count();
    auto end = chrono::time_point_cast<chrono::microseconds>(end_time_point).time_since_epoch().count();

    auto duration = end - start;
    double ms = duration * 0.001;
    double sc = ms * 0.001;

    cout << duration << "us, " << ms << "ms, " << sc << "s\n";
}
