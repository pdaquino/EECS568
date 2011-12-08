/* 
 * File:   stopwatch.hpp
 * Author: pdaquino
 *
 * Created on December 7, 2011, 4:22 PM
 */

#ifndef STOPWATCH_HPP
#define	STOPWATCH_HPP

#include <sys/time.h>

struct StopWatch {
    void start() {
        gettimeofday(&m_start, NULL);
    }
    double stop(){
        timeval end;
        gettimeofday(&end, NULL);
        return (end.tv_usec - m_start.tv_usec)/1000.0;
    }
    
private:
    timeval m_start;
};

#endif	/* STOPWATCH_HPP */

