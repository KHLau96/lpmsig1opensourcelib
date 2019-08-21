/***********************************************************************
** Copyright (C) 2019 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
**
** This file is part of the Open Motion Analysis Toolkit (OpenMAT).
**
** Redistribution and use in source and binary forms, with 
** or without modification, are permitted provided that the 
** following conditions are met:
**
** Redistributions of source code must retain the above copyright 
** notice, this list of conditions and the following disclaimer.
** Redistributions in binary form must reproduce the above copyright 
** notice, this list of conditions and the following disclaimer in 
** the documentation and/or other materials provided with the 
** distribution.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
** FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
** HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***********************************************************************/

#include "MicroMeasure.h"

#include <sys/time.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>

static const unsigned usec_per_sec = 1000000;
static const unsigned usec_per_msec = 1000;

bool QueryPerformanceFrequency(long long *frequency)
{
    *frequency = (long long)usec_per_sec;

    return true;
}

bool QueryPerformanceCounter(long long *performance_count)
{
    struct timeval time;

    assert(performance_count != NULL);

    gettimeofday(&time, NULL);
    *performance_count = time.tv_usec + time.tv_sec * (long long)usec_per_sec;

    return true;
}

MicroMeasure::MicroMeasure()
{
    long long ticksPerSecond;

    QueryPerformanceFrequency(&ticksPerSecond);
    tpm = ticksPerSecond;
    start_time = 0LL;
}

void MicroMeasure::reset(void)
{
    long long tick;

    QueryPerformanceCounter(&tick);
    start_time = tick;
}

long long MicroMeasure::measure(void)
{
    long long tick;

    QueryPerformanceCounter(&tick);

    return (((tick - start_time) * 1000LL * 1000LL) / tpm);
}

void MicroMeasure::sleep(long long s_t)
{
    reset();

    while (measure() < s_t)
    {
    }
}
