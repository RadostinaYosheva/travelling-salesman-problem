#include <iostream>

#if __linux__ != 0
#include <time.h>

static uint64_t timer_nsec() {
#if defined(CLOCK_MONOTONIC_RAW)
	const clockid_t clockid = CLOCK_MONOTONIC_RAW;

#else
	const clockid_t clockid = CLOCK_MONOTONIC;

#endif

	timespec t;
	clock_gettime(clockid, &t);

	return t.tv_sec * 1000000000UL + t.tv_nsec;
}

#elif _WIN64 != 0
#define NOMINMAX
#include <Windows.h>

static uint64_t timer_nsec() {

	static LARGE_INTEGER freq;
	static BOOL once = QueryPerformanceFrequency(&freq);

	LARGE_INTEGER t;
	QueryPerformanceCounter(&t);

	return 1000000000ULL * t.QuadPart / freq.QuadPart;
}

#elif __APPLE__ != 0
#include <mach/mach_time.h>

static uint64_t timer_nsec() {

    static mach_timebase_info_data_t tb;
    if (0 == tb.denom)
		mach_timebase_info(&tb);

    const uint64_t t = mach_absolute_time();

    return t * tb.numer / tb.denom;
}

#endif
