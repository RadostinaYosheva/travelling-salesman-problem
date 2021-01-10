#include "engine.h"
#include <sys/time.h>
#include <sys/resource.h>

long getMemoryUsage()
{
    rusage myUsage;

    getrusage(RUSAGE_SELF, &myUsage);

    return myUsage.ru_maxrss;
}

// TODO: Make an engine class and in main have one line "engine.run();"
int main()
{
    Engine engine;
    engine.run();
    
    return 0;
}