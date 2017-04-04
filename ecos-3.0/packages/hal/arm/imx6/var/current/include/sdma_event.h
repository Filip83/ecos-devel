#include <pkgconf/hal_arm.h>
#if (CYGPKG_HAL_SMP_CPU_MAX == 2)
#include "sdma_event2.h"
#else
#include "sdma_event4.h"
#endif
