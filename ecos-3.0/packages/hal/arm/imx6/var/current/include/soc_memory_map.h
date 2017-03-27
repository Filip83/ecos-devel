#include <pkgconf/hal_arm.h>
#if (CYGPKG_HAL_SMP_CPU_MAX == 2)
#include "soc_memory_map2.h"
#else
#include "soc_memory_map4.h"
#endif
