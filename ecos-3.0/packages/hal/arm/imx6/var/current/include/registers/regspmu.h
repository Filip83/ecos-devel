#include <pkgconf/hal_arm.h>
#if (CYGPKG_HAL_SMP_CPU_MAX == 2)
#include "registers2/regspmu.h"
#else
#include "registers4/regspmu.h"
#endif
