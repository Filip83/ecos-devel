#include <pkgconf/hal_arm.h>
#if (CYGPKG_HAL_SMP_CPU_MAX == 2)
#include "registers2/regsipu.h"
#else
#include "registers4/regsipu.h"
#endif
