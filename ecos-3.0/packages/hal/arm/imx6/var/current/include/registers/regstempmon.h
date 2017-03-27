#include <pkgconf/hal_arm.h>
#if (CYGPKG_HAL_SMP_CPU_MAX == 2)
#include "registers2/regstempmon.h"
#else
#include "registers4/regstempmon.h"
#endif
