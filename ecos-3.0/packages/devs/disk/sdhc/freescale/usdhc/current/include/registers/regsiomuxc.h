#include <pkgconf/hal_arm.h>
#if (CYGPKG_HAL_SMP_CPU_MAX == 2)
#include "registers2/regsiomuxc.h"
#else
#include "registers4/regsiomuxc.h"
#endif