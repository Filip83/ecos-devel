#include <pkgconf/hal_arm.h>
#if (CYGPKG_HAL_SMP_CPU_MAX == 2)
#include "registers2/regsgpmi.h"
#else
#include "registers4/regsgpmi.h"
#endif