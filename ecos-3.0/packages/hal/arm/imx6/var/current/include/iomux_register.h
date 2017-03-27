#include <pkgconf/hal_arm.h>
#if (CYGPKG_HAL_SMP_CPU_MAX == 2)
#include "iomux_register2.h"
#else
#include "iomux_register4.h"
#endif
