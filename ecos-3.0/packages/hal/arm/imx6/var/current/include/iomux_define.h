#include <pkgconf/hal_arm.h>
#if (CYGPKG_HAL_SMP_CPU_MAX == 2)
#include "iomux_define2.h"
#else
#include "iomux_define4.h"
#endif
