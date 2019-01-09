// eCos memory layout

#ifndef __ASSEMBLER__
#include <cyg/infra/cyg_type.h>
#include <stddef.h>

#endif

#define CYGMEM_REGION_ram 0x1FFF0000
#define CYGMEM_REGION_ram_SIZE (CYGHWR_HAL_KINETIS_SRAM_SIZE-2*CYGNUM_HAL_COMMON_INTERRUPTS_STACK_SIZE)
#define CYGMEM_REGION_ram_ATTR (CYGMEM_REGION_ATTR_R | CYGMEM_REGION_ATTR_W)

#define CYGMEM_REGION_flash (0x00000000)
#define CYGMEM_REGION_flash_SIZE (CYGHWR_HAL_KINETIS_FLASH_SIZE)
#define CYGMEM_REGION_flash_ATTR (CYGMEM_REGION_ATTR_R)

#ifdef CYGHWR_HAL_KINETIS_FLEXNVM_DFLASH_SIZE
# define CYGMEM_REGION_flexnvm (0x10000000)
# define CYGMEM_REGION_flexnvm_SIZE (CYGHWR_HAL_KINETIS_FLEXNVM_DFLASH_SIZE)
# define CYGMEM_REGION_flexnvm_ATTR (CYGMEM_REGION_ATTR_R)
#endif

#if defined(CYGHWR_HAL_CORTEXM_KINETIS_EEE) && CYGHWR_HAL_CORTEXM_KINETIS_EEE
#  define CYGMEM_REGION_eeeprom0 (0x14000000)
#  define CYGMEM_REGION_eeeprom0_SIZE (CYGHWR_HAL_KINETIS_EEE0_SIZE)
#  define CYGMEM_REGION_eeeprom0_ATTR (CYGMEM_REGION_ATTR_R)
# if CYGHWR_HAL_KINETIS_EEE_SPLIT > 1
#  define CYGMEM_REGION_eeeprom1 (0x14000000 + CYGHWR_HAL_KINETIS_EEE0_SIZE )
#  define CYGMEM_REGION_eeeprom1_SIZE (CYGHWR_HAL_KINETIS_EEE1_SIZE)
#  define CYGMEM_REGION_eeeprom1_ATTR (CYGMEM_REGION_ATTR_R)
# endif
#else
# define CYGMEM_REGION_flexram (0x14000000)
# define CYGMEM_REGION_flexram_SIZE (CYGHWR_HAL_KINETIS_FLEXRAM_SIZE)
# define CYGMEM_REGION_flexram_ATTR (CYGMEM_REGION_ATTR_R | CYGMEM_REGION_ATTR_W)
#endif

#ifndef __ASSEMBLER__
extern char CYG_LABEL_NAME (__heap1) [];
#endif
#define CYGMEM_SECTION_heap1 (CYG_LABEL_NAME (__heap1))
#define CYGMEM_SECTION_heap1_SIZE (CYGMEM_REGION_ram+CYGMEM_REGION_ram_SIZE - (size_t) CYG_LABEL_NAME (__heap1))
