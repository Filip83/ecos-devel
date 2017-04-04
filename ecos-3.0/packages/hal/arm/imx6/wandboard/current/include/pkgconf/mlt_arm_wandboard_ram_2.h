// eCos memory layout

// =============================================================================

#ifndef __ASSEMBLER__
#include <cyg/infra/cyg_type.h>
#include <stddef.h>

#endif
#define CYGMEM_REGION_boot          (0x0)         // rom start
#define CYGMEM_REGION_boot_SIZE     (0x18000)     // 98K
#define CYGMEM_REGION_boot_ATTR     (CYGMEM_REGION_ATTR_R)
#define CYGMEM_REGION_ocram         (0x00900000)  //sram start
#define CYGMEM_REGION_ocram_SIZE    (0x00020000)  /128K
#define CYGMEM_REGION_ocram_ATTR    (CYGMEM_REGION_ATTR_R | CYGMEM_REGION_ATTR_W)
#define CYGMEM_REGION_ddr           (0x10000000)
#define CYGMEM_REGION_ddr_SIZE      (0x40000000) // 1 GB DDR3 SDRAM memory
#define CYGMEM_REGION_ddr_ATTR      (CYGMEM_REGION_ATTR_R | CYGMEM_REGION_ATTR_W)

#define CYGMEM_REGION_rom       CYGMEM_REGION_boot
#define CYGMEM_REGION_rom_SIZE  CYGMEM_REGION_boot_SIZE
#define CYGMEM_REGION_rom_ATTR  CYGMEM_REGION_boot_ATTR
#define CYGMEM_REGION_ram       CYGMEM_REGION_ocram
#define CYGMEM_REGION_ram_SIZE  CYGMEM_REGION_ocram_SIZE
#define CYGMEM_REGION_ram_ATTR  CYGMEM_REGION_ocram_ATTR

#ifndef __ASSEMBLER__
extern char CYG_LABEL_NAME (__heap1) [];
#endif
#define CYGMEM_SECTION_heap1 (CYG_LABEL_NAME (__heap1))
#define CYGMEM_SECTION_heap1_SIZE (CYGMEM_REGION_ddr + CYGMEM_REGION_ddr_SIZE - (size_t) CYG_LABEL_NAME (__heap1))

