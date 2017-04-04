#ifndef CYGONCE_VAR_MMU_H
#define CYGONCE_VAR_MMU_H

// All code here cam from SDK mmu.h

// -------------------------------------------------------------------------
// MMU initialization:

#include <cyg/infra/cyg_type.h>         // base types

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! @brief Memory region attributes.
typedef enum _mmu_memory_type
{
    kStronglyOrdered,
    kDevice,
    kOuterInner_WB_WA,
    kOuterInner_WT,
    kNoncacheable,
} mmu_memory_type_t;

//! @brief Memory region shareability options.
typedef enum _mmu_shareability
{
    kShareable = 1,
    kNonshareable = 0
} mmu_shareability_t;

//! @brief Access permissions for a memory region.
typedef enum _mmu_access
{
    kNoAccess,
    kROAccess,
    kRWAccess
} mmu_access_t;

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Enable the MMU.
 *
 * The L1 page tables and MMU settings must have already been configured by
 * calling mmu_init() before the MMU is enabled.
 */
void hal_mmu_enable(void);

/*!
 * @brief Disable the MMU.
 */
void hal_mmu_disable(void);

/*!
 * @brief Set up the default first-level page table.
 *
 * Initializes the L1 page table with the following regions:
 *  - 0x00000000...0x00900000 : ROM and peripherals, strongly-ordered
 *  - 0x00900000...0x00a00000 : OCRAM, strongly-ordered
 *  - For MX6DQ or MX6SDL: 0x10000000...0x90000000 : DDR, normal, outer inner, write-back, write-allocate
 *  - For MX6SL: 0x80000000...0xc0000000 : DDR, normal, outer inner, write-back, write-allocate
 *
 * If the CPU is participating in SMP, then the DDR regions are made shareable. Otherwise they
 * are marked as non-shareable.
 *
 * The TTBR0 register is set to the base of the L1 table.
 *
 * All memory domains are configured to allow client access. However, note that only domain 0 is
 * used by mmu_map_l1_range().
 */
void hal_mmu_init(void);

/*!
 * @brief Maps a range of memory in the first-level page table.
 *
 * Entries in the first-level page table are filled in for the range of virtual addresses
 * starting at @a va and continuing for @a length bytes. These virtual addreses are mapped
 * to the physical addresses starting at @a pa and continuing for @a length bytes. All table
 * entries for the range of mapped memory have the same attributes, which are selected with
 * the @a memoryType, @a isShareable, and @a access parameters.
 *
 * @param pa The base physical address of the range to which the virtual address will be mapped.
 * @param va The base virtual address of the range.
 * @param length The size of the range to be mapped, in bytes. This value must be divisible by 1MB.
 * @param memoryType The type of the memory region. This controls caching, buffering, ordering of
 *      memory accesses, and other attributes of the region.
 * @param isShareable The shareability of the physical memory. Ignored for strongly-ordered memory.
 * @param access Access permissions.
 */
void hal_mmu_map_l1_range(cyg_uint32 pa, cyg_uint32 va, cyg_uint32 length, mmu_memory_type_t memoryType, mmu_shareability_t isShareable, mmu_access_t access);

/*!
 * @brief Convert virtual address to physical.
 *
 * First attempts a priviledged read translation for the current security mode. If that fails,
 * a priviledged write translation, also for the current security mode, is attempted. If this
 * second attempt at translation fails, then false will be returned.
 *
 * @param virtualAddress Virtual address to convert to a physical address.
 * @param[out] physicalAddress This parameter is filled in with the physical address corresponding
 *      to the virtual address passed in @a virtualAddress.
 * @retval true The address returned through @a physicalAddress is valid.
 * @retval false The conversion failed for some reason.
 */
bool hal_mmu_virtual_to_physical(cyg_uint32 virtualAddress, cyg_uint32 * physicalAddress);

#if defined(__cplusplus)
}
#endif

#endif//CYGONCE_VAR_MMU_H

