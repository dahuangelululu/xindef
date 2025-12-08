/* Compatibility helpers for legacy ll_aton builds lacking epoch flag enums
 * and utility functions introduced by newer ST Edge AI generators. */

#ifndef NETWORK_EPOCH_FLAGS_COMPAT_H
#define NETWORK_EPOCH_FLAGS_COMPAT_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifndef EpochBlock_Flags_blob_encrypted
#define EpochBlock_Flags_blob_encrypted EpochBlock_Flags_blob
#endif

/* Some older ll_aton libraries do not provide the physical/virtual helper
 * macro.  Keep behavior identical by returning the original address. */
#ifndef ATON_LIB_PHYSICAL_TO_VIRTUAL_ADDR
#define ATON_LIB_PHYSICAL_TO_VIRTUAL_ADDR(addr) (addr)
#endif

/* STM32CubeIDE 1.19 配套的 NPU 库可能缺少 ec_copy_blob，
 * 这里提供一个弱定义以便链接通过，实际仅做安全 memcpy。
 *
 * 注意：ECInstr 类型在 ecloader.h 中已定义，避免在此重复 typedef。 */
#ifndef HAVE_EC_COPY_BLOB
__attribute__((weak)) static inline bool ec_copy_blob(ECInstr *dst, const uint8_t *src, unsigned int *size)
{
  if (dst == NULL || src == NULL || size == NULL)
  {
    return false;
  }

  memcpy(dst, src, (*size) * sizeof(ECInstr));
  return true;
}
#endif

#endif /* NETWORK_EPOCH_FLAGS_COMPAT_H */