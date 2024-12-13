/*
 * FPGA Interface C API 21.3 source file.
 *
 * Copyright (c) 2021,
 * National Instruments Corporation.
 * All rights reserved.
 */

#include "NiFpga.h"

/*
 * Platform specific includes.
 */
#if NiFpga_Windows
   #include <windows.h>
#elif NiFpga_Linux || NiFpga_MacOsX
   #include <stdlib.h>
   #include <stdio.h>
   #include <dlfcn.h>
#else
   #error
#endif

/*
 * Platform specific defines.
 */
#if NiFpga_Windows
   #define NiFpga_CCall   __cdecl
   #define NiFpga_StdCall __stdcall
#else
   #define NiFpga_CCall
   #define NiFpga_StdCall
#endif

/*
 * Global library handle, or NULL if the library isn't loaded.
 */
#if NiFpga_Windows
   static HMODULE NiFpga_library = NULL;
#elif NiFpga_VxWorks
   static MODULE_ID NiFpga_library = NULL;
#elif NiFpga_Linux || NiFpga_MacOsX
   static void* NiFpga_library = NULL;
#else
   #error
#endif

/*
 * CVI Resource Tracking functions.
 */
#if NiFpga_Cvi && NiFpga_Windows
#define NiFpga_CviResourceTracking 1

static char* const NiFpga_cviResourceType = "FPGA Interface C API";

typedef void* (NiFpga_CCall *NiFpga_AcquireCviResource)(void* resource,
                                                        char* type,
                                                        char* description,
                                                        ...);

static NiFpga_AcquireCviResource NiFpga_acquireCviResource = NULL;

typedef void* (NiFpga_StdCall *NiFpga_ReleaseCviResource)(void* resource);

static NiFpga_ReleaseCviResource NiFpga_releaseCviResource = NULL;
#endif

/*
 * Session management functions.
 */
static NiFpga_Status (NiFpga_CCall *NiFpga_open)(
                          const char*     path,
                          const char*     signature,
                          const char*     resource,
                          uint32_t        attribute,
                          NiFpga_Session* session) = NULL;

NiFpga_Status NiFpga_Open(const char*     path,
                          const char*     signature,
                          const char*     resource,
                          uint32_t        attribute,
                          NiFpga_Session* session)
{
   const NiFpga_Status result = NiFpga_open
                              ? NiFpga_open(path,
                                            signature,
                                            resource,
                                            attribute,
                                            session)
                              : NiFpga_Status_ResourceNotInitialized;
   #if NiFpga_CviResourceTracking
      if (NiFpga_acquireCviResource
      &&  NiFpga_IsNotError(result))
         NiFpga_acquireCviResource((void*)*session,
                                   NiFpga_cviResourceType,
                                   "NiFpga_Session %#08x",
                                   *session);
   #endif
   return result;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_close)(
                           NiFpga_Session session,
                           uint32_t       attribute) = NULL;

NiFpga_Status NiFpga_Close(NiFpga_Session session,
                           uint32_t       attribute)
{
   if (!NiFpga_close)
      return NiFpga_Status_ResourceNotInitialized;
   #if NiFpga_CviResourceTracking
      if (NiFpga_releaseCviResource)
         NiFpga_releaseCviResource((void*)session);
   #endif
   return NiFpga_close(session, attribute);
}

/*
 * FPGA state functions.
 */
static NiFpga_Status (NiFpga_CCall *NiFpga_run)(
                         NiFpga_Session session,
                         uint32_t       attribute) = NULL;

NiFpga_Status NiFpga_Run(NiFpga_Session session,
                         uint32_t       attribute)
{
   return NiFpga_run
        ? NiFpga_run(session, attribute)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_abort)(
                           NiFpga_Session session) = NULL;

NiFpga_Status NiFpga_Abort(NiFpga_Session session)
{
   return NiFpga_abort
        ? NiFpga_abort(session)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_reset)(
                           NiFpga_Session session) = NULL;

NiFpga_Status NiFpga_Reset(NiFpga_Session session)
{
   return NiFpga_reset
        ? NiFpga_reset(session)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_download)(
                              NiFpga_Session session) = NULL;

NiFpga_Status NiFpga_Download(NiFpga_Session session)
{
   return NiFpga_download
        ? NiFpga_download(session)
        : NiFpga_Status_ResourceNotInitialized;
}

/*
 * Functions to read from scalar indicators and controls.
 */
static NiFpga_Status (NiFpga_CCall *NiFpga_readBool)(
                              NiFpga_Session session,
                              uint32_t       indicator,
                              NiFpga_Bool*   value) = NULL;

NiFpga_Status NiFpga_ReadBool(NiFpga_Session session,
                              uint32_t       indicator,
                              NiFpga_Bool*   value)
{
   return NiFpga_readBool
        ? NiFpga_readBool(session, indicator, value)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readI8)(
                            NiFpga_Session session,
                            uint32_t       indicator,
                            int8_t*        value) = NULL;

NiFpga_Status NiFpga_ReadI8(NiFpga_Session session,
                            uint32_t       indicator,
                            int8_t*        value)
{
   return NiFpga_readI8
        ? NiFpga_readI8(session, indicator, value)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readU8)(
                            NiFpga_Session session,
                            uint32_t       indicator,
                            uint8_t*       value) = NULL;

NiFpga_Status NiFpga_ReadU8(NiFpga_Session session,
                            uint32_t       indicator,
                            uint8_t*       value)
{
   return NiFpga_readU8
        ? NiFpga_readU8(session, indicator, value)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readI16)(
                             NiFpga_Session session,
                             uint32_t       indicator,
                             int16_t*       value) = NULL;

NiFpga_Status NiFpga_ReadI16(NiFpga_Session session,
                             uint32_t       indicator,
                             int16_t*       value)
{
   return NiFpga_readI16
        ? NiFpga_readI16(session, indicator, value)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readU16)(
                             NiFpga_Session session,
                             uint32_t       indicator,
                             uint16_t*      value) = NULL;

NiFpga_Status NiFpga_ReadU16(NiFpga_Session session,
                             uint32_t       indicator,
                             uint16_t*      value)
{
   return NiFpga_readU16
        ? NiFpga_readU16(session, indicator, value)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readI32)(
                             NiFpga_Session session,
                             uint32_t       indicator,
                             int32_t*       value) = NULL;

NiFpga_Status NiFpga_ReadI32(NiFpga_Session session,
                             uint32_t       indicator,
                             int32_t*       value)
{
   return NiFpga_readI32
        ? NiFpga_readI32(session, indicator, value)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readU32)(
                             NiFpga_Session session,
                             uint32_t       indicator,
                             uint32_t*      value) = NULL;

NiFpga_Status NiFpga_ReadU32(NiFpga_Session session,
                             uint32_t       indicator,
                             uint32_t*      value)
{
   return NiFpga_readU32
        ? NiFpga_readU32(session, indicator, value)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readI64)(
                             NiFpga_Session session,
                             uint32_t       indicator,
                             int64_t*       value) = NULL;

NiFpga_Status NiFpga_ReadI64(NiFpga_Session session,
                             uint32_t       indicator,
                             int64_t*       value)
{
   return NiFpga_readI64
        ? NiFpga_readI64(session, indicator, value)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readU64)(
                             NiFpga_Session session,
                             uint32_t       indicator,
                             uint64_t*      value) = NULL;

NiFpga_Status NiFpga_ReadU64(NiFpga_Session session,
                             uint32_t       indicator,
                             uint64_t*      value)
{
   return NiFpga_readU64
        ? NiFpga_readU64(session, indicator, value)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readSgl)(
                             NiFpga_Session session,
                             uint32_t       indicator,
                             float*         value) = NULL;

NiFpga_Status NiFpga_ReadSgl(NiFpga_Session session,
                             uint32_t       indicator,
                             float*         value)
{
   return NiFpga_readSgl
        ? NiFpga_readSgl(session, indicator, value)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readDbl)(
                             NiFpga_Session session,
                             uint32_t       indicator,
                             double*        value) = NULL;

NiFpga_Status NiFpga_ReadDbl(NiFpga_Session session,
                             uint32_t       indicator,
                             double*        value)
{
   return NiFpga_readDbl
        ? NiFpga_readDbl(session, indicator, value)
        : NiFpga_Status_ResourceNotInitialized;
}

/*
 * Functions to write to scalar controls and indicators.
 */
static NiFpga_Status (NiFpga_CCall *NiFpga_writeBool)(
                               NiFpga_Session session,
                               uint32_t       control,
                               NiFpga_Bool    value) = NULL;

NiFpga_Status NiFpga_WriteBool(NiFpga_Session session,
                               uint32_t       control,
                               NiFpga_Bool    value)
{
   return NiFpga_writeBool
        ? NiFpga_writeBool(session, control, value)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeI8)(
                             NiFpga_Session session,
                             uint32_t       control,
                             int8_t         value) = NULL;

NiFpga_Status NiFpga_WriteI8(NiFpga_Session session,
                             uint32_t       control,
                             int8_t         value)
{
   return NiFpga_writeI8
        ? NiFpga_writeI8(session, control, value)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeU8)(
                             NiFpga_Session session,
                             uint32_t       control,
                             uint8_t        value) = NULL;

NiFpga_Status NiFpga_WriteU8(NiFpga_Session session,
                             uint32_t       control,
                             uint8_t        value)
{
   return NiFpga_writeU8
        ? NiFpga_writeU8(session, control, value)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeI16)(
                              NiFpga_Session session,
                              uint32_t       control,
                              int16_t        value) = NULL;

NiFpga_Status NiFpga_WriteI16(NiFpga_Session session,
                              uint32_t       control,
                              int16_t        value)
{
   return NiFpga_writeI16
        ? NiFpga_writeI16(session, control, value)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeU16)(
                              NiFpga_Session session,
                              uint32_t       control,
                              uint16_t       value) = NULL;

NiFpga_Status NiFpga_WriteU16(NiFpga_Session session,
                              uint32_t       control,
                              uint16_t       value)
{
   return NiFpga_writeU16
        ? NiFpga_writeU16(session, control, value)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeI32)(
                              NiFpga_Session session,
                              uint32_t       control,
                              int32_t        value) = NULL;

NiFpga_Status NiFpga_WriteI32(NiFpga_Session session,
                              uint32_t       control,
                              int32_t        value)
{
   return NiFpga_writeI32
        ? NiFpga_writeI32(session, control, value)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeU32)(
                              NiFpga_Session session,
                              uint32_t       control,
                              uint32_t       value) = NULL;

NiFpga_Status NiFpga_WriteU32(NiFpga_Session session,
                              uint32_t       control,
                              uint32_t       value)
{
   return NiFpga_writeU32
        ? NiFpga_writeU32(session, control, value)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeI64)(
                              NiFpga_Session session,
                              uint32_t       control,
                              int64_t        value) = NULL;

NiFpga_Status NiFpga_WriteI64(NiFpga_Session session,
                              uint32_t       control,
                              int64_t        value)
{
   return NiFpga_writeI64
        ? NiFpga_writeI64(session, control, value)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeU64)(
                              NiFpga_Session session,
                              uint32_t       control,
                              uint64_t       value) = NULL;

NiFpga_Status NiFpga_WriteU64(NiFpga_Session session,
                              uint32_t       control,
                              uint64_t       value)
{
   return NiFpga_writeU64
        ? NiFpga_writeU64(session, control, value)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeSgl)(
                              NiFpga_Session session,
                              uint32_t       control,
                              float          value) = NULL;

NiFpga_Status NiFpga_WriteSgl(NiFpga_Session session,
                              uint32_t       control,
                              float          value)
{
   return NiFpga_writeSgl
        ? NiFpga_writeSgl(session, control, value)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeDbl)(
                              NiFpga_Session session,
                              uint32_t       control,
                              double         value) = NULL;

NiFpga_Status NiFpga_WriteDbl(NiFpga_Session session,
                              uint32_t       control,
                              double         value)
{
   return NiFpga_writeDbl
        ? NiFpga_writeDbl(session, control, value)
        : NiFpga_Status_ResourceNotInitialized;
}

/*
 * Functions to read from array indicators and controls.
 */
static NiFpga_Status (NiFpga_CCall *NiFpga_readArrayBool)(
                                   NiFpga_Session session,
                                   uint32_t       indicator,
                                   NiFpga_Bool*   array,
                                   size_t         size) = NULL;

NiFpga_Status NiFpga_ReadArrayBool(NiFpga_Session session,
                                   uint32_t       indicator,
                                   NiFpga_Bool*   array,
                                   size_t         size)
{
   return NiFpga_readArrayBool
        ? NiFpga_readArrayBool(session, indicator, array, size)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readArrayI8)(
                                 NiFpga_Session session,
                                 uint32_t       indicator,
                                 int8_t*        array,
                                 size_t         size) = NULL;

NiFpga_Status NiFpga_ReadArrayI8(NiFpga_Session session,
                                 uint32_t       indicator,
                                 int8_t*        array,
                                 size_t         size)
{
   return NiFpga_readArrayI8
        ? NiFpga_readArrayI8(session, indicator, array, size)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readArrayU8)(
                                 NiFpga_Session session,
                                 uint32_t       indicator,
                                 uint8_t*       array,
                                 size_t         size) = NULL;

NiFpga_Status NiFpga_ReadArrayU8(NiFpga_Session session,
                                 uint32_t       indicator,
                                 uint8_t*       array,
                                 size_t         size)
{
   return NiFpga_readArrayU8
        ? NiFpga_readArrayU8(session, indicator, array, size)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readArrayI16)(
                                  NiFpga_Session session,
                                  uint32_t       indicator,
                                  int16_t*       array,
                                  size_t         size) = NULL;

NiFpga_Status NiFpga_ReadArrayI16(NiFpga_Session session,
                                  uint32_t       indicator,
                                  int16_t*       array,
                                  size_t         size)
{
   return NiFpga_readArrayI16
        ? NiFpga_readArrayI16(session, indicator, array, size)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readArrayU16)(
                                  NiFpga_Session session,
                                  uint32_t       indicator,
                                  uint16_t*      array,
                                  size_t         size) = NULL;

NiFpga_Status NiFpga_ReadArrayU16(NiFpga_Session session,
                                  uint32_t       indicator,
                                  uint16_t*      array,
                                  size_t         size)
{
   return NiFpga_readArrayU16
        ? NiFpga_readArrayU16(session, indicator, array, size)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readArrayI32)(
                                  NiFpga_Session session,
                                  uint32_t       indicator,
                                  int32_t*       array,
                                  size_t         size) = NULL;

NiFpga_Status NiFpga_ReadArrayI32(NiFpga_Session session,
                                  uint32_t       indicator,
                                  int32_t*       array,
                                  size_t         size)
{
   return NiFpga_readArrayI32
        ? NiFpga_readArrayI32(session, indicator, array, size)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readArrayU32)(
                                  NiFpga_Session session,
                                  uint32_t       indicator,
                                  uint32_t*      array,
                                  size_t         size) = NULL;

NiFpga_Status NiFpga_ReadArrayU32(NiFpga_Session session,
                                  uint32_t       indicator,
                                  uint32_t*      array,
                                  size_t         size)
{
   return NiFpga_readArrayU32
        ? NiFpga_readArrayU32(session, indicator, array, size)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readArrayI64)(
                                  NiFpga_Session session,
                                  uint32_t       indicator,
                                  int64_t*       array,
                                  size_t         size) = NULL;

NiFpga_Status NiFpga_ReadArrayI64(NiFpga_Session session,
                                  uint32_t       indicator,
                                  int64_t*       array,
                                  size_t         size)
{
   return NiFpga_readArrayI64
        ? NiFpga_readArrayI64(session, indicator, array, size)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readArrayU64)(
                                  NiFpga_Session session,
                                  uint32_t       indicator,
                                  uint64_t*      array,
                                  size_t         size) = NULL;

NiFpga_Status NiFpga_ReadArrayU64(NiFpga_Session session,
                                  uint32_t       indicator,
                                  uint64_t*      array,
                                  size_t         size)
{
   return NiFpga_readArrayU64
        ? NiFpga_readArrayU64(session, indicator, array, size)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readArraySgl)(
                                  NiFpga_Session session,
                                  uint32_t       indicator,
                                  float*         array,
                                  size_t         size) = NULL;

NiFpga_Status NiFpga_ReadArraySgl(NiFpga_Session session,
                                  uint32_t       indicator,
                                  float*         array,
                                  size_t         size)
{
   return NiFpga_readArraySgl
        ? NiFpga_readArraySgl(session, indicator, array, size)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readArrayDbl)(
                                  NiFpga_Session session,
                                  uint32_t       indicator,
                                  double*        array,
                                  size_t         size) = NULL;

NiFpga_Status NiFpga_ReadArrayDbl(NiFpga_Session session,
                                  uint32_t       indicator,
                                  double*        array,
                                  size_t         size)
{
   return NiFpga_readArrayDbl
        ? NiFpga_readArrayDbl(session, indicator, array, size)
        : NiFpga_Status_ResourceNotInitialized;
}

/*
 * Functions to write to array controls and indicators.
 */
static NiFpga_Status (NiFpga_CCall *NiFpga_writeArrayBool)(
                                    NiFpga_Session     session,
                                    uint32_t           control,
                                    const NiFpga_Bool* array,
                                    size_t             size) = NULL;

NiFpga_Status NiFpga_WriteArrayBool(NiFpga_Session     session,
                                    uint32_t           control,
                                    const NiFpga_Bool* array,
                                    size_t             size)
{
   return NiFpga_writeArrayBool
        ? NiFpga_writeArrayBool(session, control, array, size)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeArrayI8)(
                                  NiFpga_Session session,
                                  uint32_t       control,
                                  const int8_t*  array,
                                  size_t         size) = NULL;

NiFpga_Status NiFpga_WriteArrayI8(NiFpga_Session session,
                                  uint32_t       control,
                                  const int8_t*  array,
                                  size_t         size)
{
   return NiFpga_writeArrayI8
        ? NiFpga_writeArrayI8(session, control, array, size)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeArrayU8)(
                                  NiFpga_Session session,
                                  uint32_t       control,
                                  const uint8_t* array,
                                  size_t         size) = NULL;

NiFpga_Status NiFpga_WriteArrayU8(NiFpga_Session session,
                                  uint32_t       control,
                                  const uint8_t* array,
                                  size_t         size)
{
   return NiFpga_writeArrayU8
        ? NiFpga_writeArrayU8(session, control, array, size)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeArrayI16)(
                                   NiFpga_Session session,
                                   uint32_t       control,
                                   const int16_t* array,
                                   size_t         size) = NULL;

NiFpga_Status NiFpga_WriteArrayI16(NiFpga_Session session,
                                   uint32_t       control,
                                   const int16_t* array,
                                   size_t         size)
{
   return NiFpga_writeArrayI16
        ? NiFpga_writeArrayI16(session, control, array, size)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeArrayU16)(
                                   NiFpga_Session  session,
                                   uint32_t        control,
                                   const uint16_t* array,
                                   size_t          size) = NULL;

NiFpga_Status NiFpga_WriteArrayU16(NiFpga_Session  session,
                                   uint32_t        control,
                                   const uint16_t* array,
                                   size_t          size)
{
   return NiFpga_writeArrayU16
        ? NiFpga_writeArrayU16(session, control, array, size)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeArrayI32)(
                                   NiFpga_Session session,
                                   uint32_t       control,
                                   const int32_t* array,
                                   size_t         size) = NULL;

NiFpga_Status NiFpga_WriteArrayI32(NiFpga_Session session,
                                   uint32_t       control,
                                   const int32_t* array,
                                   size_t         size)
{
   return NiFpga_writeArrayI32
        ? NiFpga_writeArrayI32(session, control, array, size)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeArrayU32)(
                                   NiFpga_Session  session,
                                   uint32_t        control,
                                   const uint32_t* array,
                                   size_t          size) = NULL;

NiFpga_Status NiFpga_WriteArrayU32(NiFpga_Session  session,
                                   uint32_t        control,
                                   const uint32_t* array,
                                   size_t          size)
{
   return NiFpga_writeArrayU32
        ? NiFpga_writeArrayU32(session, control, array, size)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeArrayI64)(
                                   NiFpga_Session session,
                                   uint32_t       control,
                                   const int64_t* array,
                                   size_t         size) = NULL;

NiFpga_Status NiFpga_WriteArrayI64(NiFpga_Session session,
                                   uint32_t       control,
                                   const int64_t* array,
                                   size_t         size)
{
   return NiFpga_writeArrayI64
        ? NiFpga_writeArrayI64(session, control, array, size)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeArrayU64)(
                                   NiFpga_Session  session,
                                   uint32_t        control,
                                   const uint64_t* array,
                                   size_t          size) = NULL;

NiFpga_Status NiFpga_WriteArrayU64(NiFpga_Session  session,
                                   uint32_t        control,
                                   const uint64_t* array,
                                   size_t          size)
{
   return NiFpga_writeArrayU64
        ? NiFpga_writeArrayU64(session, control, array, size)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeArraySgl)(
                                   NiFpga_Session session,
                                   uint32_t       control,
                                   const float*   array,
                                   size_t         size) = NULL;

NiFpga_Status NiFpga_WriteArraySgl(NiFpga_Session session,
                                   uint32_t       control,
                                   const float*   array,
                                   size_t         size)
{
   return NiFpga_writeArraySgl
        ? NiFpga_writeArraySgl(session, control, array, size)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeArrayDbl)(
                                   NiFpga_Session session,
                                   uint32_t       control,
                                   const double*  array,
                                   size_t         size) = NULL;

NiFpga_Status NiFpga_WriteArrayDbl(NiFpga_Session session,
                                   uint32_t       control,
                                   const double*  array,
                                   size_t         size)
{
   return NiFpga_writeArrayDbl
        ? NiFpga_writeArrayDbl(session, control, array, size)
        : NiFpga_Status_ResourceNotInitialized;
}

/*
 * Interrupt functions.
 */
static NiFpga_Status (NiFpga_CCall *NiFpga_reserveIrqContext)(
                                       NiFpga_Session     session,
                                       NiFpga_IrqContext* context) = NULL;


NiFpga_Status NiFpga_ReserveIrqContext(NiFpga_Session     session,
                                       NiFpga_IrqContext* context)
{
   const NiFpga_Status result = NiFpga_reserveIrqContext
                              ? NiFpga_reserveIrqContext(session, context)
                              : NiFpga_Status_ResourceNotInitialized;
   #if NiFpga_CviResourceTracking
      if (NiFpga_acquireCviResource
      &&  NiFpga_IsNotError(result))
         NiFpga_acquireCviResource(*context,
                                   NiFpga_cviResourceType,
                                   "NiFpga_IrqContext 0x%p",
                                   *context);
   #endif
   return result;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_unreserveIrqContext)(
                                         NiFpga_Session    session,
                                         NiFpga_IrqContext context) = NULL;


NiFpga_Status NiFpga_UnreserveIrqContext(NiFpga_Session    session,
                                         NiFpga_IrqContext context)
{
   if (!NiFpga_unreserveIrqContext)
      return NiFpga_Status_ResourceNotInitialized;
   #if NiFpga_CviResourceTracking
      if (NiFpga_releaseCviResource)
         NiFpga_releaseCviResource(context);
   #endif
   return NiFpga_unreserveIrqContext(session, context);
}

static NiFpga_Status (NiFpga_CCall *NiFpga_waitOnIrqs)(
                                NiFpga_Session    session,
                                NiFpga_IrqContext context,
                                uint32_t          irqs,
                                uint32_t          timeout,
                                uint32_t*         irqsAsserted,
                                NiFpga_Bool*      timedOut) = NULL;

NiFpga_Status NiFpga_WaitOnIrqs(NiFpga_Session    session,
                                NiFpga_IrqContext context,
                                uint32_t          irqs,
                                uint32_t          timeout,
                                uint32_t*         irqsAsserted,
                                NiFpga_Bool*      timedOut)
{
   return NiFpga_waitOnIrqs
        ? NiFpga_waitOnIrqs(session,
                            context,
                            irqs,
                            timeout,
                            irqsAsserted,
                            timedOut)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_acknowledgeIrqs)(
                                     NiFpga_Session session,
                                     uint32_t       irqs) = NULL;

NiFpga_Status NiFpga_AcknowledgeIrqs(NiFpga_Session session,
                                     uint32_t       irqs)
{
   return NiFpga_acknowledgeIrqs
        ? NiFpga_acknowledgeIrqs(session, irqs)
        : NiFpga_Status_ResourceNotInitialized;
}

/*
 * DMA FIFO state functions.
 */
static NiFpga_Status (NiFpga_CCall *NiFpga_configureFifo)(
                                   NiFpga_Session session,
                                   uint32_t       fifo,
                                   size_t         depth) = NULL;

NiFpga_Status NiFpga_ConfigureFifo(NiFpga_Session session,
                                   uint32_t       fifo,
                                   size_t         depth)
{
   return NiFpga_configureFifo
        ? NiFpga_configureFifo(session, fifo, depth)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_configureFifo2)(
                                   NiFpga_Session session,
                                   uint32_t       fifo,
                                   size_t         requestedDepth,
                                   size_t*        actualDepth) = NULL;

NiFpga_Status NiFpga_ConfigureFifo2(NiFpga_Session session,
                                   uint32_t       fifo,
                                   size_t         requestedDepth,
                                   size_t*        actualDepth)
{
   return NiFpga_configureFifo2
        ? NiFpga_configureFifo2(session, fifo, requestedDepth, actualDepth)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_setFifoPropertyU32)(
                                   NiFpga_Session      session,
                                   uint32_t            fifo,
                                   NiFpga_FifoProperty property,
                                   uint32_t            value) = NULL;

NiFpga_Status NiFpga_SetFifoPropertyU32(NiFpga_Session      session,
                                        uint32_t            fifo,
                                        NiFpga_FifoProperty property,
                                        uint32_t            value)
{
   return NiFpga_setFifoPropertyU32
        ? NiFpga_setFifoPropertyU32(session, fifo, property, value)
        : NiFpga_Status_VersionMismatch;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_setFifoPropertyI32)(
                                   NiFpga_Session      session,
                                   uint32_t            fifo,
                                   NiFpga_FifoProperty property,
                                   int32_t             value) = NULL;

NiFpga_Status NiFpga_SetFifoPropertyI32(NiFpga_Session      session,
                                        uint32_t            fifo,
                                        NiFpga_FifoProperty property,
                                        int32_t             value)
{
   return NiFpga_setFifoPropertyI32
        ? NiFpga_setFifoPropertyI32(session, fifo, property, value)
        : NiFpga_Status_VersionMismatch;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_setFifoPropertyU64)(
                                   NiFpga_Session      session,
                                   uint32_t            fifo,
                                   NiFpga_FifoProperty property,
                                   uint64_t            value) = NULL;

NiFpga_Status NiFpga_SetFifoPropertyU64(NiFpga_Session      session,
                                        uint32_t            fifo,
                                        NiFpga_FifoProperty property,
                                        uint64_t            value)
{
   return NiFpga_setFifoPropertyU64
        ? NiFpga_setFifoPropertyU64(session, fifo, property, value)
        : NiFpga_Status_VersionMismatch;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_setFifoPropertyI64)(
                                   NiFpga_Session      session,
                                   uint32_t            fifo,
                                   NiFpga_FifoProperty property,
                                   int64_t             value) = NULL;

NiFpga_Status NiFpga_SetFifoPropertyI64(NiFpga_Session      session,
                                        uint32_t            fifo,
                                        NiFpga_FifoProperty property,
                                        int64_t             value)
{
   return NiFpga_setFifoPropertyI64
        ? NiFpga_setFifoPropertyI64(session, fifo, property, value)
        : NiFpga_Status_VersionMismatch;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_setFifoPropertyPtr)(
                                   NiFpga_Session      session,
                                   uint32_t            fifo,
                                   NiFpga_FifoProperty property,
                                   void*               value) = NULL;

NiFpga_Status NiFpga_SetFifoPropertyPtr(NiFpga_Session      session,
                                        uint32_t            fifo,
                                        NiFpga_FifoProperty property,
                                        void*               value)
{
   return NiFpga_setFifoPropertyPtr
        ? NiFpga_setFifoPropertyPtr(session, fifo, property, value)
        : NiFpga_Status_VersionMismatch;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_getFifoPropertyU32)(
                                   NiFpga_Session      session,
                                   uint32_t            fifo,
                                   NiFpga_FifoProperty property,
                                   uint32_t*           value) = NULL;

NiFpga_Status NiFpga_GetFifoPropertyU32(NiFpga_Session      session,
                                        uint32_t            fifo,
                                        NiFpga_FifoProperty property,
                                        uint32_t*           value)
{
   return NiFpga_getFifoPropertyU32
        ? NiFpga_getFifoPropertyU32(session, fifo, property, value)
        : NiFpga_Status_VersionMismatch;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_getFifoPropertyI32)(
                                   NiFpga_Session      session,
                                   uint32_t            fifo,
                                   NiFpga_FifoProperty property,
                                   int32_t*            value) = NULL;

NiFpga_Status NiFpga_GetFifoPropertyI32(NiFpga_Session      session,
                                        uint32_t            fifo,
                                        NiFpga_FifoProperty property,
                                        int32_t*            value)
{
   return NiFpga_getFifoPropertyI32
        ? NiFpga_getFifoPropertyI32(session, fifo, property, value)
        : NiFpga_Status_VersionMismatch;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_getFifoPropertyU64)(
                                   NiFpga_Session      session,
                                   uint32_t            fifo,
                                   NiFpga_FifoProperty property,
                                   uint64_t*           value) = NULL;

NiFpga_Status NiFpga_GetFifoPropertyU64(NiFpga_Session      session,
                                        uint32_t            fifo,
                                        NiFpga_FifoProperty property,
                                        uint64_t*           value)
{
   return NiFpga_getFifoPropertyU64
        ? NiFpga_getFifoPropertyU64(session, fifo, property, value)
        : NiFpga_Status_VersionMismatch;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_getFifoPropertyI64)(
                                   NiFpga_Session      session,
                                   uint32_t            fifo,
                                   NiFpga_FifoProperty property,
                                   int64_t*            value) = NULL;

NiFpga_Status NiFpga_GetFifoPropertyI64(NiFpga_Session      session,
                                        uint32_t            fifo,
                                        NiFpga_FifoProperty property,
                                        int64_t*            value)
{
   return NiFpga_getFifoPropertyI64
        ? NiFpga_getFifoPropertyI64(session, fifo, property, value)
        : NiFpga_Status_VersionMismatch;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_getFifoPropertyPtr)(
                                   NiFpga_Session      session,
                                   uint32_t            fifo,
                                   NiFpga_FifoProperty property,
                                   void**              value) = NULL;

NiFpga_Status NiFpga_GetFifoPropertyPtr(NiFpga_Session      session,
                                        uint32_t            fifo,
                                        NiFpga_FifoProperty property,
                                        void**              value)
{
   return NiFpga_getFifoPropertyPtr
        ? NiFpga_getFifoPropertyPtr(session, fifo, property, value)
        : NiFpga_Status_VersionMismatch;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_commitFifoConfiguration)(
                                   NiFpga_Session      session,
                                   uint32_t            fifo) = NULL;

NiFpga_Status NiFpga_CommitFifoConfiguration(NiFpga_Session      session,
                                             uint32_t            fifo)
{
   return NiFpga_commitFifoConfiguration
        ? NiFpga_commitFifoConfiguration(session, fifo)
        : NiFpga_Status_VersionMismatch;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_startFifo)(
                               NiFpga_Session session,
                               uint32_t       fifo) = NULL;

NiFpga_Status NiFpga_StartFifo(NiFpga_Session session,
                               uint32_t       fifo)
{
   return NiFpga_startFifo
        ? NiFpga_startFifo(session, fifo)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_stopFifo)(
                              NiFpga_Session session,
                              uint32_t       fifo) = NULL;

NiFpga_Status NiFpga_StopFifo(NiFpga_Session session,
                              uint32_t       fifo)
{
   return NiFpga_stopFifo
        ? NiFpga_stopFifo(session, fifo)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_unreserveFifo)(
                              NiFpga_Session session,
                              uint32_t       fifo) = NULL;

NiFpga_Status NiFpga_UnreserveFifo(NiFpga_Session session,
                                   uint32_t       fifo)
{
   return NiFpga_unreserveFifo
        ? NiFpga_unreserveFifo(session, fifo)
        : NiFpga_Status_VersionMismatch;
}

/*
 * Functions to read from target-to-host DMA FIFOs.
 */
static NiFpga_Status (NiFpga_CCall *NiFpga_readFifoBool)(
                                  NiFpga_Session session,
                                  uint32_t       fifo,
                                  NiFpga_Bool*   data,
                                  size_t         numberOfElements,
                                  uint32_t       timeout,
                                  size_t*        elementsRemaining) = NULL;

NiFpga_Status NiFpga_ReadFifoBool(NiFpga_Session session,
                                  uint32_t       fifo,
                                  NiFpga_Bool*   data,
                                  size_t         numberOfElements,
                                  uint32_t       timeout,
                                  size_t*        elementsRemaining)
{
   return NiFpga_readFifoBool
        ? NiFpga_readFifoBool(session,
                              fifo,
                              data,
                              numberOfElements,
                              timeout,
                              elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readFifoI8)(
                                NiFpga_Session session,
                                uint32_t       fifo,
                                int8_t*        data,
                                size_t         numberOfElements,
                                uint32_t       timeout,
                                size_t*        elementsRemaining) = NULL;

NiFpga_Status NiFpga_ReadFifoI8(NiFpga_Session session,
                                uint32_t       fifo,
                                int8_t*        data,
                                size_t         numberOfElements,
                                uint32_t       timeout,
                                size_t*        elementsRemaining)
{
   return NiFpga_readFifoI8
        ? NiFpga_readFifoI8(session,
                            fifo,
                            data,
                            numberOfElements,
                            timeout,
                            elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readFifoU8)(
                                NiFpga_Session session,
                                uint32_t       fifo,
                                uint8_t*       data,
                                size_t         numberOfElements,
                                uint32_t       timeout,
                                size_t*        elementsRemaining) = NULL;

NiFpga_Status NiFpga_ReadFifoU8(NiFpga_Session session,
                                uint32_t       fifo,
                                uint8_t*       data,
                                size_t         numberOfElements,
                                uint32_t       timeout,
                                size_t*        elementsRemaining)
{
   return NiFpga_readFifoU8
        ? NiFpga_readFifoU8(session,
                            fifo,
                            data,
                            numberOfElements,
                            timeout,
                            elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readFifoI16)(
                                 NiFpga_Session session,
                                 uint32_t       fifo,
                                 int16_t*       data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        elementsRemaining) = NULL;

NiFpga_Status NiFpga_ReadFifoI16(NiFpga_Session session,
                                 uint32_t       fifo,
                                 int16_t*       data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        elementsRemaining)
{
   return NiFpga_readFifoI16
        ? NiFpga_readFifoI16(session,
                             fifo,
                             data,
                             numberOfElements,
                             timeout,
                             elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readFifoU16)(
                                 NiFpga_Session session,
                                 uint32_t       fifo,
                                 uint16_t*      data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        elementsRemaining) = NULL;

NiFpga_Status NiFpga_ReadFifoU16(NiFpga_Session session,
                                 uint32_t       fifo,
                                 uint16_t*      data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        elementsRemaining)
{
   return NiFpga_readFifoU16
        ? NiFpga_readFifoU16(session,
                             fifo,
                             data,
                             numberOfElements,
                             timeout,
                             elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readFifoI32)(
                                 NiFpga_Session session,
                                 uint32_t       fifo,
                                 int32_t*       data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        elementsRemaining) = NULL;

NiFpga_Status NiFpga_ReadFifoI32(NiFpga_Session session,
                                 uint32_t       fifo,
                                 int32_t*       data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        elementsRemaining)
{
   return NiFpga_readFifoI32
        ? NiFpga_readFifoI32(session,
                             fifo,
                             data,
                             numberOfElements,
                             timeout,
                             elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readFifoU32)(
                                 NiFpga_Session session,
                                 uint32_t       fifo,
                                 uint32_t*      data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        elementsRemaining) = NULL;

NiFpga_Status NiFpga_ReadFifoU32(NiFpga_Session session,
                                 uint32_t       fifo,
                                 uint32_t*      data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        elementsRemaining)
{
   return NiFpga_readFifoU32
        ? NiFpga_readFifoU32(session,
                             fifo,
                             data,
                             numberOfElements,
                             timeout,
                             elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readFifoI64)(
                                 NiFpga_Session session,
                                 uint32_t       fifo,
                                 int64_t*       data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        elementsRemaining) = NULL;

NiFpga_Status NiFpga_ReadFifoI64(NiFpga_Session session,
                                 uint32_t       fifo,
                                 int64_t*       data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        elementsRemaining)
{
   return NiFpga_readFifoI64
        ? NiFpga_readFifoI64(session,
                             fifo,
                             data,
                             numberOfElements,
                             timeout,
                             elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readFifoU64)(
                                 NiFpga_Session session,
                                 uint32_t       fifo,
                                 uint64_t*      data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        elementsRemaining) = NULL;

NiFpga_Status NiFpga_ReadFifoU64(NiFpga_Session session,
                                 uint32_t       fifo,
                                 uint64_t*      data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        elementsRemaining)
{
   return NiFpga_readFifoU64
        ? NiFpga_readFifoU64(session,
                             fifo,
                             data,
                             numberOfElements,
                             timeout,
                             elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readFifoSgl)(
                                 NiFpga_Session session,
                                 uint32_t       fifo,
                                 float*         data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        elementsRemaining) = NULL;

NiFpga_Status NiFpga_ReadFifoSgl(NiFpga_Session session,
                                 uint32_t       fifo,
                                 float*         data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        elementsRemaining)
{
   return NiFpga_readFifoSgl
        ? NiFpga_readFifoSgl(session,
                             fifo,
                             data,
                             numberOfElements,
                             timeout,
                             elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readFifoDbl)(
                                 NiFpga_Session session,
                                 uint32_t       fifo,
                                 double*        data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        elementsRemaining) = NULL;

NiFpga_Status NiFpga_ReadFifoDbl(NiFpga_Session session,
                                 uint32_t       fifo,
                                 double*        data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        elementsRemaining)
{
   return NiFpga_readFifoDbl
        ? NiFpga_readFifoDbl(session,
                             fifo,
                             data,
                             numberOfElements,
                             timeout,
                             elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_readFifoComposite)(
                                 NiFpga_Session session,
                                 uint32_t       fifo,
                                 void*          data,
                                 uint32_t       bytesPerElement,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        elementsRemaining) = NULL;

NiFpga_Status NiFpga_ReadFifoComposite(NiFpga_Session session,
                                       uint32_t       fifo,
                                       void*          data,
                                       uint32_t       bytesPerElement,
                                       size_t         numberOfElements,
                                       uint32_t       timeout,
                                       size_t*        elementsRemaining)
{
   return NiFpga_readFifoComposite
        ? NiFpga_readFifoComposite(session,
                                   fifo,
                                   data,
                                   bytesPerElement,
                                   numberOfElements,
                                   timeout,
                                   elementsRemaining)
        : NiFpga_Status_VersionMismatch;
}

/*
 * Functions to write to host-to-target DMA FIFOs.
 */
static NiFpga_Status (NiFpga_CCall *NiFpga_writeFifoBool)(
                             NiFpga_Session     session,
                             uint32_t           fifo,
                             const NiFpga_Bool* data,
                             size_t             numberOfElements,
                             uint32_t           timeout,
                             size_t*            emptyElementsRemaining) = NULL;

NiFpga_Status NiFpga_WriteFifoBool(
                             NiFpga_Session     session,
                             uint32_t           fifo,
                             const NiFpga_Bool* data,
                             size_t             numberOfElements,
                             uint32_t           timeout,
                             size_t*            emptyElementsRemaining)
{
   return NiFpga_writeFifoBool
        ? NiFpga_writeFifoBool(session,
                               fifo,
                               data,
                               numberOfElements,
                               timeout,
                               emptyElementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeFifoI8)(
                                 NiFpga_Session session,
                                 uint32_t       fifo,
                                 const int8_t*  data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        emptyElementsRemaining) = NULL;

NiFpga_Status NiFpga_WriteFifoI8(NiFpga_Session session,
                                 uint32_t       fifo,
                                 const int8_t*  data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        emptyElementsRemaining)
{
   return NiFpga_writeFifoI8
        ? NiFpga_writeFifoI8(session,
                             fifo,
                             data,
                             numberOfElements,
                             timeout,
                             emptyElementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeFifoU8)(
                                 NiFpga_Session session,
                                 uint32_t       fifo,
                                 const uint8_t* data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        emptyElementsRemaining) = NULL;

NiFpga_Status NiFpga_WriteFifoU8(NiFpga_Session session,
                                 uint32_t       fifo,
                                 const uint8_t* data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        emptyElementsRemaining)
{
   return NiFpga_writeFifoU8
        ? NiFpga_writeFifoU8(session,
                             fifo,
                             data,
                             numberOfElements,
                             timeout,
                             emptyElementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeFifoI16)(
                                 NiFpga_Session session,
                                 uint32_t       fifo,
                                 const int16_t* data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        emptyElementsRemaining) = NULL;

NiFpga_Status NiFpga_WriteFifoI16(
                                 NiFpga_Session session,
                                 uint32_t       fifo,
                                 const int16_t* data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        emptyElementsRemaining)
{
   return NiFpga_writeFifoI16
        ? NiFpga_writeFifoI16(session,
                              fifo,
                              data,
                              numberOfElements,
                              timeout,
                              emptyElementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeFifoU16)(
                                NiFpga_Session  session,
                                uint32_t        fifo,
                                const uint16_t* data,
                                size_t          numberOfElements,
                                uint32_t        timeout,
                                size_t*         emptyElementsRemaining) = NULL;

NiFpga_Status NiFpga_WriteFifoU16(
                                NiFpga_Session  session,
                                uint32_t        fifo,
                                const uint16_t* data,
                                size_t          numberOfElements,
                                uint32_t        timeout,
                                size_t*         emptyElementsRemaining)
{
   return NiFpga_writeFifoU16
        ? NiFpga_writeFifoU16(session,
                              fifo,
                              data,
                              numberOfElements,
                              timeout,
                              emptyElementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeFifoI32)(
                                 NiFpga_Session session,
                                 uint32_t       fifo,
                                 const int32_t* data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        emptyElementsRemaining) = NULL;

NiFpga_Status NiFpga_WriteFifoI32(
                                 NiFpga_Session session,
                                 uint32_t       fifo,
                                 const int32_t* data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        emptyElementsRemaining)
{
   return NiFpga_writeFifoI32
        ? NiFpga_writeFifoI32(session,
                              fifo,
                              data,
                              numberOfElements,
                              timeout,
                              emptyElementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeFifoU32)(
                                NiFpga_Session  session,
                                uint32_t        fifo,
                                const uint32_t* data,
                                size_t          numberOfElements,
                                uint32_t        timeout,
                                size_t*         emptyElementsRemaining) = NULL;

NiFpga_Status NiFpga_WriteFifoU32(
                                NiFpga_Session  session,
                                uint32_t        fifo,
                                const uint32_t* data,
                                size_t          numberOfElements,
                                uint32_t        timeout,
                                size_t*         emptyElementsRemaining)
{
   return NiFpga_writeFifoU32
        ? NiFpga_writeFifoU32(session,
                              fifo,
                              data,
                              numberOfElements,
                              timeout,
                              emptyElementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeFifoI64)(
                                 NiFpga_Session session,
                                 uint32_t       fifo,
                                 const int64_t* data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        emptyElementsRemaining) = NULL;

NiFpga_Status NiFpga_WriteFifoI64(
                                 NiFpga_Session session,
                                 uint32_t       fifo,
                                 const int64_t* data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        emptyElementsRemaining)
{
   return NiFpga_writeFifoI64
        ? NiFpga_writeFifoI64(session,
                              fifo,
                              data,
                              numberOfElements,
                              timeout,
                              emptyElementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeFifoU64)(
                                NiFpga_Session  session,
                                uint32_t        fifo,
                                const uint64_t* data,
                                size_t          numberOfElements,
                                uint32_t        timeout,
                                size_t*         emptyElementsRemaining) = NULL;

NiFpga_Status NiFpga_WriteFifoU64(
                                NiFpga_Session  session,
                                uint32_t        fifo,
                                const uint64_t* data,
                                size_t          numberOfElements,
                                uint32_t        timeout,
                                size_t*         emptyElementsRemaining)
{
   return NiFpga_writeFifoU64
        ? NiFpga_writeFifoU64(session,
                              fifo,
                              data,
                              numberOfElements,
                              timeout,
                              emptyElementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeFifoSgl)(
                                 NiFpga_Session session,
                                 uint32_t       fifo,
                                 const float*   data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        emptyElementsRemaining) = NULL;

NiFpga_Status NiFpga_WriteFifoSgl(
                                 NiFpga_Session session,
                                 uint32_t       fifo,
                                 const float*   data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        emptyElementsRemaining)
{
   return NiFpga_writeFifoSgl
        ? NiFpga_writeFifoSgl(session,
                              fifo,
                              data,
                              numberOfElements,
                              timeout,
                              emptyElementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeFifoDbl)(
                                 NiFpga_Session session,
                                 uint32_t       fifo,
                                 const double*  data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        emptyElementsRemaining) = NULL;

NiFpga_Status NiFpga_WriteFifoDbl(
                                 NiFpga_Session session,
                                 uint32_t       fifo,
                                 const double*  data,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        emptyElementsRemaining)
{
   return NiFpga_writeFifoDbl
        ? NiFpga_writeFifoDbl(session,
                              fifo,
                              data,
                              numberOfElements,
                              timeout,
                              emptyElementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_writeFifoComposite)(
                                 NiFpga_Session session,
                                 uint32_t       fifo,
                                 const void*    data,
                                 uint32_t       bytesPerElement,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        emptyElementsRemaining) = NULL;

NiFpga_Status NiFpga_WriteFifoComposite(
                                 NiFpga_Session session,
                                 uint32_t       fifo,
                                 const void*    data,
                                 uint32_t       bytesPerElement,
                                 size_t         numberOfElements,
                                 uint32_t       timeout,
                                 size_t*        emptyElementsRemaining)
{
   return NiFpga_writeFifoComposite
        ? NiFpga_writeFifoComposite(session,
                              fifo,
                              data,
                              bytesPerElement,
                              numberOfElements,
                              timeout,
                              emptyElementsRemaining)
        : NiFpga_Status_VersionMismatch;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_acquireFifoReadElementsBool)(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      NiFpga_Bool**  elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining) = NULL;

NiFpga_Status NiFpga_AcquireFifoReadElementsBool(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      NiFpga_Bool**  elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining)
{
   return NiFpga_acquireFifoReadElementsBool
        ? NiFpga_acquireFifoReadElementsBool(session,
                                             fifo,
                                             elements,
                                             elementsRequested,
                                             timeout,
                                             elementsAcquired,
                                             elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_acquireFifoReadElementsI8)(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      int8_t**       elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining) = NULL;

NiFpga_Status NiFpga_AcquireFifoReadElementsI8(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      int8_t**       elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining)
{
   return NiFpga_acquireFifoReadElementsI8
        ? NiFpga_acquireFifoReadElementsI8(session,
                                           fifo,
                                           elements,
                                           elementsRequested,
                                           timeout,
                                           elementsAcquired,
                                           elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_acquireFifoReadElementsU8)(
                                     NiFpga_Session  session,
                                     uint32_t        fifo,
                                     uint8_t**       elements,
                                     size_t          elementsRequested,
                                     uint32_t        timeout,
                                     size_t*         elementsAcquired,
                                     size_t*         elementsRemaining) = NULL;

NiFpga_Status NiFpga_AcquireFifoReadElementsU8(
                                     NiFpga_Session  session,
                                     uint32_t        fifo,
                                     uint8_t**       elements,
                                     size_t          elementsRequested,
                                     uint32_t        timeout,
                                     size_t*         elementsAcquired,
                                     size_t*         elementsRemaining)
{
   return NiFpga_acquireFifoReadElementsU8
        ? NiFpga_acquireFifoReadElementsU8(session,
                                           fifo,
                                           elements,
                                           elementsRequested,
                                           timeout,
                                           elementsAcquired,
                                           elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_acquireFifoReadElementsI16)(
                                     NiFpga_Session  session,
                                     uint32_t        fifo,
                                     int16_t**       elements,
                                     size_t          elementsRequested,
                                     uint32_t        timeout,
                                     size_t*         elementsAcquired,
                                     size_t*         elementsRemaining) = NULL;

NiFpga_Status NiFpga_AcquireFifoReadElementsI16(
                                     NiFpga_Session  session,
                                     uint32_t        fifo,
                                     int16_t**       elements,
                                     size_t          elementsRequested,
                                     uint32_t        timeout,
                                     size_t*         elementsAcquired,
                                     size_t*         elementsRemaining)
{
   return NiFpga_acquireFifoReadElementsI16
        ? NiFpga_acquireFifoReadElementsI16(session,
                                            fifo,
                                            elements,
                                            elementsRequested,
                                            timeout,
                                            elementsAcquired,
                                            elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_acquireFifoReadElementsU16)(
                                    NiFpga_Session   session,
                                    uint32_t         fifo,
                                    uint16_t**       elements,
                                    size_t           elementsRequested,
                                    uint32_t         timeout,
                                    size_t*          elementsAcquired,
                                    size_t*          elementsRemaining) = NULL;

NiFpga_Status NiFpga_AcquireFifoReadElementsU16(
                                    NiFpga_Session   session,
                                    uint32_t         fifo,
                                    uint16_t**       elements,
                                    size_t           elementsRequested,
                                    uint32_t         timeout,
                                    size_t*          elementsAcquired,
                                    size_t*          elementsRemaining)
{
   return NiFpga_acquireFifoReadElementsU16
        ? NiFpga_acquireFifoReadElementsU16(session,
                                            fifo,
                                            elements,
                                            elementsRequested,
                                            timeout,
                                            elementsAcquired,
                                            elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_acquireFifoReadElementsI32)(
                                     NiFpga_Session  session,
                                     uint32_t        fifo,
                                     int32_t**       elements,
                                     size_t          elementsRequested,
                                     uint32_t        timeout,
                                     size_t*         elementsAcquired,
                                     size_t*         elementsRemaining) = NULL;

NiFpga_Status NiFpga_AcquireFifoReadElementsI32(
                                     NiFpga_Session  session,
                                     uint32_t        fifo,
                                     int32_t**       elements,
                                     size_t          elementsRequested,
                                     uint32_t        timeout,
                                     size_t*         elementsAcquired,
                                     size_t*         elementsRemaining)
{
   return NiFpga_acquireFifoReadElementsI32
        ? NiFpga_acquireFifoReadElementsI32(session,
                                            fifo,
                                            elements,
                                            elementsRequested,
                                            timeout,
                                            elementsAcquired,
                                            elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_acquireFifoReadElementsU32)(
                                    NiFpga_Session   session,
                                    uint32_t         fifo,
                                    uint32_t**       elements,
                                    size_t           elementsRequested,
                                    uint32_t         timeout,
                                    size_t*          elementsAcquired,
                                    size_t*          elementsRemaining) = NULL;

NiFpga_Status NiFpga_AcquireFifoReadElementsU32(
                                    NiFpga_Session   session,
                                    uint32_t         fifo,
                                    uint32_t**       elements,
                                    size_t           elementsRequested,
                                    uint32_t         timeout,
                                    size_t*          elementsAcquired,
                                    size_t*          elementsRemaining)
{
   return NiFpga_acquireFifoReadElementsU32
        ? NiFpga_acquireFifoReadElementsU32(session,
                                            fifo,
                                            elements,
                                            elementsRequested,
                                            timeout,
                                            elementsAcquired,
                                            elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_acquireFifoReadElementsI64)(
                                     NiFpga_Session  session,
                                     uint32_t        fifo,
                                     int64_t**       elements,
                                     size_t          elementsRequested,
                                     uint32_t        timeout,
                                     size_t*         elementsAcquired,
                                     size_t*         elementsRemaining) = NULL;

NiFpga_Status NiFpga_AcquireFifoReadElementsI64(
                                     NiFpga_Session  session,
                                     uint32_t        fifo,
                                     int64_t**       elements,
                                     size_t          elementsRequested,
                                     uint32_t        timeout,
                                     size_t*         elementsAcquired,
                                     size_t*         elementsRemaining)
{
   return NiFpga_acquireFifoReadElementsI64
        ? NiFpga_acquireFifoReadElementsI64(session,
                                            fifo,
                                            elements,
                                            elementsRequested,
                                            timeout,
                                            elementsAcquired,
                                            elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_acquireFifoReadElementsU64)(
                                    NiFpga_Session   session,
                                    uint32_t         fifo,
                                    uint64_t**       elements,
                                    size_t           elementsRequested,
                                    uint32_t         timeout,
                                    size_t*          elementsAcquired,
                                    size_t*          elementsRemaining) = NULL;

NiFpga_Status NiFpga_AcquireFifoReadElementsU64(
                                    NiFpga_Session   session,
                                    uint32_t         fifo,
                                    uint64_t**       elements,
                                    size_t           elementsRequested,
                                    uint32_t         timeout,
                                    size_t*          elementsAcquired,
                                    size_t*          elementsRemaining)
{
   return NiFpga_acquireFifoReadElementsU64
        ? NiFpga_acquireFifoReadElementsU64(session,
                                            fifo,
                                            elements,
                                            elementsRequested,
                                            timeout,
                                            elementsAcquired,
                                            elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_acquireFifoReadElementsSgl)(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      float**        elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining) = NULL;

NiFpga_Status NiFpga_AcquireFifoReadElementsSgl(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      float**        elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining)
{
   return NiFpga_acquireFifoReadElementsSgl
        ? NiFpga_acquireFifoReadElementsSgl(session,
                                            fifo,
                                            elements,
                                            elementsRequested,
                                            timeout,
                                            elementsAcquired,
                                            elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_acquireFifoReadElementsDbl)(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      double**       elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining) = NULL;

NiFpga_Status NiFpga_AcquireFifoReadElementsDbl(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      double**       elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining)
{
   return NiFpga_acquireFifoReadElementsDbl
        ? NiFpga_acquireFifoReadElementsDbl(session,
                                            fifo,
                                            elements,
                                            elementsRequested,
                                            timeout,
                                            elementsAcquired,
                                            elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_acquireFifoWriteElementsBool)(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      NiFpga_Bool**  elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining) = NULL;

NiFpga_Status NiFpga_AcquireFifoWriteElementsBool(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      NiFpga_Bool**  elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining)
{
   return NiFpga_acquireFifoWriteElementsBool
        ? NiFpga_acquireFifoWriteElementsBool(session,
                                              fifo,
                                              elements,
                                              elementsRequested,
                                              timeout,
                                              elementsAcquired,
                                              elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_acquireFifoWriteElementsI8)(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      int8_t**       elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining) = NULL;

NiFpga_Status NiFpga_AcquireFifoWriteElementsI8(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      int8_t**       elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining)
{
   return NiFpga_acquireFifoWriteElementsI8
        ? NiFpga_acquireFifoWriteElementsI8(session,
                                            fifo,
                                            elements,
                                            elementsRequested,
                                            timeout,
                                            elementsAcquired,
                                            elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_acquireFifoWriteElementsU8)(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      uint8_t**      elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining) = NULL;

NiFpga_Status NiFpga_AcquireFifoWriteElementsU8(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      uint8_t**      elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining)
{
   return NiFpga_acquireFifoWriteElementsU8
        ? NiFpga_acquireFifoWriteElementsU8(session,
                                            fifo,
                                            elements,
                                            elementsRequested,
                                            timeout,
                                            elementsAcquired,
                                            elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_acquireFifoWriteElementsI16)(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      int16_t**      elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining) = NULL;

NiFpga_Status NiFpga_AcquireFifoWriteElementsI16(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      int16_t**      elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining)
{
   return NiFpga_acquireFifoWriteElementsI16
        ? NiFpga_acquireFifoWriteElementsI16(session,
                                             fifo,
                                             elements,
                                             elementsRequested,
                                             timeout,
                                             elementsAcquired,
                                             elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_acquireFifoWriteElementsU16)(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      uint16_t**     elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining) = NULL;

NiFpga_Status NiFpga_AcquireFifoWriteElementsU16(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      uint16_t**     elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining)
{
   return NiFpga_acquireFifoWriteElementsU16
        ? NiFpga_acquireFifoWriteElementsU16(session,
                                             fifo,
                                             elements,
                                             elementsRequested,
                                             timeout,
                                             elementsAcquired,
                                             elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_acquireFifoWriteElementsI32)(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      int32_t**      elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining) = NULL;

NiFpga_Status NiFpga_AcquireFifoWriteElementsI32(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      int32_t**      elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining)
{
   return NiFpga_acquireFifoWriteElementsI32
        ? NiFpga_acquireFifoWriteElementsI32(session,
                                             fifo,
                                             elements,
                                             elementsRequested,
                                             timeout,
                                             elementsAcquired,
                                             elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_acquireFifoWriteElementsU32)(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      uint32_t**     elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining) = NULL;

NiFpga_Status NiFpga_AcquireFifoWriteElementsU32(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      uint32_t**     elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining)
{
   return NiFpga_acquireFifoWriteElementsU32
        ? NiFpga_acquireFifoWriteElementsU32(session,
                                             fifo,
                                             elements,
                                             elementsRequested,
                                             timeout,
                                             elementsAcquired,
                                             elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_acquireFifoWriteElementsI64)(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      int64_t**      elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining) = NULL;

NiFpga_Status NiFpga_AcquireFifoWriteElementsI64(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      int64_t**      elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining)
{
   return NiFpga_acquireFifoWriteElementsI64
        ? NiFpga_acquireFifoWriteElementsI64(session,
                                             fifo,
                                             elements,
                                             elementsRequested,
                                             timeout,
                                             elementsAcquired,
                                             elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_acquireFifoWriteElementsU64)(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      uint64_t**     elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining) = NULL;

NiFpga_Status NiFpga_AcquireFifoWriteElementsU64(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      uint64_t**     elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining)
{
   return NiFpga_acquireFifoWriteElementsU64
        ? NiFpga_acquireFifoWriteElementsU64(session,
                                             fifo,
                                             elements,
                                             elementsRequested,
                                             timeout,
                                             elementsAcquired,
                                             elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_acquireFifoWriteElementsSgl)(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      float**        elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining) = NULL;

NiFpga_Status NiFpga_AcquireFifoWriteElementsSgl(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      float**        elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining)
{
   return NiFpga_acquireFifoWriteElementsSgl
        ? NiFpga_acquireFifoWriteElementsSgl(session,
                                             fifo,
                                             elements,
                                             elementsRequested,
                                             timeout,
                                             elementsAcquired,
                                             elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_acquireFifoWriteElementsDbl)(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      double**       elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining) = NULL;

NiFpga_Status NiFpga_AcquireFifoWriteElementsDbl(
                                      NiFpga_Session session,
                                      uint32_t       fifo,
                                      double**       elements,
                                      size_t         elementsRequested,
                                      uint32_t       timeout,
                                      size_t*        elementsAcquired,
                                      size_t*        elementsRemaining)
{
   return NiFpga_acquireFifoWriteElementsDbl
        ? NiFpga_acquireFifoWriteElementsDbl(session,
                                             fifo,
                                             elements,
                                             elementsRequested,
                                             timeout,
                                             elementsAcquired,
                                             elementsRemaining)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_releaseFifoElements)(
                                         NiFpga_Session session,
                                         uint32_t       fifo,
                                         size_t         elements) = NULL;

NiFpga_Status NiFpga_ReleaseFifoElements(NiFpga_Session session,
                                         uint32_t       fifo,
                                         size_t         elements)
{
   return NiFpga_releaseFifoElements
        ? NiFpga_releaseFifoElements(session, fifo, elements)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_getPeerToPeerFifoEndpoint)(
                                         NiFpga_Session session,
                                         uint32_t       fifo,
                                         uint32_t*      endpoint) = NULL;

NiFpga_Status NiFpga_GetPeerToPeerFifoEndpoint(NiFpga_Session session,
                                         uint32_t       fifo,
                                         uint32_t*      endpoint)
{
   return NiFpga_getPeerToPeerFifoEndpoint
        ? NiFpga_getPeerToPeerFifoEndpoint(session, fifo, endpoint)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_mapP2PSinkFifo)(
                                    const NiFpga_Session session,
                                    const uint32_t       fifo,
                                    size_t*              size,
                                    void**               virtualAddress) = NULL;

NiFpga_Status NiFpga_MapP2PSinkFifo(const NiFpga_Session session,
                                    const uint32_t       fifo,
                                    size_t*              size,
                                    void**               virtualAddress)
{
   return NiFpga_mapP2PSinkFifo
        ? NiFpga_mapP2PSinkFifo(session, fifo, size, virtualAddress)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_unmapP2PSinkFifo)(
                                      const NiFpga_Session session,
                                      const uint32_t       fifo) = NULL;

NiFpga_Status NiFpga_UnmapP2PSinkFifo(const NiFpga_Session session,
                                      const uint32_t       fifo)
{
   return NiFpga_unmapP2PSinkFifo
        ? NiFpga_unmapP2PSinkFifo(session, fifo)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_getBitfileContents)(
                                        NiFpga_Session session,
                                        const char**   contents) = NULL;

NiFpga_Status NiFpga_GetBitfileContents(NiFpga_Session session,
                                        const char**   contents)
{
   return NiFpga_getBitfileContents
        ? NiFpga_getBitfileContents(session, contents)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_findRegisterPrivate)(
                                         NiFpga_Session session,
                                         const char* resourceName,
                                         uint32_t  expectedResourceType,
                                         uint32_t* resourceOffset) = NULL;

NiFpga_Status NiFpga_FindRegisterPrivate(NiFpga_Session session,
                                         const char* resourceName,
                                         uint32_t  expectedResourceType,
                                         uint32_t* resourceOffset)
{
   return NiFpga_findRegisterPrivate
        ? NiFpga_findRegisterPrivate(session, resourceName, expectedResourceType, resourceOffset)
        : NiFpga_Status_VersionMismatch;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_findFifoPrivate)(
                                     NiFpga_Session session,
                                     const char* resourceName,
                                     uint32_t  expectedResourceType,
                                     uint32_t* resourceOffset) = NULL;

NiFpga_Status NiFpga_FindFifoPrivate(NiFpga_Session session,
                                     const char* fifoName,
                                     uint32_t  expectedResourceType,
                                     uint32_t* resourceOffset)
{
   return NiFpga_findFifoPrivate
        ? NiFpga_findFifoPrivate(session, fifoName, expectedResourceType, resourceOffset)
        : NiFpga_Status_VersionMismatch;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_openHostMemoryBuffer)(
                                          const NiFpga_Session session,
                                          const char*          memoryName,
                                          size_t*              memorySize,
                                          void**               virtualAddress) = NULL;

NiFpga_Status NiFpga_OpenHostMemoryBuffer(const NiFpga_Session session,
                                          const char*          memoryName,
                                          size_t*              memorySize,
                                          void**               virtualAddress)
{
   return NiFpga_openHostMemoryBuffer
        ? NiFpga_openHostMemoryBuffer(session, memoryName, memorySize, virtualAddress)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_closeHostMemoryBuffer)(
                                           const NiFpga_Session session,
                                           const char*          memoryName) = NULL;

NiFpga_Status NiFpga_CloseHostMemoryBuffer(const NiFpga_Session session,
                                           const char*          memoryName)
{
   return NiFpga_closeHostMemoryBuffer
        ? NiFpga_closeHostMemoryBuffer(session, memoryName)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_openLowLatencyBuffer)(
                                          const NiFpga_Session session,
                                          const char*          memoryName,
                                          size_t*              memorySizeToHost,
                                          void**               virtualAddressToHost,
                                          size_t*              memorySizeToFpga,
                                          void**               virtualAddressToFpga) = NULL;

NiFpga_Status NiFpga_OpenLowLatencyBuffer(const NiFpga_Session session,
                                          const char*          memoryName,
                                          size_t*              memorySizeToHost,
                                          void**               virtualAddressToHost,
                                          size_t*              memorySizeToFpga,
                                          void**               virtualAddressToFpga)
{
   return NiFpga_openLowLatencyBuffer
        ? NiFpga_openLowLatencyBuffer(session, memoryName, memorySizeToHost, virtualAddressToHost, memorySizeToFpga, virtualAddressToFpga)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_closeLowLatencyBuffer)(
                                           const NiFpga_Session session,
                                           const char*          memoryName) = NULL;

NiFpga_Status NiFpga_CloseLowLatencyBuffer(const NiFpga_Session session,
                                           const char*          memoryName)
{
   return NiFpga_closeLowLatencyBuffer
        ? NiFpga_closeLowLatencyBuffer(session, memoryName)
        : NiFpga_Status_ResourceNotInitialized;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_clientFunctionCall)(
                                        NiFpga_Session session,
                                        uint32_t group,
                                        uint32_t functionId,
                                        const void* inBuffer,
                                        size_t inBufferSize,
                                        void* outBuffer,
                                        size_t outBufferSize) = NULL;

NiFpga_Status NiFpga_ClientFunctionCall(NiFpga_Session session,
                                        uint32_t group,
                                        uint32_t functionId,
                                        const void* inBuffer,
                                        size_t inBufferSize,
                                        void* outBuffer,
                                        size_t outBufferSize)
{
   return NiFpga_clientFunctionCall
        ? NiFpga_clientFunctionCall(session, group, functionId, inBuffer, inBufferSize, outBuffer, outBufferSize)
        : NiFpga_Status_VersionMismatch;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_findRegister)(
                                         NiFpga_Session session,
                                         const char* registerName,
                                         uint32_t* registerOffset) = NULL;

NiFpga_Status NiFpga_FindRegister(NiFpga_Session session,
                                         const char* registerName,
                                         uint32_t* registerOffset)
{
   return NiFpga_findRegister
        ? NiFpga_findRegister(session, registerName, registerOffset)
        : NiFpga_Status_VersionMismatch;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_findFifo)(
                                     NiFpga_Session session,
                                     const char* fifoName,
                                     uint32_t* fifoNumber) = NULL;

NiFpga_Status NiFpga_FindFifo(NiFpga_Session session,
                              const char* fifoName,
                              uint32_t* fifoNumber)
{
   return NiFpga_findFifo
        ? NiFpga_findFifo(session, fifoName, fifoNumber)
        : NiFpga_Status_VersionMismatch;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_getFpgaViState)(
                                     NiFpga_Session session,
                                     uint32_t* state) = NULL;

NiFpga_Status NiFpga_GetFpgaViState(NiFpga_Session session, uint32_t* state)
{
   return NiFpga_getFpgaViState
        ? NiFpga_getFpgaViState(session, state)
        : NiFpga_Status_VersionMismatch;
}

static NiFpga_Status (NiFpga_CCall *NiFpga_getBitfileSignature)(
                                     NiFpga_Session session,
                                     uint32_t* signature,
                                     size_t* signatureSize) = NULL;

NiFpga_Status NiFpga_GetBitfileSignature(NiFpga_Session session, uint32_t* state, size_t* signatureSize)
{
   return NiFpga_getBitfileSignature
        ? NiFpga_getBitfileSignature(session, state, signatureSize)
        : NiFpga_Status_VersionMismatch;
}

/**
 * A type large enough to hold entry point function pointer.
 */
typedef NiFpga_Status (NiFpga_CCall *NiFpga_FunctionPointer)();

/**
 * A NULL-terminated array of all entry point functions.
 */
static const struct
{
   const char* const name;
   NiFpga_FunctionPointer* const address;
   NiFpga_Bool required;
} NiFpga_functions[] =
{
   {"NiFpgaDll_Open",                (NiFpga_FunctionPointer*)&NiFpga_open, 1},
   {"NiFpgaDll_Close",               (NiFpga_FunctionPointer*)&NiFpga_close, 1},
   {"NiFpgaDll_Run",                 (NiFpga_FunctionPointer*)&NiFpga_run, 1},
   {"NiFpgaDll_Abort",               (NiFpga_FunctionPointer*)&NiFpga_abort, 1},
   {"NiFpgaDll_Reset",               (NiFpga_FunctionPointer*)&NiFpga_reset, 1},
   {"NiFpgaDll_Download",            (NiFpga_FunctionPointer*)&NiFpga_download, 1},
   {"NiFpgaDll_ReadBool",            (NiFpga_FunctionPointer*)&NiFpga_readBool, 1},
   {"NiFpgaDll_ReadI8",              (NiFpga_FunctionPointer*)&NiFpga_readI8, 1},
   {"NiFpgaDll_ReadU8",              (NiFpga_FunctionPointer*)&NiFpga_readU8, 1},
   {"NiFpgaDll_ReadI16",             (NiFpga_FunctionPointer*)&NiFpga_readI16, 1},
   {"NiFpgaDll_ReadU16",             (NiFpga_FunctionPointer*)&NiFpga_readU16, 1},
   {"NiFpgaDll_ReadI32",             (NiFpga_FunctionPointer*)&NiFpga_readI32, 1},
   {"NiFpgaDll_ReadU32",             (NiFpga_FunctionPointer*)&NiFpga_readU32, 1},
   {"NiFpgaDll_ReadI64",             (NiFpga_FunctionPointer*)&NiFpga_readI64, 1},
   {"NiFpgaDll_ReadU64",             (NiFpga_FunctionPointer*)&NiFpga_readU64, 1},
   {"NiFpgaDll_ReadSgl",             (NiFpga_FunctionPointer*)&NiFpga_readSgl, 1},
   {"NiFpgaDll_ReadDbl",             (NiFpga_FunctionPointer*)&NiFpga_readDbl, 1},
   {"NiFpgaDll_WriteBool",           (NiFpga_FunctionPointer*)&NiFpga_writeBool, 1},
   {"NiFpgaDll_WriteI8",             (NiFpga_FunctionPointer*)&NiFpga_writeI8, 1},
   {"NiFpgaDll_WriteU8",             (NiFpga_FunctionPointer*)&NiFpga_writeU8, 1},
   {"NiFpgaDll_WriteI16",            (NiFpga_FunctionPointer*)&NiFpga_writeI16, 1},
   {"NiFpgaDll_WriteU16",            (NiFpga_FunctionPointer*)&NiFpga_writeU16, 1},
   {"NiFpgaDll_WriteI32",            (NiFpga_FunctionPointer*)&NiFpga_writeI32, 1},
   {"NiFpgaDll_WriteU32",            (NiFpga_FunctionPointer*)&NiFpga_writeU32, 1},
   {"NiFpgaDll_WriteI64",            (NiFpga_FunctionPointer*)&NiFpga_writeI64, 1},
   {"NiFpgaDll_WriteU64",            (NiFpga_FunctionPointer*)&NiFpga_writeU64, 1},
   {"NiFpgaDll_WriteSgl",            (NiFpga_FunctionPointer*)&NiFpga_writeSgl, 1},
   {"NiFpgaDll_WriteDbl",            (NiFpga_FunctionPointer*)&NiFpga_writeDbl, 1},
   {"NiFpgaDll_ReadArrayBool",       (NiFpga_FunctionPointer*)&NiFpga_readArrayBool, 1},
   {"NiFpgaDll_ReadArrayI8",         (NiFpga_FunctionPointer*)&NiFpga_readArrayI8, 1},
   {"NiFpgaDll_ReadArrayU8",         (NiFpga_FunctionPointer*)&NiFpga_readArrayU8, 1},
   {"NiFpgaDll_ReadArrayI16",        (NiFpga_FunctionPointer*)&NiFpga_readArrayI16, 1},
   {"NiFpgaDll_ReadArrayU16",        (NiFpga_FunctionPointer*)&NiFpga_readArrayU16, 1},
   {"NiFpgaDll_ReadArrayI32",        (NiFpga_FunctionPointer*)&NiFpga_readArrayI32, 1},
   {"NiFpgaDll_ReadArrayU32",        (NiFpga_FunctionPointer*)&NiFpga_readArrayU32, 1},
   {"NiFpgaDll_ReadArrayI64",        (NiFpga_FunctionPointer*)&NiFpga_readArrayI64, 1},
   {"NiFpgaDll_ReadArrayU64",        (NiFpga_FunctionPointer*)&NiFpga_readArrayU64, 1},
   {"NiFpgaDll_ReadArraySgl",        (NiFpga_FunctionPointer*)&NiFpga_readArraySgl, 1},
   {"NiFpgaDll_ReadArrayDbl",        (NiFpga_FunctionPointer*)&NiFpga_readArrayDbl, 1},
   {"NiFpgaDll_WriteArrayBool",      (NiFpga_FunctionPointer*)&NiFpga_writeArrayBool, 1},
   {"NiFpgaDll_WriteArrayI8",        (NiFpga_FunctionPointer*)&NiFpga_writeArrayI8, 1},
   {"NiFpgaDll_WriteArrayU8",        (NiFpga_FunctionPointer*)&NiFpga_writeArrayU8, 1},
   {"NiFpgaDll_WriteArrayI16",       (NiFpga_FunctionPointer*)&NiFpga_writeArrayI16, 1},
   {"NiFpgaDll_WriteArrayU16",       (NiFpga_FunctionPointer*)&NiFpga_writeArrayU16, 1},
   {"NiFpgaDll_WriteArrayI32",       (NiFpga_FunctionPointer*)&NiFpga_writeArrayI32, 1},
   {"NiFpgaDll_WriteArrayU32",       (NiFpga_FunctionPointer*)&NiFpga_writeArrayU32, 1},
   {"NiFpgaDll_WriteArrayI64",       (NiFpga_FunctionPointer*)&NiFpga_writeArrayI64, 1},
   {"NiFpgaDll_WriteArrayU64",       (NiFpga_FunctionPointer*)&NiFpga_writeArrayU64, 1},
   {"NiFpgaDll_WriteArraySgl",       (NiFpga_FunctionPointer*)&NiFpga_writeArraySgl, 1},
   {"NiFpgaDll_WriteArrayDbl",       (NiFpga_FunctionPointer*)&NiFpga_writeArrayDbl, 1},
   {"NiFpgaDll_ReserveIrqContext",   (NiFpga_FunctionPointer*)&NiFpga_reserveIrqContext, 1},
   {"NiFpgaDll_UnreserveIrqContext", (NiFpga_FunctionPointer*)&NiFpga_unreserveIrqContext, 1},
   {"NiFpgaDll_WaitOnIrqs",          (NiFpga_FunctionPointer*)&NiFpga_waitOnIrqs, 1},
   {"NiFpgaDll_AcknowledgeIrqs",     (NiFpga_FunctionPointer*)&NiFpga_acknowledgeIrqs, 1},
   {"NiFpgaDll_ConfigureFifo",       (NiFpga_FunctionPointer*)&NiFpga_configureFifo, 1},
   {"NiFpgaDll_ConfigureFifo2",      (NiFpga_FunctionPointer*)&NiFpga_configureFifo2, 1},
   {"NiFpgaDll_SetFifoPropertyU32",  (NiFpga_FunctionPointer*)&NiFpga_setFifoPropertyU32, 0},
   {"NiFpgaDll_SetFifoPropertyI32",  (NiFpga_FunctionPointer*)&NiFpga_setFifoPropertyI32, 0},
   {"NiFpgaDll_SetFifoPropertyU64",  (NiFpga_FunctionPointer*)&NiFpga_setFifoPropertyU64, 0},
   {"NiFpgaDll_SetFifoPropertyI64",  (NiFpga_FunctionPointer*)&NiFpga_setFifoPropertyI64, 0},
   {"NiFpgaDll_SetFifoPropertyPtr",  (NiFpga_FunctionPointer*)&NiFpga_setFifoPropertyPtr, 0},
   {"NiFpgaDll_GetFifoPropertyU32",  (NiFpga_FunctionPointer*)&NiFpga_getFifoPropertyU32, 0},
   {"NiFpgaDll_GetFifoPropertyI32",  (NiFpga_FunctionPointer*)&NiFpga_getFifoPropertyI32, 0},
   {"NiFpgaDll_GetFifoPropertyU64",  (NiFpga_FunctionPointer*)&NiFpga_getFifoPropertyU64, 0},
   {"NiFpgaDll_GetFifoPropertyI64",  (NiFpga_FunctionPointer*)&NiFpga_getFifoPropertyI64, 0},
   {"NiFpgaDll_GetFifoPropertyPtr",  (NiFpga_FunctionPointer*)&NiFpga_getFifoPropertyPtr, 0},
   {"NiFpgaDll_CommitFifoConfiguration", (NiFpga_FunctionPointer*)&NiFpga_commitFifoConfiguration, 0},
   {"NiFpgaDll_StartFifo",           (NiFpga_FunctionPointer*)&NiFpga_startFifo, 1},
   {"NiFpgaDll_StopFifo",            (NiFpga_FunctionPointer*)&NiFpga_stopFifo, 1},
   {"NiFpgaDll_UnreserveFifo",       (NiFpga_FunctionPointer*)&NiFpga_unreserveFifo, 0},
   {"NiFpgaDll_ReadFifoBool",        (NiFpga_FunctionPointer*)&NiFpga_readFifoBool, 1},
   {"NiFpgaDll_ReadFifoI8",          (NiFpga_FunctionPointer*)&NiFpga_readFifoI8, 1},
   {"NiFpgaDll_ReadFifoU8",          (NiFpga_FunctionPointer*)&NiFpga_readFifoU8, 1},
   {"NiFpgaDll_ReadFifoI16",         (NiFpga_FunctionPointer*)&NiFpga_readFifoI16, 1},
   {"NiFpgaDll_ReadFifoU16",         (NiFpga_FunctionPointer*)&NiFpga_readFifoU16, 1},
   {"NiFpgaDll_ReadFifoI32",         (NiFpga_FunctionPointer*)&NiFpga_readFifoI32, 1},
   {"NiFpgaDll_ReadFifoU32",         (NiFpga_FunctionPointer*)&NiFpga_readFifoU32, 1},
   {"NiFpgaDll_ReadFifoI64",         (NiFpga_FunctionPointer*)&NiFpga_readFifoI64, 1},
   {"NiFpgaDll_ReadFifoU64",         (NiFpga_FunctionPointer*)&NiFpga_readFifoU64, 1},
   {"NiFpgaDll_ReadFifoSgl",         (NiFpga_FunctionPointer*)&NiFpga_readFifoSgl, 1},
   {"NiFpgaDll_ReadFifoDbl",         (NiFpga_FunctionPointer*)&NiFpga_readFifoDbl, 1},
   {"NiFpgaDll_ReadFifoComposite",   (NiFpga_FunctionPointer*)&NiFpga_readFifoComposite, 0},
   {"NiFpgaDll_WriteFifoBool",       (NiFpga_FunctionPointer*)&NiFpga_writeFifoBool, 1},
   {"NiFpgaDll_WriteFifoI8",         (NiFpga_FunctionPointer*)&NiFpga_writeFifoI8, 1},
   {"NiFpgaDll_WriteFifoU8",         (NiFpga_FunctionPointer*)&NiFpga_writeFifoU8, 1},
   {"NiFpgaDll_WriteFifoI16",        (NiFpga_FunctionPointer*)&NiFpga_writeFifoI16, 1},
   {"NiFpgaDll_WriteFifoU16",        (NiFpga_FunctionPointer*)&NiFpga_writeFifoU16, 1},
   {"NiFpgaDll_WriteFifoI32",        (NiFpga_FunctionPointer*)&NiFpga_writeFifoI32, 1},
   {"NiFpgaDll_WriteFifoU32",        (NiFpga_FunctionPointer*)&NiFpga_writeFifoU32, 1},
   {"NiFpgaDll_WriteFifoI64",        (NiFpga_FunctionPointer*)&NiFpga_writeFifoI64, 1},
   {"NiFpgaDll_WriteFifoU64",        (NiFpga_FunctionPointer*)&NiFpga_writeFifoU64, 1},
   {"NiFpgaDll_WriteFifoSgl",        (NiFpga_FunctionPointer*)&NiFpga_writeFifoSgl, 1},
   {"NiFpgaDll_WriteFifoDbl",        (NiFpga_FunctionPointer*)&NiFpga_writeFifoDbl, 1},
   {"NiFpgaDll_WriteFifoComposite",  (NiFpga_FunctionPointer*)&NiFpga_writeFifoComposite, 0},
   {"NiFpgaDll_AcquireFifoReadElementsBool",  (NiFpga_FunctionPointer*)&NiFpga_acquireFifoReadElementsBool, 1},
   {"NiFpgaDll_AcquireFifoReadElementsI8",    (NiFpga_FunctionPointer*)&NiFpga_acquireFifoReadElementsI8, 1},
   {"NiFpgaDll_AcquireFifoReadElementsU8",    (NiFpga_FunctionPointer*)&NiFpga_acquireFifoReadElementsU8, 1},
   {"NiFpgaDll_AcquireFifoReadElementsI16",   (NiFpga_FunctionPointer*)&NiFpga_acquireFifoReadElementsI16, 1},
   {"NiFpgaDll_AcquireFifoReadElementsU16",   (NiFpga_FunctionPointer*)&NiFpga_acquireFifoReadElementsU16, 1},
   {"NiFpgaDll_AcquireFifoReadElementsI32",   (NiFpga_FunctionPointer*)&NiFpga_acquireFifoReadElementsI32, 1},
   {"NiFpgaDll_AcquireFifoReadElementsU32",   (NiFpga_FunctionPointer*)&NiFpga_acquireFifoReadElementsU32, 1},
   {"NiFpgaDll_AcquireFifoReadElementsI64",   (NiFpga_FunctionPointer*)&NiFpga_acquireFifoReadElementsI64, 1},
   {"NiFpgaDll_AcquireFifoReadElementsU64",   (NiFpga_FunctionPointer*)&NiFpga_acquireFifoReadElementsU64, 1},
   {"NiFpgaDll_AcquireFifoReadElementsSgl",   (NiFpga_FunctionPointer*)&NiFpga_acquireFifoReadElementsSgl, 1},
   {"NiFpgaDll_AcquireFifoReadElementsDbl",   (NiFpga_FunctionPointer*)&NiFpga_acquireFifoReadElementsDbl, 1},
   {"NiFpgaDll_AcquireFifoWriteElementsBool", (NiFpga_FunctionPointer*)&NiFpga_acquireFifoWriteElementsBool, 1},
   {"NiFpgaDll_AcquireFifoWriteElementsI8",   (NiFpga_FunctionPointer*)&NiFpga_acquireFifoWriteElementsI8, 1},
   {"NiFpgaDll_AcquireFifoWriteElementsU8",   (NiFpga_FunctionPointer*)&NiFpga_acquireFifoWriteElementsU8, 1},
   {"NiFpgaDll_AcquireFifoWriteElementsI16",  (NiFpga_FunctionPointer*)&NiFpga_acquireFifoWriteElementsI16, 1},
   {"NiFpgaDll_AcquireFifoWriteElementsU16",  (NiFpga_FunctionPointer*)&NiFpga_acquireFifoWriteElementsU16, 1},
   {"NiFpgaDll_AcquireFifoWriteElementsI32",  (NiFpga_FunctionPointer*)&NiFpga_acquireFifoWriteElementsI32, 1},
   {"NiFpgaDll_AcquireFifoWriteElementsU32",  (NiFpga_FunctionPointer*)&NiFpga_acquireFifoWriteElementsU32, 1},
   {"NiFpgaDll_AcquireFifoWriteElementsI64",  (NiFpga_FunctionPointer*)&NiFpga_acquireFifoWriteElementsI64, 1},
   {"NiFpgaDll_AcquireFifoWriteElementsU64",  (NiFpga_FunctionPointer*)&NiFpga_acquireFifoWriteElementsU64, 1},
   {"NiFpgaDll_AcquireFifoWriteElementsSgl",  (NiFpga_FunctionPointer*)&NiFpga_acquireFifoWriteElementsSgl, 1},
   {"NiFpgaDll_AcquireFifoWriteElementsDbl",  (NiFpga_FunctionPointer*)&NiFpga_acquireFifoWriteElementsDbl, 1},
   {"NiFpgaDll_ReleaseFifoElements",          (NiFpga_FunctionPointer*)&NiFpga_releaseFifoElements, 1},
   {"NiFpgaDll_GetPeerToPeerFifoEndpoint",    (NiFpga_FunctionPointer*)&NiFpga_getPeerToPeerFifoEndpoint, 1},
   {"NiFpgaDll_MapP2PSinkFifo",               (NiFpga_FunctionPointer*)&NiFpga_mapP2PSinkFifo, 0},
   {"NiFpgaDll_UnmapP2PSinkFifo",             (NiFpga_FunctionPointer*)&NiFpga_unmapP2PSinkFifo, 0},
   {"NiFpgaDll_GetBitfileContents",           (NiFpga_FunctionPointer*)&NiFpga_getBitfileContents, 1},
   {"NiFpgaDll_FindRegisterPrivate",          (NiFpga_FunctionPointer*)&NiFpga_findRegisterPrivate, 0},
   {"NiFpgaDll_FindFifoPrivate",              (NiFpga_FunctionPointer*)&NiFpga_findFifoPrivate, 0},
   {"NiFpgaDll_OpenHmb",                      (NiFpga_FunctionPointer*)&NiFpga_openHostMemoryBuffer, 0},
   {"NiFpgaDll_CloseHmb",                     (NiFpga_FunctionPointer*)&NiFpga_closeHostMemoryBuffer, 0},
   {"NiFpgaDll_OpenLlb",                      (NiFpga_FunctionPointer*)&NiFpga_openLowLatencyBuffer, 0},
   {"NiFpgaDll_CloseLlb",                     (NiFpga_FunctionPointer*)&NiFpga_closeLowLatencyBuffer, 0},
   {"NiFpgaDll_ClientFunctionCall",           (NiFpga_FunctionPointer*)&NiFpga_clientFunctionCall, 0},
   {"NiFpgaDll_FindRegister",                 (NiFpga_FunctionPointer*)&NiFpga_findRegister, 0},
   {"NiFpgaDll_FindFifo",                     (NiFpga_FunctionPointer*)&NiFpga_findFifo, 0},
   {"NiFpgaDll_GetFpgaViState",               (NiFpga_FunctionPointer*)&NiFpga_getFpgaViState, 0},
   {NULL, NULL, 0}
};

NiFpga_Status NiFpga_Initialize(void)
{
   /* if the library isn't already loaded */
   if (!NiFpga_library)
   {
      int i;
      /* load the library */
      #if NiFpga_Windows
         NiFpga_library = LoadLibraryA("NiFpga.dll");
      #elif NiFpga_Linux || NiFpga_MacOsX
         #if NiFpga_Linux
            const char* const library = "libNiFpga.so";
         #elif NiFpga_MacOsX
            const char* const library =
               "/Library/Frameworks/NiFpga.framework/NiFpga";
         #endif
         NiFpga_library = dlopen(library, RTLD_LAZY);
         if (!NiFpga_library)
            fprintf(stderr, "Error opening %s: %s\n", library, dlerror());
      #else
         #error
      #endif
      if (!NiFpga_library)
         return NiFpga_Status_ResourceNotFound;
      /* get each exported function */
      for (i = 0; NiFpga_functions[i].name; i++)
      {
         const char* const name = NiFpga_functions[i].name;
         NiFpga_FunctionPointer* const address = NiFpga_functions[i].address;
         #if NiFpga_Windows
            *address = (NiFpga_FunctionPointer)GetProcAddress(NiFpga_library,
                                                              name);
            if (!*address && NiFpga_functions[i].required)
               return NiFpga_Status_VersionMismatch;
         #elif NiFpga_Linux || NiFpga_MacOsX
            *address = (NiFpga_FunctionPointer)dlsym(NiFpga_library, name);
            if (!*address && NiFpga_functions[i].required)
               return NiFpga_Status_VersionMismatch;
         #else
            #error
         #endif
      }
      /* enable CVI Resource Tracking, if available */
      #if NiFpga_CviResourceTracking
      {
         HMODULE engine = GetModuleHandle("cvirte.dll");
         if (!engine)
            engine = GetModuleHandle("cvi_lvrt.dll");
         if (!engine)
            engine = GetModuleHandle("instrsup.dll");
         if (engine)
         {
            NiFpga_acquireCviResource =
               (NiFpga_AcquireCviResource)
                  GetProcAddress(engine, "__CVI_Resource_Acquire");
            NiFpga_releaseCviResource =
               (NiFpga_ReleaseCviResource)
                  GetProcAddress(engine, "__CVI_Resource_Release");
            if (!NiFpga_acquireCviResource
            ||  !NiFpga_releaseCviResource)
            {
               NiFpga_acquireCviResource = NULL;
               NiFpga_releaseCviResource = NULL;
            }
         }
      }
      #endif
   }
   return NiFpga_Status_Success;
}

NiFpga_Status NiFpga_Finalize(void)
{
   /* if the library is currently loaded */
   if (NiFpga_library)
   {
      int i;
      NiFpga_Status status = NiFpga_Status_Success;
      /* unload the library */
      #if NiFpga_Windows
         if (!FreeLibrary(NiFpga_library))
            status = NiFpga_Status_ResourceNotInitialized;
      #elif NiFpga_Linux || NiFpga_MacOsX
         if (dlclose(NiFpga_library))
            status = NiFpga_Status_ResourceNotInitialized;
      #else
         #error
      #endif
      /* null out the library and each exported function */
      NiFpga_library = NULL;
      for (i = 0; NiFpga_functions[i].name; i++)
         *NiFpga_functions[i].address = NULL;
      /* null out the CVI Resource Tracking functions */
      #if NiFpga_CviResourceTracking
         NiFpga_acquireCviResource = NULL;
         NiFpga_releaseCviResource = NULL;
      #endif
      return status;
   }
   else
      return NiFpga_Status_ResourceNotInitialized;
}
