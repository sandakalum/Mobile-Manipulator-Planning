/*
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
#ifndef __LWIP_ARCH_CC_H__
#define __LWIP_ARCH_CC_H__

#define LITTLE_ENDIAN 1234

#define BYTE_ORDER LITTLE_ENDIAN

typedef unsigned char u8_t;
typedef signed char s8_t;
typedef unsigned short u16_t;
typedef signed short s16_t;
typedef unsigned int u32_t;
typedef signed int s32_t;
typedef unsigned int mem_ptr_t;

#ifndef NULL
#define NULL 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#include <stdlib.h>
#define LWIP_RAND rand

#define LWIP_PLATFORM_DIAG(x) DBG x
#define LWIP_PLATFORM_ASSERT(x) DBG(x)

#define LWIP_PROVIDE_ERRNO

#define U16_F "hu"
#define S16_F "hd"
#define X16_F "hx"
#define U32_F "lu"
#define S32_F "ld"
#define X32_F "lx"

#if 0 
/*Create compilation problems, and according to http://www.mail-archive.com/lwip-users@nongnu.org/msg06786.html, 
lwIP uses packed structures, so packing the field is not really a good idea ;) */
#define PACK_STRUCT_FIELD(x) __packed x
#else
#define PACK_STRUCT_FIELD(x) x
#endif

#define PACK_STRUCT_STRUCT
#define PACK_STRUCT_BEGIN __packed
#define PACK_STRUCT_END

#define LWIP_CHKSUM_ALGORITHM 3

#endif /* __LWIP_ARCH_CC_H__ */
