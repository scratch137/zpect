#ifndef ZPEC_H
#define ZPEC_H
/**
  \file

  Zpectrometer CPU code declarations.

  $Id: zpec.h,v 1.24 2014/03/21 15:26:15 rauch Exp $
*/
#include <basictypes.h>
#include <nettypes.h>
#include <ucos.h>

/** Swap values of arbitrary type (C-callable). */
#define ZPEC_SWAP(type, a, b) do { \
    type tmp;  tmp = a;  a = b;  b = tmp; \
  } while (0)

/** Message priorities. */
enum zpec_log_enum {
  ZPEC_LOG_ERROR = 1, /**< Error   message priority level. */
  ZPEC_LOG_WARN  = 2, /**< Warning message priority level. */
  ZPEC_LOG_INFO  = 3, /**< Info    message priority level. */
  ZPEC_LOG_DEBUG = 4, /**< Debug   message priority level. */
};

/** Sends a message to stderr and the message server. */
#define zpec_msg(level, fn, ...) do { \
    char msg[256];  int len = 0; \
    if (fn) { len = siprintf(msg, "%s() {%s:%d}: ", fn, __FILE__, __LINE__); } \
    siprintf(msg+len, __VA_ARGS__); zpec_log(msg, level); \
  } while (0)

/** Post a debug message (short format). */
#define zpec_debug(/*msg,*/ ...)    zpec_msg(ZPEC_LOG_DEBUG,  0, __VA_ARGS__)

/** Post a debug message (long format---includes localization). */
#define zpec_debug_fn(/*msg,*/ ...) zpec_msg(ZPEC_LOG_DEBUG, fn, __VA_ARGS__)

/** Post an informational message (short format). */
#define zpec_info(/*msg,*/ ...)     zpec_msg(ZPEC_LOG_INFO,  0, __VA_ARGS__)

/** Post an informational message (long format---includes localization). */
#define zpec_info_fn(/*msg,*/ ...)  zpec_msg(ZPEC_LOG_INFO, fn, __VA_ARGS__)

/** Post a warning message (short format). */
#define zpec_warn(/*msg,*/ ...)     zpec_msg(ZPEC_LOG_WARN,  0, __VA_ARGS__)

/** Post a warning message (long format---includes localization). */
#define zpec_warn_fn(/*msg,*/ ...)  zpec_msg(ZPEC_LOG_WARN, fn, __VA_ARGS__)

/** Post an error message (short format). */
#define zpec_error(/*msg,*/ ...)    zpec_msg(ZPEC_LOG_ERROR,  0, __VA_ARGS__)

/** Post an error message (long format---includes localization). */
#define zpec_error_fn(/*msg,*/ ...) zpec_msg(ZPEC_LOG_ERROR, fn, __VA_ARGS__)


#ifdef __cplusplus
extern "C" {
#endif

/**
  Unique 32-bit pattern used to verify flash contents.
  Increment this value whenever flash_struct is changed so that
  zpec_readFlash() can support backward-compatibility.
*/
#define FLASH_SIGNATURE 0x12345679U

/** WASP hardware variant. */
typedef enum zpec_hw_t_enum {
  ZPEC_HW_GBT,    /**< Greenbank Zpectrometer backend. */
  ZPEC_HW_RLT,    /**< RLT Terahertz telescope backend. */
  ZPEC_HW_POW,    /**< Zpectrometer power supply module (YADL card). */
  ZPEC_HW_ARG,    /**< Argus backend. */
  ZPEC_HW_NTYPES  /**< Placeholder value, keep last. */
} zpec_hw_t;

/** Flash memory configuration parameters (8K maximum). */
typedef struct flash_struct {
  unsigned long signature;  /**< Unique bit-pattern to verify validity. */
  unsigned short serialNo,  /**< Zpectrometer serial number. */
		         valid;     /**< Structure validity flag. */
  zpec_hw_t      hw;        /**< Hardware variant. */
  float          gvdiv;     /**< Gate voltage divider ratio. */
  float          LNAsets[96]; /**< 6 values for each of 16 pixels. */
  char	         atten[16]; /**< receiver attenuations. */
  char           sb[16];    /**< receiver sideband setting. */
  float          yigFit[2]; /**< yig tuning linear fit parmeters: slope and intercept. */
} flash_t;

extern void zpec_readFlash(flash_t *flashData);
extern int zpec_writeFlash(const flash_t *flashData);


/* Dynamic HTML callbacks (C++ source with C linkage). */
extern void
  dhtml_getSerialNo(int sock, const char *url);


/* Miscellaneous utilities (C source). */
extern void zpec_usleep(unsigned usec);
extern void zpec_log(const char *msg, int level);
extern void zpec_write_retry(int fd, const void *buf, unsigned nbytes,
                             const char *fn);
extern void zpec_write_if_full(int fd, void *buf, unsigned *nbytes,
                               unsigned maxbytes, const char *fn);
extern void zpec_write_strcpy(int fd, void *buf, const char *str,
			      unsigned maxbytes, const char *fn);

extern BYTE zpec_change_prio(BYTE prio);

extern int zpec_getSerialNo(void);
extern int zpec_interrupt(int fd);
extern int zpec_post_msg(const char *msg, int level);

extern int zpec_log2_fix16(unsigned x);
extern int zpec_log10_fix16(unsigned x);

extern unsigned zpec_parse_spec(char *spec, int specList[/*maxElements*/],
			        int minValue, int maxValue,
				unsigned maxElements);

extern const char *zpec_severity(int level);
extern const char *zpec_version(void);

extern char *zpec_iptostr(char *str, IPADDR addr);
extern char *zpec_print_fixed(char str[/*16*/], int fixed, int scale);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  /* ZPEC_H */
