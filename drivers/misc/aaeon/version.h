#ifndef __AAEON_VERSION_H__
#define __AAEON_VERSION_H__

#define MAX_STRING_LEN 50
#define OS_VERSION "9"
#define OS_TYPE "Debian"
#define OS_COMPATIBILITY "AM335X"

extern char bootloader_vendor[MAX_STRING_LEN];
extern char bootloader_version[MAX_STRING_LEN];
extern char bootloader_date[MAX_STRING_LEN];
extern char product_name[MAX_STRING_LEN];
extern char product_version[MAX_STRING_LEN];
extern char board_vendor[MAX_STRING_LEN];
extern char board_name[MAX_STRING_LEN];
extern char board_version[MAX_STRING_LEN];
extern char build_version[MAX_STRING_LEN];

int aa_proc_version_init(void);

#endif
