/*****************************************************************************/

#ifndef _IOCTL_TEST_H
#define _IOCTL_TEST_H

#include <linux/ioctl.h>

struct ioctl_arg {
        unsigned int reg;
        unsigned int val;
};

/* 這裡要找一個沒用到的號碼，請參考 Documentation/ioctl/ioctl-number.txt */
#define IOC_MAGIC '\x66'

/* 您要的動作 */
#define IOCTL_VALSET      _IOW(IOC_MAGIC, 0, struct ioctl_arg)
#define IOCTL_VALGET      _IOR(IOC_MAGIC, 1, struct ioctl_arg)
#define IOCTL_VALGET_NUM  _IOR(IOC_MAGIC, 2, int)
#define IOCTL_VALSET_NUM  _IOW(IOC_MAGIC, 3, int)

#define IOCTL_VAL_MAXNR 3

#endif
/*****************************************************************************/
