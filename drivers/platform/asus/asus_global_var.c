/*
** =========================================================================
** File:
**     asus_global_var.c
**
** Description:
**     Export function for getting ASUS GLOBAL VAR.
**
** =========================================================================
*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/unistd.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/timer.h>
#include <linux/uaccess.h>
#include <linux/asus_global_var.h>

#define CMDLINE_MAX_LENGTH  4096
#define CMDLINE_PATH        "/proc/cmdline"
#define CMDLINE_PJ_ID       "androidboot.id.prj="
#define CMDLINE_HW_ID	    "androidboot.id.stage="
#define CMDLINE_SKU_ID	    "androidboot.id.sku="
#define CMDLINE_DDR_ID	    "androidboot.id.ddr="
#define CMDLINE_NFC_ID	    "androidboot.id.nfc="
#define CMDLINE_RF_ID	    "androidboot.id.rf="
#define CMDLINE_FP_ID	    "androidboot.id.fp="
#define CMDLINE_BC_ID	    "androidboot.id.bc="
#define CMDLINE_SECDISP_ID  "androidboot.id.sec_disp="

enum EXTERN_DEVICE_PROJID g_EXTERN_ASUS_prjID = EXTERN_PROJECT_INVALID;
enum EXTERN_DEVICE_HWID g_EXTERN_ASUS_hwID = EXTERN_HW_REV_INVALID;
enum EXTERN_DEVICE_SKUID g_EXTERN_ASUS_skuID = EXTERN_SKU_ID_INVALID;
enum EXTERN_DEVICE_DDRID g_EXTERN_ASUS_ddrID = EXTERN_DDR_VENDOR_INVALID;
enum EXTERN_DEVICE_NFCID g_EXTERN_ASUS_nfcID = EXTERN_NFC_VENDOR_INVALID;
enum EXTERN_DEVICE_RFID g_EXTERN_ASUS_rfID = EXTERN_RF_SKU_INVALID;
enum EXTERN_DEVICE_FPID g_EXTERN_ASUS_fpID = EXTERN_FP_VENDOR_INVALID;
enum EXTERN_DEVICE_BCID g_EXTERN_ASUS_bcID = EXTERN_BC_ID_INVALID;
enum EXTERN_DEVICE_SECDISPID g_EXTERN_ASUS_secdispID = EXTERN_SEC_DISP_ID_INVALID;

EXPORT_SYMBOL(g_EXTERN_ASUS_prjID);
EXPORT_SYMBOL(g_EXTERN_ASUS_hwID);
EXPORT_SYMBOL(g_EXTERN_ASUS_skuID);
EXPORT_SYMBOL(g_EXTERN_ASUS_ddrID);
EXPORT_SYMBOL(g_EXTERN_ASUS_nfcID);
EXPORT_SYMBOL(g_EXTERN_ASUS_rfID);
EXPORT_SYMBOL(g_EXTERN_ASUS_fpID);
EXPORT_SYMBOL(g_EXTERN_ASUS_bcID);
EXPORT_SYMBOL(g_EXTERN_ASUS_secdispID);

mm_segment_t oldfs;

static struct mutex mA;

struct file *openFile(char *path,int flag,int mode)
{
	struct file *fp;

	fp=filp_open(path, flag, 0);
	if (fp)
		return fp;
	else
		return NULL;
}

int readFile(struct file *fp,char *buf,int readlen)
{
	if (fp->f_op && fp->f_op->read)
		return fp->f_op->read(fp,buf,readlen, &fp->f_pos);
	else
		return -1;
}

int closeFile(struct file *fp)
{
	filp_close(fp,NULL);
	return 0;
}

void initKernelEnv(void)
{
	oldfs = get_fs();
	set_fs(KERNEL_DS);
}

static void parsing_cmdline(void)
{
	char cmdline_buf[CMDLINE_MAX_LENGTH];
	char *pstr;
	char value[16];
	struct file *fp;
	int ret;
	int proj_id    = 0;
	int hw_id      = 0;
	int sku_id     = 0;
	int ddr_id     = 0;
	int nfc_id     = 0;
	int rf_id      = 0;
	int fp_id      = 0;
	int bc_id      = 0;
	int secdisp_id = 0;

	mutex_lock(&mA);
	/***** read /proc/cmdline *****/
	initKernelEnv();

	fp=openFile(CMDLINE_PATH,O_RDONLY,0);
	if (fp!=NULL)
	{
		memset(cmdline_buf,0,CMDLINE_MAX_LENGTH);
		if ((ret=readFile(fp,cmdline_buf,CMDLINE_MAX_LENGTH))>0)
			printk("[BOOTING] cmdline_buf:%s\n",cmdline_buf);
		else
			printk("[BOOTING] read file error %d\n",ret);

		closeFile(fp);
	}

	set_fs(oldfs);

	/***** PJ_ID *****/
	pstr = strstr(cmdline_buf, CMDLINE_PJ_ID);
	if (pstr) {
		memcpy(value, pstr+strlen(CMDLINE_PJ_ID), 1);
		if (kstrtoint(value, 16, &proj_id)) {
			printk("[BOOTING] Error to parsing: PJ_ID\n");
		} else {
			g_EXTERN_ASUS_prjID = proj_id;
		}
	}else {
		printk("[BOOTING] Error to find PJ_ID cmdline\n");
	}

	/***** HW_ID *****/
	pstr = strstr(cmdline_buf, CMDLINE_HW_ID);
	if (pstr) {
		memcpy(value, pstr+strlen(CMDLINE_HW_ID), 1);
		if (kstrtoint(value, 16, &hw_id)) {
			printk("[BOOTING] Error to parsing: HW_ID\n");
		} else {
			if (g_EXTERN_ASUS_prjID >= 9) {
				switch (hw_id)
				{
					case 0:
						g_EXTERN_ASUS_hwID = EXTERN_HW_REV_ANAKIN2_ER;
					break;

					case 1:
						g_EXTERN_ASUS_hwID = EXTERN_HW_REV_ANAKIN2_PR;
					break;

					case 2:
						g_EXTERN_ASUS_hwID = EXTERN_HW_REV_ANAKIN2_MP;
					break;
				}
			} else {
				g_EXTERN_ASUS_hwID = hw_id;
			}
		}
	}else {
		printk("[BOOTING] Error to find HW_ID cmdline\n");
	}

	/***** SKU_ID *****/
	pstr = strstr(cmdline_buf, CMDLINE_SKU_ID);
	if (pstr) {
		memcpy(value, pstr+strlen(CMDLINE_SKU_ID), 1);
		if (kstrtoint(value, 16, &sku_id)) {
			printk("[BOOTING] Error to parsing: SKU_ID\n");
		} else {
			g_EXTERN_ASUS_skuID = sku_id;
		}
	}else {
		printk("[BOOTING] Error to find SKU_ID cmdline\n");
	}

	/***** DDR_ID *****/
	pstr = strstr(cmdline_buf, CMDLINE_DDR_ID);
	if (pstr) {
		memcpy(value, pstr+strlen(CMDLINE_DDR_ID), 1);
		if (kstrtoint(value, 16, &ddr_id)) {
			printk("[BOOTING] Error to parsing: DDR_ID\n");
		} else {
			g_EXTERN_ASUS_ddrID = ddr_id;
		}
	}else {
		printk("[BOOTING] Error to find DDR_ID cmdline\n");
	}

	/***** NFC_ID *****/
	pstr = strstr(cmdline_buf, CMDLINE_NFC_ID);
	if (pstr) {
		memcpy(value, pstr+strlen(CMDLINE_NFC_ID), 1);
		if (kstrtoint(value, 16, &nfc_id)) {
			printk("[BOOTING] Error to parsing: NFC_ID\n");
		} else {
			g_EXTERN_ASUS_nfcID = nfc_id;
		}
	}else {
		printk("[BOOTING] Error to find NFC_ID cmdline\n");
	}

	/***** RF_ID *****/
	pstr = strstr(cmdline_buf, CMDLINE_RF_ID);
	if (pstr) {
		memcpy(value, pstr+strlen(CMDLINE_RF_ID), 1);
		if (kstrtoint(value, 16, &rf_id)) {
			printk("[BOOTING] Error to parsing: RF_ID\n");
		} else {
			g_EXTERN_ASUS_rfID = rf_id;
		}
	}else {
		printk("[BOOTING] Error to find RF_ID cmdline\n");
	}

	/***** FP_ID *****/
	pstr = strstr(cmdline_buf, CMDLINE_FP_ID);
	if (pstr) {
		memcpy(value, pstr+strlen(CMDLINE_FP_ID), 1);
		if (kstrtoint(value, 16, &fp_id)) {
			printk("[BOOTING] Error to parsing: FP_ID\n");
		} else {
			g_EXTERN_ASUS_fpID = fp_id;
        }
	}else {
		printk("[BOOTING] Error to find FP_ID cmdline\n");
	}

	/***** BC_ID *****/
	pstr = strstr(cmdline_buf, CMDLINE_BC_ID);
	if (pstr) {
		memcpy(value, pstr+strlen(CMDLINE_BC_ID), 1);
		if (kstrtoint(value, 16, &bc_id)) {
			printk("[BOOTING] Error to parsing: BC_ID\n");
		} else {
			g_EXTERN_ASUS_bcID = bc_id;
		}
	}else {
		printk("[BOOTING] Error to find BC_ID cmdline\n");
	}

	/***** SEC_DISP_ID *****/
	pstr = strstr(cmdline_buf, CMDLINE_SECDISP_ID);
	if (pstr) {
		memcpy(value, pstr+strlen(CMDLINE_SECDISP_ID), 1);
		if (kstrtoint(value, 16, &secdisp_id)) {
			printk("[BOOTING] Error to parsing: SECDISP_ID\n");
		} else {
			g_EXTERN_ASUS_secdispID = secdisp_id;
		}
	}else {
		printk("[BOOTING] Error to find SECDISP_ID cmdline\n");
	}

	mutex_unlock(&mA);
}

/* Module info */
MODULE_AUTHOR("ASUS SunnyJY Chen");
MODULE_DESCRIPTION("ASUS_GLOBAL_VAR Kernel Module");
MODULE_LICENSE("GPL v2");

static int __init asus_global_var_init(void)
{
	mutex_init(&mA);
	printk("[BOOTING] asus_global_var_init +++.\n");
	parsing_cmdline();
	printk("[BOOTING] PJ_ID=%d\n", g_EXTERN_ASUS_prjID);
	printk("[BOOTING] HW_ID=%d\n", g_EXTERN_ASUS_hwID);
	printk("[BOOTING] SKU_ID=%d\n", g_EXTERN_ASUS_skuID);
	printk("[BOOTING] DDR_ID=%d\n", g_EXTERN_ASUS_ddrID);
	printk("[BOOTING] NFC_ID=%d\n", g_EXTERN_ASUS_nfcID);
	printk("[BOOTING] RF_ID=%d\n", g_EXTERN_ASUS_rfID);
	printk("[BOOTING] FP_ID=%d\n", g_EXTERN_ASUS_fpID);
	printk("[BOOTING] BC_ID=%d\n", g_EXTERN_ASUS_bcID);
	printk("[BOOTING] SEC_DISP_ID=%d\n", g_EXTERN_ASUS_secdispID);
	printk("[BOOTING] asus_global_var_init ---.\n");
	return 0;
}

static void __exit asus_global_var_exit(void)
{
	printk("[BOOTING] asus_global_var_exit.\n");
}

early_initcall(asus_global_var_init);
module_exit(asus_global_var_exit);
