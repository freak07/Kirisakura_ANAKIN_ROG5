#include <linux/fs.h>

#if 1//def AW_CALI_STORE_EXAMPLE
/*write cali to persist file example*/
#define AWINIC_CALI_FILE  "/mnt/vendor/persist/aw_cali.bin"
#define AWINIC_RTP_CALI_FILE  "/mnt/vendor/persist/aw_rtp_cali.bin"
//#define AW_INT_DEC_DIGIT 10
static int aw8697_write_cali_to_file(unsigned int cali_re, int channel, int type)
{
	struct file *fp;
	char buf[50] = {0};
	loff_t pos = 0;

	if (type == 0)
	fp = filp_open(AWINIC_CALI_FILE, O_RDWR | O_CREAT, 0644);
	else if (type == 1)
		fp = filp_open(AWINIC_RTP_CALI_FILE, O_RDWR | O_CREAT, 0644);

	if (IS_ERR(fp)) {
		pr_err("%s:channel:%d open %s failed!\n",__func__, channel, type ? AWINIC_RTP_CALI_FILE:AWINIC_CALI_FILE);
		return -EINVAL;
	}
	//if (channel == AW882XX_CHANNLE_RIGHT)
	//	pos = AW_INT_DEC_DIGIT;

	snprintf(buf, PAGE_SIZE, "%d", cali_re);

	kernel_write(fp, buf, strlen(buf), &pos);

	pr_info("%s: channel:%d buf:%s type:%d cali:%d\n",
		__func__, channel, buf, type, cali_re);

	filp_close(fp, NULL);
	return 0;
}

static int aw8697_get_cali_from_file(unsigned int *cali_re, int channel, int type)
{
	struct file *fp;
	/*struct inode *node;*/
	int f_size;
	char *buf;
	unsigned int int_cali_re = 0;

	loff_t pos = 0;

	if (type == 0)
	fp = filp_open(AWINIC_CALI_FILE, O_RDWR, 0);
	else if (type == 1)
		fp = filp_open(AWINIC_RTP_CALI_FILE, O_RDWR, 0);

	if (IS_ERR(fp)) {
		//pr_err("%s:channel:%d open %s failed!\n",__func__, channel, AWINIC_CALI_FILE);
		pr_err("%s:channel:%d open %s failed!\n",__func__, channel, type ? AWINIC_RTP_CALI_FILE:AWINIC_CALI_FILE);
		return -EINVAL;
	}

	//if (channel == AW882XX_CHANNLE_RIGHT)
	//	pos = AW_INT_DEC_DIGIT;

	/*node = fp->f_dentry->d_inode;*/
	/*f_size = node->i_size;*/
	f_size = sizeof(unsigned int);

	buf = kzalloc(f_size + 1, GFP_ATOMIC);
	if (!buf) {
		pr_err("%s: channel:%d malloc mem %d failed!\n",
			__func__, channel, f_size);
		filp_close(fp, NULL);
		return -EINVAL;
	}

	kernel_read(fp, buf, f_size, &pos);

	if (sscanf(buf, "%d", &int_cali_re) == 1)
		*cali_re = int_cali_re;
	else
		*cali_re = 0x7777;

	pr_info("%s: channel:%d buf:%s type:%d cali:%d\n",
		__func__, channel, buf, type, int_cali_re);

	filp_close(fp, NULL);

	return  0;

}
#endif
