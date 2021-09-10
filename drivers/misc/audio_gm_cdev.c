/*================================================================
*   Copyright (C) 2018 TC Ltd. All rights reserved.
*   
*   File Name ：audio_gm_cdev.c
*   Author    ：tombinfan
*   Date      ：2018-12-20
*   Descriptor：
*
================================================================*/

#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/timer.h>
#include <asm/atomic.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>

#define NUM_MINORS              2
#define AUDIO_GM_CDEV_NAME      "audio"
#define GRAPHIC_CDEV_NAME       "graphic"
#define AUDIO_GM_CLASS_NAME     "game_mode"

#define PARAM_INDEX_DEFAULT         0
#define PARAM_INDEX_GAMEMODE        1
#define STR_GAMEMODE_1              "GameMode=true"
#define STR_GAMEMODE_0              "GameMode=false"

#define PARAM_INDEX_BTSTEREO        2
#define STR_BTSTEREO_1              "BTStereo=true"
#define STR_BTSTEREO_0              "BTStereo=false"

#define PARAM_INDEX_MEDIAVOLUME     3
#define STR_MEDIAVOLUME_1           "MediaVolume=true"
#define STR_MEDIAVOLUME_0           "MediaVolume=false"

#define PARAM_INDEX_DOUBLETALK      4
#define STR_DOUBLETALK_1            "DoubleTalk=true"
#define STR_DOUBLETALK_0            "DoubleTalk=false"

#define PARAM_INDEX_RNNANS          5
#define STR_RNNANS_1                "RNNANS=true"
#define STR_RNNANS_0                "RNNANS=false"

#define PARAM_INDEX_GRAPHIC_LL      6

static int audio_gm_major = 0;

#define PARAM_NUM_MAX   10
#define BUFF_LEN        256

struct audio_gm_dev 
{
    int param_index;
};
 
static struct cdev g_cdev;
static char g_buff[PARAM_NUM_MAX][BUFF_LEN];
 
int audio_gm_open(struct inode *inode, struct file *filp)   
{
    struct audio_gm_dev *audio_gm_devp;

    audio_gm_devp = kmalloc(sizeof(struct audio_gm_dev), GFP_KERNEL);
    if(!audio_gm_devp){
        printk(KERN_ERR "Error add audio_gm\n");
        return -ENOMEM;
    }
 
    memset(audio_gm_devp, 0, sizeof(struct audio_gm_dev));

    filp->private_data = audio_gm_devp; 
    return 0;
}
 
int audio_gm_release(struct inode *inode, struct file *filp)    
{
    struct audio_gm_dev *dev = filp->private_data;

    if(NULL != dev){
        kfree(dev);
    }

    return 0;
}
 
static ssize_t audio_gm_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    unsigned int cnt = count;
    struct audio_gm_dev *dev = filp->private_data;
    int p_index = dev->param_index;

    if(p_index < 0 || p_index >= PARAM_NUM_MAX){
        printk(KERN_ERR "%s index invalid %d\n", __func__, p_index);
        return -EFAULT;
    }

    if(cnt > BUFF_LEN){
        cnt = BUFF_LEN;
    }

    if(copy_to_user(buf, (void *)g_buff[p_index], cnt)){
        printk(KERN_ERR "======== audio_gm_read failed\n");
        ret = -EFAULT;
    }else{
        ret = cnt;
    }

    return ret;
}

static int audio_gm_set_buff(const char *buf, unsigned int cnt, int index)
{
    if(index < 0 || index >= PARAM_NUM_MAX){
        printk(KERN_ERR "%s index invalid %d\n", __func__, index);
        return -EINVAL;
    }

    if(cnt > BUFF_LEN){
        cnt = BUFF_LEN;
    }

    memset(g_buff[index], 0x00, BUFF_LEN);
    memcpy(g_buff[index], buf, cnt);

    return cnt;
}

static ssize_t audio_gm_write(struct file *filp, const char __user *buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    unsigned int cnt = count;
    struct audio_gm_dev *dev = filp->private_data;
    int p_index = dev->param_index;
    char t_buf[BUFF_LEN] = {0x00};

    if(NULL == buf || cnt < 0){
        return -EINVAL;
    }

    if(cnt > BUFF_LEN)
        cnt = BUFF_LEN;

    if(copy_from_user(t_buf, buf, cnt)){
        printk(KERN_ERR "audio_gm_write failed: memcpy\n");
        return -EFAULT;
    }

    ret = audio_gm_set_buff(t_buf, cnt, p_index);

    if(0 == strncmp(t_buf, STR_GAMEMODE_1, strlen(STR_GAMEMODE_1))){
        audio_gm_set_buff(STR_MEDIAVOLUME_1, strlen(STR_MEDIAVOLUME_1), PARAM_INDEX_MEDIAVOLUME); 
        audio_gm_set_buff(STR_DOUBLETALK_1, strlen(STR_DOUBLETALK_1), PARAM_INDEX_DOUBLETALK); 
    }else if(0 == strncmp(t_buf, STR_GAMEMODE_0, strlen(STR_GAMEMODE_0))){
        audio_gm_set_buff(STR_MEDIAVOLUME_0, strlen(STR_MEDIAVOLUME_0), PARAM_INDEX_MEDIAVOLUME); 
        audio_gm_set_buff(STR_DOUBLETALK_0, strlen(STR_DOUBLETALK_0), PARAM_INDEX_DOUBLETALK); 
    }

    return ret;
}

static loff_t audio_gm_llseek(struct file *filp, loff_t offset, int whence)
{ 
    loff_t newpos;      
    struct audio_gm_dev *dev = filp->private_data;
    int *p_index = &dev->param_index;

    switch(whence) {
      case 0: /* SEEK_SET */
        newpos = offset;
        break;

      case 1: /* SEEK_CUR */
        newpos = *p_index + offset;    
        break;

      case 2: /* SEEK_END */
        newpos = PARAM_NUM_MAX - 1 + offset;
        break;

      default: /* can't happen */
        return -EINVAL;
    }
    if ((newpos < 0) || (newpos >= PARAM_NUM_MAX))
        return -EINVAL;
        
    *p_index = newpos;
    return newpos;
}

static const struct file_operations audio_gm_fops =
{
    .owner = THIS_MODULE,
    .llseek = audio_gm_llseek,
    .open = audio_gm_open,
    .release = audio_gm_release,
    .read = audio_gm_read,
    .write = audio_gm_write,
};
 
static void audio_gm_setup_cdev(struct cdev *cdev, int index)
{
    int err = -1;
    int devno = MKDEV(audio_gm_major, index);

    cdev_init(cdev, &audio_gm_fops);
    cdev->owner = THIS_MODULE;
    cdev->ops = &audio_gm_fops;
    err = cdev_add(cdev, devno, 1);
    if(err){
        printk(KERN_ERR "Error %d add audio_gm %d\n", err, index); 
    }
}

static int audio_param_show(char *buf, int len, int index)
{
    if(NULL == buf){
        printk(KERN_ERR "%s param buf invalid\n", __func__);
        return -EFAULT;
    }

    if(index < PARAM_INDEX_DEFAULT || index > PARAM_NUM_MAX){
        printk(KERN_ERR "%s param index invalid\n", __func__);
        return -EFAULT;
    }

    if(len < strlen(g_buff[index])){
        printk(KERN_ERR "%s param len invalid\n", __func__);
        return -EFAULT;
    }

    if(len > BUFF_LEN)
        len = BUFF_LEN;

    return sprintf(buf, "%s", g_buff[index]);
}

static int audio_param_store(const char *buf, int len, int index)
{
    if(NULL == buf){
        printk(KERN_ERR "%s param buf invalid\n", __func__);
        return -EFAULT;
    }

    if(index < PARAM_INDEX_DEFAULT || index > PARAM_NUM_MAX){
        printk(KERN_ERR "%s param index invalid\n", __func__);
        return -EFAULT;
    }

    if(len > BUFF_LEN)
        len = BUFF_LEN;

    printk(KERN_INFO "%s will write %s to %d\n", __func__, buf, index);
    return snprintf(g_buff[index], BUFF_LEN - 1, "%s", buf);
}

static ssize_t gm_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return audio_param_show(buf, BUFF_LEN, PARAM_INDEX_GAMEMODE);
}

static ssize_t gm_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    ssize_t ret = -1;
    char mv_buf[BUFF_LEN] = {0x00};
    char dt_buf[BUFF_LEN] = {0x00};

    ret = audio_param_store(buf, count, PARAM_INDEX_GAMEMODE);
    if(ret < 0){
        goto out;
    }

    if(0 == strncmp(buf, STR_GAMEMODE_1, strlen(STR_GAMEMODE_1))){
        strncpy(mv_buf, STR_MEDIAVOLUME_1, strlen(STR_MEDIAVOLUME_1));
        strncpy(dt_buf, STR_DOUBLETALK_1, strlen(STR_DOUBLETALK_1));
    }else if(0 == strncmp(buf, STR_GAMEMODE_0, strlen(STR_GAMEMODE_0))){
        strncpy(mv_buf, STR_MEDIAVOLUME_0, strlen(STR_MEDIAVOLUME_0));
        strncpy(dt_buf, STR_DOUBLETALK_0, strlen(STR_DOUBLETALK_0));
    }else{
        strncpy(mv_buf, buf, count);
        strncpy(dt_buf, buf, count);
    }

    ret = audio_param_store(mv_buf, BUFF_LEN, PARAM_INDEX_MEDIAVOLUME);
    if(ret < 0){
        goto out;
    }

    ret = audio_param_store(dt_buf, BUFF_LEN, PARAM_INDEX_DOUBLETALK);

out:
    return ret;
}

static ssize_t rnn_ans_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return audio_param_show(buf, BUFF_LEN, PARAM_INDEX_RNNANS);
}

static ssize_t rnn_ans_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    return audio_param_store(buf, count, PARAM_INDEX_RNNANS);
}

static ssize_t media_volume_show(struct device *dev, struct device_attribute* attr, char *buf)
{
    return audio_param_show(buf, BUFF_LEN, PARAM_INDEX_MEDIAVOLUME);
}

static ssize_t media_volume_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    return audio_param_store(buf, count, PARAM_INDEX_MEDIAVOLUME);
}

static ssize_t doubletalk_show(struct device *dev, struct device_attribute* attr, char *buf)
{
    return audio_param_show(buf, BUFF_LEN, PARAM_INDEX_DOUBLETALK);
}

static ssize_t doubletalk_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    return audio_param_store(buf, count, PARAM_INDEX_DOUBLETALK);
}

static ssize_t btstereo_show(struct device *dev, struct device_attribute* attr, char *buf)
{
    return audio_param_show(buf, BUFF_LEN, PARAM_INDEX_BTSTEREO);
}

static ssize_t btstereo_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    return audio_param_store(buf, count, PARAM_INDEX_BTSTEREO);
}

static ssize_t graphic_ll_show(struct device *dev, struct device_attribute* attr, char *buf)
{

    return audio_param_show(buf, BUFF_LEN, PARAM_INDEX_GRAPHIC_LL);
}

static ssize_t graphic_ll_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    return audio_param_store(buf, count, PARAM_INDEX_GRAPHIC_LL);
}

static struct device_attribute audio_gm_enable_attrs = {
    .attr = {.name = "enable", .mode = 0664},     
    .show    = gm_enable_show,
    .store    = gm_enable_store,
};

static struct device_attribute audio_gm_rnnans_attrs = {
    .attr = {.name = "rnn_ans", .mode = 0664},     
    .show    = rnn_ans_show,
    .store    = rnn_ans_store,
};

static struct device_attribute audio_gm_mediavolume_attrs = {
    .attr = {.name = "media_volume", .mode = 0664},     
    .show    = media_volume_show,
    .store    = media_volume_store,
};

static struct device_attribute audio_gm_doubletalk_attrs = {
    .attr = {.name = "double_talk", .mode = 0664},     
    .show    = doubletalk_show,
    .store    = doubletalk_store,
};

static struct device_attribute audio_gm_btstereo_attrs = {
    .attr = {.name = "bt_stereo", .mode = 0664},     
    .show    = btstereo_show,
    .store    = btstereo_store,
};

static struct device_attribute graphic_attrs = {
    .attr = {.name = "low_latency", .mode = 0664},
    .show    = graphic_ll_show,
    .store    = graphic_ll_store,
};

static struct class *audio_gm_class;
static struct device *audio_dev;
static struct device *graphic_dev;
int audio_gm_init(void)
{
    int ret = -1;
    int i = 0;
    dev_t devno = MKDEV(audio_gm_major, 0);

    if(audio_gm_major){
        ret = register_chrdev_region(devno, 1, AUDIO_GM_CDEV_NAME); 
    }
    else {
        ret = alloc_chrdev_region(&devno, 0, NUM_MINORS, AUDIO_GM_CDEV_NAME);
        audio_gm_major = MAJOR(devno);
    }
    if(ret < 0){
        printk(KERN_ERR "======== audio_gm_init failed\n");
        return ret;
    }

    audio_gm_setup_cdev(&g_cdev, 0);

    audio_gm_class = class_create(THIS_MODULE, AUDIO_GM_CLASS_NAME);
    if(IS_ERR(audio_gm_class)){
        printk(KERN_ERR "%s create audio class failed", __func__);
        return PTR_ERR(audio_gm_class);
    }

    audio_dev = device_create(audio_gm_class, NULL, MKDEV(audio_gm_major, 0), NULL, AUDIO_GM_CDEV_NAME);
    if(NULL == audio_dev){
        printk(KERN_ERR "%s create audio device failed", __func__);
        return -1;
    }

    graphic_dev = device_create(audio_gm_class, audio_dev, MKDEV(audio_gm_major, 1), NULL, GRAPHIC_CDEV_NAME);
    if(NULL == graphic_dev){
        printk(KERN_ERR "%s create graphic device failed", __func__);
    }

    if(sysfs_create_file(&(audio_dev->kobj), &audio_gm_enable_attrs.attr)){
        return -1;
    }
    if(sysfs_create_file(&(audio_dev->kobj), &audio_gm_rnnans_attrs.attr)){
        return -1;
    }
    if(sysfs_create_file(&(audio_dev->kobj), &audio_gm_mediavolume_attrs.attr)){
        return -1;
    }
    if(sysfs_create_file(&(audio_dev->kobj), &audio_gm_doubletalk_attrs.attr)){
        return -1;
    }
    if(sysfs_create_file(&(audio_dev->kobj), &audio_gm_btstereo_attrs.attr)){
        return -1;
    }
    
    if(NULL != graphic_dev && sysfs_create_file(&(graphic_dev->kobj), &graphic_attrs.attr)){
        printk(KERN_ERR "%s sysfs create failed for graphic device", __func__);
        return -1;
    }

    for(i = 0; i < PARAM_NUM_MAX; i ++){
        memset((void *)g_buff[i], 0x00, BUFF_LEN);
    }

    return 0;
}
 
void audio_gm_exit(void)
{
    cdev_del(&g_cdev);
    class_destroy(audio_gm_class);
    unregister_chrdev_region(MKDEV(audio_gm_major,0),1);
}
 
MODULE_LICENSE("Dual BSD/GPL");
module_init(audio_gm_init);
module_exit(audio_gm_exit);
