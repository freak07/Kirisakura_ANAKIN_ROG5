#include "asus_cam_sensor.h"
#include <linux/proc_fs.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/delay.h>
#include "cam_sensor_core.h"
#include "cam_eeprom_dev.h"//EEPROM

#undef  pr_fmt
#define pr_fmt(fmt) "SENSOR-ATD %s(): " fmt, __func__

#define MAX_CAMERA_ID CAMERA_4

static struct cam_sensor_ctrl_t *g_sensor_ctrl[MAX_CAMERA_ID+1]={0};
static uint16_t g_sensor_id[MAX_CAMERA_ID+1]={0};
static uint8_t  g_sensor_init_state[MAX_CAMERA_ID+1]={0};
static uint32_t g_module_changed[MAX_CAMERA_ID + 1];

#include "asus_cam_sensor_def.h"
#include "asus_cam_sensor_utils.c"
#include "asus_cam_sensor_spec.c"

uint8_t g_cam_cci_check = 0; //ASUS_BSP Jason "Add for camera cci debug"
uint8_t g_cam_csi_check = 0;  //ASUS_BSP Bryant "Add for camera csi debug"


#define DUAL_CALI_START 0x821
#define DUAL_CALI_END	0x1020
#define DUAL_CALI_SIZE (DUAL_CALI_END-DUAL_CALI_START+1)
//ASUS_BSP +++ Combine dit cali data and EEPROM data into EEPROM_CTRL->mapdata
#define PDAF_CALI_START 0x21
#define PDAF_CALI_IMX686_START 0x222
#define PDAF_CALI_SIZE 0x3E1
//ASUS_BSP Ryan add for ov24b1q eeprom combine+++
#define OV24B1Q_BIN_SIZE 0x45D
#define OV24B1Q_EEPROM_MAP_START 0x00
#define OV24B1Q_EEPROM_MAP_END	0x266
#define OV24B1Q_EEPROM_MAP_SIZE (OV24B1Q_EEPROM_MAP_END - OV24B1Q_EEPROM_MAP_START+1)
//ASUS_BSP Ryan add for ov24b1q eeprom combine---

/*sm8150 only*/
#define REMOSAIC_CALI_START 0x1023
#define REMOSAIC_CALI_END 0x1BF4
#define REMOSAIC_CALI_SIZE (REMOSAIC_CALI_END-REMOSAIC_CALI_START+1)
//ASUS_BSP --- Combine dit cali data and EEPROM data into EEPROM_CTRL->mapdata

#define DIT_FILE_SIZE 0x471
#define DIT_FILE_AF_START 0x45D
#define DIT_FILE_AF_CHECK 0x465

//ASUS_BSP Byron +++
#define CALI_AF_INFINITY_START 0x0
#define CALI_AF_MACRO_START 0x4

#define AF_INFINITY_SIZE 1
#define AF_MACRO_SIZE 1

#define LRC_START 0x21
#define LRC_END 0x221
#define LRC_SIZE (LRC_END - LRC_START +1)
//ASUS_BSP Byron ---


#define MODULE_ID_ADDR   7165  //Mdoule ID address in share eeprom ex: OV13B10 Module ID:"0x74", addr:"7165" in EEPROM 0

#define EEPROM_SIZE 8192  //ASUS_BSP Bryant
#define EEPROM_MODULEID_OFFSET 8
#define SHARE_EEPROM_MODULEID_OFFSET 0
#define OTP_MODULEID_OFFSET 8
#define DIT_DUT_BIN "dut_%s.bin"
#define PROC_EEPROM "driver/eeprom%d"
#define PROC_DIT_EEPROM "driver/dit_eeprom%d"
#define PROC_OTP_NAME "driver/otp%d"
#define DIT_CALI_TXT "cali_%s.txt"
#define EEPROM_SENSORVERSION_OFFSET 0x1d //ASUS_BSP Jason_yeh suporrt sensor version features
typedef struct
{
	const uint8_t  eeprom_module_Id;
	char physicalSensorModuleName[20];
	const int32_t  af_infinity_start;
	const int32_t  af_macro_start;
	const int32_t  pdaf_start;
	const int32_t  remosaic_start;
	const int32_t  lrc_start;
	const uint32_t lrc_size;
	const uint8_t  dualCali_num;
	bool  module_changed;
	bool  suporrt_sensor_version;
}eeprom_group_t;


//ASUS_BSP Byron EEprom [Step.1] Add your camera module here +++
//Use OTP: EEPROM index must > 8 in dtsi. EEPROM size >= OTP_DATA_LEN_BYTE*3

static eeprom_group_t  g_eeprom_group[] =
{
  //MODULE_ID  SENSOR_NAME  AF_INFINITY_START_ADDR  AF_MACRO_START_ADDR
  {0x73,       "IMX686_R",  CALI_AF_INFINITY_START, CALI_AF_MACRO_START,
		//PDAF_CALI_START_ADDR    REMOSAIC_CALI_START_ADDR     LRC_START
		  PDAF_CALI_IMX686_START, REMOSAIC_CALI_START,         LRC_START,
		//LRC_SIZE  DUAL_NUM  IS_CHANGED  IsSuporrtVersion
		  LRC_SIZE, 1,        0,          0},


  //MODULE_ID  SENSOR_NAME  AF_INFINITY_START_ADDR  AF_MACRO_START_ADDR
  {0x74,       "OV13B_R",   -1,                     -1,
		//PDAF_CALI_START_ADDR    REMOSAIC_CALI_START_ADDR     LRC_START
		  -1,                     -1,                          -1,
		//LRC_SIZE  DUAL_NUM  IS_CHANGED  IsSuporrtVersion
		  0,        0,        0,          0},


  //MODULE_ID  SENSOR_NAME  AF_INFINITY_START_ADDR  AF_MACRO_START_ADDR
  {0x72,       "OV24B1Q_R", -1,                     -1,
		//PDAF_CALI_START_ADDR    REMOSAIC_CALI_START_ADDR     LRC_START
		  -1,                     OV24B1Q_EEPROM_MAP_START,    -1,
		//LRC_SIZE  DUAL_NUM  IS_CHANGED  IsSuporrtVersion
		  0,        0,        0,          0},


  //MODULE_ID  SENSOR_NAME  AF_INFINITY_START_ADDR  AF_MACRO_START_ADDR
  {0x6F,       "OV8856_R" , -1,                     -1,
		//PDAF_CALI_START_ADDR    REMOSAIC_CALI_START_ADDR     LRC_START
		  -1,                     -1,                          -1,
		//LRC_SIZE  DUAL_NUM  IS_CHANGED  IsSuporrtVersion
		  0,        0,        0,          0},

  //MODULE_ID  SENSOR_NAME  AF_INFINITY_START_ADDR  AF_MACRO_START_ADDR
  {0x6E,       "OV08A10_P",  CALI_AF_INFINITY_START, CALI_AF_MACRO_START,
		//PDAF_CALI_START_ADDR    REMOSAIC_CALI_START_ADDR     LRC_START
		  PDAF_CALI_START, -1,         -1,
		//LRC_SIZE  DUAL_NUM  IS_CHANGED  IsSuporrtVersion
		  0, 0,        0,          0},

  //MODULE_ID  SENSOR_NAME  AF_INFINITY_START_ADDR  AF_MACRO_START_ADDR
  {0x6B,       "IMX686_P",  CALI_AF_INFINITY_START, CALI_AF_MACRO_START,
               //PDAF_CALI_START_ADDR    REMOSAIC_CALI_START_ADDR     LRC_START
                 PDAF_CALI_IMX686_START, REMOSAIC_CALI_START,         LRC_START,
               //LRC_SIZE  DUAL_NUM  IS_CHANGED  IsSuporrtVersion
                 LRC_SIZE, 1,        0,          0},

};
//ASUS_BSP Byron EEprom [Step.1] Add your camera module here ---

typedef struct
{
	uint8_t *af_infinity;
	uint8_t *af_macro;
	uint8_t *pdaf;
	uint8_t *remosaic;
	uint8_t *lrc;
}eeprom_addrMap;

#define EEPROM_GROUP_SIZE (sizeof(g_eeprom_group)/sizeof(eeprom_group_t))

bool check_eeprom_module_in_table(struct cam_eeprom_ctrl_t * e_ctrl,uint8_t *eeprom_module_group_index) {
	uint8_t i;
	for(i=0;i<EEPROM_GROUP_SIZE;i++) {
		if(e_ctrl->cal_data.mapdata[EEPROM_MODULEID_OFFSET] == g_eeprom_group[i].eeprom_module_Id) {
			pr_info("module_Id(0x%x) in table[%u]",e_ctrl->cal_data.mapdata[EEPROM_MODULEID_OFFSET],i);
			*eeprom_module_group_index = i;
			return true;
		}
	}
	pr_err("No match module_id(0x%x) in table, please add this module id you need in table g_eeprom_group !!!\n",e_ctrl->cal_data.mapdata[EEPROM_MODULEID_OFFSET]);
	return false;
}


//camera_res +++
static int modules_proc_read(struct seq_file *buf, void *v)
{
	int rc = 0;
	struct cam_sensor_ctrl_t *s_ctrl= (struct cam_sensor_ctrl_t *)buf->private;

	if(!s_ctrl)
	{
		seq_printf(buf, "s_ctrl is NULL\n");
		pr_err("s_ctrl is NULL!\n");
	}
	else
	{
		seq_printf(buf, "%s\n", get_sensor_name(s_ctrl->sensordata->slave_info.sensor_id));
	}
	return rc;
}

static int module_proc_open(struct inode *inode, struct	 file *file)
{
	return single_open(file, modules_proc_read, PDE_DATA(inode));
}
static struct file_operations camera_module_fops = {
	.owner = THIS_MODULE,
	.open = module_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static char *g_module_proc_file_name[MAX_CAMERA_ID + 1]=
{
	PROC_MODULE_CAMERA0,
	PROC_MODULE_CAMERA1,
	PROC_MODULE_CAMERA2,
	PROC_MODULE_CAMERA3,
	PROC_MODULE_CAMERA4,
};

static void module_create(uint16_t camera_id)
{
	char* file_name = g_module_proc_file_name[camera_id];
	create_proc_file(file_name, &camera_module_fops, g_sensor_ctrl[camera_id]);
}
//camera_res ---

//camera resolution +++
static int resolution_proc_read(struct seq_file *buf, void *v)
{

	int rc = 0;
	struct cam_sensor_ctrl_t *s_ctrl= (struct cam_sensor_ctrl_t *)buf->private;

	if(!s_ctrl)
	{
		seq_printf(buf, "s_ctrl is NULL!\n");
		pr_err("s_ctrl is NULL!\n");
	}
	else
	{
		seq_printf(buf, "%dM\n", get_sensor_resolution(s_ctrl->sensordata->slave_info.sensor_id));
	}

	return rc;
}

static int resolution_proc_open(struct inode *inode, struct	 file *file)
{
	return single_open(file, resolution_proc_read, PDE_DATA(inode));
}
static struct file_operations camera_resolution_fops = {
	.owner = THIS_MODULE,
	.open = resolution_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static char *g_resolution_proc_file_name[MAX_CAMERA_ID + 1]=
{
	PROC_RESOUTION_CAMERA0,
	PROC_RESOUTION_CAMERA1,
	PROC_RESOUTION_CAMERA2,
	PROC_RESOUTION_CAMERA3,
	PROC_RESOUTION_CAMERA4,
};

static void resolution_proc_create(uint16_t camera_id)
{
	char* file_name = g_resolution_proc_file_name[camera_id];
	create_proc_file(file_name, &camera_resolution_fops, g_sensor_ctrl[camera_id]);
}
//camera resolution ---

//camera status +++
static int status_proc_read(struct seq_file *buf, void *v)
{

	int rc = 0;
	struct cam_sensor_ctrl_t *s_ctrl= (struct cam_sensor_ctrl_t *)buf->private;

	if(!s_ctrl)
	{
		seq_printf(buf, "s_ctrl is NULL!\n");
		pr_err("s_ctrl is NULL!\n");
	}
	else
	{
		if(cam_sensor_test_i2c((struct cam_sensor_ctrl_t *)buf->private))
			seq_printf(buf, "%s\n%s\n", "ERROR: i2c r/w test fail","Driver version:");
		else
			seq_printf(buf, "%s\n%s\n%s 0x%x\n", "ACK:i2c r/w test ok","Driver version:","sensor_id:", g_sensor_id[s_ctrl->id]);
	}

	return rc;
}

static int status_proc_open(struct inode *inode, struct	 file *file)
{
	return single_open(file, status_proc_read, PDE_DATA(inode));
}
static struct file_operations camera_status_fops = {
	.owner = THIS_MODULE,
	.open = status_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static char *g_status_proc_file_name[MAX_CAMERA_ID + 1]=
{
	PROC_STATUS_CAMERA0,
	PROC_STATUS_CAMERA1,
	PROC_STATUS_CAMERA2,
	PROC_STATUS_CAMERA3,
	PROC_STATUS_CAMERA4,
};

static void status_proc_create(uint16_t camera_id)
{
	char* file_name = g_status_proc_file_name[camera_id];
	create_proc_file(file_name, &camera_status_fops, g_sensor_ctrl[camera_id]);
}
//camera status ---


static sensor_define_t project_camera[MAX_CAMERA_ID+1] = {
	{SENSOR_ID_IMX686, DIRECTION_REAR},
	{SENSOR_ID_OV24B1Q, DIRECTION_FRONT},//ASUS_BSP Ryan add for cam 1
	{SENSOR_ID_OV13B10, DIRECTION_REAR},
	{SENSOR_ID_OV8856, DIRECTION_REAR},
};
static char g_sensors_res_string[64];

static void fill_sensors_res_string(void)
{
	//asume each side has MAX_CAMERA_ID+1 cameras
	uint8_t front_cam_res[MAX_CAMERA_ID+1];
	uint8_t rear_cam_res[MAX_CAMERA_ID+1];

	int i;
	int front_count=0;
	int rear_count=0;

	char *p = g_sensors_res_string;
	int offset = 0;

	memset(front_cam_res,0,sizeof(front_cam_res));
	memset(rear_cam_res,0,sizeof(rear_cam_res));

	for(i=0;i<MAX_CAMERA_ID+1;i++)
	{
		if(project_camera[i].direction == DIRECTION_FRONT)
		{
			front_cam_res[front_count++] = get_sensor_resolution(project_camera[i].sensor_id);
		}
		else if(project_camera[i].direction == DIRECTION_REAR)
		{
			rear_cam_res[rear_count++] = get_sensor_resolution(project_camera[i].sensor_id);
		}
	}

	sort_cam_res(front_cam_res,front_count);
	sort_cam_res(rear_cam_res,rear_count);

	//format: front0+front1+...+frontN-rear0+rear2+...+rearN
	for(i=0;i<front_count;i++)
	{
		if(i==0)
			offset+=sprintf(p+offset, "%dM",front_cam_res[i]);
		else
			offset+=sprintf(p+offset, "+%dM",front_cam_res[i]);
	}

	for(i=0;i<rear_count;i++)
	{
		if(i==0)
			offset+=sprintf(p+offset, "-%dM",rear_cam_res[i]);
		else
			offset+=sprintf(p+offset, "+%dM",rear_cam_res[i]);
	}

	offset+=sprintf(p+offset,"\n");
}

static int sensors_res_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%s",g_sensors_res_string);
	return 0;
}

static int sensors_res_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, sensors_res_proc_read, NULL);
}

static struct file_operations sensors_res_fops = {
	.owner = THIS_MODULE,
	.open = sensors_res_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void sensors_res_create(void)
{
	fill_sensors_res_string();
	create_proc_file(PROC_SENSORS_RES,&sensors_res_fops,NULL);
}

//camera_res ---

//OTP +++
typedef struct
{
	uint32_t num_map;
	uint32_t num_data;
	uint8_t *mapdata;
	uint8_t module_id;
}eeprom_info_t;

static eeprom_info_t g_eeprom_info[EEPROM_GROUP_SIZE];
static eeprom_info_t g_dit_eeprom_info[EEPROM_GROUP_SIZE];
static uint8_t g_otp_data_banks[MAX_CAMERA_ID + 1][OTP_DATA_LEN_BYTE*3];
static uint8_t g_otp_data_override_done[MAX_CAMERA_ID + 1] = {0};

static int otp_proc_read(struct seq_file *buf, void *v)
{
	int i;
	int bank;

	uint8_t* p_otp_data;
	uint32_t camera_id = 0;

	struct cam_sensor_ctrl_t *s_ctrl = (struct cam_sensor_ctrl_t *)buf->private;

	if(s_ctrl == NULL)
	{
		seq_printf(buf, "s_ctrl is NULL!\n");
		return 0;
	}

	camera_id = s_ctrl->id;

/*  debug: It will read OTP when cat otp every time
	pr_info("read_sensor_otp :E \n");
	read_sensor_otp(g_sensor_ctrl[camera_id],&g_otp_data_banks[camera_id][0]);
	pr_info("read_sensor_otp :X \n");
*/

	p_otp_data = &g_otp_data_banks[camera_id][0];


	/***************************
	* Camera 0 & 2 share EEPROM 0
	* Copy eeprom data to otp
	****************************/
	if((camera_id == 0 || camera_id == 2) && !g_otp_data_override_done[camera_id])
	{

		if(g_eeprom_info[0].mapdata == NULL || g_eeprom_info[0].num_data <= 0 )
		{
			pr_err("eeprom_data of camera %d is NULL! not override otp\n", camera_id);
		}else if ( g_eeprom_info[0].mapdata[MODULE_ID_ADDR] != 0x74 ){ //Check 13M (camera_id == 2)Module ID:"0x74", addr:"7165"  in EEPROM 0
			pr_err("g_eeprom_info[0].mapdata[%d] = 0x%02X Module ID NO match 0x74\n", MODULE_ID_ADDR, g_eeprom_info[0].mapdata[MODULE_ID_ADDR]);
		}else{
			pr_info("override_otp_from_eeprom  \n");

			override_otp_from_eeprom(p_otp_data, g_eeprom_info[0].mapdata, camera_id);
			g_otp_data_override_done[camera_id] = 1;
		}
	}


	for(bank=0;bank<3;bank++) //otp have 3 bank data, print data
	{
		if(!g_otp_data_override_done[camera_id])
			p_otp_data = &g_otp_data_banks[camera_id][OTP_DATA_LEN_BYTE*bank];

		for( i = 0; i < OTP_DATA_LEN_WORD; i++)//show 32 bytes, although one bank have 64 bytes
		{
			seq_printf(buf,"0x%02X ",p_otp_data[i]);
			if( (i&7) == 7)
				seq_printf(buf,"\n");
		}
		if(bank<2)
			seq_printf(buf ,"\n");
	}

	//pr_err("[eeporm_debug] otp_proc_read, module_id = 0x%02X \n", p_otp_data[8]);
	//pr_err("[eeporm_debug] otp_proc_read, vendor_id = 0x%02X \n", p_otp_data[9]);


	return 0;
}

static int otp_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, otp_proc_read, PDE_DATA(inode));
}

static struct file_operations otp_fops = {
	.owner = THIS_MODULE,
	.open = otp_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static char *g_otp_proc_file_name[MAX_CAMERA_ID + 1]=
{	PROC_OTP_Camera0,
	PROC_OTP_Camera1,
	PROC_OTP_Camera2,
	PROC_OTP_Camera3,
	PROC_OTP_Camera4,
};

static void otp_dump_create(uint16_t camera_id)
{
	char* file_name = g_otp_proc_file_name[camera_id];

	if(g_sensor_init_state[camera_id])
	{
		remove_proc_entry(file_name, NULL);
		pr_info("remove otp proc entry done  \n");
	}

	if(read_sensor_otp(g_sensor_ctrl[camera_id], &g_otp_data_banks[camera_id][0]) < 0)
	{
		pr_err("read otp for camera sensor %d failed!\n",camera_id);
	}

	create_proc_file(file_name, &otp_fops, g_sensor_ctrl[camera_id]);

}
//OTP ---

void cam_eeprom_read_from_otp(struct cam_eeprom_ctrl_t *e_ctrl, int camera_id) {

	int i,j;

	if (e_ctrl->cal_data.num_data < OTP_DATA_LEN_BYTE*3)
	{
		pr_err("eeprom data size  %d < OTP data size %d", e_ctrl->cal_data.num_data, OTP_DATA_LEN_BYTE*3);
		return;
	}

	for (i=2; i>=0; i--)
	{
		if(g_otp_data_banks[camera_id][i*OTP_DATA_LEN_BYTE + OTP_MODULEID_OFFSET] == 0x00 ||
			g_otp_data_banks[camera_id][i*OTP_DATA_LEN_BYTE + OTP_MODULEID_OFFSET] == 0xFF)
		{
			//the block was not written data
			pr_info("[eeporm_debug] camera id %d otp black[%d] module ID 0x00 or 0xFF, the block was not written data", camera_id, i);
		}else{
			for(j=0; j<=OTP_DATA_LEN_BYTE; j++)
			{
				e_ctrl->cal_data.mapdata[j] = g_otp_data_banks[camera_id][i*OTP_DATA_LEN_BYTE+j];
			}
			break;
		}
	}
}



static uint8_t g_camera_id;
static uint32_t g_camera_reg_addr;
static uint32_t g_camera_reg_val;
static enum camera_sensor_i2c_type g_camera_data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
static uint8_t g_camera_sensor_operation = 0;
static int sensor_i2c_debug_read(struct seq_file *buf, void *v)
{
	uint32_t reg_val = 0xEEEE;
	int rc;

	if(g_sensor_ctrl[g_camera_id] == NULL)
	{
		seq_printf(buf,"Camera ID %d Not Exist!\n",g_camera_id);
		return 0;
	}
	mutex_lock(&(g_sensor_ctrl[g_camera_id]->cam_sensor_mutex));
	seq_printf(buf,"=========================\n\
                    Camera %d, PowerState %d\n\
                    Camera %d, PowerState %d\n\
                    Camera %d, PowerState %d\n\
                    Camera %d, PowerState %d\n\
                    Camera %d, PowerState %d\n\
                    =========================\n",
	CAMERA_0,g_sensor_ctrl[CAMERA_0]?g_sensor_ctrl[CAMERA_0]->power_state:-1,
	CAMERA_1,g_sensor_ctrl[CAMERA_1]?g_sensor_ctrl[CAMERA_1]->power_state:-1,
	CAMERA_2,g_sensor_ctrl[CAMERA_2]?g_sensor_ctrl[CAMERA_2]->power_state:-1,
	CAMERA_3,g_sensor_ctrl[CAMERA_3]?g_sensor_ctrl[CAMERA_3]->power_state:-1,
	CAMERA_4,g_sensor_ctrl[CAMERA_4]?g_sensor_ctrl[CAMERA_4]->power_state:-1
	);
	if(g_sensor_ctrl[g_camera_id]->power_state != 1)
	{
		seq_printf(buf,"Camera ID %d POWER DOWN!\n",g_camera_id);
	}
	else
	{
		if(g_camera_data_type == CAMERA_SENSOR_I2C_TYPE_BYTE){
			rc = cam_sensor_read_byte(g_sensor_ctrl[g_camera_id],g_camera_reg_addr,&reg_val);
		}else{
			rc = cam_sensor_read_word(g_sensor_ctrl[g_camera_id],g_camera_reg_addr,&reg_val);
			}
		if(g_camera_sensor_operation == 1)//write
		{
			if(reg_val == g_camera_reg_val)
			{
				pr_info("read back the same value as write!\n");
			}
			else
			{
				pr_err("write value 0x%x and read back value 0x%x not same!\n",
						g_camera_reg_val,reg_val
				);
			}
		}
		seq_printf(buf,"Camera %d reg 0x%04x val 0x%x\n",g_camera_id,g_camera_reg_addr,reg_val);
	}
	mutex_unlock(&(g_sensor_ctrl[g_camera_id]->cam_sensor_mutex));
	return 0;
}

static int sensor_i2c_debug_open(struct inode *inode, struct  file *file)
{
	return single_open(file, sensor_i2c_debug_read, NULL);
}

static ssize_t sensor_i2c_debug_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t ret_len;
	int n;
	char messages[32]="";
	uint32_t val[4];
	int rc;

	ret_len = len;
	if (len > 32) {
		len = 32;
	}
	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	n = sscanf(messages,"%x %x %x %x",&val[0],&val[1],&val[2],&val[3]);

	if(n < 2 || n > 4 )
	{
		rc = -1;
		pr_err("Invalid Argument count %d!\n",n);
		goto RETURN;
	}
	else if( n == 2)
	{
		//camera id, reg addr
		g_camera_id = val[0];
		g_camera_reg_addr = val[1];

		g_camera_data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
		g_camera_sensor_operation = 0;
	}
	else if( n == 3)
	{
		//camera id, reg addr, data type
		g_camera_id = val[0];
		g_camera_reg_addr = val[1];
		if(val[2] == 1)
			g_camera_data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
		else if(val[2] == 2)
			g_camera_data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
		else
			g_camera_data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
		g_camera_sensor_operation = 0;
	}
	else if( n == 4)
	{
		//camera id, reg addr, data type, reg val
		g_camera_id = val[0];
		g_camera_reg_addr = val[1];
		if(val[2] == 1)
			g_camera_data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
		else if(val[2] == 2)
			g_camera_data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
		else
			g_camera_data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
		g_camera_reg_val = val[3];
		g_camera_sensor_operation = 1;
	}

	if(g_camera_id > MAX_CAMERA_ID)
	{
		pr_err("Invalid Sensor ID %d!\n",g_camera_id);
		g_camera_id = CAMERA_0;//reset to default ID
		goto RETURN;
	}

	if(g_sensor_ctrl[g_camera_id] == NULL)
	{
		pr_err("Sensor ID %d Not Exist!\n",g_camera_id);
		g_camera_id = CAMERA_0;//reset to default ID
		goto RETURN;
	}

	mutex_lock(&(g_sensor_ctrl[g_camera_id]->cam_sensor_mutex));
	if(g_sensor_ctrl[g_camera_id]->power_state != 1)
	{
		pr_err("Please power up Camera Sensor %d first!\n",g_camera_id);
		goto RETURN;
	}
	pr_info(" gona %s Camera ID %d reg 0x%04x, data type %s\n",
			g_camera_sensor_operation ? "WRITE":"READ",
			g_camera_id,
			g_camera_reg_addr,
			g_camera_data_type == CAMERA_SENSOR_I2C_TYPE_BYTE?"BYTE":"WORD"
			);


	if(g_camera_sensor_operation == 1)
	{
		if(g_camera_data_type == CAMERA_SENSOR_I2C_TYPE_BYTE)
			rc = cam_sensor_write_byte(g_sensor_ctrl[g_camera_id],g_camera_reg_addr,g_camera_reg_val);
		else
			rc = cam_sensor_write_word(g_sensor_ctrl[g_camera_id],g_camera_reg_addr,g_camera_reg_val);

		if(rc < 0)
		{
			pr_err("write 0x%x to camera id %d addr 0x%04x FAIL\n",g_camera_reg_val,g_camera_id,g_camera_reg_addr);
		}
		else
		{
			pr_info("write 0x%x to camera id %d addr 0x%04x OK\n",g_camera_reg_val,g_camera_id,g_camera_reg_addr);
		}
	}
RETURN:
	mutex_unlock(&(g_sensor_ctrl[g_camera_id]->cam_sensor_mutex));
	return ret_len;
}
static const struct file_operations sensor_i2c_debug_fops = {
	.owner = THIS_MODULE,
	.open = sensor_i2c_debug_open,
	.read = seq_read,
	.write = sensor_i2c_debug_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static void sensor_i2c_create(void)
{
	create_proc_file(PROC_SENSOR_I2C_RW,&sensor_i2c_debug_fops,NULL);
}

static uint32_t g_eeprom_reg_addr = 0x0000;
static uint32_t g_eeprom_id = 0;//camera id
static struct cam_eeprom_ctrl_t* g_eeprom_ctrl[EEPROM_GROUP_SIZE] = {0};

static int eeprom_read(struct cam_eeprom_ctrl_t *e_ctrl, uint32_t reg_addr, uint32_t * reg_data)
{
	int rc;

	rc = camera_io_dev_read(&(e_ctrl->io_master_info),
								reg_addr,
								reg_data,
								CAMERA_SENSOR_I2C_TYPE_WORD,//addr_type
								CAMERA_SENSOR_I2C_TYPE_BYTE);//data_type
	if(rc < 0)
	{
		pr_err("EEPROM read reg 0x%x failed! rc = %d\n",reg_addr,rc);
	}
	else
	{
		pr_info("EEPROM read reg 0x%x get val 0x%x\n",reg_addr,*reg_data);
	}
	return rc;
}

static int eeprom_i2c_debug_read(struct seq_file *buf, void *v)
{
	uint32_t reg_val = 0xEEEE;
	int rc;

	struct cam_eeprom_ctrl_t * e_ctrl = g_eeprom_ctrl[g_eeprom_id];

	if(e_ctrl == NULL)
	{
		seq_printf(buf,"e_ctrl for eeprom %d is NULL!\n",g_eeprom_id);
		return 0;
	}
	mutex_lock(&(g_sensor_ctrl[g_eeprom_id]->cam_sensor_mutex));
	if(g_sensor_ctrl[g_eeprom_id]->power_state != 1)
	{
		pr_err("Please Power UP Camera Sensor %d for eeprom %d!\n",g_eeprom_id,g_eeprom_id);
		seq_printf(buf,"EEPROM %d POWER DOWN!\n",g_eeprom_id);
	}
	else
	{
		rc = eeprom_read(e_ctrl,g_eeprom_reg_addr,&reg_val);
		if(rc < 0)
		{
			seq_printf(buf,"read reg 0x%04x failed, rc = %d\n",g_eeprom_reg_addr,rc);
		}
		else
			seq_printf(buf,"0x%x\n",reg_val);
	}
	mutex_unlock(&(g_sensor_ctrl[g_eeprom_id]->cam_sensor_mutex));
	return 0;
}

static int eeprom_i2c_debug_open(struct inode *inode, struct  file *file)
{
	return single_open(file, eeprom_i2c_debug_read, NULL);
}

static ssize_t eeprom_i2c_debug_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t ret_len;
	char messages[16]="";

	ret_len = len;

	if (len > 16) {
		len = 16;
	}

	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages,"%x",&g_eeprom_reg_addr);

	pr_info("Gona read EEPROM reg addr 0x%04x\n",g_eeprom_reg_addr);

	return ret_len;
}

static const struct file_operations eeprom_i2c_debug_fops = {
	.owner = THIS_MODULE,
	.open = eeprom_i2c_debug_open,
	.read = seq_read,
	.write = eeprom_i2c_debug_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static void eeprom_i2c_create(void)
{
	create_proc_file(PROC_EEPROM_I2C_R,&eeprom_i2c_debug_fops,NULL);
}



int32_t get_file_size(const char *filename, uint64_t* size)
{
	struct kstat stat;
	mm_segment_t fs;
	int rc = 0;

	stat.size = 0;

	fs = get_fs();
	set_fs(KERNEL_DS);

	rc = vfs_stat(filename,&stat);
	if(rc < 0)
	{
		pr_err("vfs_stat(%s) failed, rc = %d\n",filename,rc);
		rc = -1;
		goto END;
	}

	*size = stat.size;
END:
	set_fs(fs);
	return rc;
}


static int put_dual_cali_bin_buffer(struct seq_file *buf, uint32_t dual_cali_size)
{
	int ret = 0;
	uint64_t i, size;
	if (get_file_size(DUAL_CALI_BIN, &size) == 0 && size == dual_cali_size) {
		uint8_t* pBuffer;
		pBuffer = kzalloc(sizeof(uint8_t) * DUAL_CALI_SIZE, GFP_KERNEL);
		if (pBuffer) {
			ret = read_file_into_buffer(DUAL_CALI_BIN, pBuffer, size);
			if (ret == dual_cali_size) {
				for (i = 0; i < size; i++)
				{
					seq_putc(buf, pBuffer[i]);
				}
			}
			kfree(pBuffer);
		}
	}
	return ret;
}
//ASUS_BSP Byron Add for load defult dual cali bin +++
static int put_default_dual_cali_bin_buffer(struct seq_file *buf, uint32_t dual_cali_size)
{
	int ret = 0;
	uint64_t i, size;
	if (get_file_size("/vendor/bin/dualcam_default_cali.bin", &size) == 0 && size == dual_cali_size) {
		uint8_t* pBuffer;
		pBuffer = kzalloc(sizeof(uint8_t) * dual_cali_size, GFP_KERNEL);
		if (pBuffer) {
			ret = read_file_into_buffer("/vendor/bin/dualcam_default_cali.bin", pBuffer, size);
			pr_err("Byron check ret = %d\n",ret);
			if (ret == dual_cali_size) {
				for (i = 0; i < size; i++)
				{
					seq_putc(buf, pBuffer[i]);
				}
			}
			kfree(pBuffer);
		}
	}
	return ret;
}
//ASUS_BSP Byron Add for load defult dual cali bin ---
static int dual_cali_read_general(struct seq_file *buf, void *v, uint32_t dual_cali_start, uint32_t dual_cali_end, uint32_t dual_cali_size)
{
	int i;
	int report_eeprom_dual=0; //ASUS_BSP Byron Add for load defult dual cali bin
	eeprom_info_t * eeprom_info = (eeprom_info_t *)buf->private;

	if (put_dual_cali_bin_buffer(buf, dual_cali_size) == dual_cali_size){
		pr_err("Use dual cali bin file Success !!\n");
		return 0;
	}

	if(eeprom_info == NULL || eeprom_info->mapdata == NULL)
	{
		//seq_printf(buf,"eeprom info invalid!\n");
		put_default_dual_cali_bin_buffer(buf, dual_cali_size);	  //ASUS_BSP Byron Add for load defult dual cali bin
		return 0;
	}

	if(eeprom_info->num_data < dual_cali_end+1)
	{
		//seq_printf(buf,"eeprom data not cover all dual cali data!\n");
		put_default_dual_cali_bin_buffer(buf, dual_cali_size);	  //ASUS_BSP Byron Add for load defult dual cali bin
		return 0;
	}

	for(i=dual_cali_start;i<=dual_cali_end;i++)
	{
		//ASUS_BSP Byron Add for load defult dual cali bin +++
		if(eeprom_info->mapdata[i] == 0 || eeprom_info->mapdata[i] == 0xFF){
			report_eeprom_dual ++;
		}
		//ASUS_BSP Byron Add for load defult dual cali bin ---
		seq_putc(buf, eeprom_info->mapdata[i]);
	}
	//ASUS_BSP Byron Add for load defult dual cali bin +++
	if(report_eeprom_dual >= 2000){
		buf->count = 0;
		pr_err("Use default dual calib bin instead of eeprom dual data\n");
		put_default_dual_cali_bin_buffer(buf, dual_cali_size);
	}
	//ASUS_BSP Byron Add for load defult dual cali bin ---
	return 0;
}


static int dual_cali_read(struct seq_file *buf, void *v)
{

	dual_cali_read_general(buf, v, DUAL_CALI_START, DUAL_CALI_END, DUAL_CALI_SIZE);
	return 0;
}


static int dual_cali_read3x(struct seq_file *buf, void *v)
{
#define DUAL_CALI_START_3X 0x821
#define DUAL_CALI_END_3X 0x1020
#define DUAL_CALI_SIZE_3X (DUAL_CALI_END_3X-DUAL_CALI_START_3X+1)
	dual_cali_read_general(buf, v, DUAL_CALI_START_3X, DUAL_CALI_END_3X, DUAL_CALI_SIZE_3X);
	return 0;
}

static int dual_cali_read1x(struct seq_file *buf, void *v)
{
#define DUAL_CALI_START_1X 0x1021
#define DUAL_CALI_END_1X 0x1820
#define DUAL_CALI_SIZE_1X (DUAL_CALI_END_1X-DUAL_CALI_START_1X+1)
		dual_cali_read_general(buf, v, DUAL_CALI_START_1X, DUAL_CALI_END_1X, DUAL_CALI_SIZE_1X);
		return 0;
}

static int dual_cali_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dual_cali_read, PDE_DATA(inode));
}

static int dual_cali_open3x(struct inode *inode, struct  file *file)
{
	return single_open(file, dual_cali_read3x, PDE_DATA(inode));
}

static int dual_cali_open1x(struct inode *inode, struct  file *file)
{
	return single_open(file, dual_cali_read1x, PDE_DATA(inode));
}


static const struct file_operations dual_cali_fops = {
	.owner = THIS_MODULE,
	.open = dual_cali_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations dual_cali_fops3x = {
	.owner = THIS_MODULE,
	.open = dual_cali_open3x,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
static const struct file_operations dual_cali_fops1x = {
	.owner = THIS_MODULE,
	.open = dual_cali_open1x,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


//create_proc_file for Arcsoft_calibration (for camera alignment: e.g. Bokeh, SAT)
static void dual_cali_dump_remove(uint8_t moduleGroupIndex) {
	switch (g_eeprom_group[moduleGroupIndex].dualCali_num) {
		case 1:
			remove_proc_entry(PROC_ARCSOFT_CALI, NULL);
			break;
		case 2:
			remove_proc_entry(PROC_ARCSOFT_CALI_3x, NULL);
			remove_proc_entry(PROC_ARCSOFT_CALI_1x, NULL);
			break;
		default:
				break;
		};
}
static void dual_cali_dump_create(uint8_t moduleGroupIndex)
{
	switch (g_eeprom_group[moduleGroupIndex].dualCali_num) {
		case 1:
			proc_create_data(PROC_ARCSOFT_CALI, 0444, NULL, &dual_cali_fops, &g_eeprom_info[moduleGroupIndex]);
			break;
		case 2:
			proc_create_data(PROC_ARCSOFT_CALI_3x, 0444, NULL, &dual_cali_fops3x, &g_eeprom_info[moduleGroupIndex]);
			proc_create_data(PROC_ARCSOFT_CALI_1x, 0444, NULL, &dual_cali_fops1x, &g_eeprom_info[moduleGroupIndex]);
			break;
		default:
			pr_info("%s has no dual cali data dump node\n", g_eeprom_group[moduleGroupIndex].physicalSensorModuleName);
		};
}
//dual cali data ---

#define AFC_SIZE 6
uint8_t AFC_golden_data[AFC_SIZE] =
{
	0x04,
	0x07,
	0x04,
	0x87,
	0x05,
	0xD4
};

uint8_t is_AFC_data_valid(uint8_t moduleGroupIndex)
{
	uint8_t is_AFC_valid = 0;
	int AF_Infinity_Start = g_eeprom_group[moduleGroupIndex].af_infinity_start;
	int i;

	if(g_eeprom_info[moduleGroupIndex].mapdata && AF_Infinity_Start != -1)
	{
		//AF high bytes can not be 0xFF

		for(i=0x0; i<0x07; i=i+2)
		{
			if((g_eeprom_info[moduleGroupIndex].mapdata[AF_Infinity_Start + i] != 0xFF))
			{
				is_AFC_valid = 1;
			}
		}

	}

	pr_info("module id(0x%x) valid? (%d)\n", g_eeprom_group[moduleGroupIndex].eeprom_module_Id, is_AFC_valid);
	return is_AFC_valid;
}

static void copy_to_dit_eeprom(struct cam_eeprom_ctrl_t *e_ctrl, uint8_t moduleGroupIndex)
{
	if(g_dit_eeprom_info[moduleGroupIndex].mapdata == NULL)
	{
		g_dit_eeprom_info[moduleGroupIndex].mapdata = kzalloc(e_ctrl->cal_data.num_data, GFP_KERNEL);
		if(!g_dit_eeprom_info[moduleGroupIndex].mapdata)
		{
			pr_err("alloc memory for DIT eeprom mapdata failed!\n");
			return;
		}
	}
	pr_info("copy eeprom to dit_eeprom e_ctrl->cal_data.num_data = %d", e_ctrl->cal_data.num_data);
	memcpy(g_dit_eeprom_info[moduleGroupIndex].mapdata, e_ctrl->cal_data.mapdata, e_ctrl->cal_data.num_data);
}

void getSpecificDITEEpromFileDir(uint8_t moduleGroupIndex, char *dit_dut_factory_file_dir, char *dit_dut_golden_file_dir) {
	char dit_dut_bin_name[20];
	sprintf(dit_dut_bin_name, DIT_DUT_BIN, g_eeprom_group[moduleGroupIndex].physicalSensorModuleName);
	pr_info("[DIT_EEPROM] moduleGroupIndex(%u)  dit_dut_bin_name(%s)\n", moduleGroupIndex, dit_dut_bin_name);
	sprintf(dit_dut_factory_file_dir, "/vendor/factory/%s", dit_dut_bin_name);
	sprintf(dit_dut_golden_file_dir, "/vendor/lib64/camera/%s", dit_dut_bin_name);
	pr_info("[DIT_EEPROM] factory file dir(%s), golden file dir(%s)\n", dit_dut_factory_file_dir, dit_dut_golden_file_dir);
	return;
}

static void compareCameraSN (uint8_t moduleGroupIndex)
{

	char dit_cali_txt_name[20];
	char filename_cali[40];
	uint16_t cameraidbuffer[OTP_ID_LEN];
	int i;
	int SN_Offset=0;

	static uint8_t buf_cali_id[128];
	static uint16_t buf_cali_id_tmp[OTP_ID_LEN];
	static uint8_t otp_data[OTP_ID_LEN];

	struct file *fp;
	loff_t pos = 0;

	memset(buf_cali_id_tmp, 0, sizeof(buf_cali_id_tmp));
	memset(buf_cali_id, 0, 128);
	memset(otp_data, 0, sizeof(otp_data));

	sprintf(dit_cali_txt_name, DIT_CALI_TXT, g_eeprom_group[moduleGroupIndex].physicalSensorModuleName);
	sprintf(filename_cali, "/vendor/factory/%s", dit_cali_txt_name);

	pr_info("[DIT_EEPROM]  ModuleName %s filename_cali %s", g_eeprom_group[moduleGroupIndex].physicalSensorModuleName, filename_cali);

	fp = filp_open(filename_cali, O_RDONLY,S_IRWXU | S_IRWXG | S_IRWXO);

	if (IS_ERR(fp)) {
		pr_err("[DIT_EEPROM]  open( %s ) failed\n", filename_cali);
		g_eeprom_group[moduleGroupIndex].module_changed = 0;
		return;
	}


	pos = 0;
	kernel_read(fp, buf_cali_id, 128, &pos);

	filp_close(fp, NULL);
	pr_info("[DIT_EEPROM]  buf_cali_id %s, pos %d", buf_cali_id, (int)pos);

	if (g_eeprom_info[moduleGroupIndex].mapdata == NULL) {

		pr_err("[DIT_EEPROM] camera %s eeprom mapdata did not exist\n", g_eeprom_group[moduleGroupIndex].physicalSensorModuleName);
		return;

	} else {

		//The otpdata is copy from eeprom

		SN_Offset = EEPROM_MODULEID_OFFSET + 2;

		memcpy(otp_data, g_eeprom_info[moduleGroupIndex].mapdata + SN_Offset, OTP_ID_LEN);

		pr_info("[DIT_EEPROM] camera %s  otp_data[%d] %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X", g_eeprom_group[moduleGroupIndex].physicalSensorModuleName, moduleGroupIndex,
		otp_data[0], otp_data[1], otp_data[2], otp_data[3], otp_data[4], otp_data[5],
		otp_data[6], otp_data[7], otp_data[8], otp_data[9], otp_data[10], otp_data[11]);

		sprintf((char *)cameraidbuffer, "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
		otp_data[0], otp_data[1], otp_data[2], otp_data[3],	otp_data[4], otp_data[5],
		otp_data[6], otp_data[7], otp_data[8], otp_data[9], otp_data[10], otp_data[11]);

		pr_info("cameraidbuffer = %s", cameraidbuffer);
	}


	sscanf(buf_cali_id, "%24s", buf_cali_id_tmp);
	pr_info("[DIT_EEPROM]  buf_cali_id %s", buf_cali_id_tmp);


	for (i = 0; i < OTP_ID_LEN; i++) {
		if (buf_cali_id_tmp[i] != cameraidbuffer[i]) {
			pr_info("[DIT_EEPROM] buf_cali_id_tmp[%d] != cameraidbuffer[%d]", i, i);
			g_eeprom_group[moduleGroupIndex].module_changed = 1;
			return;
		}
	}
	g_eeprom_group[moduleGroupIndex].module_changed = 0;

}

void getAddrMapByMoudleID(uint8_t module_Id, uint8_t moduleGroupIndex, eeprom_addrMap *map) {

	// 1. AF (infinity & macro)
	if(!is_AFC_data_valid(moduleGroupIndex))
	{
			#ifdef CAM_FACTORY_CONFIG
				pr_err("[DIT_EEPROM] AFC in EEPROM invalid! FACTORY build, not override golden data\n");
			#else
				pr_err("[DIT_EEPROM] AFC in EEPROM invalid! Override with golden data for userspace...\n");
				map->af_infinity = AFC_golden_data;
				map->af_macro = AFC_golden_data+4;
			#endif
	}
	else
	{
		pr_info("[DIT_EEPROM] module_id 0x%x, copy AF data from module eeprom", module_Id);
		if(g_eeprom_group[moduleGroupIndex].af_infinity_start != -1)
		{

			map->af_infinity = g_eeprom_info[moduleGroupIndex].mapdata + g_eeprom_group[moduleGroupIndex].af_infinity_start;
		}

		if(g_eeprom_group[moduleGroupIndex].af_macro_start != -1)
		{
			map->af_macro = g_eeprom_info[moduleGroupIndex].mapdata + g_eeprom_group[moduleGroupIndex].af_macro_start;
		}
	}


	// 2. LRT
	if(g_eeprom_group[moduleGroupIndex].lrc_start != -1)
	{
		map->lrc = g_eeprom_info[moduleGroupIndex].mapdata + g_eeprom_group[moduleGroupIndex].lrc_start;
	}

	// 3. PDAF
	if(g_eeprom_group[moduleGroupIndex].pdaf_start != -1)
	{
		map->pdaf= g_eeprom_info[moduleGroupIndex].mapdata + g_eeprom_group[moduleGroupIndex].pdaf_start;
	}


	// 4. REMOSAIC
	if(g_eeprom_group[moduleGroupIndex].remosaic_start != -1)
	{
		map->remosaic = g_eeprom_info[moduleGroupIndex].mapdata + g_eeprom_group[moduleGroupIndex].remosaic_start;
	}


	return;
}

//ASUS_BSP For Combine dit calibration file to device eeprom
static int cover_dit_cal_data(struct cam_eeprom_ctrl_t *e_ctrl, uint8_t moduleGroupIndex)
{
	int num;
	char dit_dut_factory_file_dir[50];
	char dit_dut_golden_file_dir[50];
	char* load_file_name;
	uint8_t module_id = g_eeprom_group[moduleGroupIndex].eeprom_module_Id;
	eeprom_addrMap map = {0};
	uint8_t *targetStart = e_ctrl->cal_data.mapdata + DIT_FILE_SIZE;
	uint8_t eeprom_val=0; //ASUS_BSP Jason_yeh suporrt sensor version features
	uint8_t sensor_version=0; //ASUS_BSP Jason_yeh suporrt sensor version features
	getSpecificDITEEpromFileDir(moduleGroupIndex, dit_dut_factory_file_dir, dit_dut_golden_file_dir);

	// compare if the two id are matched: dit_dut_factory_file_dir: vendor/factory/cali_id.txt
	// if not, use golden calibration file
	// if any file not exit, use original factory calibration file

	compareCameraSN(moduleGroupIndex);


	if(g_eeprom_group[moduleGroupIndex].module_changed)
	{
		load_file_name = dit_dut_golden_file_dir;
	}
	else
	{
		load_file_name = dit_dut_factory_file_dir;
	}

	pr_info("[DIT_EEPROM] module_id 0x%x Start read DIT DUT File", module_id);

	if((num = read_file_into_buffer(load_file_name, e_ctrl->cal_data.mapdata, EEPROM_SIZE)) >0)
	{
		pr_info("[DIT_EEPROM] read %d bytes from %s module_id = 0x%x", num, load_file_name, module_id);
	}else if((num = read_file_into_buffer(dit_dut_golden_file_dir, e_ctrl->cal_data.mapdata, EEPROM_SIZE)) >0)
	{
		pr_info("[DIT_EEPROM] read %s file fail, read %d bytes from golden file %s module_id = 0x%x", dit_dut_factory_file_dir, num, dit_dut_golden_file_dir, module_id);
	}else{
		/*If there is no dit bin ,set user space eeprom data to 0*/
		pr_info("[DIT_EEPROM] read %s failed, rc %d", dit_dut_golden_file_dir, num);
	    pr_info("[DIT_EEPROM] There is no dit bin ,set user space eeprom data to 0 ");
	    memset(e_ctrl->cal_data.mapdata, 0, e_ctrl->cal_data.num_data);
	}


	//ASUS_BSP Byron EEprom [Step.2] If your dit file addr or size different than common, you need to customize by case module_id:+++
	switch(module_id)
	{
		/*
		ex:
		case 0x6F:
			getAddrMapByMoudleID(module_id,moduleGroupIndex,&map);
			break;
		*/

		default:
			getAddrMapByMoudleID(module_id, moduleGroupIndex, &map);
			pr_info("mapdata(0x%x) infinity(0x%x) macro(0x%x) lrc(0x%x) pdaf(0x%x) remosaic(0x%x)", g_eeprom_info[moduleGroupIndex].mapdata, map.af_infinity, map.af_macro, map.lrc, map.pdaf, map.remosaic);
			if(e_ctrl->cal_data.mapdata[DIT_FILE_AF_CHECK]==0 && e_ctrl->cal_data.mapdata[DIT_FILE_AF_CHECK+1]==0) {
				if(map.af_infinity != NULL)
				{
					memcpy(e_ctrl->cal_data.mapdata + DIT_FILE_AF_START + 1, map.af_infinity, AF_INFINITY_SIZE);
					memcpy(e_ctrl->cal_data.mapdata + DIT_FILE_AF_START, map.af_infinity+1, AF_INFINITY_SIZE);
				}
				if(map.af_macro != NULL)
				{
					memcpy(e_ctrl->cal_data.mapdata + DIT_FILE_AF_START + 3, map.af_macro, AF_MACRO_SIZE);
					memcpy(e_ctrl->cal_data.mapdata + DIT_FILE_AF_START + 2, map.af_macro+1, AF_MACRO_SIZE);
				}
			}

			if(map.lrc != NULL && g_eeprom_group[moduleGroupIndex].lrc_size != 0)
			{
				memcpy(targetStart, map.lrc, g_eeprom_group[moduleGroupIndex].lrc_size);
				targetStart += g_eeprom_group[moduleGroupIndex].lrc_size;
			}

			if(map.pdaf != NULL)
			{
				memcpy(targetStart, map.pdaf, PDAF_CALI_SIZE);
				targetStart += PDAF_CALI_SIZE;
			}

			if(map.remosaic != NULL)
			{
				memcpy(targetStart, map.remosaic, REMOSAIC_CALI_SIZE);
				targetStart += REMOSAIC_CALI_SIZE;
			}
			//ASUS_BSP +++Jason_yeh suporrt sensor version features
			if(g_eeprom_group[moduleGroupIndex].suporrt_sensor_version)
			{// for imx686_R imx686_H imx686_L
				eeprom_val= *(g_eeprom_info[moduleGroupIndex].mapdata + EEPROM_SENSORVERSION_OFFSET);
				switch(eeprom_val)
				{
					case 3://Cut0.91: 3.0
						sensor_version = 1;
						break;
					case 4://Cut1.0: 4.0
						sensor_version = 1;
						break;
					default:
						sensor_version = 0;
					break;
				}
				//memcpy(targetStart,&sensor_version,1);

				*((uint32_t*)targetStart) =  sensor_version;
				pr_info("DIT_FILE_SIZE=%d  lrc_size=%d  PDAF_CALI_SIZE=%d  REMOSAIC_CALI_SIZE=%d  sensorversion val=%d eeprom_val=%d\n",DIT_FILE_SIZE,g_eeprom_group[moduleGroupIndex].lrc_size,PDAF_CALI_SIZE,REMOSAIC_CALI_SIZE,*targetStart,eeprom_val);
				targetStart +=sizeof(uint32_t);
			}
			//ASUS_BSP ---Jason_yeh suporrt sensor version features

			break;
	}


	//ASUS_BSP Byron EEprom [Step.2] If your dit file cover addr different than common, you need to customize by case module_id: ---
	copy_to_dit_eeprom(e_ctrl, moduleGroupIndex);
	return 0;
}

static int save_actual_eeprom_info(struct cam_eeprom_ctrl_t *e_ctrl, uint8_t moduleGroupIndex)
{
	g_eeprom_info[moduleGroupIndex].module_id = g_eeprom_group[moduleGroupIndex].eeprom_module_Id;
	g_eeprom_info[moduleGroupIndex].num_map = e_ctrl->cal_data.num_map;
	g_eeprom_info[moduleGroupIndex].num_data = e_ctrl->cal_data.num_data;
	if(g_eeprom_info[moduleGroupIndex].mapdata == NULL)
	{
		g_eeprom_info[moduleGroupIndex].mapdata = kzalloc(e_ctrl->cal_data.num_data, GFP_KERNEL);
		if(!g_eeprom_info[moduleGroupIndex].mapdata)
		{
			pr_err("alloc memory for eeprom mapdata failed!\n");
			return -1;
		}
	}
	memcpy(g_eeprom_info[moduleGroupIndex].mapdata, e_ctrl->cal_data.mapdata, e_ctrl->cal_data.num_data);
	return 0;
}

static int eeprom_proc_read(struct seq_file *buf, void *v)
{
	int i;
	eeprom_info_t *eeprom = (eeprom_info_t *)buf->private;

	pr_info("eeprom info, module_id(0x%x) , num_map %d, num_data %d\n",
			eeprom->module_id, eeprom->num_map, eeprom->num_data);
	if(!eeprom->mapdata)
	{
		seq_printf(buf,"eeprom data is NULL\n");
	}
	else
	{
		for( i = 0; i < eeprom->num_data; i++)
		{
			seq_printf(buf, "0x%04X 0x%02X\n",i,eeprom->mapdata[i]);
		}
	}
	return 0;
}
static int eeprom_proc_open(struct inode *inode, struct	 file *file)
{
	return single_open(file, eeprom_proc_read, PDE_DATA(inode));
}

static struct file_operations eeprom_proc_fops = {
	.owner = THIS_MODULE,
	.open = eeprom_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int eeprom_to_otp_read(struct seq_file *buf, void *v)
{
	int i,j,num;
	eeprom_info_t *eeprom = (eeprom_info_t*)buf->private;

	pr_info("eeprom info, module_id(0x%x) , num_map %d, num_data %d\n",
			eeprom->module_id, eeprom->num_map, eeprom->num_data);
	if(!eeprom->mapdata)
	{
		seq_printf(buf,"eeprom data is NULL\n");
	}
	else
	{
		num=0;
		switch(eeprom->module_id)
		{
			case 0x68: //ov13855

				for( i = 0; i < 4; i++)
				{
					if ( i == 0 )
					{
						for( j = 0; j < 8; j++)
						{
							seq_printf(buf, "0x00 ");
							num++;
						}
					}
					else
					{
						for( j = 0; j < 8; j++)
						{
							seq_printf(buf, "0x%02X ",eeprom->mapdata[num++]);
						}
					}
					seq_printf(buf, "\n");
				}
				break;

			default:
				for( i = 0; i < 4; i++)
				{
					for( j = 0; j < 8; j++)
					{
						seq_printf(buf, "0x%02X ",eeprom->mapdata[num++]);
					}
					seq_printf(buf, "\n");
				}
		}
	}

	return 0;
}
static int eeprom_to_otp_open(struct inode *inode, struct  file *file)
{
	return single_open(file, eeprom_to_otp_read, PDE_DATA(inode));
}


static struct file_operations eeprom_to_otp_fops = {
	.owner = THIS_MODULE,
	.open = eeprom_to_otp_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dit_eeprom_proc_read(struct seq_file *buf, void *v)
{
	int i;
	eeprom_info_t *eeprom = (eeprom_info_t *)buf->private;

	pr_info("[DIT_EEPROM], module_id(0x%x), num_map %d, num_data %d\n",
			eeprom->module_id, eeprom->num_map, eeprom->num_data);
	if(!eeprom->mapdata)
	{
		seq_printf(buf,"eeprom data is NULL\n");
	}
	else
	{
		for( i = 0; i < eeprom->num_data; i++)
		{
			seq_printf(buf, "0x%04X 0x%02X\n", i, eeprom->mapdata[i]);
		}
	}
	return 0;
}
static int dit_eeprom_proc_open(struct inode *inode, struct	 file *file)
{
	return single_open(file, dit_eeprom_proc_read, PDE_DATA(inode));
}

static struct file_operations dit_eeprom_proc_fops = {
	.owner = THIS_MODULE,
	.open = dit_eeprom_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void set_actual_afc_data(uint8_t moduleGroupIndex)
{
	uint32_t dac_infinity, dac_macro;

	if(is_AFC_data_valid(moduleGroupIndex))
	{
		dac_macro = g_eeprom_info[moduleGroupIndex].mapdata[4] << 8 | g_eeprom_info[moduleGroupIndex].mapdata[5];
		dac_infinity = g_eeprom_info[moduleGroupIndex].mapdata[0] << 8 | g_eeprom_info[moduleGroupIndex].mapdata[1];
	}
	else
	{
		dac_macro = AFC_golden_data[4] << 8 | AFC_golden_data[5];
		dac_infinity = AFC_golden_data[0] << 8 | AFC_golden_data[1];
	}

	pr_info("physical sensor module name(%s) dac (marcro,infinity) = (%u,%u)\n", g_eeprom_group[moduleGroupIndex].physicalSensorModuleName, dac_macro, dac_infinity);
	//set_ois_afc_data_from_eeprom(dac_macro,dac_infinity, g_eeprom_group[moduleGroupIndex].physicalSensorModuleName);
}

static void set_dit_afc_data(uint8_t moduleGroupIndex)
{
	#define AF_INFINITY_FORWARD 0x45D
	#define AF_MACRO_FORWARD 0x45F

	uint32_t dac_macro;
	uint32_t dac_infinity;
	if(g_dit_eeprom_info[moduleGroupIndex].mapdata != NULL)
	{
		dac_infinity = g_dit_eeprom_info[moduleGroupIndex].mapdata[AF_INFINITY_FORWARD]; //low byte
		dac_infinity |= g_dit_eeprom_info[moduleGroupIndex].mapdata[AF_INFINITY_FORWARD+1] << 8; //high byte
		dac_macro = g_dit_eeprom_info[moduleGroupIndex].mapdata[AF_MACRO_FORWARD]; //low byte
		dac_macro |= g_dit_eeprom_info[moduleGroupIndex].mapdata[AF_MACRO_FORWARD+1] << 8; //high byte
		pr_info("physical sensor module name(%s) dac (marcro,infinity) = (%u,%u)\n",g_eeprom_group[moduleGroupIndex].physicalSensorModuleName,dac_macro,dac_infinity);
		//set_ois_dit_afc_data_from_eeprom(dac_macro, dac_infinity,g_eeprom_group[moduleGroupIndex].physicalSensorModuleName);
		return;
	}
	pr_err("physical sensor module name(%s) , no dit eeprom info to read \n",g_eeprom_group[moduleGroupIndex].physicalSensorModuleName);
	//set_ois_dit_afc_data_from_eeprom(0, 0,g_eeprom_group[moduleGroupIndex].physicalSensorModuleName);
}
static void getAllFileNameByModuleID(uint32_t soc_index, uint8_t moduleGroupIndex, char *procEEpromName, char *ditProcEEpromName, char *procOtpName) {

	sprintf(procEEpromName, PROC_EEPROM, soc_index);
	sprintf(ditProcEEpromName, PROC_DIT_EEPROM, soc_index);
	sprintf(procOtpName, PROC_OTP_NAME, soc_index);
	pr_info("procEEpromName = %s, ditProcEEpromName = %s procOtpName = %s\n", procEEpromName, ditProcEEpromName, procOtpName);

	return;
}

void eeprom_dump_create(struct cam_eeprom_ctrl_t * e_ctrl,uint8_t moduleGroupIndex)
{
	char* file_name;
	char* dit_file_name;
	char procEEpromName[30];
	char ditProcEEpromName[30];
	char procOtpName[30];
	char* otp_name;//Asus Bsp Ryan ++
	if(e_ctrl == NULL)
	{
		pr_err("e_ctrl is NULL!\n");
		return;
	}

	if(moduleGroupIndex == EEPROM_GROUP_SIZE) {
		pr_err("No match module_id(%u) in table, return directly\n", e_ctrl->cal_data.mapdata[EEPROM_MODULEID_OFFSET]);
		return;
	}

	pr_info("moduleGroupIndex(%u) eeprom_soc_index(%u)\n",moduleGroupIndex, e_ctrl->soc_info.index);
	getAllFileNameByModuleID(e_ctrl->soc_info.index, moduleGroupIndex, procEEpromName, ditProcEEpromName, procOtpName);

	pr_info("[DIT_EEPROM]  moduleGroupIndex(%d)  procEEpromName:%s  ditProcEEpromName:%s  procOtpName:%s", moduleGroupIndex, procEEpromName, ditProcEEpromName, procOtpName);

	if(g_eeprom_ctrl[moduleGroupIndex] != NULL)//if loaded. free it first
	{
		pr_info("[DIT_EEPROM] loaded. free it first ");
		//remove proc files
		remove_proc_entry(ditProcEEpromName, NULL);
		remove_proc_entry(procEEpromName, NULL);
		remove_proc_entry(procOtpName, NULL);
		dual_cali_dump_remove(moduleGroupIndex);

		//free g_dit_eeprom_info, g_eeprom_info, g_eeprom_ctrl
		kfree(g_dit_eeprom_info[moduleGroupIndex].mapdata);
		kfree(g_eeprom_info[moduleGroupIndex].mapdata);
		g_dit_eeprom_info[moduleGroupIndex].num_map = 0;
		g_dit_eeprom_info[moduleGroupIndex].num_data = 0;
		g_dit_eeprom_info[moduleGroupIndex].mapdata = NULL;
		g_dit_eeprom_info[moduleGroupIndex].module_id= 0;

		g_eeprom_info[moduleGroupIndex].num_map = 0;
		g_eeprom_info[moduleGroupIndex].num_data = 0;
		g_eeprom_info[moduleGroupIndex].mapdata = NULL;
		g_eeprom_info[moduleGroupIndex].module_id = 0;

		g_eeprom_ctrl[moduleGroupIndex] = NULL;
	}


	if(g_eeprom_ctrl[moduleGroupIndex] == NULL)//first time
	{
		save_actual_eeprom_info(e_ctrl, moduleGroupIndex); //save module eeprom data to g_eeprom_ctrl
		g_eeprom_ctrl[moduleGroupIndex] = e_ctrl;
		file_name = procEEpromName;
		dit_file_name = ditProcEEpromName;
		otp_name = procOtpName;//ASUS BSP ryan ++ FOR Cam 1 eeprom

		//-Save value in g_dit_eeprom_info
		g_dit_eeprom_info[moduleGroupIndex].module_id = g_eeprom_group[moduleGroupIndex].eeprom_module_Id;
		g_dit_eeprom_info[moduleGroupIndex].num_map = e_ctrl->cal_data.num_map;
		g_dit_eeprom_info[moduleGroupIndex].num_data = e_ctrl->cal_data.num_data;

		//- create proc files
		create_proc_file(file_name, &eeprom_proc_fops, &g_eeprom_info[moduleGroupIndex]);  //creat  driver/eeprom file
		create_proc_file(otp_name, &eeprom_to_otp_fops, &g_eeprom_info[moduleGroupIndex]); // eeprom to otp   "driver/otp"
		create_proc_file(dit_file_name, &dit_eeprom_proc_fops, &g_dit_eeprom_info[moduleGroupIndex]);   //"driver/dit_eeprom"


		//TODO Randy need to add proc file(PROC_ARCSOFT_CALI_1x,PROC_ARCSOFT_CALI_3x) here
		dual_cali_dump_create(moduleGroupIndex);  //create_proc_file for Arcsoft_calibration (for camera alignment: e.g. Bokeh, SAT)

		//- Merge EEProm + DIT_Calibration file
		cover_dit_cal_data(e_ctrl, moduleGroupIndex);
		set_actual_afc_data(moduleGroupIndex);
		set_dit_afc_data(moduleGroupIndex); //pass macro/infinity calibration to OIS
	}
	else
	{
		pr_info("EEPROM debug file for moduleGroupIndex %u already created\n",moduleGroupIndex);
	}
}
//Module Changed ++++

static int module_changed_read(struct seq_file *buf, void *v)
{
	uint32_t* module_changed = (uint32_t *)buf->private;
	seq_printf(buf, "%d\n",*module_changed);
	return 0;
}

static int module_changed_open(struct inode *inode, struct file *file)
{
	return single_open(file, module_changed_read, PDE_DATA(inode));
}

static ssize_t module_changed_write(struct file *dev, const char *buf, size_t len, loff_t *ppos)
{
	ssize_t ret_len;
	char messages[8]="";

	uint32_t val;

	uint32_t *module_changed = PDE_DATA(file_inode(dev));

	ret_len = len;
	if (len > 8) {
		len = 8;
	}
	if (copy_from_user(messages, buf, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages, "%d", &val);

	if(val == 0)
		*module_changed = 0;
	else
		*module_changed = 1;

	return ret_len;
}

static const struct file_operations module_changed_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= module_changed_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= module_changed_write,
};

static char *g_module_changed_file_name[MAX_CAMERA_ID + 1]=
{	PROC_MODULE_CHANGE_REAR,
	PROC_MODULE_CHANGE_FRONT,
	PROC_MODULE_CHANGE_REAR2,
	PROC_MODULE_CHANGE_REAR3,
	PROC_MODULE_CHANGE_REAR4
};

void module_changed_create(uint16_t camera_id)
{
	char* file_name = g_module_changed_file_name[camera_id];
	create_proc_file(file_name, &module_changed_proc_fops, &g_module_changed[camera_id]);
}

//Module Changed ----

//ASUS_BSP +++ "Add for camera csi debug"
static ssize_t cam_csi_check_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	buff = kzalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	pr_err("camera csi check %d\n", g_cam_csi_check);

	len += sprintf(buff, "%d\n", g_cam_csi_check);
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}


static ssize_t cam_csi_check_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	memset(messages, 0, sizeof(messages));

	if (len > 256)
		len = 256;
	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if (strncmp(messages, "0", 1) == 0) {
		g_cam_csi_check = 0;
		pr_err("[CAM] write g_cam_csi_check is %d\n", g_cam_csi_check);
	}

	return len;
}


static struct file_operations cam_csi_check_fops = {
	.read  = cam_csi_check_read,
	.write = cam_csi_check_write,
};

static void cam_csi_debug_create(void)
{
	create_proc_file(PROC_CSI_CHECK, &cam_csi_check_fops, NULL);
}
//ASUS_BSP --- "Add for camera csi debug"


//ASUS_BSP +++ "Add for camera cci debug"
static ssize_t cam_cci_check_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	buff = kzalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	pr_err("camera cci check %d\n", g_cam_cci_check);

	len += sprintf(buff, "%d\n", g_cam_cci_check);
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static ssize_t cam_cci_check_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	memset(messages, 0, sizeof(messages));

	if (len > 256)
		len = 256;
	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if (strncmp(messages, "0", 1) == 0) {
		g_cam_cci_check = 0;
		pr_err("[CAM] write g_cam_cci_check is %d\n", g_cam_cci_check);
	}

	return len;
}


static struct file_operations cam_cci_check_fops = {
	.read  = cam_cci_check_read,
	.write = cam_cci_check_write,
};

static void cam_cci_debug_create(void)
{
	create_proc_file(PROC_CCI_CHECK, &cam_cci_check_fops, NULL);
}

//ASUS_BSP --- "Add for camera cci debug"

void asus_cam_sensor_init(struct cam_sensor_ctrl_t *s_ctrl)
{

	pr_info("CAMERA ID %d init :E", s_ctrl->id);
	if(!s_ctrl)
	{
		pr_err("s_ctrl is NULL\n");
		return;
	}

	if(s_ctrl->id > MAX_CAMERA_ID)
	{
		pr_err("s_ctrl id %d invalid!\n",s_ctrl->id);
		return;
	}
	if(!g_sensor_init_state[s_ctrl->id])
	{
		g_sensor_ctrl[s_ctrl->id] = s_ctrl;
		g_sensor_id[s_ctrl->id] = s_ctrl->sensordata->slave_info.sensor_id;
		pr_info("CAMERA ID %d, Sensor id 0x%X E\n", s_ctrl->id, g_sensor_id[s_ctrl->id]);

		module_create(s_ctrl->id);
		resolution_proc_create(s_ctrl->id);
		status_proc_create(s_ctrl->id);
		module_changed_create(s_ctrl->id);

		if(s_ctrl->id == CAMERA_0)
		{
			sensor_i2c_create();
			eeprom_i2c_create();//for debug
			sensors_res_create();//for shipping image
			cam_csi_debug_create();
			cam_cci_debug_create();
		}

		if(s_ctrl->sensordata->slave_info.sensor_id == 0x885A)
		{
			otp_dump_create(s_ctrl->id);
		}


		pr_info("CAMERA ID %d, Sensor id 0x%X X\n",s_ctrl->id, g_sensor_id[s_ctrl->id]);
		g_sensor_init_state[s_ctrl->id] = 1;
	}
	else
	{
		pr_err("camera id %d already inited!\n",s_ctrl->id);

		if(s_ctrl->sensordata->slave_info.sensor_id == 0x885A)
		{
			pr_info("otp_dump_create again");
			otp_dump_create(s_ctrl->id);
		}
	}


	pr_info("CAMERA ID %d   init :X", s_ctrl->id);
}
