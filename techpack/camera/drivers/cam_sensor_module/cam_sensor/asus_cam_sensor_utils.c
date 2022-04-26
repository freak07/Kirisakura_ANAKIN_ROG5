typedef struct
{
	uint16_t id;
	char	 name[8];
	uint8_t	 resolution;//M
	uint32_t test_reg_addr;//reg addr to do i2c R/W test
}sensor_db_t;

static sensor_db_t g_sensor_info[] =
{
	{SENSOR_ID_IMX686, "IMX686", 64, 0x0686},
	{SENSOR_ID_OV8856, "OV8856", 8, 0x885a},
	//{SENSOR_ID_OV13855, "OV13855", 13, 0xD855},
	{SENSOR_ID_OV13B10, "OV13B10", 13, 0x0D42},
	{SENSOR_ID_OV24B1Q, "OV24B1Q", 24, 0x2442},
};


typedef enum
{
	DIRECTION_REAR,
	DIRECTION_FRONT
}sensor_direct_t;

typedef struct
{
	uint16_t		 sensor_id;
	sensor_direct_t	 direction;
}sensor_define_t;

static void sort_cam_res(uint8_t* cam_res, int size)
{
	int i,j;
	uint8_t temp;
	for(i=0;i<size-1;i++)
	{
		for(j=0;j<size-1-i;j++)//little to tail
		{
			if(cam_res[j]<cam_res[j+1])
			{
				temp = cam_res[j];
				cam_res[j] = cam_res[j+1];
				cam_res[j+1] = temp;
			}
		}
	}
}

static char * get_sensor_name(uint16_t sensor_id)
{
	int i;
	for(i=0;i<sizeof(g_sensor_info)/sizeof(sensor_db_t);i++)
	{
		if(g_sensor_info[i].id == sensor_id)
		{
			return g_sensor_info[i].name;
		}
	}
	return "Unknown Module";
}

static uint8_t get_sensor_resolution(uint16_t sensor_id)
{
	int i;
	for(i=0;i<sizeof(g_sensor_info)/sizeof(sensor_db_t);i++)
	{
		if(g_sensor_info[i].id == sensor_id)
		{
			return g_sensor_info[i].resolution;
		}
	}
	return 0;
}
static uint32_t get_sensor_test_reg_addr(uint16_t sensor_id)
{
	int i;
	for(i=0;i<sizeof(g_sensor_info)/sizeof(sensor_db_t);i++)
	{
		if(g_sensor_info[i].id == sensor_id)
		{
			return g_sensor_info[i].test_reg_addr;
		}
	}
	return 0;
}

static const char * submodule2string(enum sensor_sub_module sub_module)
{
	switch(sub_module)
	{
		case SUB_MODULE_SENSOR:
			return "SENSOR";
		case SUB_MODULE_ACTUATOR:
			return "ACTUATOR";
		case SUB_MODULE_EEPROM:
			return "EEPROM";
		case SUB_MODULE_LED_FLASH:
			return "FLASH";
		case SUB_MODULE_CSID:
			return "CSID";
		case SUB_MODULE_CSIPHY:
			return "CSIPHY";
		case SUB_MODULE_OIS:
			return "OIS";
		case SUB_MODULE_EXT:
			return "EXT";
		case SUB_MODULE_MAX:
		default:
			return "InvalidSubModule";
	}
}
//just for EEPROM and Actuator to determine file name

uint16_t get_camera_sensor_id(uint32_t camera_id) {
	pr_info("cameraID(%d) get sensorID = 0x%x\n",camera_id, g_sensor_ctrl[camera_id]->sensordata->slave_info.sensor_id);
	return  g_sensor_ctrl[camera_id]->sensordata->slave_info.sensor_id;
}

int get_camera_id_for_submodule(enum sensor_sub_module sub_module, uint32_t index, uint32_t* camera_id)
{
	int i;

	if(sub_module >= SUB_MODULE_MAX)
	{
		pr_err("invalid sub module type %d!\n",sub_module);
		return -1;
	}

	for(i=0;i<=MAX_CAMERA_ID;i++)
	{
		if(g_sensor_ctrl[i] && g_sensor_ctrl[i]->sensordata->subdev_id[sub_module]!= -1)
		{
			if(g_sensor_ctrl[i]->sensordata->subdev_id[sub_module] == index)
			{
				//what if two camera share one submodule? such like eeprom?
				//return first camera id matched
				//one camera have one specific type submodule
				*camera_id = i;
				pr_err("got camera id %d for submodule %s, index %d sensor_probe_addr_type =0x%x\n",
						*camera_id,
						submodule2string(sub_module),
						index,
						g_sensor_ctrl[i]->sensor_probe_addr_type
						);
				return 0;
			}
		}
	}
	return -2;
}

static void create_proc_file(const char *PATH, const struct file_operations* f_ops, void *data)
{
	struct proc_dir_entry *pde;

	pde = proc_create_data(PATH, 0666, NULL, f_ops, data);
	if(pde)
	{
		pr_info("create(%s) done\n", PATH);
	}
	else
	{
		pr_err("create(%s) failed!\n", PATH);
	}
}

static int cam_sensor_read_byte(struct cam_sensor_ctrl_t *ctrl, uint32_t reg_addr, uint32_t* reg_data)
{
	int rc = 0;

	rc = camera_io_dev_read(&(ctrl->io_master_info),
								reg_addr,
								reg_data,
								CAMERA_SENSOR_I2C_TYPE_WORD,//addr_type
								CAMERA_SENSOR_I2C_TYPE_BYTE);//data_type
	if(rc < 0)
	{
		pr_err("read reg 0x%04x failed, rc = %d\n",reg_addr,rc);
	}

	return rc;
}

static int cam_sensor_read_word(struct cam_sensor_ctrl_t *ctrl, uint32_t reg_addr, uint32_t* reg_data)
{
	int rc = 0;

	rc = camera_io_dev_read(&(ctrl->io_master_info),
								reg_addr,
								reg_data,
								CAMERA_SENSOR_I2C_TYPE_WORD,//addr_type
								CAMERA_SENSOR_I2C_TYPE_WORD);//data_type
	if(rc < 0)
	{
		pr_err("read reg 0x%04x failed, rc = %d\n",reg_addr,rc);
	}
	return rc;
}

static int cam_sensor_write_byte(struct cam_sensor_ctrl_t *ctrl, uint32_t reg_addr, uint32_t reg_data)
{
	int rc = 0;

	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array data;

	data.reg_addr = reg_addr;
	data.reg_data = reg_data;
	data.delay = 0;
	data.data_mask = 0;//how to use? not used so far

	write_setting.reg_setting = &data;
	write_setting.size = 1;
	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.delay = 0;

	rc = camera_io_dev_write(&(ctrl->io_master_info),&write_setting);
	if(rc < 0)
	{
		pr_err("write 0x%x to reg 0x%x failed! rc = %d",
						data.reg_data,data.reg_addr,rc);
	}
	return rc;
}
static int cam_sensor_write_word(struct cam_sensor_ctrl_t * ctrl, uint32_t reg_addr, uint32_t reg_data)
{
	int rc = 0;

	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array data;

	data.reg_addr = reg_addr;
	data.reg_data = reg_data;
	data.delay = 0;
	data.data_mask = 0;//how to use?

	write_setting.reg_setting = &data;
	write_setting.size = 1;
	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	write_setting.delay = 0;

	rc = camera_io_dev_write(&(ctrl->io_master_info),&write_setting);
	if(rc < 0)
	{
		pr_err("write 0x%x to reg 0x%x failed! rc = %d",
						data.reg_data,data.reg_addr,rc);
	}
	return rc;
}

int cam_sensor_read_seq_bytes(struct cam_sensor_ctrl_t * ctrl, uint32_t reg_addr, uint8_t* reg_data, uint32_t size)
{
	int rc = 0;

	if(size > I2C_REG_DATA_MAX)
	{
		pr_err("read size %d too large, max seq size is %d\n",size,I2C_REG_DATA_MAX);
		return -1;
	}

	rc = camera_io_dev_read_seq(&(ctrl->io_master_info),
								reg_addr,
								reg_data,
								CAMERA_SENSOR_I2C_TYPE_WORD,
								CAMERA_SENSOR_I2C_TYPE_BYTE,
								size);
	if(rc < 0)
	{
		pr_err("seq read %d bytes from reg 0x%04x failed, rc = %d\n",size,reg_addr,rc);
	}
	return rc;
}

static int cam_sensor_test_i2c(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc;
	int i;
	int full_check = 1;//whether power up/down to do full check
	uint32_t test_reg_addr;
	uint32_t test_reg_val;
	int check_result = -1;

	mutex_lock(&(s_ctrl->cam_sensor_mutex));

	if(s_ctrl->power_state == 1)
	{
		full_check = 0;//just read/write
	}

	if(full_check)
	{
		rc = cam_sensor_power_up(s_ctrl);
		if (rc < 0) {
			pr_err("power up failed");
			goto POWER_DOWN;
		}
	}

	test_reg_addr = get_sensor_test_reg_addr(s_ctrl->sensordata->slave_info.sensor_id);

	for(i=0;i<5;i++)
	{
		check_result = cam_sensor_read_word(s_ctrl, test_reg_addr, &test_reg_val);
		if (check_result < 0)
		{
			pr_err("check read failed!n");
			break;
		}
		check_result = cam_sensor_write_word(s_ctrl, test_reg_addr, test_reg_val);
		if (check_result < 0)
		{
			pr_err("check write failed!n");
			break;
		}
	}

POWER_DOWN:
	if(full_check)
	{
		rc = cam_sensor_power_down(s_ctrl);
		if (rc < 0) {
			pr_err("power down failed");
		}
	}

	mutex_unlock(&(s_ctrl->cam_sensor_mutex));
	return check_result;
}
static int read_file_into_buffer(const char *filename, uint8_t* data, uint32_t size)
{
	struct file *fp;
	loff_t pos;
	int num=0;

	fp = filp_open(filename,O_RDONLY,S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR(fp)) {
		pr_err("open(%s) failed\n", filename);
		return -1;
	}

	pos = 0;
	num = kernel_read(fp, data, size, &pos);

#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT
	filp_close(fp, NULL);
#endif
	pr_info("[DIT_EEPROM] read %s file, read %d bytes ",filename, num);

	return num;
}
