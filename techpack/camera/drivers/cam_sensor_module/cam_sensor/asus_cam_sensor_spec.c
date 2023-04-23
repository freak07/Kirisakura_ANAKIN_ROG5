static void delay_ms(uint32_t time)
{
	usleep_range(time*1000,time*1000+time*10);
}
static int read_otp_ov13855(struct cam_sensor_ctrl_t *s_ctrl ,uint8_t *otp_data)
{
	int rc;
	int i;

	uint32_t start_addr_production;
	uint32_t start_addr_otp;
	uint32_t end_addr;

	uint8_t  internal_data[16];

	start_addr_production = 0x7000;
	start_addr_otp = 0x7220;
	end_addr = 0x727F;



	//reset sensor as default
	rc = cam_sensor_write_byte(s_ctrl,0x0103,0x01);
	if(rc < 0)
	{
		pr_err("Reset sensor as default failed!\n");
		goto END;
	}

	rc = cam_sensor_write_byte(s_ctrl,0x5000,0x00);
	if(rc < 0)
	{
		pr_err("Write 0x5000 >00 failed!\n");
		goto END;
	}

	//0x3D84[6] 1, Manual mode(partial)  0x3D84[7] 1, Program disable
	rc = cam_sensor_write_byte(s_ctrl,0x3D84,0xC0);
	if(rc < 0)
	{
		pr_err("Write 0x3D84 failed!\n");
		goto END;
	}

	//[3] 1, OTP power up load data enable;  [1] 1, OTP power up load setting enable
	rc = cam_sensor_write_byte(s_ctrl,0x3D85,0x0A);
	if(rc < 0)
	{
		pr_err("Write 0x3D85 failed!\n");
		goto END;
	}

	//[0x3D8C 0x3D8D] set 0x73C0
	rc = cam_sensor_write_byte(s_ctrl,0x3D8C,0x73);
	if(rc < 0)
	{
		pr_err("Write 0x3D8C failed!\n");
		goto END;
	}
	rc = cam_sensor_write_byte(s_ctrl,0x3D8D,0xC0);
	if(rc < 0)
	{
		pr_err("Write 0x3D8C failed!\n");
		goto END;
	}

	//set address range, MSB in lower address
	rc = cam_sensor_write_byte(s_ctrl,0x3D88,0x70);
	if(rc == 0)
		rc = cam_sensor_write_byte(s_ctrl,0x3D89,0x00);
	if(rc == 0)
		rc = cam_sensor_write_byte(s_ctrl,0x3D8A,0x72);
	if(rc == 0)
		rc = cam_sensor_write_byte(s_ctrl,0x3D8B,0x7F);
	if(rc < 0)
	{
		pr_err("Set address range failed!\n");
		goto END;
	}


	//trigger auto_load, stream mode enable; write 0x01 to 0x3D81, OTP_load_enable
	rc = cam_sensor_write_byte(s_ctrl, 0x0100, 0x01);
	if(rc == 0)
		rc = cam_sensor_write_byte(s_ctrl, 0x3D81, 0x01);
	if(rc < 0)
	{
		pr_err("Trigger load failed!\n");
		goto END;
	}

	delay_ms(20);


	rc = cam_sensor_read_seq_bytes(s_ctrl,start_addr_production,internal_data,16);
	if(rc < 0)
	{
		pr_err("Read internal 16 bytes data failed!\n");
		goto END;
	}
	else
	{
		pr_info("Production ID is [0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x]\n",
				internal_data[0],internal_data[1],internal_data[2],internal_data[3],internal_data[4],internal_data[5],
				internal_data[6],internal_data[7],internal_data[8],internal_data[9],internal_data[10],internal_data[11],
				internal_data[12],internal_data[13],internal_data[14],internal_data[15]
				);
	}


	for(i=0;i<3;i++)
	{

		rc = cam_sensor_read_seq_bytes(s_ctrl,
                                       start_addr_otp+(i*OTP_DATA_LEN_WORD),//read 32 bytes
                                       otp_data+(i*OTP_DATA_LEN_BYTE),//store in 64 bytes
                                       OTP_DATA_LEN_WORD);

		if(rc < 0)
		{
			pr_err("Read OTP data bank %d failed!\n",i);
			goto END;
		}
	}


	rc = cam_sensor_write_byte(s_ctrl,0x5000,0x10);
	if(rc < 0)
	{
		pr_err("Write 0x5000 > 10 failed!\n");
		goto END;
	}
END:
	return rc;
}


static int read_otp_ov8856(struct cam_sensor_ctrl_t *s_ctrl ,uint8_t *otp_data)
{
	int rc;
	int i;

	uint32_t start_addr;
	uint32_t end_addr;
	uint32_t reg_value;

	uint8_t  internal_data[16];
	pr_err("read_otp_ov8856 E\n");
	//reset sensor as default
	rc = cam_sensor_write_byte(s_ctrl,0x0103,0x01);
	if(rc < 0)
	{
		pr_err("Reset sensor as default failed!\n");
		goto END;
	}

	//set 0x5001[3] 0, OTP option disable
	rc = cam_sensor_read_byte(s_ctrl,0x5001,&reg_value);
	if(rc < 0)
	{
		pr_err("Read 0x5001 failed!\n");
		goto END;
	}
	rc = cam_sensor_write_byte(s_ctrl,0x5001,reg_value & (~0x08));
	if(rc < 0)
	{
		pr_err("Write 0x5001 failed!\n");
		goto END;
	}

	//0x3D84[6] 1, Manual mode(partial)
	rc = cam_sensor_write_byte(s_ctrl,0x3D84,0xC0);
	if(rc < 0)
	{
		pr_err("Write 0x3D84 failed!\n");
		goto END;
	}

	//[2] 1, OTP power up load data enable;  [1] 1, OTP power up load setting enable
	rc = cam_sensor_write_byte(s_ctrl,0x3D85,0x06);
	if(rc < 0)
	{
		pr_err("Write 0x3D85 failed!\n");
		goto END;
	}

	start_addr = 0x7000;
	end_addr = start_addr+OTP_DATA_LEN_WORD*3-1+0x10; //0x7065

	//set address range, MSB in lower address
	rc = cam_sensor_write_byte(s_ctrl,0x3D88,(start_addr & 0xff00)>>8);
	if(rc == 0)
		rc = cam_sensor_write_byte(s_ctrl,0x3D89,(start_addr & 0xff));
	if(rc == 0)
		rc = cam_sensor_write_byte(s_ctrl,0x3D8A,(end_addr & 0xff00)>>8);
	if(rc == 0)
		rc = cam_sensor_write_byte(s_ctrl,0x3D8B,(end_addr & 0xff));
	if(rc < 0)
	{
		pr_err("Set address range failed!\n");
		goto END;
	}

	//trigger auto_load, stream mode enable; write 0x01 to 0x3D81, OTP_load_enable
	rc = cam_sensor_write_byte(s_ctrl, 0x0100, 0x01);
	if(rc == 0)
		rc = cam_sensor_write_byte(s_ctrl, 0x3D81, 0x01);
	if(rc < 0)
	{
	pr_err("Trigger load failed!\n");
		goto END;
	}
	delay_ms(20);

	rc = cam_sensor_read_seq_bytes(s_ctrl,start_addr,internal_data,16);
	if(rc < 0)
	{
		pr_err("Read internal 16 bytes data failed!\n");
		goto END;
	}
	else
	{
		pr_info("Internal data is [0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x]\n",
				internal_data[0],internal_data[1],internal_data[2],internal_data[3],internal_data[4],internal_data[5],
				internal_data[6],internal_data[7],internal_data[8],internal_data[9],internal_data[10],internal_data[11],
				internal_data[12],internal_data[13],internal_data[14],internal_data[15]
				);

	}

	start_addr += 0x10;
	for(i=0;i<3;i++)
	{
		rc = cam_sensor_read_seq_bytes(s_ctrl,
                                       start_addr+i*OTP_DATA_LEN_WORD,//read 32 bytes
                                       otp_data+i*OTP_DATA_LEN_BYTE,//store in 64 bytes
                                       OTP_DATA_LEN_WORD);
		if(rc < 0)
		{
			pr_err("Read OTP data bank %d failed!\n",i);
			goto END;
		}
	}
	pr_err("read_otp_ov8856 X\n");
END:
	return rc;
}


static int read_sensor_otp(struct cam_sensor_ctrl_t *s_ctrl ,uint8_t *otp_data)
{
	int rc;
	uint8_t auto_power;//whether need power up/down
	int read_result = -1;

	if(!s_ctrl)
	{
		pr_err("s_ctrl is NULL!\n");
		return -EINVAL;
	}

	//this is called from sensor probe cmd, no need use mutex
	if(s_ctrl->power_state == 1)
		auto_power = 0;
	else
		auto_power = 1;

	if(auto_power)
	{
		rc = cam_sensor_power_up(s_ctrl);
		if (rc < 0) {
			pr_err("power up failed");
			goto END;
		}
	}

	switch(s_ctrl->sensordata->slave_info.sensor_id)
	{
		case SENSOR_ID_OV13855:
			 read_result = read_otp_ov13855(s_ctrl, otp_data);
			 break;

		case SENSOR_ID_OV8856:
			 read_result = read_otp_ov8856(s_ctrl, otp_data);
			 break;

		default:
			 pr_err("not support for sensor id 0x%x\n", s_ctrl->sensordata->slave_info.sensor_id);
			 read_result = -1;
			 break;
	}

	if(auto_power)
	{
		rc = cam_sensor_power_down(s_ctrl);
		if (rc < 0) {
			pr_err("power down failed");
		}
	}

END:
	return read_result;
}

static void override_otp_from_eeprom(uint8_t * otp_data, uint8_t * eeprom_data, uint32_t camera_id)
{
    //camera 0,2 share the eeprom but camera 2 does not need AF data
	if(camera_id == CAMERA_0)//IMX686
	{
		pr_info("override OTP %d bytes for CAMERA 0 include AF DATA\n", OTP_DATA_LEN_WORD);
		memcpy(otp_data, eeprom_data, OTP_DATA_LEN_WORD);
	}
	else if(camera_id == CAMERA_2)//OV13B10
	{
		pr_info("override OTP %d bytes for CAMERA 2 NO AF DATA\n", OTP_DATA_LEN_WORD-8);
		memcpy(otp_data, eeprom_data+7165, OTP_DATA_LEN_WORD);
	}
}
