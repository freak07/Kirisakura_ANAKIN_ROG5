/*
 * Copyright (c) 2020, ASUS. All rights reserved.
 */

#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/syscalls.h>
#include <drm/drm_anakin.h>

#include "dsi_anakin.h"
#if defined(CONFIG_PXLW_IRIS)
#include "iris/dsi_iris6_api.h"
#endif

#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT
#include <sde_encoder_phys.h>

struct dsi_display *g_display;
char g_reg_buffer[REG_BUF_SIZE];
int display_commit_cnt = COMMIT_FRAMES_COUNT;
extern char g_lcd_unique_id[10];
// FOD feature
bool has_fod_masker;
bool old_has_fod_masker;
bool has_fod_spot;  //flag indicate the fod spot layer exist
bool old_has_fod_spot;
extern int fod_spot_ui_ready; //flag indicate the fod spot has shown on screen
// CHARGER mode
extern bool g_Charger_mode;
// DC mode
bool dc_fixed_bl;
// GAMMA register
char g_gamma_B7_cmd[32];
char g_gamma_B9_cmd[32];

// check display or panal valid
static inline bool display_panel_valid(void)
{
	if (!g_display || !g_display->panel) {
		DSI_LOG("[%pS] display or panel is not valid.\n", __builtin_return_address(0));
		return false;
	}

	return true;
}

// write panel command
static void set_tcon_cmd(char *cmd, short len)
{
	int i = 0;
	ssize_t rc;
	struct dsi_cmd_desc cmds;
	struct mipi_dsi_msg tcon_cmd = {0, 0x15, 0, 0, 0, len, cmd, 0, NULL};
	const struct mipi_dsi_host_ops *ops = g_display->panel->host->ops;

	if (!g_display || !g_display->panel) {
		DSI_LOG("Invalid params\n");
		return;
	}

	for(i=0; i<1; i++)
		DSI_LOG("cmd[%d] = 0x%02x\n", i, cmd[i]);

	if(len > 2)
		tcon_cmd.type = 0x39;

	mutex_lock(&g_display->display_lock);
	dsi_panel_acquire_panel_lock(g_display->panel);

	if (g_display->ctrl[0].ctrl->current_state.controller_state == DSI_CTRL_ENGINE_ON) {
		// cmds assigned
		cmds.msg = tcon_cmd;
		cmds.last_command = 1;
		cmds.post_wait_ms = 1;
		if (cmds.last_command) {
			cmds.msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;
		}

		rc = ops->transfer(g_display->panel->host, &cmds.msg);
		if (rc < 0) {
			DSI_LOG("tx cmd transfer failed rc=%d\n", rc);
		}
	} else {
		DSI_LOG("DSI_CTRL_ENGINE is off\n");
	}

	dsi_panel_release_panel_lock(g_display->panel);
	mutex_unlock(&g_display->display_lock);
}

// read panel command
static void get_tcon_cmd(char cmd, int rlen)
{
	char tmp[256];
	int i = 0, rc = 0, start = 0;
	u8 *tx_buf, *return_buf, *status_buf;

	struct dsi_cmd_desc cmds;
	struct mipi_dsi_msg tcon_cmd = {0, 0x06, 0, 0, 0, sizeof(cmd), NULL, rlen, NULL};
	const struct mipi_dsi_host_ops *ops = g_display->panel->host->ops;

	if (!g_display || !g_display->panel) {
		DSI_LOG("Invalid params\n");
		return;
	}

	mutex_lock(&g_display->display_lock);
	dsi_panel_acquire_panel_lock(g_display->panel);

	if (g_display->ctrl[0].ctrl->current_state.controller_state == DSI_CTRL_ENGINE_ON) {
		// buffer assigned
		tx_buf = &cmd;
		return_buf = kcalloc(rlen, sizeof(unsigned char), GFP_KERNEL);
		status_buf = kzalloc(SZ_4K, GFP_KERNEL);
		memset(status_buf, 0x0, SZ_4K);

		// cmds assigned
		cmds.msg = tcon_cmd;
		cmds.last_command = 1;
		cmds.post_wait_ms = 0;
		if (cmds.last_command) {
			cmds.msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;
		}
		cmds.msg.flags |= MIPI_DSI_MSG_CMD_READ;

		cmds.msg.tx_buf = tx_buf;
		cmds.msg.rx_buf = status_buf;
		cmds.msg.rx_len = rlen;
		memset(g_reg_buffer, 0, REG_BUF_SIZE*sizeof(char));

		rc = ops->transfer(g_display->panel->host, &cmds.msg);
		if (rc <= 0) {
			DSI_LOG("rx cmd transfer failed rc=%d\n", rc);
		} else {
			memcpy(return_buf + start, status_buf, rlen);
			start += rlen;

			for(i=0; i<rlen; i++) {
				memset(tmp, 0, 256*sizeof(char));
				snprintf(tmp, sizeof(tmp), "0x%02x = 0x%02x\n", cmd, return_buf[i]);
				strcat(g_reg_buffer,tmp);
			}
		}
	} else {
		DSI_LOG("DSI_CTRL_ENGINE is off\n");
	}

	dsi_panel_release_panel_lock(g_display->panel);
	mutex_unlock(&g_display->display_lock);
}

// copy from dsi_panel_tx_cmd_set()
static int dsi_anakin_tx_cmd_set(struct dsi_panel *panel,
				enum dsi_cmd_set_type type)
{
	int rc = 0, i = 0;
	ssize_t len;
	struct dsi_cmd_desc *cmds;
	u32 count;
	enum dsi_cmd_set_state state;
	struct dsi_display_mode *mode;
	const struct mipi_dsi_host_ops *ops = panel->host->ops;

	if (!panel || !panel->cur_mode)
		return -EINVAL;

	mode = panel->cur_mode;

	cmds = mode->priv_info->cmd_sets[type].cmds;
	count = mode->priv_info->cmd_sets[type].count;
	state = mode->priv_info->cmd_sets[type].state;

	if (count == 0) {
		DSI_LOG("[%s] No commands to be sent for state(%d)\n",
			 panel->name, type);
		goto error;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_mp_panel()) {
		if (iris_get_abyp_mode_blocking() == IRIS_PT_MODE) {
			rc = iris_pt_send_panel_cmd(panel, &(mode->priv_info->cmd_sets[type]));
			return rc;
		}
	}
#endif

	for (i = 0; i < count; i++) {
		if (state == DSI_CMD_SET_STATE_LP)
			cmds->msg.flags |= MIPI_DSI_MSG_USE_LPM;

		if (cmds->last_command)
			cmds->msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;

		if (type == DSI_CMD_SET_VID_TO_CMD_SWITCH)
			cmds->msg.flags |= MIPI_DSI_MSG_ASYNC_OVERRIDE;

		len = ops->transfer(panel->host, &cmds->msg);
		if (len < 0) {
			rc = len;
			DSI_LOG("failed to set cmds(%d), rc=%d\n", type, rc);
			goto error;
		}
		if (cmds->post_wait_ms)
			usleep_range(cmds->post_wait_ms*1000,
					((cmds->post_wait_ms*1000)+10));
		cmds++;
	}
error:
	// only for FOD HBM ON
	if (type == DSI_CMD_SET_FOD_HBM_ON || type == DSI_CMD_SET_POST_FOD_HBM_ON) {
		panel->ktime0 = ktime_get();
		//DSI_LOG("hbm on start on %d (us). +++", (u32)ktime_to_us(g_display->panel->ktime0));
	}
	return rc;
}

// send HBM command to panel
static int dsi_anakin_set_hbm(struct dsi_panel *panel, bool enable)
{
	int rc = 0;

	if (!panel) {
		DSI_LOG("invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);
	if (!panel->panel_initialized)
		goto exit;

	if (enable) {
		rc = dsi_anakin_tx_cmd_set(panel, DSI_CMD_SET_HBM_ON);
		if (rc)
			DSI_LOG("[%s] failed to send DSI_CMD_SET_HBM_ON cmd, rc=%d\n",
				   panel->name, rc);
	} else {
		rc = dsi_anakin_tx_cmd_set(panel, DSI_CMD_SET_HBM_OFF);
		if (rc)
			DSI_LOG("[%s] failed to send DSI_CMD_SET_HBM_OFF cmd, rc=%d\n",
				   panel->name, rc);
	}
exit:
	mutex_unlock(&panel->panel_lock);
	return rc;
}

// called for /proc/hbm_mode
static void display_set_hbm_mode(int mode)
{
	if (mode == g_display->panel->panel_hbm_mode) {
		DSI_LOG("hbm mode same.\n");
		return;
	}

	g_display->panel->panel_hbm_mode = mode;
	DSI_LOG("hbm mode is (%d)\n", g_display->panel->panel_hbm_mode);
	dsi_anakin_set_hbm(g_display->panel, mode);
}

// exit idle mode
static void display_exit_idle_mode()
{
	int rc = 0;

	DSI_LOG("power_mode/aod_state are (%d, %d)\n",
			g_display->panel->power_mode, g_display->panel->aod_state);
	// SDE_MODE_DPMS_LP1    1
	// SDE_MODE_DPMS_LP2    2
	// checking aod_state
	if ((g_display->panel->power_mode != 1 &&
			g_display->panel->power_mode != 2) && !g_display->panel->aod_state)
		return;

	rc = dsi_anakin_tx_cmd_set(g_display->panel, DSI_CMD_SET_NOLP);
	if (rc)
		DSI_LOG("[%s] failed to send DSI_CMD_SET_NOLP cmd, rc=%d\n",
			   g_display->panel->name, rc);
	else {
		g_display->panel->fod_in_doze = true;
		g_display->panel->aod_state = false;
		g_display->panel->panel_aod_last_bl = 0;
	}
}

// send Fod HBM command to panel
static int dsi_anakin_set_fod_hbm(struct dsi_panel *panel, bool enable)
{
	int rc = 0;

	if (!panel) {
		DSI_LOG("invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);
	if (!panel->panel_initialized)
		goto exit;

	if (enable) {
		// exit idle mode if enter doze before
		display_exit_idle_mode();

		// to aviod ghbm without mask
		if (panel->fod_in_doze) {
			DSI_LOG("set display off first\n");
			rc = dsi_anakin_tx_cmd_set(panel, DSI_CMD_SET_LP1);
			if (rc)
				DSI_LOG("[%s] failed to send DSI_CMD_SET_LP1 cmd, rc=%d\n",
					   panel->name, rc);
			else if (panel->cur_mode->timing.refresh_rate == 90) {
				rc = dsi_anakin_tx_cmd_set(panel, DSI_CMD_SET_TIMING_SWITCH);
				if (rc)
					DSI_LOG("[%s] failed to send DSI_CMD_SET_TIMING_SWITCH cmd, rc=%d\n",
					   panel->name, rc);
			}
		} else {
			DSI_LOG("set display on directly\n");
			rc = dsi_anakin_tx_cmd_set(panel, DSI_CMD_SET_FOD_HBM_ON);
			if (rc)
				DSI_LOG("[%s] failed to send DSI_CMD_SET_FOD_HBM_ON cmd, rc=%d\n",
					   panel->name, rc);
		}
	} else {
		dsi_anakin_restore_backlight();

		if (g_display->panel->panel_hbm_mode) {
			DSI_LOG("hbm on after fod\n");
			rc = dsi_anakin_tx_cmd_set(panel, DSI_CMD_SET_DIM_DLY_ON);
			if (rc)
				DSI_LOG("[%s] failed to send DSI_CMD_SET_DIM_DLY_ON cmd, rc=%d\n",
					   panel->name, rc);
		} else {
			rc = dsi_anakin_tx_cmd_set(panel, DSI_CMD_SET_FOD_HBM_OFF);
			if (rc)
				DSI_LOG("[%s] failed to send DSI_CMD_SET_FOD_HBM_OFF cmd, rc=%d\n",
					   panel->name, rc);
		}
	}

exit:
	mutex_unlock(&panel->panel_lock);
	return rc;
}

// called for /proc/globalHbm
static void display_set_fod_hbm_mode(int mode)
{
	if (mode == g_display->panel->panel_fod_hbm_mode) {
		DSI_LOG("fod hbm mode same.\n");
		return;
	}

	g_display->panel->panel_fod_hbm_mode = mode;
	DSI_LOG("fod hbm mode is (%d)\n", g_display->panel->panel_fod_hbm_mode);
	dsi_anakin_set_fod_hbm(g_display->panel, mode);
}

// called for FOD flow, as /proc/globalHbm
static void display_set_fod_hbm(void)
{
	if (!display_panel_valid())
		return;

	if (!g_display->panel->panel_is_on) {
		DSI_LOG("display is off.\n");
		return;
	}

	DSI_LOG("FOD: global hbm <fod> (%d) +++ \n", g_display->panel->allow_panel_fod_hbm);
	dsi_anakin_set_fod_hbm(g_display->panel, g_display->panel->allow_panel_fod_hbm);
	anakin_drm_notify(ASUS_NOTIFY_GHBM_ON_REQ, g_display->panel->allow_panel_fod_hbm);
	DSI_LOG("FOD: global hbm <fod> (%d) --- \n", g_display->panel->allow_panel_fod_hbm);

	// reset fod hbm pending immediately, need be guard by lock
	g_display->panel->allow_fod_hbm_process = false;
}

// send Dimming Smooth command to panel
void dsi_anakin_set_dimming_smooth(struct dsi_panel *panel, u32 backlight)
{
	int rc = 0;

	// don't allow in fod hbm
	if (panel->allow_panel_fod_hbm == 1)
		return;

	if (panel->aod_state ||panel->allow_fod_hbm_process || backlight == 0) {
		panel->panel_bl_count = 0;
		return;
	}

	if (panel->panel_bl_count == 1) {
		DSI_LOG("restore dimming smooth\n");
		rc = dsi_anakin_tx_cmd_set(panel, DSI_CMD_SET_DIMMING_SMOOTH);
		if (rc)
			DSI_LOG("[%s] failed to send DSI_CMD_SET_DIMMING_SMOOTH cmd, rc=%d\n",
				   panel->name, rc);
	}

	panel->panel_bl_count++;
}

// panel_reg_rw_ops() - read/write/show panel register
static ssize_t panel_reg_rw(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char *messages, *tmp, *cur;
	char *token, *token_par;
	char *put_cmd;
	bool flag = 0; /* w/r type : w=1, r=0 */
	int *store;
	int i = 0, cnt = 0, cmd_cnt = 0;
	int ret = 0;
	uint8_t str_len = 0;

	messages = (char*) kmalloc(len*sizeof(char), GFP_KERNEL);
	if(!messages)
		return -EFAULT;

	tmp = (char*) kmalloc(len*sizeof(char), GFP_KERNEL);
	memset(tmp, 0, len*sizeof(char));
	store =  (int*) kmalloc((len/MIN_LEN)*sizeof(int), GFP_KERNEL);
	put_cmd = (char*) kmalloc((len/MIN_LEN)*sizeof(char), GFP_KERNEL);
	memset(g_reg_buffer, 0, REG_BUF_SIZE*sizeof(char));

	/* add '\0' to end of string */
	if (copy_from_user(messages, buff, len)) {
		ret = -1;
		goto error;
	}

	cur = messages;
	*(cur+len-1) = '\0';
	DSI_LOG("%s +++\n", cur);

	if (strncmp(cur, "w", 1) == 0)
		flag = true;
	else if(strncmp(cur, "r", 1) == 0)
		flag = false;
	else {
		ret = -1;
		goto error;
	}

	while ((token = strsep(&cur, "wr")) != NULL) {
		str_len = strlen(token);

		if(str_len > 0) { /* filter zero length */
			if(!(strncmp(token, ",", 1) == 0) || (str_len < MAX_LEN)) {
				ret = -1;
				goto error;
			}

			memset(store, 0, (len/MIN_LEN)*sizeof(int));
			memset(put_cmd, 0, (len/MIN_LEN)*sizeof(char));
			cmd_cnt++;

			/* register parameter */
			while ((token_par = strsep(&token, ",")) != NULL) {
				if(strlen(token_par) > MIN_LEN) {
					ret = -1;
					goto error;
				}
				if(strlen(token_par)) {
					sscanf(token_par, "%x", &(store[cnt]));
					cnt++;
				}
			}

			for(i=0; i<cnt; i++)
				put_cmd[i] = store[i]&0xff;

			if(flag) {
				DSI_LOG("write panel command\n");
				set_tcon_cmd(put_cmd, cnt);
			}
			else {
				DSI_LOG("read panel command\n");
				get_tcon_cmd(put_cmd[0], store[1]);
			}

			if(cur != NULL) {
				if (*(tmp+str_len) == 'w')
					flag = true;
				else if (*(tmp+str_len) == 'r')
					flag = false;
			}
			cnt = 0;
		}

		memset(tmp, 0, len*sizeof(char));

		if(cur != NULL)
			strcpy(tmp, cur);
	}

	if(cmd_cnt == 0) {
		ret = -1;
		goto error;
	}

	ret = len;

error:
	DSI_LOG("len = %d ---\n", ret);
	kfree(messages);
	kfree(tmp);
	kfree(store);
	kfree(put_cmd);
	return ret;
}

static ssize_t panel_reg_show(struct file *file, char __user *buf,
                 size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	buff = kzalloc(SZ_4K, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	len += sprintf(buff, "%s\n", g_reg_buffer);
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static struct file_operations panel_reg_rw_ops = {
	.write = panel_reg_rw,
	.read = panel_reg_show,
};

// panel_vendor_id_ops() - show panel vendor id
static ssize_t panel_vendor_id_show(struct file *file, char __user *buf,
				 size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	buff = kzalloc(SZ_4K, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;
	len += sprintf(buff, "%s\n", g_display->panel->panel_vendor_id);
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static struct file_operations panel_vendor_id_ops = {
	.read = panel_vendor_id_show,
};

// panel uid read
static ssize_t lcd_unique_id_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	buff = kzalloc(256, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	len += sprintf(buff, "%s\n", g_lcd_unique_id);
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static struct file_operations lcd_unique_id_ops = {
	.read = lcd_unique_id_read,
};

// panel fps read
static ssize_t panel_fps_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	//DSI_LOG("refreshrate  %d\n",g_display->panel->cur_mode->timing.refresh_rate);
	buff = kzalloc(256, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	len += sprintf(buff, "%d\n", g_display->panel->cur_mode->timing.refresh_rate);
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static struct file_operations panel_fps_ops = {
	.read = panel_fps_read,
};

// hbm_mode_ops() - set HBM on/off & read HBM status
static ssize_t hbm_mode_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	memset(messages, 0, sizeof(messages));

	if (len > 256)
		len = 256;
	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if (!display_panel_valid())
		return -EINVAL;

	if (g_display->panel->panel_is_on &&
			(g_display->panel->allow_panel_fod_hbm == 0)) {
		if (strncmp(messages, "0", 1) == 0) {
			display_set_hbm_mode(0);
		} else if (strncmp(messages, "1", 1) == 0) {
			display_set_hbm_mode(1);
		} else {
			DSI_LOG("don't match any hbm mode.\n");
		}
	} else {
		DSI_LOG("unable to set in display off or fod hbm\n");
		g_display->panel->panel_hbm_mode = 0;
	}

	return len;
}

static ssize_t hbm_mode_read(struct file *file, char __user *buf,
							 size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	if (*ppos)
		return 0;

	buff = kzalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	//DSI_LOG("hbm mode is %d\n", g_display->panel->panel_hbm_mode);

	len += sprintf(buff, "%d\n", g_display->panel->panel_hbm_mode);
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static struct file_operations hbm_mode_ops = {
	.write = hbm_mode_write,
	.read  = hbm_mode_read,
};

// send dimming command to panel
static int dsi_anakin_set_dimming(struct dsi_panel *panel, bool enable)
{
	int rc = 0;

	if (!panel) {
		DSI_LOG("invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);
	if (!panel->panel_initialized)
		goto exit;

	if (enable)
		rc = dsi_anakin_tx_cmd_set(panel, DSI_CMD_SET_DIMMING_SPEED_1);
	else
		rc = dsi_anakin_tx_cmd_set(panel, DSI_CMD_SET_DIMMING_SPEED_20);

exit:
	mutex_unlock(&panel->panel_lock);
	return rc;
}

static ssize_t dimming_speed_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	int rc = 0;
	memset(messages, 0, sizeof(messages));

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if (!display_panel_valid())
		return -EFAULT;

	DSI_LOG("dimming speed write +++ val:%s \n",messages);

	if (g_display->panel->panel_is_on
		&& !(g_display->panel->allow_panel_fod_hbm || g_display->panel->allow_fod_hbm_process)) {
		if (strncmp(messages, "1", 1) == 0) {
			rc = dsi_anakin_set_dimming(g_display->panel, true);
		} else if (strncmp(messages, "20", 20) == 0) {
			rc = dsi_anakin_set_dimming(g_display->panel, false);
		} else {
			DSI_LOG("doesn't match any dimming speed .\n");
		}
	} else {
		DSI_LOG("unable to set in display off\n");
	}

	if(rc) {
		DSI_LOG("panel set dimming speed failed\n");
	}

	return len;
}

static struct file_operations dimming_speed_ops = {
	.write = dimming_speed_write,
};

static ssize_t lcd_brightness_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	u32 DC_mode = 0;

	memset(messages, 0, sizeof(messages));

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if (!display_panel_valid())
		return -EFAULT;


	sscanf(messages, "%u", &DC_mode);

	//DSI_LOG("dc lcd brightess write (%d) +++ \n",DC_mode);
	if(g_display->panel->dc_mode == 0 && DC_mode==1) {
		g_display->panel->dc_bl_delay = true;
	}
	g_display->panel->dc_mode = DC_mode;
	if(DC_mode == 0) {
		dc_fixed_bl = false;
	}
	return len;
}

static struct file_operations lcd_brightness_ops = {
	.write = lcd_brightness_write,
};

// global_hbm_mode_ops() - set Fod HBM on/off & read Fod HBM status
static ssize_t global_hbm_mode_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	memset(messages, 0, sizeof(messages));

	if (len > 256)
		len = 256;
	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if (!display_panel_valid())
		return -EINVAL;

	if (g_display->panel->panel_is_on) {
		if (strncmp(messages, "0", 1) == 0) {
			display_set_fod_hbm_mode(0);
		} else if (strncmp(messages, "1", 1) == 0) {
			display_set_fod_hbm_mode(1);
		} else {
			DSI_LOG("don't match any hbm mode.\n");
		}
	} else {
		DSI_LOG("unable to set in display off\n");
		g_display->panel->panel_fod_hbm_mode = 0;
	}

	return len;
}

static ssize_t global_hbm_mode_read(struct file *file, char __user *buf,
							 size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	if (*ppos)
		return 0;

	buff = kzalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	DSI_LOG("fod hbm mode is %d\n", g_display->panel->panel_fod_hbm_mode);

	len += sprintf(buff, "%d\n", g_display->panel->panel_fod_hbm_mode);
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static struct file_operations global_hbm_mode_ops = {
	.write = global_hbm_mode_write,
	.read  = global_hbm_mode_read,
};

// to support DSI_CTRL_CMD_READ if MIPI_DSI_MSG_CMD_READ is enabled
u32 dsi_anakin_support_cmd_read_flags(u32 flags)
{
	u32 ret_flags = 0;

	if (flags & MIPI_DSI_MSG_CMD_READ) {
		ret_flags |= (DSI_CTRL_CMD_LAST_COMMAND | DSI_CTRL_CMD_FETCH_MEMORY | DSI_CTRL_CMD_READ);
		//DSI_LOG("DSI_CTRL_CMD is 0x%x\n", ret_flags);
	}

	return ret_flags;
}

// to show & clear frame commit count
void dsi_anakin_clear_commit_cnt(void)
{
	display_commit_cnt = COMMIT_FRAMES_COUNT;
}

void dsi_anakin_frame_commit_cnt(struct drm_crtc *crtc)
{
	static int dc_delay_frames = 0;
	static int aod_delay_frames = 0;
	if (display_commit_cnt > 0 && !strcmp(crtc->name, "crtc-0")) {
		DSI_LOG("fbc%d\n", display_commit_cnt);
		display_commit_cnt--;
	}

	if(g_display->panel->aod_delay && g_display->panel->aod_state) {
		if(g_display->panel->dc_bl_delay) {
			dc_delay_frames = 0;
			g_display->panel->dc_bl_delay = false;
		}
		aod_delay_frames++;
		// for receive msg case only
		if(aod_delay_frames >= 5 && !(g_display->panel->allow_panel_fod_hbm || g_display->panel->allow_fod_hbm_process)) {
			dsi_anakin_need_aod_reset(g_display->panel);
			aod_delay_frames = 0;
		}
	}

	if(g_display->panel->dc_mode && g_display->panel->dc_bl_delay && !g_display->panel->aod_state) {
		dc_delay_frames++;
		if (dc_delay_frames == 4) {
			DSI_LOG("fixed bl for dc mode \n");
			dc_delay_frames = 0;
			dc_fixed_bl = true;
			g_display->panel->dc_bl_delay = false;
			if (g_display->panel->panel_last_backlight < 224) {
				dsi_panel_set_backlight(g_display->panel, 224);
			}
			else {
				dsi_anakin_restore_backlight();
			}
		}
	}

	//if(display_commit_cnt == 2 && !g_display->panel->aod_state) {
	//	DSI_LOG("restore Dimming Smooth");
	//	dsi_anakin_set_dimming_smooth(g_display->panel);
	//}
}

// to initial asus display parameters
void dsi_anakin_display_init(struct dsi_display *display)
{
	g_display = display;
	dsi_anakin_parse_panel_vendor_id(g_display->panel);

	g_display->panel->panel_hbm_mode = 0;
	g_display->panel->panel_fod_hbm_mode = 0;
	g_display->panel->allow_panel_fod_hbm = 0;
	g_display->panel->allow_fod_hbm_process = false;
	g_display->panel->panel_is_on = false;
	g_display->panel->panel_last_backlight = 0;
	g_display->panel->panel_aod_last_bl = 0;
	g_display->panel->panel_bl_count = 0;
	g_display->panel->aod_state = false;
	g_display->panel->mode_change_bl_blocked = false;
	g_display->panel->dc_bl_delay = false;
	g_display->panel->fod_in_doze = false;
	g_display->panel->err_fg_irq_is_on = false;
	g_display->panel->is_gamma_get = false;
	g_display->panel->is_gamma_change = false;
	g_display->panel->is_first_gamma_set = false;
	g_display->panel->aod_delay = false;
	atomic_set(&g_display->panel->is_spot_ready, 0);

	proc_create(PANEL_REGISTER_RW, 0640, NULL, &panel_reg_rw_ops);
	proc_create(PANEL_VENDOR_ID, 0640, NULL, &panel_vendor_id_ops);
	proc_create(PANEL_FPS, 0666, NULL, &panel_fps_ops);
	proc_create(LCD_UNIQUE_ID, 0444, NULL, &lcd_unique_id_ops);
	proc_create(HBM_MODE, 0666, NULL, &hbm_mode_ops);
	proc_create(GLOBAL_HBM_MODE, 0666, NULL, &global_hbm_mode_ops);
	proc_create(DIMMING_SPEED, 0666, NULL, &dimming_speed_ops);
	proc_create(LCD_BACKLIGNTNESS, 0666, NULL, &lcd_brightness_ops);
}

// to parse panel_vendor_id from panel dtsi
void dsi_anakin_parse_panel_vendor_id(struct dsi_panel *panel)
{
	struct dsi_parser_utils *utils = &panel->utils;

	panel->panel_vendor_id = utils->get_property(utils->data,
			"qcom,mdss-dsi-panel-vendor-id", NULL);
	DSI_LOG("panel vendor id = %s", panel->panel_vendor_id);
}

// to store panel status & reset display parameter
void dsi_anakin_set_panel_is_on(bool on)
{
	g_display->panel->panel_is_on = on;

	if (on == false) {
		g_display->panel->panel_hbm_mode = 0;
		g_display->panel->panel_fod_hbm_mode = 0;
		g_display->panel->allow_panel_fod_hbm = 0;
		g_display->panel->allow_fod_hbm_process = false;
		g_display->panel->aod_state = false;
		g_display->panel->panel_aod_last_bl = 0;
		g_display->panel->panel_bl_count = 0;
		g_display->panel->fod_in_doze = false;
		g_display->panel->mode_change_bl_blocked = false;
		g_display->panel->err_fg_irq_is_on = false;
		has_fod_masker = false;
		old_has_fod_masker = false;
		has_fod_spot = false;
		old_has_fod_spot = false;
		anakin_drm_notify(ASUS_NOTIFY_SPOT_READY, 0);
		anakin_drm_notify(ASUS_NOTIFY_GHBM_ON_READY, 0);
		anakin_drm_notify(ASUS_NOTIFY_GHBM_ON_REQ, 0);
		g_display->panel->is_gamma_change = false;
		g_display->panel->aod_delay = false;
		atomic_set(&g_display->panel->is_spot_ready, 0);
	}
}

static void dsi_anakin_aod_backlight(struct dsi_panel *panel)
{
	int rc = 0;
	if (!panel) {
		DSI_LOG("invalid params\n");
		return;
	}

	// skip if fod hbm is processing
	if (panel->aod_state && !(panel->allow_panel_fod_hbm || panel->allow_fod_hbm_process)) {
		// for bl=61 & bl=1
		if (panel->panel_last_backlight == 61 || panel->panel_last_backlight == 9) {
			if (panel->panel_last_backlight == 61 && panel->panel_last_backlight != panel->panel_aod_last_bl) {
				DSI_LOG("set AOD HIGH command\n");
				rc = dsi_anakin_tx_cmd_set(panel, DSI_CMD_SET_AOD_HIGH);
				panel->panel_aod_last_bl = panel->panel_last_backlight;
				panel->aod_mode = 1;
			} else if (panel->panel_last_backlight == 9 && panel->panel_last_backlight != panel->panel_aod_last_bl) {
				DSI_LOG("set AOD LOW command\n");
				rc = dsi_anakin_tx_cmd_set(panel, DSI_CMD_SET_AOD_LOW);
				panel->panel_aod_last_bl = panel->panel_last_backlight;
				panel->aod_mode = 0;
			}
		} else { // to prevent display off
			DSI_LOG("set AOD Other command\n");
			rc = dsi_anakin_tx_cmd_set(panel, DSI_CMD_SET_AOD_OTHER);
		}

		if (rc)
			DSI_LOG("unable to set AOD command\n");
	}
}

// to record & restore user's last backlight
void dsi_anakin_record_backlight(u32 bl_lvl)
{
	if (bl_lvl == 0)
		return;

	g_display->panel->panel_last_backlight = bl_lvl;
	//#define SDE_MODE_DPMS_LP1	1      sde_drm.h
	//#define SDE_MODE_DPMS_LP2	2

	if ((g_display->panel->power_mode == 1) ||
		(g_display->panel->power_mode == 2)) {
		dsi_anakin_aod_backlight(g_display->panel);
	}
}

void dsi_anakin_restore_backlight(void)
{
	int rc = 0;

	DSI_LOG("restore bl=%d\n", g_display->panel->panel_last_backlight);
	rc = dsi_panel_set_backlight(g_display->panel, g_display->panel->panel_last_backlight);
	if (rc)
		DSI_LOG("unable to set backlight\n");
}

// called from sde_crtc_atomic_set_property, ready for FOD ON/OFF
void anakin_crtc_fod_masker_spot(struct drm_crtc *crtc, int idx, uint64_t val)
{
	if (!strcmp(crtc->name, "crtc-0")) {
		switch (idx) {
		case CRTC_PROP_FOD_MASKER:
			has_fod_masker = val;
			if (old_has_fod_masker == false && has_fod_masker == true) {
				g_display->panel->allow_panel_fod_hbm = 1;
				g_display->panel->allow_fod_hbm_process = true;
				DSI_LOG("FOD:OFF->ON");
			} else if (old_has_fod_masker == true && has_fod_masker == false) {
				g_display->panel->allow_panel_fod_hbm = 0;
				g_display->panel->allow_fod_hbm_process = true;
				DSI_LOG("FOD:ON->OFF");
			}
			old_has_fod_masker = has_fod_masker;
			break;
		case CRTC_PROP_FOD_SPOT:
			has_fod_spot = val;
			if (old_has_fod_spot == false && has_fod_spot == true) {
				DSI_LOG("FOD SPOT:OFF->ON");
			} else if (old_has_fod_spot == true && has_fod_spot == false) {
				DSI_LOG("FOD SPOT:ON->OFF");
			}
			old_has_fod_spot = has_fod_spot;
			break;
		default:
			break;
		}
	}
}

// called from sde_crtc_commit_kickoff, to enable panel global HBM
void anakin_crtc_display_commit(struct drm_encoder *encoder, struct drm_crtc *crtc)
{

	if (g_display->panel->allow_fod_hbm_process && !strcmp(crtc->name, "crtc-0")) {
		DSI_LOG("FOD HBM setting +++\n");
		sde_encoder_wait_for_event(encoder, MSM_ENC_VBLANK);
		display_set_fod_hbm();

		if (g_display->panel->allow_panel_fod_hbm == 1) {
			// need delay time, waiting for fine tune
			anakin_drm_notify(ASUS_NOTIFY_GHBM_ON_READY, 1);
			g_display->panel->panel_fod_hbm_mode = 1;
			DSI_LOG("panel_fod_hbm_mode set to 1");
		} else if (g_display->panel->allow_panel_fod_hbm == 0) {
			// need delay time, waiting for fine tune
			anakin_drm_notify(ASUS_NOTIFY_GHBM_ON_READY, 0);
			g_display->panel->panel_fod_hbm_mode = 0;
			DSI_LOG("panel_fod_hbm_mode set to 0");
		}
		DSI_LOG("FOD HBM setting ---\n");
	}
}

// add for spot ready after hbm ramping to the highest
// type 1:from first spot; 2:from crtc
static void dsi_anakin_set_notify_spot_ready(int type)
{
	uint32_t delta_us = 0;
	uint32_t frame_us = 0;
	uint32_t delay = 0;
	int frame_delay = 2;
	ktime_t ktime1;
	bool is_ready = false;

	if (!display_panel_valid())
		return;

	// for spot layer is ready
	if (!atomic_read(&g_display->panel->is_spot_ready))
		return;

	ktime1 = ktime_get(); //current frame time
	delta_us = (u32)ktime_to_us(ktime1) - (u32)ktime_to_us(g_display->panel->ktime0);
	frame_us = 1000000 / g_display->panel->cur_mode->timing.refresh_rate;
	is_ready = (delta_us >= frame_us) ? true : false;

	// fod_spot_ui_ready represent to SYS NODE
	if (is_ready && !fod_spot_ui_ready) {
		// spec is 1 frame, but fine tune is 2 frames
		if (delta_us < (frame_delay*frame_us) && type == 2) {
			delay = ((frame_delay*frame_us) - delta_us);
			usleep_range(delay, delay);
		}
		DSI_LOG("tooks %d (us) ---", (delta_us+delay));
		anakin_drm_notify(ASUS_NOTIFY_SPOT_READY, 1);
	}
}

// called from sde_crtc.c
void anakin_set_notify_spot_ready(struct drm_crtc *crtc) {

	if (strcmp(crtc->name, "crtc-0"))
		return;

	dsi_anakin_set_notify_spot_ready(2);
}

// called from _msm_drm_commit_work_cb, to notify spot ready
// type 0: commit_for_fod_spot
//      1: report_fod_spot_disappear
bool anakin_atomic_get_spot_status(int type)
{
	if (type == 0) {
		if (has_fod_spot && !atomic_read(&g_display->panel->is_spot_ready)) {
			DSI_LOG("commit FOD spot to panel +++ \n");
			return true;
		}
	} else if (type == 1) {
		if (!has_fod_spot && atomic_read(&g_display->panel->is_spot_ready))
			return true;
	}

	return false;
}

void anakin_atomic_set_spot_status(int type)
{
	int rc = 0;

	if (type == 0) {
		int period_ms = 1000000 / g_display->panel->cur_mode->timing.refresh_rate;
		DSI_LOG("commit FOD spot to panel (%d) --- \n", period_ms);

		// display on after ASUS_NOTIFY_SPOT_READY
		if (g_display->panel->fod_in_doze) {
			rc = dsi_anakin_tx_cmd_set(g_display->panel, DSI_CMD_SET_POST_FOD_HBM_ON);
			if (rc) {
				DSI_LOG("[%s] failed to send DSI_CMD_SET_POST_FOD_HBM_ON cmd, rc=%d\n",
					   g_display->panel->name, rc);
			} else {
				g_display->panel->fod_in_doze = false;
				atomic_set(&g_display->panel->is_spot_ready, 1);
				//anakin_drm_notify(ASUS_NOTIFY_SPOT_READY, 1);
			}
		} else {
			atomic_set(&g_display->panel->is_spot_ready, 1);
			//anakin_drm_notify(ASUS_NOTIFY_SPOT_READY, 1);
		}
	} else if (type == 1) {
		anakin_drm_notify(ASUS_NOTIFY_SPOT_READY, 0);
		atomic_set(&g_display->panel->is_spot_ready, 0);
		DSI_LOG("removed fod spot \n");
	}

	dsi_anakin_set_notify_spot_ready(1);
}

// add for charger mode to skip iris fw
bool anakin_get_charger_mode(void)
{
	return g_Charger_mode;
}

// add for set min backlight to 2
u32 dsi_anakin_backlightupdate(u32 bl_lvl)
{
	if (g_display->panel->dc_mode && dc_fixed_bl && bl_lvl <= 224 && bl_lvl != 0 && !g_display->panel->aod_state){
		DSI_LOG("dc_mode on fixed bl = 224");
		return 224;
	}
	if (bl_lvl == 1) {
		return 2;
	}
	else if (bl_lvl == 9) {
		return 10;
	}
	else {
		return bl_lvl;
	}
}

void anakin_set_err_fg_irq_state(bool state) {
	if(g_display->panel->panel_is_on || !state) {
		g_display->panel->err_fg_irq_is_on = state;
		if(state) {
			DSI_LOG("set err fg irq as on");
			g_display->panel->esd_fail = true;
		}
	}
}

bool anakin_get_err_fg_irq_state(void) {
	if(g_display->panel->panel_is_on) {
		return g_display->panel->err_fg_irq_is_on;
	}
	else {
		return false;
	}
}

// add for gamma changing solution
static void anakin_gamma_write_cmd(struct dsi_display *display, char *cmd, short len) {
	ssize_t rc;
	struct dsi_cmd_desc cmds;
	struct mipi_dsi_msg tcon_cmd   = {0, 0x39, 0, 0, 0, len, cmd, 0, NULL};
	const struct mipi_dsi_host_ops *ops = display->panel->host->ops;

	if(len > 2)
		tcon_cmd.type = 0x39;

	// cmds assigned
	cmds.msg = tcon_cmd;
	cmds.last_command = 1;
	cmds.post_wait_ms = 1;
	if (cmds.last_command) {
		cmds.msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;
	}

	rc = ops->transfer(display->panel->host, &cmds.msg);
	if (rc < 0) {
		DSI_LOG("write Gamma cmd (%x) failed rc=%d\n", cmd[0], rc);
	}
}

static int anakin_gamma_read_cmd(struct dsi_display *display, char cmd, int rlen, int position)
{
	int i = 0, rc = 0, start = 0;
	u8 *tx_buf, *return_buf, *status_buf;

	struct dsi_cmd_desc cmds;
	struct mipi_dsi_msg tcon_cmd = {0, 0x06, 0, 0, 0, sizeof(cmd), NULL, rlen, NULL};
	const struct mipi_dsi_host_ops *ops = display->panel->host->ops;

	if(!(cmd == 0xB7 || cmd == 0xB9)) {
		DSI_LOG("not Gamma cmd (%x)\n", cmd);
		return -1;
	}

	// buffer assigned
	tx_buf = &cmd;

	// allocate buffer
	return_buf = kcalloc(rlen, sizeof(unsigned char), GFP_KERNEL);
	status_buf = kzalloc(SZ_4K, GFP_KERNEL);
	memset(status_buf, 0x0, SZ_4K);

	// cmds assigned
	cmds.msg = tcon_cmd;
	cmds.last_command = 1;
	cmds.post_wait_ms = 0;
	if (cmds.last_command) {
		cmds.msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;
	}
	cmds.msg.flags |= MIPI_DSI_MSG_CMD_READ;

	cmds.msg.tx_buf = tx_buf;
	cmds.msg.rx_buf = status_buf;
	cmds.msg.rx_len = rlen;

	rc = ops->transfer(display->panel->host, &cmds.msg);
	if (rc <= 0) {
		DSI_LOG("read Gamma cmd (%x) failed rc=%d\n", cmd, rc);
		goto error;
	} else {
		memcpy(return_buf + start, status_buf, rlen);
		start += rlen;
		for(i=0; i<rlen; i++) {
			if (cmd == 0xB7)
				g_gamma_B7_cmd[i+position] = return_buf[i];
			else if (cmd == 0xB9)
				g_gamma_B9_cmd[i+position] = return_buf[i];
		}
	}

error:
	kfree(return_buf);
	kfree(status_buf);
	return rc;
}

static void anakin_gamma_calculate(void) {
	int i = 0, rlen = 32;
	short R_tmp = 0, G_tmp = 0, B_tmp = 0;
	char value_B9[32] = { 1, 13, 13, 0, 7, 13, 12, 0, 4, 12, 12, 0, 2, 10, 6, 0,
				   0, 7, 3, 0, 0, 5, 0, 0, 0, 5, 0, 0, 0, 5, 0, 0};
	char value_B7[32] = { 2, 9, 12, 0, 0, 9, 12, 0, 0, 7, 6, 0, 0, 3, 1, 0,
				   0, 1, 1, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 4, 0, 0};

	DSI_LOG("Gamma calculate +++\n");
	// 0xB7
	for (i=0; i<rlen; i+=4) {
		// RGB
		R_tmp = ((g_gamma_B7_cmd[i] & 0x3F) << 0x4) | ((g_gamma_B7_cmd[i+1] & 0xF0) >> 0x4);
		G_tmp = ((g_gamma_B7_cmd[i+1] & 0x0F) << 0x6) | ((g_gamma_B7_cmd[i+2] & 0xFC) >> 0x2);
		B_tmp = ((g_gamma_B7_cmd[i+2] & 0x03) << 0x8) | (g_gamma_B7_cmd[i+3] & 0xFF);
		//DSI_LOG("before: R_tmp (%x), G_tmp (%x), B_tmp (%x)\n", R_tmp, G_tmp, B_tmp);

		if (i == 0) // R[0] is plus value
			R_tmp = R_tmp + value_B7[i];
		else
			R_tmp = R_tmp - value_B7[i];
		G_tmp = G_tmp - value_B7[i+1];
		B_tmp = B_tmp - value_B7[i+2];
		//DSI_LOG("after: R_tmp (%x), G_tmp (%x), B_tmp (%x)\n", R_tmp, G_tmp, B_tmp);

		// restore
		g_gamma_B7_cmd[i] = (R_tmp >> 0x4) | (g_gamma_B7_cmd[0] & 0xC0);
		g_gamma_B7_cmd[i+1] = ((R_tmp & 0x0F) << 0x4) | (G_tmp >> 0x6);
		g_gamma_B7_cmd[i+2] = ((G_tmp & 0x3F) << 0x2) | (B_tmp >> 0x8);
		g_gamma_B7_cmd[i+3] = B_tmp & 0xFF;
	}

	// 0xB9
	for (i=0; i<rlen; i+=4) {
		// RGB
		R_tmp = ((g_gamma_B9_cmd[i] & 0x3F) << 0x4) | ((g_gamma_B9_cmd[i+1] & 0xF0) >> 0x4);
		G_tmp = ((g_gamma_B9_cmd[i+1] & 0x0F) << 0x6) | ((g_gamma_B9_cmd[i+2] & 0xFC) >> 0x2);
		B_tmp = ((g_gamma_B9_cmd[i+2] & 0x03) << 0x8) | (g_gamma_B9_cmd[i+3] & 0xFF);
		//DSI_LOG("before: R_tmp (%x), G_tmp (%x), B_tmp (%x)\n", R_tmp, G_tmp, B_tmp);

		R_tmp = R_tmp - value_B9[i];
		G_tmp = G_tmp - value_B9[i+1];
		B_tmp = B_tmp - value_B9[i+2];
		//DSI_LOG("after: R_tmp (%x), G_tmp (%x), B_tmp (%x)\n", R_tmp, G_tmp, B_tmp);

		// restore
		g_gamma_B9_cmd[i] = (R_tmp >> 0x4) | (g_gamma_B9_cmd[0] & 0xC0);
		g_gamma_B9_cmd[i+1] = ((R_tmp & 0x0F) << 0x4) | (G_tmp >> 0x6);
		g_gamma_B9_cmd[i+2] = ((G_tmp & 0x3F) << 0x2) | (B_tmp >> 0x8);
		g_gamma_B9_cmd[i+3] = B_tmp & 0xFF;
	}
	DSI_LOG("Gamma calculate --\n");
}

void anakin_change_gamma_setting(struct dsi_display *display) {
	static char level2_enable_cmd[3] = {0xF0, 0x5A, 0x5A};
	static char level2_disable_cmd[3] = {0xF0, 0xA5, 0xA5};
	static char gamma_update[2] = {0xF7, 0x0F};
	static char shift_B7_offset[2] = {0xB0, 0xAF}; // 176th
	static char shift_B9_offset[2] = {0xB0, 0x2D}; // 46th
	static char gamma_B7_cmd = 0xB7;
	static char gamma_B9_cmd = 0xB9;

	char gamma_B7_wb_cmd[33] = {0};
	char gamma_B9_wb_cmd[33] = {0};
	int rlen = 32, i = 0;

	if (!display || !display->panel) {
		DSI_LOG("Invalid params\n");
		return;
	}

	if (display->panel->is_gamma_change)
		return;

	DSI_LOG("change Gamma register +++\n");

	mutex_lock(&display->display_lock);
	dsi_panel_acquire_panel_lock(display->panel);

	// level2 key access enable
	anakin_gamma_write_cmd(display, level2_enable_cmd, sizeof(level2_enable_cmd));

	// write 0xB7, 32 parameter
	// offset to 176 parameter
	anakin_gamma_write_cmd(display, shift_B7_offset, sizeof(shift_B7_offset));
	gamma_B7_wb_cmd[0] = gamma_B7_cmd;
	for(i=1; i<rlen+1; i++) {
		gamma_B7_wb_cmd[i] = g_gamma_B7_cmd[i-1];
	}
	anakin_gamma_write_cmd(display, gamma_B7_wb_cmd, sizeof(gamma_B7_wb_cmd));

	// write 0xB9, 32 parameter
	// offset to 46 parameter
	anakin_gamma_write_cmd(display, shift_B9_offset, sizeof(shift_B9_offset));
	gamma_B9_wb_cmd[0] = gamma_B9_cmd;
	for(i=1; i<rlen+1; i++) {
		gamma_B9_wb_cmd[i] = g_gamma_B9_cmd[i-1];
	}
	anakin_gamma_write_cmd(display, gamma_B9_wb_cmd, sizeof(gamma_B9_wb_cmd));

	// update
	anakin_gamma_write_cmd(display, gamma_update, sizeof(gamma_update));

	// level2 key access disable
	anakin_gamma_write_cmd(display, level2_disable_cmd, sizeof(level2_disable_cmd));

	display->panel->is_gamma_change = true;
	dsi_panel_release_panel_lock(display->panel);
	mutex_unlock(&display->display_lock);
	DSI_LOG("change Gamma register ---\n");
}

void anakin_get_gamma_setting(struct dsi_display *display) {
	static char level2_enable_cmd[3] = {0xF0, 0x5A, 0x5A};
	static char level2_disable_cmd[3] = {0xF0, 0xA5, 0xA5};
	static char shift_B7_offset[2] = {0xB0, 0xAF}; // 175th
	static char shift_B9_offset[2] = {0xB0, 0x2D}; // 45th

	static char gamma_B7_cmd = 0xB7;
	static char gamma_B9_cmd = 0xB9;
	int rlen[4] = {10, 10, 10, 2};
	int i = 0;
	int ret = 0;
	int position_B7 = 0;
	int position_B9 = 0;

	if (!display || !display->panel) {
		DSI_LOG("Invalid params\n");
		return;
	}

	if (display->panel->is_gamma_get)
		return;

	DSI_LOG("get Gamma register +++\n");
	mutex_lock(&display->display_lock);
	dsi_panel_acquire_panel_lock(display->panel);

	// level2 key access enable
	anakin_gamma_write_cmd(display, level2_enable_cmd, sizeof(level2_enable_cmd));

	// read 0xB9, 32 parameter
	// only can read 10 bytes after offset
	DSI_LOG("read Gamma 0xB9 +++\n");
	for (i=0; i<4; i++) {
		anakin_gamma_write_cmd(display, shift_B9_offset, sizeof(shift_B9_offset));
		ret = anakin_gamma_read_cmd(display, gamma_B9_cmd, rlen[i], position_B9);
		if (ret <= 0)
			return;
		shift_B9_offset[1] = shift_B9_offset[1] + 10;
		position_B9 = position_B9 + 10;
	}
	DSI_LOG("read Gamma 0xB9 ---\n");

	// read 0xB7, 32 parameter
	// only can read 10 bytes after offset
	DSI_LOG("read Gamma 0xB7 +++\n");
	for (i=0; i<4; i++) {
		anakin_gamma_write_cmd(display, shift_B7_offset, sizeof(shift_B7_offset));
		anakin_gamma_read_cmd(display, gamma_B7_cmd, rlen[i], position_B7);
		if (ret <= 0)
			return;
		shift_B7_offset[1] = shift_B7_offset[1] + 10;
		position_B7 = position_B7 + 10;
	}
	DSI_LOG("read Gamma 0xB7 ---\n");

	// level2 key access disable
	anakin_gamma_write_cmd(display, level2_disable_cmd, sizeof(level2_disable_cmd));

	// calculating gamma
	anakin_gamma_calculate();

	display->panel->is_gamma_get = true;
	dsi_panel_release_panel_lock(display->panel);
	mutex_unlock(&display->display_lock);
	DSI_LOG("get Gamma register ---\n");
}

bool anakin_need_change_gamma(void)
{
	// ascii code
	// only change in panel id is before 0x41
	if (g_lcd_unique_id[6] < 0x34 || g_lcd_unique_id[7] <= 0x30)
		return true;
	else
		return false;
}

void dsi_anakin_need_aod_reset(struct dsi_panel *panel)
{
	int rc = 0;

	mutex_lock(&panel->panel_lock);
	if (!panel->panel_initialized)
		goto exit;

	if(panel->panel_last_backlight == 9 || !panel->aod_mode) {
		DSI_LOG("notify set AOD LOW command\n");
		rc = dsi_anakin_tx_cmd_set(panel, DSI_CMD_SET_AOD_LOW);
		panel->aod_mode = 0;
	} else if(panel->panel_last_backlight == 61 || panel->aod_mode) {
		DSI_LOG("notify set AOD HIGH command\n");
		rc = dsi_anakin_tx_cmd_set(panel, DSI_CMD_SET_AOD_HIGH);
		panel->aod_mode = 1;
	}

	if(rc) {
		DSI_LOG("failed to set notify AOD command");
	}

	panel->panel_aod_last_bl = panel->panel_last_backlight;
	panel->aod_delay = false;

exit:
	mutex_unlock(&panel->panel_lock);
	return;
}
#endif