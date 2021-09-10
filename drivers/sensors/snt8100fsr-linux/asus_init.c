#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#define CONFIG_I2C_MODULE
#include <linux/i2c.h>
#include "device.h"
#include "memory.h"
#include "serial_bus.h"
#include "main.h"
#include "event.h"
#include "hardware.h"
#include "sysfs.h"
#include "utils.h"
#include "config.h"
#include "debug.h"
#include "sonacomm.h"
#include "workqueue.h"
#include "asus_init.h"

int SNT_SUSPEND_FLAG = 1;
int grip_write_fail_count = 0;
int grip_write_fail_reset_trigger = 20;

//Reset Function
void check_i2c_error(void){
	grip_write_fail_count++;
	if(grip_write_fail_count >= grip_write_fail_reset_trigger){
		ASUSEvtlog("[Grip] read/write fail, count=%d, call reset function\n", grip_write_fail_count);
		queue_delayed_work(asus_wq, &rst_gpio_wk, msecs_to_jiffies(1000));
		/* reset grip_write_fail_count in Reset_func */
		grip_write_fail_count = 0;
	}
}

void snt_set_pinctrl(struct device *dev, char *str)
{
	int ret;
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *set_state;
	
	key_pinctrl = devm_pinctrl_get(dev);
	set_state = pinctrl_lookup_state(key_pinctrl, str);
	ret = pinctrl_select_state(key_pinctrl, set_state);
	if(ret < 0){
		PRINT_ERR("%s: pinctrl_select_state ERROR(%d).\n", __FUNCTION__, ret);
	} else {
		PRINT_INFO("pinctrl_lookup_state: set success");
	}
}

static struct regulator *reg_2v8;
static struct regulator *reg_1v8;
static struct regulator *reg_1v2;
//static struct regulator *reg_1v2Base;

static int grip_2v8_regulator_init(void)
{
	int ret = 0;
	reg_2v8 = regulator_get(snt8100fsr_g->dev,"vcc_grip_2v8");
		if (IS_ERR_OR_NULL(reg_2v8)) {
		ret = PTR_ERR(reg_2v8);
		PRINT_ERR("Failed to get regulator vcc_gripsensor %d\n", ret);
		return ret;
	}
	ret = regulator_set_voltage(reg_2v8, 2800000, 2800000);
	if (ret) {
		PRINT_ERR("Failed to set voltage for vcc_gripsensor reg_2v8 %d\n", ret);
		return -1;
	}

	PRINT_INFO("vcc_gripsensor regulator setting init");
	return ret;
}

static int grip_2v8_regulator_enable(void)
{
	int ret = 0, idx = 0;

	if(IS_ERR_OR_NULL(reg_2v8)){
		ret = PTR_ERR(reg_2v8);
		PRINT_ERR("Failed to get regulator vcc_gripsensor %d\n", ret);
		return ret;
	}

	ret = regulator_set_load(reg_2v8, 10000);
	if(ret < 0){
		PRINT_ERR("Failed to set load for vcc_gripsensor reg_2v8 %d\n", ret);
		return ret;
	}

	ret = regulator_enable(reg_2v8);
	if(ret){
		PRINT_ERR("Failed to enable vcc_gripsensor reg_2v8 %d\n", ret);
		return -1;
	}

	for(idx=0; idx<10; idx++){
		if(regulator_is_enabled(reg_2v8) > 0){
			PRINT_DEBUG("vcc_gripsensor regulator is enabled(idx=%d)", idx);
			break;
		}
	}
	if(idx >= 10){
		PRINT_ERR("vcc_gripsensor regulator is enabled fail(retry count >= %d)", idx);
		return -1;
	}

	PRINT_INFO("Update vcc_psensor to NPM_mode");
	return ret;
}

static int grip_2v8_regulator_disable(void)
{
	int ret = 0;

	if(IS_ERR_OR_NULL(reg_2v8)){
		ret = PTR_ERR(reg_2v8);
		PRINT_ERR("Failed to get regulator vcc_gripsensor %d\n", ret);
		return ret;
	}

	ret = regulator_set_load(reg_2v8, 0);
	if(ret < 0){
		PRINT_ERR("Failed to set load for vcc_gripsensor reg_2v8 %d\n", ret);
		return ret;
	}

	ret = regulator_disable(reg_2v8);
	if(ret){
		PRINT_ERR("Failed to enable vincentr reg_2v8 %d\n", ret);
		return -1;
	}

	PRINT_INFO("Update vcc_gripsensor to LPM_mode");
	return ret;
}


static int grip_1v2_regulator_init(void)
{
	int ret = 0;
	reg_1v2 = regulator_get(snt8100fsr_g->dev,"vcc_grip_1v2");
	if (IS_ERR_OR_NULL(reg_1v2)) {
		ret = PTR_ERR(reg_1v2);
		PRINT_ERR("Failed to get regulator vcc_gripsensor %d\n", ret);
		return ret;
	}
	ret = regulator_set_voltage(reg_1v2, 1200000, 1200000);
	if (ret) {
		PRINT_ERR("Failed to set voltage for vcc_gripsensor reg_1v2 %d\n", ret);
		return -1;
	}

	PRINT_INFO("vcc_gripsensor regulator setting init");
	return ret;
}

static int grip_1v2_regulator_enable(void)
{
	int ret = 0, idx = 0;

	if(IS_ERR_OR_NULL(reg_1v2)){
		ret = PTR_ERR(reg_1v2);
		PRINT_ERR("Failed to get regulator vcc_gripsensor %d\n", ret);
		return ret;
	}

	ret = regulator_set_load(reg_1v2, 10000);
	if(ret < 0){
		PRINT_ERR("Failed to set load for vcc_gripsensor reg_1v2 %d\n", ret);
		return ret;
	}

	ret = regulator_enable(reg_1v2);
	if(ret){
		PRINT_ERR("Failed to enable vcc_gripsensor reg_1v2 %d\n", ret);
		return -1;
	}

	for(idx=0; idx<10; idx++){
		if(regulator_is_enabled(reg_1v2) > 0){
			PRINT_DEBUG("vcc_gripsensor regulator is enabled(idx=%d)", idx);
			break;
		}
	}
	if(idx >= 10){
		PRINT_ERR("vcc_gripsensor regulator is enabled fail(retry count >= %d)", idx);
		return -1;
	}

	PRINT_INFO("Update vcc_psensor to NPM_mode");
	return ret;
}

static int grip_1v2_regulator_disable(void)
{
	int ret = 0;

	if(IS_ERR_OR_NULL(reg_1v2)){
		ret = PTR_ERR(reg_1v2);
		PRINT_ERR("Failed to get regulator vcc_gripsensor %d\n", ret);
		return ret;
	}

	ret = regulator_set_load(reg_1v2, 0);
	if(ret < 0){
		PRINT_ERR("Failed to set load for vcc_gripsensor reg_1v2 %d\n", ret);
		return ret;
	}

	ret = regulator_disable(reg_1v2);
	if(ret){
		PRINT_ERR("Failed to enable vincentr reg_1v2 %d\n", ret);
		return -1;
	}

	PRINT_INFO("Update vcc_gripsensor to LPM_mode");
	return ret;
}

static int grip_1v8_regulator_init(void)
{
	int ret = 0;
	reg_1v8 = regulator_get(snt8100fsr_g->dev,"vcc_grip_1v8");
	if (IS_ERR_OR_NULL(reg_1v8)) {
		ret = PTR_ERR(reg_1v8);
		PRINT_ERR("Failed to get regulator vcc_gripsensor %d\n", ret);
		return ret;
	}
	ret = regulator_set_voltage(reg_1v8, 1800000, 1800000);
	if (ret) {
		PRINT_ERR("Failed to set voltage for vcc_gripsensor reg_1v8 %d\n", ret);
		return -1;
	}

	PRINT_INFO("vcc_gripsensor regulator setting init");
	return ret;
}

static int grip_1v8_regulator_enable(void)
{
	int ret = 0, idx = 0;

	if(IS_ERR_OR_NULL(reg_1v8)){
		ret = PTR_ERR(reg_1v8);
			PRINT_ERR("Failed to get regulator vcc_gripsensor %d\n", ret);
		return ret;
	}

	ret = regulator_set_load(reg_1v8, 10000);
	if(ret < 0){
		PRINT_ERR("Failed to set load for vcc_gripsensor reg_1v8 %d\n", ret);
		return ret;
	}

	ret = regulator_enable(reg_1v8);
	if(ret){
		PRINT_ERR("Failed to enable vcc_gripsensor reg_1v8 %d\n", ret);
		return -1;
	}

	for(idx=0; idx<10; idx++){
		if(regulator_is_enabled(reg_1v8) > 0){
			PRINT_DEBUG("vcc_gripsensor regulator is enabled(idx=%d)", idx);
			break;
		}
	}
	if(idx >= 10){
		PRINT_ERR("vcc_gripsensor regulator is enabled fail(retry count >= %d)", idx);
		return -1;
	}

	PRINT_INFO("Update vcc_psensor to NPM_mode");
	return ret;
}

static int grip_1v8_regulator_disable(void)
{
	int ret = 0;

	if(IS_ERR_OR_NULL(reg_1v8)){
		ret = PTR_ERR(reg_1v8);
		PRINT_ERR("Failed to get regulator vcc_gripsensor %d\n", ret);
		return ret;
	}

	ret = regulator_set_load(reg_1v8, 0);
	if(ret < 0){
		PRINT_ERR("Failed to set load for vcc_gripsensor reg_1v8 %d\n", ret);
		return ret;
	}

	ret = regulator_disable(reg_1v8);
	if(ret){
		PRINT_ERR("Failed to enable vincentr reg_1v8 %d\n", ret);
		return -1;
	}

	PRINT_INFO("Update vcc_gripsensor to LPM_mode");
	return ret;
}

static struct grip_asus_struct grip_2v8_asus_snt = {
	.grip_regulator_init = grip_2v8_regulator_init,
	.grip_regulator_enable = grip_2v8_regulator_enable,
	.grip_regulator_disable = grip_2v8_regulator_disable,
};
static struct grip_asus_struct grip_1v8_asus_snt = {
	.grip_regulator_init = grip_1v8_regulator_init,
	.grip_regulator_enable = grip_1v8_regulator_enable,
	.grip_regulator_disable = grip_1v8_regulator_disable,
};
static struct grip_asus_struct grip_1v2_asus_snt = {
	.grip_regulator_init = grip_1v2_regulator_init,
	.grip_regulator_enable = grip_1v2_regulator_enable,
	.grip_regulator_disable = grip_1v2_regulator_disable,
};

extern enum DEVICE_HWID g_ASUS_hwID;
void set_1V2_2V8_pin_func_no_work(void) {
	uint16_t reg_chidlsb, reg_chidmsb;
	int gpio_req;
	
	PRINT_INFO("1v8 pull up test");
	snt8100fsr_g->mgrip_1v8_asus_func->grip_regulator_init();
	snt8100fsr_g->mgrip_1v8_asus_func->grip_regulator_enable();
	msleep(5);

	PRINT_INFO("1v2 pull up test");
	snt8100fsr_g->mgrip_1v2_asus_func->grip_regulator_init();
	snt8100fsr_g->mgrip_1v2_asus_func->grip_regulator_enable();
	msleep(5);
	PRINT_INFO("2v8 pull up test");
	snt8100fsr_g->mgrip_2v8_asus_func->grip_regulator_init();
	snt8100fsr_g->mgrip_2v8_asus_func->grip_regulator_enable();
	msleep(5);

	if(HW_REV_ER == g_ASUS_hwID){
		PRINT_INFO("alsps_int_n_p v2000 pull-down");
		gpio_req = of_get_named_gpio(snt8100fsr_g->dev->of_node, "alsps_v2000", 0);
		gpio_request(gpio_req, "alsp_int_n_p");
		gpio_direction_output(gpio_req, 0); //output low
	}

	PRINT_INFO("rst pin pull-up");
	gpio_req = of_get_named_gpio(snt8100fsr_g->dev->of_node, SNT_RST_NAME, 0);
	gpio_request(gpio_req, "snt_rst_gpio");
	gpio_direction_output(gpio_req, 1); //output high

	PRINT_INFO("id");
	gpio_request(snt8100fsr_g->snt_id_num, "8350_gpio132");
	gpio_direction_output(snt8100fsr_g->snt_id_num, 1); //output high
	msleep(10);

	read_register(snt8100fsr_g, REGISTER_CHIP_ID_LSB, &reg_chidlsb);
	PRINT_INFO("REGISTER_CHIP_ID_LSB = 0x%x", reg_chidlsb);
	read_register(snt8100fsr_g, REGISTER_CHIP_ID_MSB, &reg_chidmsb);
	PRINT_INFO("REGISTER_CHIP_ID_MSB = 0x%x", reg_chidmsb);

	snt_set_pinctrl(snt8100fsr_g->dev, GRIP_TRIG1_NO_PULL);
	snt_set_pinctrl(snt8100fsr_g->dev, GRIP_TRIG2_NO_PULL);
	snt_set_pinctrl(snt8100fsr_g->dev, GRIP_INT_SUSPEND);
	snt_set_pinctrl(snt8100fsr_g->dev, GRIP_WAKE_SUSPEND);
	return;
}

void asus_init_probe(void){
	int ret;
	//struct delayed_work en_pwr_wk;
	struct Grip_DPC_status *Grip_DPC_status_t;
	struct grip_status *grip_state_t;

	//Initialization for main i2c device only, should be put after this line
	Grip_DPC_status_t = memory_allocate(sizeof(*Grip_DPC_status_t), GFP_KERNEL);
	Grip_DPC_status_t->Condition = 0xa710;
	Grip_DPC_status_t->High = 0x14;
	Grip_DPC_status_t->Low = 0x5;
	Grip_DPC_status_g = Grip_DPC_status_t;
	grip_state_t = memory_allocate(sizeof(*grip_state_t), GFP_KERNEL);

	/* fw status default value */
	snt8100fsr_g->grip_fw_loading_status = false;

	Grip_DPC_status_g = Grip_DPC_status_t;
	memset(grip_state_t, -1, sizeof(*grip_state_t));
	grip_status_g = grip_state_t;

	snt8100fsr_g->mgrip_2v8_asus_func = &grip_2v8_asus_snt;
	snt8100fsr_g->mgrip_1v8_asus_func = &grip_1v8_asus_snt;
	snt8100fsr_g->mgrip_1v2_asus_func = &grip_1v2_asus_snt;

	asus_wq = create_workqueue(asus_grip_queue);
	if (!asus_wq) {
		PRINT_CRIT("Unable to create_workqueue(%s)", asus_grip_queue);
		return;
	}

	INIT_DELAYED_WORK(&check_stuck_wake, check_stuck_semaphore);
	INIT_DELAYED_WORK(&rst_recovery_wk, grip_dump_status_func);
	INIT_DELAYED_WORK(&rst_gpio_wk, Reset_Func);
	snt8100fsr_g->snt_wakelock = wakeup_source_register(NULL, "snt_wakelock");

	set_1V2_2V8_pin_func_no_work();
	
	/* Clay ioctl +++*/
	ret = sntSensor_miscRegister();
	if (ret < 0) {
		PRINT_INFO("creat misc fail");
	}

	create_Grip_en_proc_file();
	create_Grip_frame_proc_file();
	create_Grip_raw_en_proc_file();
	
	/* Tap proc */
	create_Grip_Tap_En_proc_file();
	create_Grip_Tap_Force_proc_file();
	create_Grip_Tap_min_pos_proc_file();
	create_Grip_Tap_max_pos_proc_file();
	create_Grip_Tap_slope_window_proc_file();
	create_Grip_Tap_slope_tap_force_proc_file();
	create_Grip_Tap_slope_release_force_proc_file();
	create_Grip_Tap_delta_tap_force_proc_file();
	create_Grip_Tap_delta_release_force_proc_file();
	create_Grip_Tap_vib_en_proc_file();

	create_Grip_Squeeze_en_proc_file();
	create_Grip_Squeeze_force_proc_file();
	create_Grip_Squeeze_short_dur_proc_file();
	create_Grip_Squeeze_long_dur_proc_file();
	create_Grip_Squeeze_drop_rate_proc_file();
	create_Grip_Squeeze_drop_total_proc_file();
	create_Grip_Squeeze_up_rate_proc_file();
	create_Grip_Squeeze_up_total_proc_file();
	
	create_Grip_Slide_en_proc_file();
	create_Grip_Slide_dist_proc_file();
	create_Grip_Slide_2nd_dist_proc_file();
	create_Grip_Slide_force_proc_file();
	create_Grip_Slide_min_pos_proc_file();
	create_Grip_Slide_max_pos_proc_file();
	create_Grip_Slide_vib_en_proc_file();
	create_Grip_Slide_tap_priority_proc_file();
	
	create_Grip_Swipe_en_proc_file();
	create_Grip_Swipe_velocity_proc_file();
	create_Grip_Swipe_len_proc_file();
	create_Grip_Swipe_min_pos_proc_file();
	create_Grip_Swipe_max_pos_proc_file();

	/******* Dynamic Loading FW ********/
	create_Grip_FW_VER_proc_file();
	create_Grip_FW_RESULT_proc_file();
	create_Grip_set_power_proc_file();

	/* Factory requirment */
	create_Grip_I2c_Check_proc_file();
	create_Grip_Calibration_raw_data_proc_file();
	create_Grip_Disable_WakeLock_proc_file();
	create_Grip_ReadK_proc_file();
#ifdef ASUS_ZS673KS_PROJECT
	create_Grip_Cal_Read_proc_file();
#endif

}

