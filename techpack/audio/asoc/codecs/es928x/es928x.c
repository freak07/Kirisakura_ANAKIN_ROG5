/*
*	ES928x SABRE HiFi ALSA Codec Driver
*	Copyright (C) 2019 Samuel Schaefer <samuel.schaefer@esstech.com>, ESS Technology Inc.
*
*
*	This program is free software; you can redistribute it and/or modify
*	it under the terms of the GNU General Public License as published by
*	the Free Software Foundation; either version 2 of the License, or
*	(at your option) any later version.
*
*	This program is distributed in the hope that it will be useful,
*	but WITHOUT ANY WARRANTY; without even the implied warranty of
*	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*	GNU General Public License for more details.
*
*	You should have received a copy of the GNU General Public License along
*	with this program; if not, write to the Free Software Foundation, Inc.,
*	51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <sound/jack.h>
#include <linux/of_gpio.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/proc_fs.h> //Austin +++
//#include <soc/internal.h>
#include "es928x.h"

#define ESS_DEBUG "driver/audio_debug"
#define FW_RETRY_TIME 3
#define ES928X_SND_JACK_HEADSET SND_JACK_MECHANICAL|SND_JACK_HEADPHONE|SND_JACK_MICROPHONE
#define ES928X_SND_JACK_HEADPHONE SND_JACK_MECHANICAL|SND_JACK_HEADPHONE
#define ES928X_REG_MAX 255
#define ES928X_SYSFS_ENTRY_MAX_LEN 4096
static char *fw_name = "ES928XA.ROM";
static char *fw_name_v = "ES928X.ROM";

struct es928x_priv *g_es928x;
static int audbg_read_ctrl = 0;
static unsigned int es928x_fw_build = 0;

module_param(fw_name, charp, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(fw_name, "ES928xA firmware file name");
MODULE_FIRMWARE("ES928XA.ROM");

module_param(fw_name_v, charp, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(fw_name, "ES928x firmware file name");
MODULE_FIRMWARE("ES928X.ROM");

static bool es928x_readable(struct device *dev, unsigned int reg)
{
	return true;
}

static bool es928x_writeable(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 216 ... 230:
		return false;
	default:
		return true;
	}
}

static const struct regmap_config es928x_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.readable_reg = es928x_readable,
	.writeable_reg = es928x_writeable,
	.max_register = 255,
	.cache_type = REGCACHE_NONE,
};

struct es928x_pll_config {
	unsigned int Fout;
	unsigned int Fin;
	u8 Ni;
	u8 Nf;
	u8 No;
};

static const struct es928x_pll_config es928x_pll_div[] = {
	//   49.152 MHz   //
	{49152000,	11289600,	14,	61,	2	},	//	49.190 MHz
	{49152000,	12288000,	11,	44,	2	},	//	49.152 MHz
	{49152000,	20000000,	24,	59,	2	},	//	49.167 MHz
	{49152000,	22000000,	26,	58,	2	},	//	49.077 MHz
	{49152000,	22579200,	28,	61,	2	},	//	49.190 MHz
	{49152000,	24000000,	21,	43,	2	},	//	49.143 MHz
	{49152000,	24576000,	21,	42,	2	},	//	49.152 MHz
	{49152000,	26000000,	27,	51,	2	},	//	49.111 MHz
	{49152000,	28000000,	33,	58,	2	},	//	49.212 MHz
	{49152000,	30000000,	36,	59,	2	},	//	49.167 MHz
	{49152000,	32000000,	28,	43,	2	},	//	49.143 MHz
	{49152000,	33868800,	31,	45,	2	},	//	49.164 MHz
	{49152000,	34000000,	36,	52,	2	},	//	49.111 MHz
	{49152000,	36000000,	41,	56,	2	},	//	49.171 MHz
	{49152000,	36864000,	33,	44,	2	},	//	49.152 MHz
	{49152000,	38000000,	34,	44,	2	},	//	49.176 MHz
	{49152000,	40000000,	35,	43,	2	},	//	49.143 MHz
	{49152000,	42000000,	47,	55,	2	},	//	49.149 MHz
	{49152000,	44000000,	51,	57,	2	},	//	49.176 MHz
	{49152000,	45158400,	45,	49,	2	},	//	49.172 MHz
	{49152000,	46000000,	44,	47,	2	},	//	49.136 MHz
	{49152000,	48000000,	42,	43,	2	},	//	49.143 MHz
	{49152000,	49152000,	41,	41,	2	},	//	49.152 MHz
	{49152000,	50000000,	59,	58,	2	},	//	49.153 MHz
	//   45.1584 MHz   //
	{45158400,	11289600,	10,	40,	2	},	//	45.158 MHz
	{45158400,	12288000,	12,	44,	2	},	//	45.056 MHz
	{45158400,	20000000,	23,	52,	2	},	//	45.217 MHz
	{45158400,	22000000,	19,	39,	2	},	//	45.158 MHz
	{45158400,	22579200,	19,	38,	2	},	//	45.158 MHz
	{45158400,	24000000,	25,	47,	2	},	//	45.120 MHz
	{45158400,	24576000,	25,	46,	2	},	//	45.220 MHz
	{45158400,	26000000,	23,	40,	2	},	//	45.217 MHz
	{45158400,	28000000,	31,	50,	2	},	//	45.161 MHz
	{45158400,	30000000,	26,	39,	2	},	//	45.000 MHz
	{45158400,	32000000,	34,	48,	2	},	//	45.176 MHz
	{45158400,	33868800,	30,	40,	2	},	//	45.158 MHz
	{45158400,	34000000,	40,	53,	2	},	//	45.050 MHz
	{45158400,	36000000,	43,	54,	2	},	//	45.209 MHz
	{45158400,	36864000,	40,	49,	2	},	//	45.158 MHz
	{45158400,	38000000,	37,	44,	2	},	//	45.189 MHz
	{45158400,	40000000,	39,	44,	2	},	//	45.128 MHz
	{45158400,	42000000,	40,	43,	2	},	//	45.150 MHz
	{45158400,	44000000,	38,	39,	2	},	//	45.158 MHz
	{45158400,	45158400,	38,	38,	2	},	//	45.158 MHz
	{45158400,	46000000,	55,	54,	2	},	//	45.164 MHz
	{45158400,	48000000,	51,	48,	2	},	//	45.176 MHz
	{45158400,	49152000,	49,	45,	2	},	//	45.140 MHz
	{45158400,	50000000,	62,	56,	2	},	//	45.161 MHz
	//   24.576 MHz   //
	{24576000,	11289600,	14,	61,	4	},	//	24.595 MHz
	{24576000,	12288000,	11,	44,	4	},	//	24.576 MHz
	{24576000,	20000000,	24,	59,	4	},	//	24.583 MHz
	{24576000,	22000000,	26,	58,	4	},	//	24.538 MHz
	{24576000,	22579200,	28,	61,	4	},	//	24.595 MHz
	{24576000,	24000000,	21,	43,	4	},	//	24.571 MHz
	{24576000,	24576000,	21,	42,	4	},	//	24.576 MHz
	{24576000,	26000000,	27,	51,	4	},	//	24.556 MHz
	{24576000,	28000000,	33,	58,	4	},	//	24.606 MHz
	{24576000,	30000000,	36,	59,	4	},	//	24.583 MHz
	{24576000,	32000000,	28,	43,	4	},	//	24.571 MHz
	{24576000,	33868800,	31,	45,	4	},	//	24.582 MHz
	{24576000,	34000000,	36,	52,	4	},	//	24.556 MHz
	{24576000,	36000000,	41,	56,	4	},	//	24.585 MHz
	{24576000,	36864000,	33,	44,	4	},	//	24.576 MHz
	{24576000,	38000000,	34,	44,	4	},	//	24.588 MHz
	{24576000,	40000000,	35,	43,	4	},	//	24.571 MHz
	{24576000,	42000000,	47,	55,	4	},	//	24.574 MHz
	{24576000,	44000000,	51,	57,	4	},	//	24.588 MHz
	{24576000,	45158400,	45,	49,	4	},	//	24.586 MHz
	{24576000,	46000000,	44,	47,	4	},	//	24.568 MHz
	{24576000,	48000000,	42,	43,	4	},	//	24.571 MHz
	{24576000,	49152000,	41,	41,	4	},	//	24.576 MHz
	{24576000,	50000000,	59,	58,	4	},	//	24.576 MHz
	//   22.5792 MHz   //
	{22579200,	11289600,	10,	40,	4	},	//	22.579 MHz
	{22579200,	12288000,	15,	69,	5	},	//	22.610 MHz
	{22579200,	20000000,	17,	48,	5	},	//	22.588 MHz
	{22579200,	22000000,	19,	39,	4	},	//	22.579 MHz
	{22579200,	22579200,	19,	38,	4	},	//	22.579 MHz
	{22579200,	24000000,	20,	47,	5	},	//	22.560 MHz
	{22579200,	24576000,	27,	62,	5	},	//	22.574 MHz
	{22579200,	26000000,	29,	63,	5	},	//	22.593 MHz
	{22579200,	28000000,	31,	50,	4	},	//	22.581 MHz
	{22579200,	30000000,	34,	64,	5	},	//	22.588 MHz
	{22579200,	32000000,	34,	60,	5	},	//	22.588 MHz
	{22579200,	33868800,	30,	40,	4	},	//	22.579 MHz
	{22579200,	34000000,	41,	68,	5	},	//	22.556 MHz
	{22579200,	36000000,	44,	69,	5	},	//	22.582 MHz
	{22579200,	36864000,	32,	49,	5	},	//	22.579 MHz
	{22579200,	38000000,	35,	52,	5	},	//	22.583 MHz
	{22579200,	40000000,	34,	48,	5	},	//	22.588 MHz
	{22579200,	42000000,	40,	43,	4	},	//	22.575 MHz
	{22579200,	44000000,	38,	39,	4	},	//	22.579 MHz
	{22579200,	45158400,	38,	38,	4	},	//	22.579 MHz
	{22579200,	46000000,	44,	54,	5	},	//	22.582 MHz
	{22579200,	48000000,	51,	48,	4	},	//	22.588 MHz
	{22579200,	49152000,	54,	62,	5	},	//	22.574 MHz
	{22579200,	50000000,	62,	56,	4	},	//	22.581 MHz
	//   12.288 MHz //
	{12288000,	11289600,	10,	49,	9	},	//	12.293 MHz
	{12288000,	12288000,	11,	44,	8	},	//	12.288 MHz
	{12288000,	20000000,	17,	47,	9	},	//	12.288 MHz
	{12288000,	22000000,	22,	43,	7	},	//	12.286 MHz
	{12288000,	22579200,	21,	40,	7	},	//	12.288 MHz
	{12288000,	24000000,	23,	53,	9	},	//	12.290 MHz
	{12288000,	24576000,	21,	42,	8	},	//	12.288 MHz
	{12288000,	26000000,	26,	43,	7	},	//	12.286 MHz
	{12288000,	28000000,	28,	43,	7	},	//	12.286 MHz
	{12288000,	30000000,	30,	43,	7	},	//	12.286 MHz
	{12288000,	32000000,	28,	43,	8	},	//	12.286 MHz
	{12288000,	33868800,	31,	45,	8	},	//	12.291 MHz
	{12288000,	34000000,	34,	43,	7	},	//	12.286 MHz
	{12288000,	36000000,	36,	43,	7	},	//	12.286 MHz
	{12288000,	36864000,	33,	44,	8	},	//	12.288 MHz
	{12288000,	38000000,	38,	43,	7	},	//	12.286 MHz
	{12288000,	40000000,	34,	47,	9	},	//	12.288 MHz
	{12288000,	42000000,	47,	55,	8	},	//	12.287 MHz
	{12288000,	44000000,	44,	43,	7	},	//	12.286 MHz
	{12288000,	45158400,	49,	60,	9	},	//	12.288 MHz
	{12288000,	46000000,	46,	43,	7	},	//	12.286 MHz
	{12288000,	48000000,	46,	53,	9	},	//	12.290 MHz
	{12288000,	49152000,	41,	41,	8	},	//	12.288 MHz
	{12288000,	50000000,	59,	58,	8	},	//	12.288 MHz
	//   11.2896 MHz   //
	{11289600,	11289600,	10,	40,	8	},	//	11.290 MHz
	{11289600,	12288000,	15,	62,	9	},	//	11.287 MHz
	{11289600,	20000000,	17,	48,	10	},	//	11.294 MHz
	{11289600,	22000000,	19,	39,	8	},	//	11.289 MHz
	{11289600,	22579200,	19,	38,	8	},	//	11.290 MHz
	{11289600,	24000000,	26,	55,	9	},	//	11.282 MHz
	{11289600,	24576000,	30,	62,	9	},	//	11.287 MHz
	{11289600,	26000000,	22,	43,	9	},	//	11.293 MHz
	{11289600,	28000000,	31,	50,	8	},	//	11.290 MHz
	{11289600,	30000000,	34,	64,	10	},	//	11.294 MHz
	{11289600,	32000000,	34,	54,	9	},	//	11.294 MHz
	{11289600,	33868800,	30,	40,	8	},	//	11.290 MHz
	{11289600,	34000000,	41,	68,	10	},	//	11.278 MHz
	{11289600,	36000000,	44,	69,	10	},	//	11.291 MHz
	{11289600,	36864000,	32,	49,	10	},	//	11.290 MHz
	{11289600,	38000000,	35,	52,	10	},	//	11.291 MHz
	{11289600,	40000000,	37,	47,	9	},	//	11.291 MHz
	{11289600,	42000000,	40,	43,	8	},	//	11.287 MHz
	{11289600,	44000000,	38,	39,	8	},	//	11.289 MHz
	{11289600,	45158400,	38,	38,	8	},	//	11.290 MHz
	{11289600,	46000000,	44,	54,	10	},	//	11.291 MHz
	{11289600,	48000000,	51,	48,	8	},	//	11.294 MHz
	{11289600,	49152000,	60,	62,	9	},	//	11.287 MHz
	{11289600,	50000000,	62,	56,	8	},	//	11.290 MHz
};



static const char *es928x_adc_l_mux_select[] = {"No Input", "HPA Out Left", "AUX Left"};
static const char *es928x_adc_r_mux_select[] = {"No Input", "HPA Out Right", "AUX Right"};
static const char *es928x_adc_mic_mux_select[] = {"No Input", "Mic", "Ground"};
static SOC_ENUM_SINGLE_DECL(es928x_adc_l_mux_enum,
	ES928X_REG_MUX_ADC_L_MUX_CFG, ES928X_MUX_L_INPUT_SEL_SHIFT, es928x_adc_l_mux_select);
static SOC_ENUM_SINGLE_DECL(es928x_adc_r_mux_enum,
	ES928X_REG_MUX_ADC_R_MUX_CFG, ES928X_MUX_R_INPUT_SEL_SHIFT, es928x_adc_r_mux_select);
static SOC_ENUM_SINGLE_DECL(es928x_adc_mic_mux_enum,
	ES928X_REG_MUX_ADC_MIC_MUX_CFG, ES928X_MUX_MIC_INPUT_SEL_SHIFT, es928x_adc_mic_mux_select);

static const char *es9328x_adc_preamp_gain_texts[] = {"0dB", "+6dB",
	"+12dB", "+18dB", "+24dB", "+30dB"};
static const unsigned int es9328x_adc_preamp_gain_values[] = { 1, 2, 4, 8, 16, 32};
static SOC_VALUE_ENUM_SINGLE_DECL(es928x_adc_l_preamp_gain_enum,
	ES928X_REG_MUX_ADC_L_MUX_CFG, ES928X_MUX_L_GAIN_SHIFT, ES928X_MUX_L_GAIN_MASK,
	es9328x_adc_preamp_gain_texts, es9328x_adc_preamp_gain_values);
static SOC_VALUE_ENUM_SINGLE_DECL(es928x_adc_r_preamp_gain_enum,
	ES928X_REG_MUX_ADC_R_MUX_CFG, ES928X_MUX_R_GAIN_SHIFT, ES928X_MUX_R_GAIN_MASK,
	es9328x_adc_preamp_gain_texts, es9328x_adc_preamp_gain_values);
static SOC_VALUE_ENUM_SINGLE_DECL(es928x_adc_mic_preamp_gain_enum,
	ES928X_REG_MUX_ADC_MIC_MUX_CFG, ES928X_MUX_MIC_GAIN_SHIFT, ES928X_MUX_MIC_GAIN_MASK,
	es9328x_adc_preamp_gain_texts, es9328x_adc_preamp_gain_values);

static const char *es928x_adc_input_sel[] = {"ADC Left", "ADC Right", "PDM1 Left",
	"PDM1 Right", "ADC Mic", "PDM2 Left", "PDM2 Right"};
static SOC_ENUM_SINGLE_DECL(es928x_adc_ch1_enum,
	ES928X_REG_ADC_CH_IN_SEL_1, ES928X_ADC_CH1_SEL_SHIFT, es928x_adc_input_sel);
static SOC_ENUM_SINGLE_DECL(es928x_adc_ch2_enum,
	ES928X_REG_ADC_CH_IN_SEL_1, ES928X_ADC_CH2_SEL_SHIFT, es928x_adc_input_sel);
static SOC_ENUM_SINGLE_DECL(es928x_adc_ch3_enum,
	ES928X_REG_ADC_CH_IN_SEL_1, ES928X_ADC_CH3_SEL_LSB_SHIFT, es928x_adc_input_sel);
static SOC_ENUM_SINGLE_DECL(es928x_adc_ch4_enum,
	ES928X_REG_ADC_CH_IN_SEL_2, ES928X_ADC_CH4_SEL_SHIFT, es928x_adc_input_sel);

static DECLARE_TLV_DB_SCALE(adc_tlv, 0, -50, 0);
static DECLARE_TLV_DB_SCALE(dac_tlv, 0, -50, 0);

static int es928x_asp_dac_coeff_mem_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct es928x_priv *es928x = snd_soc_component_get_drvdata(component);
	struct es928x_asp *asp = es928x->dac_asp;
	int i;

	for(i = 0; i < ARRAY_SIZE(asp->coefficients); i++)
		ucontrol->value.integer.value[0] = asp->coefficients[i];
	return 0;
}

static int es928x_asp_dac_coeff_mem_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct es928x_priv *es928x = snd_soc_component_get_drvdata(component);
	struct es928x_asp *asp = es928x->dac_asp;
	int ret= 0;


	mutex_lock(&es928x->lock);
	//pr_debug("%s value: %d\n", __func__, (int)ucontrol->value.integer.value[asp->coeff_addr]);
	asp->coefficients[asp->coeff_addr] = ucontrol->value.integer.value[0];
	asp->coeff_addr++;
	if(asp->coeff_addr >= ARRAY_SIZE(asp->coefficients))
	{
		pr_err("%s invalid coefficient address\n", __func__);
		asp->coeff_addr = 0;
		ret = -EINVAL;
	}
	mutex_unlock(&es928x->lock);
	return ret;
}

static int es928x_asp_dac_program_mem_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct es928x_priv *es928x = snd_soc_component_get_drvdata(component);
	struct es928x_asp *asp = es928x->dac_asp;
	int i;

	for(i = 0; i < ARRAY_SIZE(asp->program); i++)
		ucontrol->value.integer.value[0] = asp->program[i];
	return 0;
}

static int es928x_asp_dac_program_mem_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct es928x_priv *es928x = snd_soc_component_get_drvdata(component);
	struct es928x_asp *asp = es928x->dac_asp;
	int ret = 0;

	mutex_lock(&es928x->lock);
	//pr_debug("%s value: %d\n", __func__, (int)ucontrol->value.integer.value[asp->program_addr]);
		asp->program[asp->program_addr] = (u8) ucontrol->value.integer.value[0];
	asp->program_addr++;
	if(asp->program_addr >= ARRAY_SIZE(asp->program))
	{
		pr_err("%s invalid program address\n", __func__);
		asp->program_addr = 0;
		ret = -EINVAL;
	}
	mutex_unlock(&es928x->lock);
	return ret;
}

static int es928x_asp_configure_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct es928x_priv *es928x = snd_soc_component_get_drvdata(component);
	struct es928x_asp *asp = es928x->dac_asp;
	int i;
	u8 asp_en;
	u8 coeff[8];

	pr_debug("%s value: %ld\n", __func__, ucontrol->value.integer.value[0]);
	mutex_lock(&es928x->lock);
	if(ucontrol->value.integer.value[0])
	{
		asp_en = snd_soc_component_read32(component, ES928X_REG_ASP_EN);
		asp_en |= ES928X_ASP_DAC_PROGRAM_EN_MASK;
		regmap_write(es928x->regmap, ES928X_REG_ASP_EN, asp_en);
		//Write Program Memory
		udelay(50);
		for(i = 0; i < asp->program_addr; i++)
		{
			//pr_debug("%s i: %d program: %d\n", __func__,i, asp->program[i]);
			regmap_write(es928x->regmap, ES928X_REG_ASP_PROGRAM_ADDR, (u8)i);
			regmap_write(es928x->regmap, ES928X_REG_ASP_PROGRAM, asp->program[i]);
			asp_en |= ES928X_ASP_DAC_PROGRAM_WE_MASK;
			regmap_write(es928x->regmap, ES928X_REG_ASP_EN, asp_en);
			udelay(50);
			asp_en &= ~ES928X_ASP_DAC_PROGRAM_WE_MASK;
			regmap_write(es928x->regmap, ES928X_REG_ASP_EN, asp_en);
		}

		udelay(50);
		//Write Coefficient Memory
		for(i = 0; i < asp->coeff_addr; i += 2)
		{
			//pr_debug("%s i: %d coeff: %d, %d\n", __func__,i, asp->coefficients[i],asp->coefficients[i+1]);
			regmap_write(es928x->regmap, ES928X_REG_ASP_DATA_ADDR, i >> 1);
			coeff[0] = (asp->coefficients[i] >> 0) & 0xFF;
			coeff[1] = (asp->coefficients[i] >> 8) & 0xFF;
			coeff[2] = (asp->coefficients[i] >> 16) & 0xFF;
			coeff[3] = (asp->coefficients[i] >> 24) & 0xFF;
			coeff[4] = (asp->coefficients[i + 1] >> 0) & 0xFF;
			coeff[5] = (asp->coefficients[i + 1] >> 8) & 0xFF;
			coeff[6] = (asp->coefficients[i + 1] >> 16) & 0xFF;
			coeff[7] = (asp->coefficients[i + 1] >> 24) & 0xFF;

			regmap_bulk_write(es928x->regmap, ES928X_REG_ASP_DATA,
				coeff, ARRAY_SIZE(coeff));
			asp_en |= ES928X_ASP_DAC_COEFF_WE_MASK;
			regmap_write(es928x->regmap, ES928X_REG_ASP_EN, asp_en);
			udelay(50);
			asp_en &= ~ES928X_ASP_DAC_COEFF_WE_MASK;
			regmap_write(es928x->regmap, ES928X_REG_ASP_EN, asp_en);
		}

		asp_en &= ~ES928X_ASP_DAC_PROGRAM_EN_MASK;
		regmap_write(es928x->regmap, ES928X_REG_ASP_EN, asp_en);
	}
	else
	{
		asp->program_addr = 0;
		asp->coeff_addr = 0;
	}
	mutex_unlock(&es928x->lock);

	return 0;
}

static int es928x_hph_impedance_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	int zl, zr;
	bool hphr;
	struct soc_mixer_control *mc;
	struct es928x_jdet_priv *es928x_jdet = g_es928x->jdet;
	struct snd_soc_component *component = es928x_jdet->component;
	struct es928x_priv *es928x = snd_soc_component_get_drvdata(component);
	u8 zdet_val[3];
	int load_types[7] = {-1, 8, 16, 32, 64, 300, 100000};
	int left_load, right_load;
	zl = zr = 0;

	if(es928x_jdet->jack_inserted)
	{
		regmap_bulk_read(es928x->regmap, ES928X_REG_MCU_LOAD_MIC_STATUS, zdet_val, 3);
		switch (zdet_val[1])
		{
			case 117 ... 127:
				left_load = 2;
				break;
			case 106 ... 116:
				left_load = 3;
				break;
			case 82 ... 105:
				left_load = 4;
				break;
			case 51 ... 81:
				left_load = 5;
				break;
			case 0 ... 50:
				left_load = 6;
				break;
			default:
				left_load = 0;
				break;
		}

		switch (zdet_val[2])
		{
			case 117 ... 127:
				right_load = 2;
				break;
			case 106 ... 116:
				right_load = 3;
				break;
			case 82 ... 105:
				right_load = 4;
				break;
			case 51 ... 81:
				right_load = 5;
				break;
			case 0 ... 50:
				right_load = 6;
				break;
			default:
				right_load = 0;
				break;
		}

		dev_info(es928x->dev, "Impedance Measured with codes 0x%x, 0x%x. load_types[%d]=(%d) l(%d) r(%d)\n",
			zdet_val[1], zdet_val[2], (int)(zdet_val[0] >> 4), load_types[(int)(zdet_val[0] >> 4)],load_types[left_load], load_types[right_load]);


		zl = load_types[left_load];
		zr = load_types[right_load];
	}

	mc = (struct soc_mixer_control *)(kcontrol->private_value);
	hphr = mc->shift;

	dev_info(es928x->dev, "%s: zl=%d(ohms), zr=%d(ohms)\n", __func__,
		zl, zr);
	ucontrol->value.integer.value[0] = hphr ? zr : zl;

	return 0;
}


static int es928x_asp_configure_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
		return 0;
}

static int es928x_vol_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(component);
	int ret;

	snd_soc_dapm_mutex_lock(dapm);
	//pr_debug("%s\n", __func__);
	ret = snd_soc_get_volsw_range(kcontrol, ucontrol);

	snd_soc_dapm_mutex_unlock(dapm);

	return ret;
}

static int es928x_vol_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(component);
	struct es928x_priv *es928x = snd_soc_component_get_drvdata(component);
	int ret;

	snd_soc_dapm_mutex_lock(dapm);
	//pr_debug("%s\n", __func__);
	ret = snd_soc_put_volsw_range(kcontrol, ucontrol);
	regmap_update_bits(es928x->regmap, ES928X_REG_DAC_THD_IIR,
		ES928X_RUN_VOLUME_MASK, ES928X_RUN_VOLUME_MASK);
	regmap_update_bits(es928x->regmap, ES928X_REG_DAC_THD_IIR,
		ES928X_RUN_VOLUME_MASK, ~ES928X_RUN_VOLUME_MASK);

	snd_soc_dapm_mutex_unlock(dapm);

	return ret;
}

static const struct snd_kcontrol_new es928x_controls[] = {
	SOC_SINGLE_RANGE_TLV("ADC CH1 Volume", ES928X_REG_ADC_VOL_1,
			ES928X_ADC_VOLUME1_SHIFT, 0, 255, 0, adc_tlv),
	SOC_SINGLE_RANGE_TLV("ADC CH2 Volume", ES928X_REG_ADC_VOL_2,
			ES928X_ADC_VOLUME2_SHIFT, 0, 255, 0, adc_tlv),
	SOC_SINGLE_RANGE_TLV("ADC CH3 Volume", ES928X_REG_ADC_VOL_3,
			ES928X_ADC_VOLUME3_SHIFT, 0, 255, 0, adc_tlv),
	SOC_SINGLE_RANGE_TLV("ADC CH4 Volume", ES928X_REG_ADC_VOL_4,
			ES928X_ADC_VOLUME4_SHIFT, 0, 255, 0, adc_tlv),
	SOC_SINGLE_RANGE_EXT_TLV("DAC 1 Volume", ES928X_REG_DAC_VOL_1,
		0, 0, 255, 0, es928x_vol_get, es928x_vol_put, dac_tlv),
	SOC_SINGLE_RANGE_EXT_TLV("DAC 2 Volume", ES928X_REG_DAC_VOL_2,
		0, 0, 255, 0, es928x_vol_get, es928x_vol_put, dac_tlv),
	SOC_ENUM("ADC MIC Gain", es928x_adc_mic_preamp_gain_enum),
	SOC_ENUM("ADC Left Gain", es928x_adc_l_preamp_gain_enum),
	SOC_ENUM("ADC Right Gain", es928x_adc_r_preamp_gain_enum),
	SOC_SINGLE("DAC ASP", ES928X_REG_DAC_THD_IIR, ES928X_DAC_ASP_ENABLE_SHIFT,1,0),
	SOC_SINGLE_EXT("DAC ASP Coefficients", SND_SOC_NOPM, 0, 0x7FFFFFFF, 0,
		es928x_asp_dac_coeff_mem_get, es928x_asp_dac_coeff_mem_put),
	SOC_SINGLE_EXT("DAC ASP Program", SND_SOC_NOPM, 0,  255, 0,
		es928x_asp_dac_program_mem_get, es928x_asp_dac_program_mem_put),
	SOC_SINGLE_EXT("HPHL Impedance", 0, 0, UINT_MAX, 0,
		       es928x_hph_impedance_get, NULL),
	SOC_SINGLE_EXT("HPHR Impedance", 0, 1, UINT_MAX, 0,
		       es928x_hph_impedance_get, NULL),
	SOC_SINGLE_BOOL_EXT("DAC ASP Initialize", 0,
		es928x_asp_configure_get, es928x_asp_configure_put),
	SOC_SINGLE("DAC ASP Custom Address RAM2", ES928X_REG_DAC_ASP_CUSTOM_RAM2_ADDR_DAC,
		ES928X_ASP_CUSTOM_RAM2_ADDR_DAC_SHIFT, ES928X_ASP_CUSTOM_RAM2_ADDR_DAC_SHIFT, 0),
	SOC_SINGLE("DAC ASP Custom Address RAM1", ES928X_REG_DAC_ASP_CUSTOM_RAM1_ADDR_DAC,
		ES928X_ASP_CUSTOM_RAM1_ADDR_DAC_SHIFT, ES928X_ASP_CUSTOM_RAM1_ADDR_DAC_MASK, 0),
	SOC_SINGLE("MIC Mix Path", ES928X_REG_MCU_MASTER_CTRL, 4, 1, 0),
	SOC_SINGLE("DRE Rate", ES928X_REG_DAC_DRE_CFG,
		ES928X_DAC_DRE_DECAY_RATE_SHIFT, 31, 0),
};

static const struct snd_soc_dapm_widget es928x_dapm_widgets[] = {
	SND_SOC_DAPM_SUPPLY("Supply", SND_SOC_NOPM, 0, 0,
				NULL, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN("AIFINL", "ES928x Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("AIFINR", "ES928x Playback", 1, SND_SOC_NOPM, 0, 0),
	//SND_SOC_DAPM_DAC("DAC", NULL, ES928X_REG_MCU_MASTER_CTRL, 0, 0),
	SND_SOC_DAPM_DAC_E("DAC", NULL, ES928X_REG_MCU_MASTER_CTRL, 0, 0, NULL, SND_SOC_DAPM_POST_REG),

	SND_SOC_DAPM_OUTPUT("OUT"),

	SND_SOC_DAPM_INPUT("MIC"),
	SND_SOC_DAPM_ADC("ADC Mic", NULL, ES928X_REG_MCU_MASTER_CTRL, 1, 0),
	SND_SOC_DAPM_AIF_OUT("AIFOUTL", "ES928x Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIFOUTR", "ES928x Capture", 1, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_MICBIAS("MIC BIAS", SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route es928x_dapm_routes[] = {
	{ "MIC BIAS", NULL, "MIC" },
	{ "ADC Mic", NULL, "MIC BIAS"},
	{ "AIFOUTL", NULL, "ADC Mic" },
	{ "AIFOUTR", NULL, "ADC Mic" },

	{ "DAC", NULL, "AIFINL" },
	{ "DAC", NULL, "AIFINR" },
	{ "OUT", NULL, "DAC" },

	{ "DAC", NULL, "Supply" },
	{ "ADC Mic", NULL, "Supply" },
};

static int es928x_hw_reset(struct es928x_priv *es928x, int value)
{
	int ret;
	ret = gpio_is_valid(es928x->rst_gpio);
	if (!ret) {
		dev_err(es928x->dev,"rst_gpio %d invalid.\n",es928x->rst_gpio);
		return -EINVAL;
	}
//	if(!value)
//	{
//		regmap_update_bits(es928x->regmap, ES928X_REG_JDET_CONFIG,
//			ES928X_JDET_HOLD_MASK, ~ES928X_JDET_HOLD_MASK);
//	}
	dev_info(es928x->dev, "%s value: %d (%pS)", __func__, value, __builtin_return_address(0));
	gpio_set_value_cansleep(es928x->rst_gpio, value);

	return ret;
}

/* FW Routines */
static int es928x_fw_load_new(const struct firmware *fw, struct es928x_priv *es928x)
{
	unsigned short seg_length, seg_addr, max_seg_length = 0;
	const u8 *romptr;
	u8 *verify;
	int ret = 0;
	//int i;

	if (!fw) {
		dev_err(es928x->dev, "Firmware not found. FW will not be initialized!\n");
		return -EIO;
	}

	if (*fw->data != 0x45 || fw->size < ES928X_FW_ROM_SIZE){
		dev_err(es928x->dev, "Firmware not valid or corrupt.\n");
		return -EINVAL;
	}

	regmap_write(es928x->regmap, ES928X_REG_FW_CUST_STATUS, 0x01);

	romptr = fw->data + 16 + ES928X_FW_ROM_SIZE;
	while(1) //Write to ES928x
	{
		seg_length = *(romptr++) & 0x00FF;
		seg_length |= (*(romptr++) << 8) & 0xFF00;

		if(!seg_length)
			break;

		if(seg_length > max_seg_length)
			max_seg_length = seg_length;

		regmap_write(es928x->regmap, ES928X_REG_FW_CUST_WRITE_ADDR_L, *romptr);
		seg_addr = *(romptr++) & 0x00FF;
		regmap_write(es928x->regmap, ES928X_REG_FW_CUST_WRITE_ADDR_H, *romptr);
		seg_addr |= (*(romptr++) << 8) & 0xFF00;
		regmap_bulk_write(es928x->regmap, ES928X_REG_FW_CUST_WRITE_DATA, romptr, seg_length);
		romptr += seg_length;
	}

	romptr = fw->data + 16 + ES928X_FW_ROM_SIZE;
	verify = devm_kzalloc(es928x->dev, max_seg_length*sizeof(u8), GFP_KERNEL);

	if(!verify)
		return -ENOMEM;


	while(1) //Verify ES928x
	{
		seg_length = *(romptr++) & 0x00FF;
		seg_length |= (*(romptr++) << 8) & 0xFF00;
		if(!seg_length)
			break;

		regmap_write(es928x->regmap, ES928X_REG_FW_CUST_WRITE_ADDR_L, *romptr);
		seg_addr = *(romptr++) & 0x00FF;
		regmap_write(es928x->regmap, ES928X_REG_FW_CUST_WRITE_ADDR_H, *romptr);
		seg_addr |= (*(romptr++) << 8) & 0xFF00;

		//dev_dbg(es928x->dev, "seg_length: %x (%u) addr: %x (%u)\n", seg_length, seg_length, seg_addr, seg_addr);
		ret = regmap_bulk_read(es928x->regmap, ES928X_REG_FW_CUST_WRITE_DATA, verify, seg_length);
		if (ret)
		{
			dev_err(es928x->dev, "firmware readback failed with %d\n", ret);
			goto exit;
		}

		if(memcmp(romptr, verify, seg_length))
		{
			dev_err(es928x->dev, "%s firmware verification failed at 0x%x.\n", fw_name, seg_addr);
			regmap_write(es928x->regmap, ES928X_REG_FW_CUST_STATUS, 0x04);
			ret = -1;
			goto exit;
		}
		romptr += seg_length;

	}
	regmap_write(es928x->regmap, ES928X_REG_FW_CUST_STATUS, 0x02);
	msleep_interruptible(50);
	dev_info(es928x->dev, "Custom ROM load complete. Device will now start using %s\n", fw_name);
exit:
	devm_kfree(es928x->dev, verify);
	return ret;
}

static void es928x_fw_load_update(const struct firmware *fw, struct es928x_priv *es928x)
{
	unsigned int fw_status, cntr = 0;
	unsigned short addr = 0;
	int i=0;
	if (!fw) {
		dev_err(es928x->dev, "%s firmware not loaded. FW will not be updated\n", fw_name);
		return;
	}
	if (*fw->data != 0x45){
		dev_err(es928x->dev, "%s firmware not valid.\n", fw_name);
		return;
	}

	while(addr < ES928X_FW_ROM_SIZE)
	{
		fw_status = 0;
		regmap_write(es928x->regmap, ES928X_REG_MCU_FW_DATA, addr & 0xFF);
		regmap_write(es928x->regmap, ES928X_REG_MCU_FW_DATA, (addr >> 8) & 0xFF);
		if(fw->size - addr > ES928X_FW_BLOCK_SIZE)
		{
			regmap_bulk_write(es928x->regmap, ES928X_REG_MCU_FW_DATA, fw->data + addr, ES928X_FW_BLOCK_SIZE);
		}
		else
		{
			regmap_bulk_write(es928x->regmap, ES928X_REG_MCU_FW_DATA, fw->data + addr, (fw->size - addr));
		}

		if( ((es928x->fw->minor << 16) | es928x->fw->build) >= ES928X_FW_VERSION_1)
		{
			regmap_read(es928x->regmap, ES928X_REG_MCU_FW_CNTR, &cntr);
			if(cntr != (ES928X_FW_BLOCK_SIZE + 2))
			{
				dev_err(es928x->dev, "FW block size: %d\n", cntr);
				i++;
				if (i > 5)
				{
					dev_err(es928x->dev, "Device not responding to I2C communication.\n");
					return;
				}
				regmap_write(es928x->regmap, ES928X_REG_MCU_FW_STATUS, 0xF);
				continue;
			}
			i = 0;
		}

		addr += ES928X_FW_BLOCK_SIZE;
		regmap_write(es928x->regmap, ES928X_REG_MCU_FW_STATUS, 0);
		while(!fw_status)
		{
			msleep_interruptible(5);
			regmap_read(es928x->regmap, ES928X_REG_MCU_FW_STATUS, &fw_status);
			i++;
			if (i > 1000) {
				dev_err(es928x->dev, "%s: MCU FW status check failed exceed 5 secs. \n", __func__);
				return;
			}
		}

		if(fw_status & ES928X_FW_BLOCK_VERIFICTION_FAIL)
		{
			dev_err(es928x->dev, "Memory Write Failed. Memory did not respond.\n");
			return;
		}

		if(fw_status & ES928X_FW_BLOCK_READBACK_FAIL)
		{
			dev_err(es928x->dev, "Memory Verification Failed.\n");
			return;
		}
		//dev_err(es928x->dev, "delay %d x 5ms = %u ms for this block write.\n", i, i*5);
		i=0;
	}

	dev_info(es928x->dev, "FW update complete.\n");
}

static void es928x_power_on(struct es928x_priv *es928x)
{
	dev_err(es928x->dev, "%s ", __func__);
	gpio_set_value_cansleep(es928x->gpio_3p3, 1);
	msleep(1);
	gpio_set_value_cansleep(es928x->gpio_1p8, 1);
	msleep(5);
}

static void es928x_shut_down(struct es928x_priv *es928x)
{
	dev_err(es928x->dev, "%s ", __func__);
	es928x_hw_reset(es928x, 0);
	msleep(10);
	gpio_set_value_cansleep(es928x->gpio_1p8, 0);
	gpio_set_value_cansleep(es928x->gpio_3p3, 0);
	msleep(10);
}

static int es928x_restart_and_check_fw(const struct firmware *fw, struct es928x_priv *es928x)
{
	int retry = FW_RETRY_TIME;
	struct es928x_fw es928x_fw;
	u8 build[2];
	int ret=0;

	dev_err(es928x->dev, "%s:\n", __func__);

	if (!fw) {
		dev_err(es928x->dev, "%s not found. Default FW used!\n", fw_name);
		return -EINVAL;
	}

	if (*fw->data != 0x45){
		dev_err(es928x->dev, "%s firmware not valid.\n", fw_name);
		return -EINVAL;
	}

	while (retry--) {
		msleep_interruptible(10);
		es928x_shut_down(es928x);
		msleep_interruptible(10);
		es928x_power_on(es928x);
		es928x_hw_reset(es928x, 1);
		dev_err(es928x->dev, "%s: delay 1 sec and load fw\n", __func__);
		msleep(1000);
		regmap_read(es928x->regmap, ES928X_REG_MCU_FW_VERSION_MAJOR, &es928x_fw.major);
		regmap_read(es928x->regmap, ES928X_REG_MCU_FW_VERSION_MINOR, &es928x_fw.minor);
		regmap_bulk_read(es928x->regmap, ES928X_REG_MCU_FW_VERSION_LSB_BUILD, build, 2);
		es928x_fw.build = build[0] | (build[1] << 8);
		dev_info(es928x->dev, "%s: HW:%d.%d Build: %04x (%d), FW: %d.%d Build: %02x%02x\n",
		__func__, es928x_fw.major, es928x_fw.minor, es928x_fw.build, es928x_fw.build,
		fw->data[5], fw->data[4], fw->data[11], fw->data[10]);

		if(es928x_fw.major == (unsigned int) fw->data[5] &&
			es928x_fw.minor == (unsigned int) fw->data[4] &&
			build[0] == fw->data[10] && build[1] == fw->data[11]){
			dev_info(es928x->dev, "%s: FW Versions match.\n", __func__);
			return 0;
		}
		dev_err(es928x->dev, "%s: FW versions is not match , retry fw update\n", __func__);
		if( !(es928x_fw.major | es928x_fw.minor | es928x_fw.build)) {//First Boot
			ret = es928x_fw_load_new(fw, es928x);
			if (ret)
				return -EINVAL;
		}

		es928x_fw_load_update(fw, es928x);
	}

	return -EINVAL;
}

static void es928x_fw_load_version(const struct firmware *fw, void *context)
{
	struct es928x_priv *es928x = context;
	struct es928x_fw es928x_fw;
	int init_status = 0;
	u8 build[2];

	if (!fw) {
		dev_err(es928x->dev, "Firmware file not found. Default FW used!\n");
		return;
	}

	if (*fw->data != 0x45){
		dev_err(es928x->dev, "Firwmare not valid or corrupted.\n");
		return;
	}

	mutex_lock(&es928x->lock);
	regmap_read(es928x->regmap, ES928X_REG_MCU_FW_VERSION_MAJOR, &es928x_fw.major);
	regmap_read(es928x->regmap, ES928X_REG_MCU_FW_VERSION_MINOR, &es928x_fw.minor);
	regmap_bulk_read(es928x->regmap, ES928X_REG_MCU_FW_VERSION_LSB_BUILD, build, 2);

	es928x_fw.build = build[0] | (build[1] << 8);
	dev_info(es928x->dev, "HW:%d.%d Build: %04x (%d), FW: %d.%d Build: %02x%02x\n",
		es928x_fw.major, es928x_fw.minor, es928x_fw.build, es928x_fw.build,
		fw->data[5], fw->data[4], fw->data[11], fw->data[10]);

	if(es928x_fw.major == (unsigned int) fw->data[5] &&
			es928x_fw.minor == (unsigned int) fw->data[4] &&
			build[0] == fw->data[10] &&
			build[1] == fw->data[11]){
		dev_info(es928x->dev, "FW Versions match.\n");
		goto exit;
	}

	if( !(es928x_fw.major | es928x_fw.minor | es928x_fw.build)) //First Boot
	{
		dev_info(es928x->dev, "fw update to RAM begin\n");
		init_status = es928x_fw_load_new(fw, es928x);
		dev_info(es928x->dev, "fw update to RAM end\n");
	}

	es928x->fw = &es928x_fw;

	if(!init_status)
	{
		dev_info(es928x->dev, "fw update to flash begin\n");
		es928x_fw_load_update(fw, es928x);
		dev_info(es928x->dev, "fw update to flash end\n");
		init_status = es928x_restart_and_check_fw(fw, es928x);
		if(init_status) {
			dev_info(es928x->dev, "es928x_restart_and_check_fw fail init_status(%d) \n", init_status);
			goto exit;
		}
	} else {
		dev_err(es928x->dev, "es928x_fw_load_new failed with error: %d\n", init_status);
		goto exit;
	}

exit:
	//es928x_hw_reset(es928x, 0);
	//msleep_interruptible(1);
	//es928x_hw_reset(es928x, 1);

	//msleep_interruptible(250);

	release_firmware(fw);
	mutex_unlock(&es928x->lock);


	es928x_fw_build = es928x_fw.build;
}

static void es928x_fw_get_version_work_fn(struct work_struct *work)
{
	struct es928x_priv *es928x =
		container_of(work, struct es928x_priv, firmware_work);
	int ret, i;
	int boot_flag = 0, rev_id = 0;
	int id = 0;
	es928x_hw_reset(es928x, 1);
	dev_err(es928x->dev, "pull rst pin high and load fw\n");
	//msleep(1000); //SS This is not required.
	dev_info(es928x->dev, "MODULE_VERSION 1.9.6 \n");
	for(i=0;i<7;i++) {
		msleep(100);
		regmap_read(es928x->regmap, ES928X_REG_RB_CHIP_ID_RB, &id);
		dev_err(es928x->dev, "%s: chip id(%x)\n", __func__, id);
		if (id == 0x0b)
			break;
	}
	if (i == 7)
		return;

	for(i=0;i<7;i++) {
		regmap_read(es928x->regmap, ES928X_REG_MCU_INTERRUPT_FLAGS, &boot_flag);
		dev_err(es928x->dev, "%s: boot flag id(%x)\n", __func__, boot_flag);
		if ((boot_flag & ES928X_BOOT_FINISHED_FLAG) == ES928X_BOOT_FINISHED_FLAG)
			break;
		msleep(100);
	}
	if(i == 7) {
		dev_warn(es928x->dev, "%s: id read retry timeout id(%x), es928x is not booting as expected. Forcing firwmare load now...\n", __func__, boot_flag);
		//return 0;  // This will occur when EEPROM is empty on first load, or update to 0459
	}

	regmap_read(es928x->regmap, ES928X_REG_REV_ID_RB, &rev_id);

	if(!rev_id) //VL Version
	{
		dev_info(es928x->dev, "Detected ES9280C version. Loading %s\n", fw_name_v);
		ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
			fw_name_v, es928x->dev, GFP_KERNEL, es928x, es928x_fw_load_version);
	}
	else
	{
		dev_info(es928x->dev, "Detected ES9280AC version. Loading %s\n", fw_name);
		ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
			fw_name, es928x->dev, GFP_KERNEL, es928x, es928x_fw_load_version);
	}

	if (ret)
	{
		dev_err(es928x->dev, "firmware request failed. %d\n", ret);
		es928x_hw_reset(es928x, 0);
	}
}

/* Jack Detect Routines */

/*
static void es928x_btn_work_fn(struct work_struct *work)
{
	struct es928x_jdet_priv *es928x_jdet =
		container_of(work, struct es928x_jdet_priv, btn_work);
	struct snd_soc_component *component = es928x_jdet->component;
	unsigned int btn_result = 0;

	pr_info("%s: Enter\n", __func__);
	btn_result = snd_soc_component_read32(component, ES928X_REG_MCU_BUTTON_STATUS);
		pr_info("%s: Reporting long button press event, btn_result: %d\n",
			 __func__, btn_result);
		snd_soc_jack_report(es928x_jdet->jack,
				btn_result << 8, 0xF000);

}
*/
static void es928x_zdet_work_fn(struct work_struct *work)
{
	struct es928x_jdet_priv *es928x_jdet;
	struct delayed_work *dwork;
	struct snd_soc_component *component;
	struct es928x_priv *es928x;
	u8 zdet_val[3];
	unsigned int zdet_status = 0, cnt=15;
	char *load_types[7] = {"LOAD_UNKNOWN", "LOAD_8", "LOAD_16", "LOAD_32", "LOAD_64", "LOAD_300", "LOAD_100K"};
	int left_load, right_load;

	dwork = to_delayed_work(work);
	es928x_jdet = container_of(dwork, struct es928x_jdet_priv, zdet_work);
	if(!es928x_jdet)
		return;
	if(!es928x_jdet->component)
		return;
	component = es928x_jdet->component;
	es928x = snd_soc_component_get_drvdata(component);


	while(!(zdet_status & ES928X_ZMEAS_STATUS))
	{
		msleep_interruptible(100);
		if(!es928x_jdet->jack_inserted)
		{
			dev_info(es928x->dev, "%s - jack was removed before impedance was measured!\n",__func__);
			return;
		}
		regmap_read(es928x->regmap, ES928X_REG_MCU_INTERRUPT_FLAGS, &zdet_status);
		if(!cnt--)
		{
			dev_err(es928x->dev, "Impedance detection failed. Impedance values unreliable!\n");
			return;
		}
	}

	regmap_bulk_read(es928x->regmap, ES928X_REG_MCU_LOAD_MIC_STATUS, zdet_val, 3);

	switch (zdet_val[1])
	{
		case 117 ... 127:
			left_load = 2;
			break;
		case 106 ... 116:
			left_load = 3;
			break;
		case 82 ... 105:
			left_load = 4;
			break;
		case 51 ... 81:
			left_load = 5;
			break;
		case 0 ... 50:
			left_load = 6;
			break;
		default:
			left_load = 0;
			break;
	}

	switch (zdet_val[2])
	{
		case 117 ... 127:
			right_load = 2;
			break;
		case 106 ... 116:
			right_load = 3;
			break;
		case 82 ... 105:
			right_load = 4;
			break;
		case 51 ... 81:
			right_load = 5;
			break;
		case 0 ... 50:
			right_load = 6;
			break;
		default:
			right_load = 0;
			break;
	}
	dev_info(es928x->dev, "Impedance Measured with codes 0x%x, 0x%x. Loads: %s, %s (fw: %s)\n",
		zdet_val[1], zdet_val[2],
		load_types[left_load], load_types[right_load],
		load_types[(int)(zdet_val[0] >> 4)]);
}

/*
static void es928x_refresh_work_fn(struct work_struct *work)
{
	struct es928x_jdet_priv *es928x_jdet =
		container_of(work, struct es928x_jdet_priv, refresh_work);
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(es928x_jdet->component);
	int ret = 0;

	ret = snd_soc_dapm_sync(dapm);
	pr_debug("%s with %d\n",__func__, ret);
}
*/
static void es928x_poweroff_work_fn(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct es928x_jdet_priv *es928x_jdet =
		container_of(dwork, struct es928x_jdet_priv, poweroff_work);
	struct snd_soc_component *component;
	struct es928x_priv *es928x;
	unsigned int dac_en;
	int ret;

	if(!es928x_jdet)
		return;
	component = es928x_jdet->component;
	es928x = snd_soc_component_get_drvdata(component);

	dev_info(es928x->dev,"%s jack_inserted(%d) rst_gpio(%d)\n",
	__func__, (int)es928x_jdet->jack_inserted, (int)gpio_get_value(es928x->rst_gpio));
	if(!es928x_jdet->jack_inserted && gpio_get_value(es928x->rst_gpio))
	{
		ret = regmap_read(es928x->regmap, ES928X_REG_MCU_MASTER_CTRL, &dac_en);

		if(!(dac_en & (ES928X_DAC_EN_MASK|ES928X_ADC_EN_MASK)) || ret < 0)
		{
			dev_info(es928x->dev,"%s power off with dac_en=%u\n",__func__, dac_en);
			es928x_hw_reset(es928x, 0);
		} else
			dev_info(es928x->dev,"%s power off fail with dac_en=%x\n",__func__, dac_en);
	}
}

static void es928x_jdet_work_fn(struct work_struct *work)
{
	struct es928x_jdet_priv *es928x_jdet = container_of(work, struct es928x_jdet_priv, jdet_work);
	struct es928x_priv *es928x;
	struct snd_soc_component *component;
	unsigned int flags;
	u8 button;
	//u8 impedance_dac_codes[2];
	int report = 0, mask = 0, ret, i=0;

	if(!es928x_jdet)
		return;
	component = es928x_jdet->component;
	es928x = snd_soc_component_get_drvdata(es928x_jdet->component);

	ret = regmap_read(es928x->regmap, ES928X_REG_MCU_INTERRUPT_FLAGS, &flags);
	while(!(flags & 0x80))
	{
		dev_err(es928x->dev, "flag retry %d.\n", flags);
		if(!flags)
		{
			udelay(2000);
		}
		if(i++ > 20)
		{
			dev_err(es928x->dev, "flag error %d\n", flags);
			break;
		}
		ret = regmap_read(es928x->regmap, ES928X_REG_MCU_INTERRUPT_FLAGS, &flags);
	}

	dev_err(es928x->dev, "jdet_work with status: %x\n", flags);

	if(!es928x_jdet->jack_inserted  && (flags & ES928X_JACK_IN))
	{
		//New Jack Insertion
		es928x_jdet->jack_inserted = true;
		cancel_delayed_work(&es928x_jdet->poweroff_work);
		es928x_hw_reset(es928x, 1);
		es928x_jdet->zmeas_done = false;
		dev_err(es928x->dev, "New Jack Insertion,\n");

		mask |= SND_JACK_MECHANICAL;
		report |= SND_JACK_MECHANICAL;
		if(!es928x_jdet->debug_mode)
			snd_soc_jack_report(es928x_jdet->jack, report, mask);
		else
			dev_err(es928x->dev, "%s skip report jack insert/remove event in debug mode\n", __func__);

	} else if((es928x_jdet->jack_inserted && !(flags & ES928X_JACK_IN)) ||
		ret < 0)
	{
		//Jack Removal
		es928x_jdet->jack_inserted = false;
		es928x_jdet->zmeas_done = false;
		report = 0;
		mask = ES928X_JDET_REPORT_ALL_MASK;
		es928x_jdet->current_plug = ES928X_PLUG_TYPE_NONE;
		dev_info(es928x->dev, "Jack Removed,\n");
		if(!es928x_jdet->debug_mode)
			snd_soc_jack_report(es928x_jdet->jack, report, mask);
		else
			dev_err(es928x->dev, "%s skip report jack insert/remove event in debug mode\n", __func__);
		schedule_delayed_work(&es928x_jdet->poweroff_work, msecs_to_jiffies(2000));
		return;
	}

	if(!es928x_jdet->zmeas_done && (flags & ES928X_HP_TYPE_STATUS))
	{
		//Read Z-Imp Registers to recover impedance
		//schedule_work(&es928x_jdet->zdet_work);
		schedule_delayed_work(&es928x_jdet->zdet_work, msecs_to_jiffies(100));

		mask |= SND_JACK_HEADSET;
		report |= SND_JACK_HEADSET;

		if (flags & ES928X_MIC_IN_STATUS)
		{
			if(es928x_jdet->current_plug == ES928X_PLUG_TYPE_HEADPHONE && !es928x_jdet->debug_mode)
				snd_soc_jack_report(es928x_jdet->jack, 0, ES928X_JDET_REPORT_ALL_MASK); //Remove first
			es928x_jdet->current_plug = ES928X_PLUG_TYPE_HEADSET;
			dev_info(es928x->dev, "4-Pole jack detected.\n");

		}
		else
		{
			es928x_jdet->current_plug = ES928X_PLUG_TYPE_HEADPHONE;
			dev_info(es928x->dev, "3-Pole jack detected.\n");
			report &= ~SND_JACK_MICROPHONE;

		}
		if(!es928x_jdet->debug_mode)
			snd_soc_jack_report(es928x_jdet->jack, report, mask);
		else
			dev_err(es928x->dev, "%s skip report jack insert/remove event in debug mode\n", __func__);
		es928x_jdet->zmeas_done = true;
		//schedule_work(&es928x_jdet->refresh_work);

	}else if(!(flags & ES928X_HP_TYPE_STATUS))
	{
		es928x_jdet->zmeas_done = false; //Indicates a new mic detection. For slow inserts
	}
	else if((flags & (ES928X_BUTTON_PRESS | ES928X_BUTTON_RELEASE)) && es928x_jdet->zmeas_done)
	{
		button = snd_soc_component_read32(component, ES928X_REG_MCU_BUTTON_STATUS);
		if(flags & ES928X_BUTTON_PRESS){
			dev_info(es928x->dev, "Button: %d Pressed\n",button);
			mask |= (button << 8);
			report |= (button << 8);
			if(!es928x_jdet->debug_mode)
				snd_soc_jack_report(es928x_jdet->jack, report, mask);
			else
				dev_err(es928x->dev, "%s skip report jack insert/remove event in debug mode\n", __func__);
		}
		if(flags & ES928X_BUTTON_RELEASE)
		{
			dev_info(es928x->dev, "Button: %d Released\n",button);
			mask |= (button << 8);
			report &= ~(button << 8);
			if(!es928x_jdet->debug_mode)
				snd_soc_jack_report(es928x_jdet->jack, report, mask);
			else
				dev_err(es928x->dev, "%s skip report jack insert/remove event in debug mode\n", __func__);
		}
		snd_soc_component_write(component, ES928X_REG_MCU_BUTTON_STATUS, button);
	}
}

void es928x_jdet_jack_det(struct snd_soc_component *component, struct snd_soc_jack *jack)
{
	struct es928x_priv *es928x = snd_soc_component_get_drvdata(component);
	struct es928x_jdet_priv *es928x_jdet;
	unsigned boot_flag = 0, wait = 0;
	int ret;

	dev_info(es928x->dev, "%s\n", __func__);

	es928x_jdet = es928x->jdet;


	es928x_jdet->jack = jack;
	es928x_jdet->jack_inserted = false;

	/* Send an initial empty report */
	snd_soc_jack_report(jack, 0, ES928X_JDET_REPORT_ALL_MASK);

	es928x_hw_reset(es928x, 1);
	msleep(20);
	ret = regmap_read(es928x->regmap, ES928X_REG_MCU_INTERRUPT_FLAGS, &boot_flag);
	while(((boot_flag & ES928X_BOOT_FINISHED_FLAG) != ES928X_BOOT_FINISHED_FLAG) || ret)
	{
		msleep(100);
		if(wait++ > 7)
		{
				dev_err(es928x->dev, "%s cannot get boot flag. Firwmare will be updated.\n", __func__);
				schedule_work(&es928x->firmware_work);
				return;
		}
		ret = regmap_read(es928x->regmap, ES928X_REG_MCU_INTERRUPT_FLAGS, &boot_flag);
	}
	mutex_lock(&es928x->lock);
	if(!wait)
	{
		//regmap_write(es928x->regmap, ES928X_REG_MCU_MASTER_CTRL, ES928X_JDET_RRUN_MASK);
		regmap_update_bits(es928x->regmap, ES928X_REG_IO_GPIO_13_14_CFG,
			ES928X_GPIO13_CFG_MASK, 0x07);
	}
	regmap_update_bits(es928x->regmap, ES928X_REG_IO_GPIO_13_14_CFG,
			ES928X_GPIO13_CFG_MASK, 0x0F);

	schedule_delayed_work(&es928x_jdet->poweroff_work, msecs_to_jiffies(100));
	mutex_unlock(&es928x->lock);

}
EXPORT_SYMBOL_GPL(es928x_jdet_jack_det);



static irqreturn_t es928x_jdet_irq_thread(int irq, void *data)
{
	struct es928x_jdet_priv *es928x_jdet = data;
	struct es928x_priv *es928x = snd_soc_component_get_drvdata(es928x_jdet->component);
	struct snd_soc_component *component = es928x_jdet->component;

	schedule_work(&es928x_jdet->jdet_work);

	mutex_lock(&es928x->lock);
	snd_soc_component_update_bits(component, ES928X_REG_IO_GPIO_13_14_CFG,
		ES928X_GPIO13_CFG_MASK, 0x07);
	mutex_unlock(&es928x->lock);


	return IRQ_HANDLED;

}

int insure_debug_write() {
	struct es928x_jdet_priv *es928x_jdet;
	pr_err("[es928x] insure_debug_write: enter\n");

	if (g_es928x->jdet == NULL || strlen(g_es928x->audio_debug) == 0)
		return 1;

	es928x_jdet = g_es928x->jdet;

	if (strncmp(g_es928x->audio_debug, "1", 1) == 0) {
		pr_err("[es928x] debug mode disable irq(%d)\n", es928x_jdet->irq);
		gpio_set_value_cansleep(g_es928x->gpio_audio_debug, 0); /* enable uart log, disable audio */
		es928x_jdet->debug_mode = true;
	} else if (strncmp(g_es928x->audio_debug, "0", 1) == 0) {
		pr_err("[es928x] headset mode enable irq(%d)\n", es928x_jdet->irq);
		gpio_set_value_cansleep(g_es928x->gpio_audio_debug, 1);	/* disable uart log, enable audio */
		es928x_jdet->debug_mode = false;
	}
	return 0;
}

int es928x_jdet_init(struct snd_soc_component *component)
{
	struct es928x_priv *es928x = snd_soc_component_get_drvdata(component);
	struct es928x_jdet_priv *es928x_jdet;
	struct i2c_client *i2c = to_i2c_client(component->dev);
	int ret = 0;

	es928x_jdet = devm_kzalloc(es928x->dev, sizeof(*es928x_jdet), GFP_KERNEL);
	if (!es928x_jdet)
		return -ENOMEM;

	dev_err(es928x->dev, "%s, i2c->irq: %d\n", __func__, i2c->irq);
	es928x_jdet->irq = i2c->irq;
	es928x_jdet->current_plug = ES928X_PLUG_TYPE_NONE;
	es928x_jdet->debug_mode = true;

	INIT_WORK(&es928x_jdet->jdet_work, es928x_jdet_work_fn);
	INIT_DELAYED_WORK(&es928x_jdet->zdet_work, es928x_zdet_work_fn);
	INIT_DELAYED_WORK(&es928x_jdet->poweroff_work, es928x_poweroff_work_fn);

	dev_info(es928x->dev, "%s\n", __func__);
	ret = request_threaded_irq(es928x_jdet->irq, NULL,
				   es928x_jdet_irq_thread,
				    IRQF_TRIGGER_RISING | IRQF_ONESHOT, //| IRQF_NO_SUSPEND,
				   "es928x-jdet", es928x_jdet);
	if (ret) {
		dev_err(es928x->dev, "Failed to request IRQ: %d\n", ret);
		return ret;
	} else{
		ret = enable_irq_wake(es928x_jdet->irq);
		if(ret) {
			dev_err(es928x->dev, "Failed to set wake enable IRQ %d\n", es928x_jdet->irq);
			return ret;
		}
	}
	es928x_jdet->component = component;
	es928x->jdet = es928x_jdet;
	ret = insure_debug_write(); //ASUS BSP Jackson +++
	if(ret)
		dev_err(es928x->dev, "%s insure_debug_write: jdet not init or property not set.\n", __func__);
	return 0;
}
EXPORT_SYMBOL_GPL(es928x_jdet_init);

void es928x_jdet_exit(struct snd_soc_component *component)
{
	struct es928x_priv *es928x = snd_soc_component_get_drvdata(component);
	struct es928x_jdet_priv *es928x_jdet = es928x->jdet;

	free_irq(es928x_jdet->irq, es928x_jdet);
	//devm_kfree(&es928x->jdet);
}
EXPORT_SYMBOL_GPL(es928x_jdet_exit);

static int es928x_nco_config(struct es928x_priv *es928x, unsigned int rate, unsigned int clock_frequency)
{
	unsigned long long dpll_p;
	u8 nco_num[4];
	int div = 0;

	while((rate * 128 << div) < clock_frequency &&
			div < 4 &&
			(clock_frequency >> div) > ES928X_CLOCK_MIN_FREQ)
		div++;

	if (!div)
		return -EINVAL;

	dpll_p = ((unsigned long long)rate << (--div))  << 32;
	do_div(dpll_p, clock_frequency);
	nco_num[0] = dpll_p & 0xFF;
	nco_num[1] = (dpll_p >> 8) & 0xFF;
	nco_num[2] = (dpll_p >> 16) & 0xFF;
	nco_num[3] = (dpll_p >> 24) & 0xFF;

	regmap_bulk_write(es928x->regmap, ES928X_REG_DAC_FORCE_DPLL_NUM, nco_num, 4);
	return div;
}

static void es928x_nco_disable(struct es928x_priv *es928x)
{
	u8 clear[] = {0,0,0,0};
	regmap_bulk_write(es928x->regmap, ES928X_REG_DAC_FORCE_DPLL_NUM, clear, 4);
}

static int es928x_pll_config(struct es928x_priv *es928x, unsigned int freq_in, unsigned int freq_request)
{
	int i;
	dev_err(es928x->dev, "%s freq_in(%u) freq_request(%u)\n", __func__, freq_in, freq_request);

	for(i = 0; i < ARRAY_SIZE(es928x_pll_div); i++)
	{
		if(freq_request == es928x_pll_div[i].Fout && freq_in == es928x_pll_div[i].Fin)
		{
			regmap_update_bits(es928x->regmap,ES928X_REG_MCU_MASTER_CTRL,
				ES928X_PLL_EN_MASK,ES928X_PLL_EN_MASK);
			udelay(500);
			regmap_update_bits(es928x->regmap, ES928X_REG_PLL_VCO_CTRL,
				ES928X_CP_PDB_MASK|ES928X_VCO_PDB_MASK,
				~(ES928X_CP_PDB_MASK|ES928X_VCO_PDB_MASK));
			regmap_write(es928x->regmap, ES928X_REG_PLL_CFG_DIV_FB,
				es928x_pll_div[i].Nf);
			regmap_update_bits(es928x->regmap,
				ES928X_REG_PLL_CFG_DIV_IN_1,
				ES928X_CLK_IN_DIV_LSB_MASK,
				(es928x_pll_div[i].Ni & 0x0F) << ES928X_CLK_IN_DIV_LSB_SHIFT);
			regmap_update_bits(es928x->regmap,
				ES928X_REG_PLL_CFG_DIV_IN_2,
				ES928X_CLK_IN_DIV_MSB_MASK,
				((es928x_pll_div[i].Ni >> 4) & 0x0F) << ES928X_CLK_IN_DIV_MSB_SHIFT);
			regmap_write(es928x->regmap,
				ES928X_REG_PLL_CFG_DIV_OUT_1, es928x_pll_div[i].No);
			regmap_update_bits(es928x->regmap, ES928X_REG_PLL_VCO_CTRL,
				ES928X_CP_PDB_MASK|ES928X_VCO_PDB_MASK,
				ES928X_CP_PDB_MASK|ES928X_VCO_PDB_MASK);
			return es928x_pll_div[i].Fout;
		}
	}
	dev_err(es928x->dev, "%s no pll configuration found for input %u and output %u\n",__func__, freq_in, freq_request);
	return -EINVAL;

}

static int es928x_startup(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *dai)
{
	struct es928x_priv *es928x = snd_soc_component_get_drvdata(dai->component);
	dev_err(es928x->dev, "%s, rate: %d, sample_bits: %d, name: %s\n",
		__func__, dai->rate, dai->sample_bits, dai->name);
	if(substream->runtime)
	{
		dev_dbg(es928x->dev, "%s runtime info: rate: %d, frame_bits: %d, sample_bits: %d\n",
			__func__, substream->runtime->rate, substream->runtime->frame_bits, substream->runtime->sample_bits);
	}

	return 0;
}

static void es928x_shutdown(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *dai)
{
	struct es928x_priv *es928x = snd_soc_component_get_drvdata(dai->component);
	struct es928x_jdet_priv *es928x_jdet = es928x->jdet;
	unsigned int status;

	dev_info(es928x->dev, "%s, name: %s, jack inserted: %d\n",
		__func__, dai->name, es928x_jdet->jack_inserted);
	if(!es928x_jdet->jack_inserted)
	{
		regmap_read(es928x->regmap, ES928X_REG_MCU_MASTER_CTRL, &status);
		if(!(status & (ES928X_DAC_EN_MASK | ES928X_ADC_EN_MASK))) //Wait until both playback and recording paths are finished.
		{
			es928x_hw_reset(es928x, 0);
		}
	}
}

static int es928x_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	int ret = 0, div = 1, clk_gear=0, clk_div=0, clk = 0;
	struct snd_soc_component *component = dai->component;
	struct es928x_priv *es928x = snd_soc_component_get_drvdata(component);
	unsigned int rate = params_rate(params);
	unsigned int rate_128 = rate *128;

	mutex_lock(&es928x->lock); //Austin+++

	if(!substream)
	{
		dev_err(es928x->dev, "%s snd_pcm_substream=NULL. Sample rates not set\n", __func__);
		mutex_unlock(&es928x->lock); //Austin+++
		return ret;
	}

	dev_err(es928x->dev, "%s with stream: %d rx_i2s_master: %d\n", __func__, substream->stream, es928x->rx_i2s_master);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
	{
		if(es928x->rx_i2s_master) //ES928x is I2S Master
		{
			//regmap_update_bits(es928x->regmap, ES928X_REG_DAC_CLK_GEAR,
			//	ES928X_DAC_AUTO_CLK_GEAR_EN_MASK,
			//	~ES928X_DAC_AUTO_CLK_GEAR_EN_MASK); //Disable Automatic clock gearing
			for(clk = XOSC; clk < CLK_END; clk++)
			{
				dev_err(es928x->dev,"clock: %d. rate: %u. clock: %d, mod: %d\n",
					clk, rate, es928x->clocks[clk], es928x->clocks[clk] % rate_128);
				if(es928x->clocks[clk] && !(es928x->clocks[clk] % rate_128))
					break;
			}

			if(clk < CLK_END)
			{
				for(div = 0; div < 7; div++)
				{
					if(rate_128 << div == es928x->clocks[clk])
					{
						if(div > clk_gear)
							clk_div = div-clk_gear;
						else
							clk_div = 0;

						regmap_write(es928x->regmap, ES928X_REG_DAC_CLK_GEAR,
							clk_gear << ES928X_DAC_CLK_GEAR_SHIFT);
						regmap_update_bits(es928x->regmap, ES928X_REG_DAC_MASTER_MODE_CFG,
							ES928X_DAC_MASTER_MODE_DIV_MASK,
							clk_div << ES928X_DAC_MASTER_MODE_DIV_SHIFT);
						regmap_update_bits(es928x->regmap, ES928X_REG_DAC_SYNC_SET,
							ES928X_DAC_SYNC_MODE_MASK, ES928X_DAC_SYNC_MODE_MASK);
						es928x_nco_disable(es928x);
						break;
					}
					if (es928x->clocks[clk] >> (clk_gear + 1) > ES928X_CLOCK_MIN_FREQ &&
							clk_gear < 3)
						clk_gear++;
				}
			}

			if(clk == CLK_END || div >= 7) //No exact clock rate match. Use NCO mode to generate I2S
			{
				clk = XOSC; //Only XTAL can be NCO source
				clk_gear = es928x_nco_config(es928x, rate, es928x->clocks[clk]);
				if (clk_gear < 0)
				{
					dev_err(es928x->dev, "MCLK/FS (%u, %u)combination not supported.\n", rate, es928x->clocks[clk]);
					mutex_unlock(&es928x->lock); //Austin+++
					return clk_gear;
				}
				regmap_write(es928x->regmap, ES928X_REG_DAC_CLK_GEAR,
							clk_gear << ES928X_DAC_CLK_GEAR_SHIFT);
				regmap_update_bits(es928x->regmap, ES928X_REG_DAC_SYNC_SET,
					ES928X_DAC_SYNC_MODE_MASK, ~ES928X_DAC_SYNC_MODE_MASK);
			}

			//regmap_update_bits(es928x->regmap, ES928X_REG_DAC_MASTER_MODE_CFG,
			//		ES928X_DAC_MASTER_MODE_ENABLE_MASK, ES928X_DAC_MASTER_MODE_ENABLE_MASK);
			regmap_update_bits(es928x->regmap, ES928X_REG_SYS_CLK_CTRL_SIGS,
				ES928X_SEL_DAC_CLKIN_MASK, clk << ES928X_SEL_DAC_CLKIN_SHIFT);
		}
		else{
			dev_err(es928x->dev, "%s with stream: %d ES928x is not I2S Master\n", __func__, substream->stream);
			regmap_update_bits(es928x->regmap, ES928X_REG_DAC_MASTER_MODE_CFG,
					ES928X_DAC_MASTER_MODE_ENABLE_MASK, ~ES928X_DAC_MASTER_MODE_ENABLE_MASK);
			regmap_update_bits(es928x->regmap, ES928X_REG_DAC_CLK_GEAR,
				ES928X_DAC_AUTO_CLK_GEAR_EN_MASK,
				ES928X_DAC_AUTO_CLK_GEAR_EN_MASK);

			for(clk = XOSC; clk < CLK_END; clk++)
			{
				if (es928x->clocks[clk] > rate_128)
				{
					while((rate_128 << clk_gear) < es928x->clocks[clk] && es928x->clocks[clk] >> clk_gear > ES928X_CLOCK_MIN_FREQ)
						clk_gear++;
					regmap_update_bits(es928x->regmap, ES928X_REG_DAC_CLK_GEAR,
						ES928X_DAC_MAX_CLK_GEAR_MASK,
						(--clk_gear) << ES928X_DAC_MAX_CLK_GEAR_SHIFT);
					regmap_update_bits(es928x->regmap, ES928X_REG_SYS_CLK_CTRL_SIGS,
						ES928X_SEL_DAC_CLKIN_MASK, clk << ES928X_SEL_DAC_CLKIN_SHIFT);
					break;
				}
			}
			if(clk == CLK_END)
			{
				dev_err(es928x->dev, "Sample rate %u not supported.\n", rate);
				mutex_unlock(&es928x->lock); //Austin+++
				return -EINVAL;
			}
		}
		regmap_update_bits(es928x->regmap, ES928X_REG_SYS_CLK_CTRL_SIGS,
			ES928X_ENB_DAC_CLKIN_MASK, ~ES928X_ENB_DAC_CLKIN_MASK);
	}
	else //Capture Mode
	{
		if(es928x->tx_i2s_master)
		{
			dev_err(es928x->dev, "capture rate (%u)\n", rate);

			if(rate < 32000 || rate > 96000)
			{
				dev_err(es928x->dev, "Sample rate %u not supported.\n", rate);
				mutex_unlock(&es928x->lock); //Austin+++
				return -EINVAL;
			}

			for(clk = XOSC; clk < CLK_END; clk++)
			{
				if(es928x->clocks[clk] && !(es928x->clocks[clk] % (rate*128)))
					break;
			}

			if(clk == CLK_END) //No exact clock rate match. Use PLL to generate clock
			{
				clk = XOSC; //Only XTAL can be NCO source
				dev_err(es928x->dev, "No exact clock rate match. Use PLL to generate clock\n");
				switch(rate){
					case 44100:
					case 48000:
						div = 4;
						break;
					case 88200:
					case 96000:
						div = 2;
						break;
					default:
						dev_err(es928x->dev, "%s sample rate %d not supported by codec.\n", __func__, rate);
						mutex_unlock(&es928x->lock); //Austin+++
						return -EINVAL;
				}

				ret = es928x_pll_config(es928x, es928x->clocks[clk], rate * div * 128);
				if(ret > 0)
				{
					clk = PLL;
				}
				else
				{
					dev_err(es928x->dev, "PLL configuration failed. (%d)\n", ret);
					mutex_unlock(&es928x->lock); //Austin+++
					return -EINVAL;
				}
			}
			else
			{
				dev_err(es928x->dev, "es928x->clocks[clk] (%u).\n", es928x->clocks[clk]);
				for(div = 0; div < 4; div++)
				{
					if(rate * (128 << div) == es928x->clocks[clk])
					{
						break;
					}
					else if(div == 3)
					{
						dev_err(es928x->dev, "Invalid sample rate %u\n",rate);
						mutex_unlock(&es928x->lock); //Austin+++
						return -EINVAL;
					}
				}
			}
		}
		else
		{
			dev_err(es928x->dev, "I2S Capture is only supported in master mode.\n");
		}
		regmap_update_bits(es928x->regmap, ES928X_REG_SYS_CLK_CTRL_SIGS,
			ES928X_ENB_ADC_CLKIN_MASK, ~ES928X_ENB_ADC_CLKIN_MASK);
		regmap_update_bits(es928x->regmap, ES928X_REG_ADC_CLK_SEL_ADC_EN,
			ES928X_SEL_ADC_CLK_DIV_MASK,
			div << ES928X_SEL_ADC_CLK_DIV_SHIFT);
		regmap_update_bits(es928x->regmap, ES928X_REG_ADC_CLK_SEL_ADC_EN,
			ES928X_SEL_ADC_CLKIN_MASK, clk << ES928X_SEL_ADC_CLKIN_SHIFT);


	}

	mutex_unlock(&es928x->lock); //Austin+++
	return 0;
}

static int es928x_trigger(struct snd_pcm_substream *substream, int cmd,
			   struct snd_soc_dai *dai)
{

	dev_err(g_es928x->dev, "%s cmd(%d)\n", __func__, cmd);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		break;
	default: break;
	}

	return 0;
}

static int es928x_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct es928x_priv *es928x = snd_soc_component_get_drvdata(dai->component);
	dev_dbg(es928x->dev, "%s format: %d\n", __func__, fmt);

	return 0;
}

static const struct snd_soc_dai_ops es928x_dai_ops = {
	.startup	= es928x_startup,
	.shutdown	= es928x_shutdown,
	.hw_params	= es928x_hw_params,
	.trigger	= es928x_trigger,
	.set_fmt	= es928x_set_fmt,
};


static struct snd_soc_dai_driver es928x_i2s_dai[] = {
	{
		.name = "es928x_i2s_rx",
		.playback = {
		     .stream_name = "ES928x Playback",
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = SNDRV_PCM_RATE_8000_384000,
		     .formats = ES928X_FORMATS,
		},
		.ops = &es928x_dai_ops,
	},
	{
		.name = "es928x_i2s_tx",
		.capture = {
			.stream_name = "ES928x Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000,
			.formats = ES928X_FORMATS,
		},
		.ops = &es928x_dai_ops,
	},
};


static int es928x_component_driver_probe(struct snd_soc_component *component)
{
	struct es928x_priv *es928x = snd_soc_component_get_drvdata(component);
	int ret = 0;
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(component);
	dev_info(es928x->dev, "%s\n", __func__);
	es928x->component = component;
	ret = es928x_jdet_init(es928x->component);

	snd_soc_dapm_enable_pin(dapm, "OUT");
	snd_soc_dapm_enable_pin(dapm, "MIC");


	snd_soc_dapm_ignore_suspend(dapm, "ES928x Playback");
	snd_soc_dapm_ignore_suspend(dapm, "ES928x Capture");
	snd_soc_dapm_ignore_suspend(dapm, "Supply");
	snd_soc_dapm_ignore_suspend(dapm, "AIFINL");
	snd_soc_dapm_ignore_suspend(dapm, "AIFINR");
	snd_soc_dapm_ignore_suspend(dapm, "DAC");
	snd_soc_dapm_ignore_suspend(dapm, "OUT");
	snd_soc_dapm_ignore_suspend(dapm, "MIC");
	snd_soc_dapm_ignore_suspend(dapm, "ADC Mic");
	snd_soc_dapm_ignore_suspend(dapm, "AIFOUTL");
	snd_soc_dapm_ignore_suspend(dapm, "AIFOUTR");
	snd_soc_dapm_ignore_suspend(dapm, "MIC BIAS");


	snd_soc_dapm_sync(dapm);
	return 0;
}

static void es928x_component_driver_remove(struct snd_soc_component *component)
{
	struct es928x_priv *es928x = snd_soc_component_get_drvdata(component);
	dev_info(es928x->dev, "%s\n", __func__);
}

static int es928x_component_driver_suspend(struct snd_soc_component *component)
{
	struct es928x_priv *es928x = snd_soc_component_get_drvdata(component);
	dev_info(es928x->dev, "%s\n", __func__);

	return 0;
}

static int es928x_component_driver_resume(struct snd_soc_component *component)
{
	struct es928x_priv *es928x = snd_soc_component_get_drvdata(component);
	dev_err(es928x->dev, "%s\n", __func__);
	//regmap_update_bits(es928x->regmap, ES928X_REG_IO_GPIO_13_14_CFG,
	//		ES928X_GPIO13_CFG_MASK, 0x07);

	//regmap_update_bits(es928x->regmap, ES928X_REG_IO_GPIO_13_14_CFG,
	//		ES928X_GPIO13_CFG_MASK, 0x0F);

	return 0;
}


static struct snd_soc_component_driver es928x_component_driver = {
	.probe 				= es928x_component_driver_probe,
	.remove 			= es928x_component_driver_remove,
	.suspend			= es928x_component_driver_suspend,
	.resume				= es928x_component_driver_resume,
	.name				= "es928x_codec",
	.controls			= es928x_controls,
	.num_controls		= ARRAY_SIZE(es928x_controls),
	.dapm_widgets		= es928x_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(es928x_dapm_widgets),
	.dapm_routes		= es928x_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(es928x_dapm_routes),
};

#ifdef ES928X_CHAR_DEV_DRV
#define ES928X_CDEV_MAX_LENGTH 256
#define ES928X_CDEV_NAME     "ess_debug"
static struct cdev *es928x_cdev;
static struct es928x_priv *g_es928x_cdev;
struct class *es928x_cdev_class;
static int es928x_cdev_major;
static int es928x_cdev_opened;

static int es928x_cdev_open(struct inode *in, struct file *filp)
{
	struct es928x_priv *p_es928x = g_es928x_cdev;

	if (es928x_cdev_opened) {
		dev_info(p_es928x->dev, "%s device is already opened\n", "es928x_cdev");
		return -EINVAL;
	}
	filp->private_data = (void *)p_es928x;
	es928x_cdev_opened++;
	return 0;
}

static int es928x_cdev_release(struct inode *in, struct file *filp)
{
	filp->private_data = NULL;
	es928x_cdev_opened--;
	return 0;
}

static ssize_t es928x_cdev_read(struct file *filp, char __user *buf,
    size_t count, loff_t *offset)
{
    struct es928x_priv *p_es928x = (struct es928x_priv *)filp->private_data;
    char reg_addr;
    char read_value[ES928X_CDEV_MAX_LENGTH + 1];
    unsigned int value = 0;
    ssize_t ret = 0;

    ret = copy_from_user(&reg_addr, buf, 1);
	if (ret != 0) {
		dev_err(p_es928x->dev, "read: copy_from_user failure\n");
		return -EINVAL;
	}

    if(count == 1)
    {
        ret = regmap_read(p_es928x->regmap, reg_addr, &value);
        if (ret >= 0)
        {
            read_value[0] = value;
        }
    }
    else if(count > 1 && count <= ES928X_CDEV_MAX_LENGTH)
    {
        ret = regmap_bulk_read(p_es928x->regmap, reg_addr, read_value, count);
    }
    else
    {
        dev_err(p_es928x->dev, "Max %d bytes can be read at one time.\n", ES928X_CDEV_MAX_LENGTH);
        return -EINVAL;
    }

    if (ret < 0)
    {
        dev_err(p_es928x->dev, "%s, ret=%d, count=%zu error happened!\n",
			__func__, (int)ret, count);
        return -EIO;
    }

    if (copy_to_user(buf, read_value, count) != 0) {
		dev_err(p_es928x->dev, "copy_to_user failed\n");
		return -EINVAL;
	}

    ret = count;
    return ret;
}

static ssize_t es928x_cdev_write(struct file *filp, const char __user *buf,
	size_t count, loff_t *offset)
{
    struct es928x_priv *p_es928x = (struct es928x_priv *)filp->private_data;
    static char write_data[ES928X_CDEV_MAX_LENGTH + 1];
    static unsigned int reg_addr;
    int ret = 0;

    if (count > ES928X_CDEV_MAX_LENGTH)
    {
        dev_err(p_es928x->dev, "Max %d bytes can be written at one time.\n", ES928X_CDEV_MAX_LENGTH);
        return -EINVAL;
    }

    ret = copy_from_user(write_data, buf, count);
    if (ret != 0)
    {
		dev_err(p_es928x->dev, "write: copy_from_user failure\n");
		return -EINVAL;
	}

    reg_addr = write_data[0];
    if(count == 2)
    {
        ret = regmap_write(p_es928x->regmap, reg_addr, write_data[1]);
    }
    else if (count > 2)
    {
        ret = regmap_bulk_write(p_es928x->regmap, reg_addr, &write_data[1], count - 1);
    }

    if (ret < 0)
    {
        dev_err(p_es928x->dev, "%s, %d, ret=%d, count=%zu, ERROR Happen\n", __func__,
            __LINE__, ret, count);
            return -EIO;
    }

    return count;
}

static long es928x_cdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct es928x_priv *p_es928x = (struct es928x_priv *)filp->private_data;

    dev_info(p_es928x->dev, "%s, cmd=0x%x\n", __func__, cmd);

    return 0;
}

static const struct file_operations es928x_cdev_fops = {
	.owner = THIS_MODULE,
	.open = es928x_cdev_open,
	.release = es928x_cdev_release,
	.read = es928x_cdev_read,
	.write = es928x_cdev_write,
	.unlocked_ioctl = es928x_cdev_ioctl,
};

int es928x_cdev_init(void *dev_priv)
{
    int ret = 0;
    dev_t dev = MKDEV(es928x_cdev_major, 0);

    g_es928x_cdev = (struct es928x_priv *)dev_priv;
    dev_dbg(g_es928x_cdev->dev, "Initializing es928x_cdev debug driver.\n");

	ret = alloc_chrdev_region(&dev, 0, 1, ES928X_CDEV_NAME);
    if (ret < 0)
    {
        dev_err(g_es928x_cdev->dev, "Could not allocate device major number: %d.\n",
            es928x_cdev_major);
        return ret;
    }

    es928x_cdev_class = class_create(THIS_MODULE, ES928X_CDEV_NAME);
    es928x_cdev_major = MAJOR(dev);
    dev_info(g_es928x_cdev->dev, "Allocated major number:%d\n",
        es928x_cdev_major);

    es928x_cdev = cdev_alloc();
    cdev_init(es928x_cdev, &es928x_cdev_fops);
    es928x_cdev->owner = THIS_MODULE;
    es928x_cdev->ops = &es928x_cdev_fops;

    if (device_create(es928x_cdev_class, NULL, dev, NULL, ES928X_CDEV_NAME) == NULL)
		dev_err(g_es928x_cdev->dev, "Device creation failed\n");

	if (cdev_add(es928x_cdev, dev, 1) < 0) {
		dev_err(g_es928x_cdev->dev, "es928x_cdev_driver: cdev_add failed\n");
		unregister_chrdev_region(dev, 1);
		es928x_cdev = NULL;
		return 1;
	}
	dev_info(g_es928x_cdev->dev, "Registered ESS Character Device Driver driver, Major number:%d\n",
        es928x_cdev_major);

    return ret;
}
#endif

static ssize_t es928x_reg_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	return 1;
}

static ssize_t es928x_reg_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct es928x_jdet_priv *es928x_jdet = g_es928x->jdet;
	struct es928x_priv *es928x = g_es928x;
	int reg_addr, read_dump_value, len = 0;
	mutex_lock(&es928x->lock);
	if (!(es928x_jdet->jack_inserted))
		es928x_hw_reset(es928x, 1);
	for (reg_addr = 0; reg_addr <= ES928X_REG_MAX; reg_addr++)
	{
		regmap_read(es928x->regmap, reg_addr, &read_dump_value);
		len += snprintf(buf+len, ES928X_SYSFS_ENTRY_MAX_LEN - len, "Reg %d:  %d\n", reg_addr, read_dump_value);
	}
	if (!(es928x_jdet->jack_inserted))
		es928x_hw_reset(es928x, 0);
	mutex_unlock(&es928x->lock);
	return len;
}

static DEVICE_ATTR(es928x_reg, 0644, es928x_reg_show, es928x_reg_store);

static struct attribute *es928x_attributes[] = {
	&dev_attr_es928x_reg.attr,
	NULL
};

static struct attribute_group es928x_attribute_group = {
	.attrs = es928x_attributes
};

static void read_dump(struct es928x_priv *es928x)
{
	struct es928x_jdet_priv *es928x_jdet = es928x->jdet;
	int reg_addr, read_dump_value = 0;
	mutex_lock(&es928x->lock);
	//If headset not detected, pull high reset pin
	if (!(es928x_jdet->jack_inserted)){
		es928x_hw_reset(es928x, 1);
		msleep_interruptible(1000);
	}
	for (reg_addr = 0; reg_addr <= ES928X_REG_MAX; reg_addr++)
	{
		regmap_read(es928x->regmap, reg_addr, &read_dump_value);
		dev_info(es928x->dev, "Reg %d:  %d\n", reg_addr, read_dump_value);
	}
	if (!(es928x_jdet->jack_inserted))
		es928x_hw_reset(es928x, 0);
	mutex_unlock(&es928x->lock);
}

static ssize_t ess_debug_write(struct file *filp, const char __user *buff, size_t len, loff_t *off)
{
	char messages[256];
	struct es928x_jdet_priv *es928x_jdet;

	memset(messages, 0, sizeof(messages));
	if (len > 256)
		len = 256;
	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	memset(g_es928x->audio_debug, 0, sizeof(g_es928x->audio_debug));
	strcpy(g_es928x->audio_debug, messages);

	if (g_es928x->jdet == NULL)
		return len;

	es928x_jdet = g_es928x->jdet;

	if (strncmp(messages, "1", 1) == 0) {
		pr_err("[es928x] debug mode disable irq(%d)\n", es928x_jdet->irq);
		gpio_set_value_cansleep(g_es928x->gpio_audio_debug, 0); /* enable uart log, disable audio */
		if (es928x_jdet->jack_inserted)
			snd_soc_jack_report(es928x_jdet->jack, 0, ES928X_JDET_REPORT_ALL_MASK);
		es928x_jdet->debug_mode = true;
	} else if (strncmp(messages, "0", 1) == 0) {
		pr_err("[es928x] headset mode enable irq(%d)\n", es928x_jdet->irq);
		gpio_set_value_cansleep(g_es928x->gpio_audio_debug, 1);	/* disable uart log, enable audio */
		if (es928x_jdet->jack_inserted) {
			snd_soc_jack_report(es928x_jdet->jack, SND_JACK_MECHANICAL, SND_JACK_MECHANICAL);
			if (es928x_jdet->current_plug == ES928X_PLUG_TYPE_HEADSET)
				snd_soc_jack_report(es928x_jdet->jack, ES928X_SND_JACK_HEADSET, ES928X_SND_JACK_HEADSET);
			else if (es928x_jdet->current_plug == ES928X_PLUG_TYPE_HEADPHONE)
				snd_soc_jack_report(es928x_jdet->jack, ES928X_SND_JACK_HEADPHONE, ES928X_SND_JACK_HEADPHONE);
		}
		es928x_jdet->debug_mode = false;
	} else if (strncmp(messages, "2", 1) == 0) {
		pr_err("es928x force reload FW\n");
		schedule_work(&g_es928x->firmware_work);
		//ret = es928x_fw_get_version(g_es928x);
	} else if (strncmp(messages, "3", 1) == 0) {
		dev_err(g_es928x->dev, "hs virtual insertion \n");
		//snd_soc_jack_report(es928x_jdet->jack, SND_JACK_MECHANICAL, SND_JACK_MECHANICAL);
		snd_soc_jack_report(es928x_jdet->jack, SND_JACK_MECHANICAL|ES928X_SND_JACK_HEADSET, SND_JACK_MECHANICAL|ES928X_SND_JACK_HEADSET);
	} else if (strncmp(messages, "4", 1) == 0) {
		dev_err(g_es928x->dev, "hs virtual remove\n");
		snd_soc_jack_report(es928x_jdet->jack, 0, ES928X_JDET_REPORT_ALL_MASK);
	} else if (strncmp(messages, "dump",4) == 0) {
		//read register values
		read_dump(g_es928x);
	} else if (strncmp(messages, "5",1) == 0) {
		es928x_hw_reset(g_es928x, 0);
	} else if (strncmp(messages, "fw",1) == 0) {
		audbg_read_ctrl = 1;
	} else if (strncmp(messages, "driver",1) == 0) {
		audbg_read_ctrl = 2;
	} else
		pr_err("es928x proc write out of case\n");

	return len;
}

static ssize_t ess_debug_read(struct file *filp, char __user *buff, size_t len, loff_t *off)
{
	char messages[256];
	struct es928x_jdet_priv *es928x_jdet = g_es928x->jdet;

	if (*off)
		return 0;

	memset(messages, 0, sizeof(messages));
	if (len > 256)
		len = 256;
	if (audbg_read_ctrl == 0) {
		if (es928x_jdet->debug_mode)
			sprintf(messages, "audio debug mode\n");
		else {
			switch (es928x_jdet->current_plug) {
			case ES928X_PLUG_TYPE_HEADSET:
				sprintf(messages, "1\n");
				break;
			case ES928X_PLUG_TYPE_HEADPHONE:
				sprintf(messages, "2\n");
				break;
			default:
				sprintf(messages, "0\n");
				break;
			}
		}
	} else if (audbg_read_ctrl == 1) {
		sprintf(messages, "%04x\n",es928x_fw_build);
		audbg_read_ctrl = 0;
	} else if (audbg_read_ctrl == 2) {
		sprintf(messages, "1.9.6\n");
		audbg_read_ctrl = 0;
	}

	if(copy_to_user(buff, messages, len))
		return -EFAULT;

	(*off)++;
	return len;
}

static struct file_operations ess_debug_ops = {
	.write = ess_debug_write,
	.read = ess_debug_read,
};

static int es928x_i2c_probe(struct i2c_client *i2c,
			 const struct i2c_device_id *id)
{
	struct device *dev = &i2c->dev;
	struct es928x_priv *es928x;
	struct device_node *np = i2c->dev.of_node;
	int check,ret;

	es928x = devm_kzalloc(dev, sizeof(struct es928x_priv), GFP_KERNEL);
	if (!es928x)
		return -ENOMEM;

	es928x->regmap = devm_regmap_init_i2c(i2c, &es928x_regmap);
	if (IS_ERR(es928x->regmap)) {
		return PTR_ERR(es928x->regmap);
	}

	i2c_set_clientdata(i2c, es928x);
	es928x->dev = dev;
	//Austin+++
    es928x->gpio_3p3 = of_get_named_gpio(np, "ess,es928x-3p3-en", 0);
    dev_err(es928x->dev, "ess,es928x-3p3-en (%d) \n",es928x->gpio_3p3);
	if (!gpio_is_valid(es928x->gpio_3p3)) {
		dev_err(es928x->dev, "Looking up es928x->gpio_3p3 is not valid failed\n");
	} else
	{
		ret = devm_gpio_request_one(&i2c->dev, es928x->gpio_3p3,
			GPIOF_DIR_OUT, "es928x_3p3_en");
		if(ret)
		{
			dev_err(es928x->dev, "es928x_3p3_en cannot be requested\n");
			return ret;
		}
	}

	es928x->gpio_1p8 = of_get_named_gpio(np, "ess,es928x-1p8-en", 0);
	dev_err(es928x->dev, "ess,es928x-1p8-en (%d) \n",es928x->gpio_1p8);
	if (!gpio_is_valid(es928x->gpio_1p8)) {
		dev_err(es928x->dev, "Looking up es928x->gpio_1p8 is not valid failed\n");
		ret = -EINVAL;
	} else
	{
		ret = devm_gpio_request_one(&i2c->dev, es928x->gpio_1p8,
			GPIOF_DIR_OUT, "es928x_1p8_en");
		if(ret)
		{
			dev_err(es928x->dev, "es928x_1p8_en cannot be requested, %d\n",es928x->gpio_1p8);
			return ret;
		}
	}
	es928x_shut_down(es928x);
	es928x_power_on(es928x);

	es928x->rst_gpio = of_get_named_gpio(np, "ess,es928x-rst", 0);
	if (!gpio_is_valid(es928x->rst_gpio)) {
		dev_err(es928x->dev, "Looking up %s property in node %s failed %d\n",
			"ess,es8018-rst-gpio", np->full_name, es928x->rst_gpio);
		ret = -EINVAL;
	} else
	{
		ret = devm_gpio_request_one(&i2c->dev, es928x->rst_gpio,
			GPIOF_DIR_OUT, "es928x_rst");
		if(ret)
		{
			dev_err(es928x->dev, "es928x_rst (%d) cannot be requested, %d\n",es928x->rst_gpio, ret);
			return ret;
		}
	}

	es928x->rx_i2s_master = of_property_read_bool(np, "ess,es928x-rx-i2s-master");
	es928x->tx_i2s_master = of_property_read_bool(np, "ess,es928x-tx-i2s-master");

	dev_info(es928x->dev, "%s: rx_i2s_master %d tx_i2s_master %d\n", __func__, es928x->rx_i2s_master, es928x->tx_i2s_master);

	ret = of_property_read_u32(np, "ess,es928x-xosc-freq",
				&(es928x->clocks[XOSC]));
	if (ret)
	{
		dev_dbg(es928x->dev, "%s: missing %s in dt node\n", __func__,
			"ess,es928x-xosc-freq");
		es928x->clocks[XOSC] = 0;
	}


	ret = of_property_read_u32(np, "ess,es928x-aclk1-freq",
				&(es928x->clocks[ACLK1]));
	if (ret)
	{
		dev_dbg(es928x->dev, "%s: missing %s in dt node\n", __func__,
			"ess,es928x-aclk1-freq");
		es928x->clocks[ACLK1] = 0;
	}


	ret = of_property_read_u32(np, "ess,es928x-aclk2-freq",
				&(es928x->clocks[ACLK2]));
	if (ret)
	{
		dev_dbg(es928x->dev, "%s: missing %s in dt node\n", __func__,
			"ess,es928x-aclk2-freq");
		es928x->clocks[ACLK2] = 0;
	}

	mutex_init(&es928x->lock);

	INIT_WORK(&es928x->firmware_work, es928x_fw_get_version_work_fn);
	// Check for updated FW
	schedule_work(&es928x->firmware_work);
	g_es928x = es928x;

	proc_create(ESS_DEBUG, 0777, NULL, &ess_debug_ops);//Austin +++

	//Allocate ASP Memory
	es928x->dac_asp = devm_kzalloc(dev, sizeof(struct es928x_asp), GFP_KERNEL);
	if (!es928x->dac_asp)
	{
		return -ENOMEM;
	}


	/* Register with soc framework */
	ret = snd_soc_register_component(dev, &es928x_component_driver,
						es928x_i2s_dai,
						ARRAY_SIZE(es928x_i2s_dai));
	if (ret) {
		dev_err(dev, "%s: Codec registration failed.\n",
		 __func__);
		return ret;
	}

	#ifdef ES928X_CHAR_DEV_DRV
	es928x_cdev_init(es928x);
	#endif

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	es928x->gpio_audio_debug = of_get_named_gpio(np, "ess,es928x-audio_debug", 0);
	dev_err(es928x->dev, "ess,es928x-audio_debug (%d) \n",es928x->gpio_audio_debug);
	if (!gpio_is_valid(es928x->gpio_audio_debug)) {
		dev_err(es928x->dev, "Looking up es928x->gpio_audio_debug is not valid failed\n");
		ret = -EINVAL;
	} else
	{
		ret = devm_gpio_request_one(&i2c->dev, es928x->gpio_audio_debug,
			GPIOF_DIR_OUT, "es928x_audio_debug");
		if(ret)
		{
			dev_err(es928x->dev, "es928x_audio_debug cannot be requested, %d\n",es928x->gpio_1p8);
			return ret;
		}
	}

	check = sysfs_create_group(&i2c->dev.kobj, &es928x_attribute_group); //Jackson +++

	return ret;
}

static int es928x_i2c_remove(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;

	snd_soc_unregister_component(&i2c->dev);
	pm_runtime_disable(dev);
	es928x_shut_down(g_es928x);

	return 0;
}

static const struct i2c_device_id es928x_i2c_id[] = {
	{ "es928x_codec", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, es928x_i2c_id);

static const struct of_device_id es928x_of_match[] = {
	{ .compatible = "ess,es928x_codec", },
	{ }
};
MODULE_DEVICE_TABLE(of, es928x_of_match);

static struct i2c_driver es928x_i2c_driver = {
	.probe	        = es928x_i2c_probe,
	.remove         = es928x_i2c_remove,
	.id_table       = es928x_i2c_id,
	.driver         = {
		.name           = "es928x_codec",
		.of_match_table = es928x_of_match,
		//.pm             = &es928x_pm_ops,
	},
};

module_i2c_driver(es928x_i2c_driver);

MODULE_DESCRIPTION("ES928x ASoC Codec Driver");
MODULE_AUTHOR("Samuel Schaefer <samuel.schaefer@esstech.com>");
MODULE_VERSION("1.9.6");
MODULE_LICENSE("GPL");
