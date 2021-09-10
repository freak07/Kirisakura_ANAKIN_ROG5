/*
 * Copyright (c) 2020, ASUS. All rights reserved.
 * Copy from drm_sysfs.c
 */

#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT
#include <linux/device.h>
#include <drm/drm_anakin.h>
#include <linux/delay.h>

static int g_hdr = 0;
int ghbm_on_requested;
int ghbm_on_achieved;
int fod_spot_ui_ready;
int fod_gesture_touched;
extern struct class *drm_class;
extern struct kobject* asus_class_get_kobj(struct class *cls);

static ssize_t hdr_mode_show(struct class *class,
					struct class_attribute *attr,
					char *buf)
{
	return sprintf(buf, "%d\n", g_hdr);
}

static ssize_t hdr_mode_store(struct class *class,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	if (!count)
		return -EINVAL;

	sscanf(buf, "%d", &g_hdr);

	return count;
}
static CLASS_ATTR_RW(hdr_mode);

// FOD feature
void anakin_drm_notify(int var, int value)
{
	int *selected_var = NULL;
	char *selected_var_name;

	switch (var) {
	case ASUS_NOTIFY_GHBM_ON_REQ:
		selected_var = &ghbm_on_requested;
		selected_var_name = "ghbm_on_requested";
		break;
	case ASUS_NOTIFY_GHBM_ON_READY:
		selected_var = &ghbm_on_achieved;
		selected_var_name = "ghbm_on_achieved";
		break;
	case ASUS_NOTIFY_SPOT_READY:
		selected_var = &fod_spot_ui_ready;
		selected_var_name = "spot_on_achieved";
		break;
	case ASUS_NOTIFY_FOD_TOUCHED:
		selected_var = &fod_gesture_touched;
		selected_var_name = "fod_touched";
		break;
	default:
        pr_err("[Display] unsupported drm notify variable type %d\n", var);
		return;
	}

	if (!selected_var) {
		pr_err("[Display] var is null\n");
		return;
	}

	if (*selected_var == value) {
		pr_err("[Display] value same, variable type %d, value %d\n", var, *selected_var);
	} else {
		pr_err("[Display] update variable type %d from %d to %d\n", var, *selected_var, value);
		*selected_var = value;
		sysfs_notify(asus_class_get_kobj(drm_class), NULL, selected_var_name);
	}
}
EXPORT_SYMBOL(anakin_drm_notify);

static ssize_t ghbm_on_requested_show(struct class *class,
					struct class_attribute *attr,
					char *buf)
{
	return sprintf(buf, "%d\n", ghbm_on_requested);
}

static ssize_t ghbm_on_achieved_show(struct class *class,
					struct class_attribute *attr,
					char *buf)
{
	return sprintf(buf, "%d\n", ghbm_on_achieved);
}

static ssize_t spot_on_achieved_show(struct class *class,
					struct class_attribute *attr,
					char *buf)
{
	return sprintf(buf, "%d\n", fod_spot_ui_ready);
}

static ssize_t fod_touched_show(struct class *class,
					struct class_attribute *attr,
					char *buf)
{
	return sprintf(buf, "%d\n", fod_gesture_touched);
}

static ssize_t fod_touched_store(struct class *class,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	if (!count)
		return -EINVAL;

	sscanf(buf, "%d", &fod_gesture_touched);

	return count;
}

static CLASS_ATTR_RO(ghbm_on_requested);
static CLASS_ATTR_RO(ghbm_on_achieved);
static CLASS_ATTR_RO(spot_on_achieved);
static CLASS_ATTR_RW(fod_touched);

void drm_anakin_sysfs_destroy(void) {
	class_remove_file(drm_class, &class_attr_hdr_mode);
	class_remove_file(drm_class, &class_attr_ghbm_on_requested);
	class_remove_file(drm_class, &class_attr_ghbm_on_achieved);
	class_remove_file(drm_class, &class_attr_spot_on_achieved);
	class_remove_file(drm_class, &class_attr_fod_touched);
}

int drm_anakin_sysfs_init(void)
{
	int err;

	err = class_create_file(drm_class, &class_attr_hdr_mode);
	if (err) {
		pr_err("[Display] Fail to create hdr_mode file node\n");
		goto error;
	}

	err = class_create_file(drm_class, &class_attr_ghbm_on_requested);
	if (err) {
		pr_err("[Display] Fail to create ghbm_on_requested file node\n");
		goto error;
	}

	err = class_create_file(drm_class, &class_attr_ghbm_on_achieved);
	if (err) {
		pr_err("[Display] Fail to create ghbm_on_achieved file node\n");
		goto error;
	}

	err = class_create_file(drm_class, &class_attr_spot_on_achieved);
	if (err) {
		pr_err("[Display] Fail to create fod_ui_ready file node\n");
		goto error;
	}

	err = class_create_file(drm_class, &class_attr_fod_touched);
	if (err) {
		pr_err("[Display] Fail to create fod_touched file node\n");
		goto error;
	}

	return 0;

error:
	return err;
}
bool is_DSI_mode(int vdisplay, int vtotal)
{
	if( (2448 == vdisplay) && (2463 == vtotal) ) {
		return true;
	}
	return false;
}
bool refreshrate_match(int refresh1, int refresh2)
{
	if( refresh1 == refresh2 ) {
		return true;
	}
	return false;
}
#endif
