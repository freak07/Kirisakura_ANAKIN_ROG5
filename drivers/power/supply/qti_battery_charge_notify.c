#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/notifier.h>
#include <linux/power_supply.h>
#include <linux/mutex.h>

static BLOCKING_NOTIFIER_HEAD(qti_charge_notifier_list);
/**
 * qti_charge_register_notify - register a notifier callback whenever a qti_charge change happens
 * @nb: pointer to the notifier block for the callback events.
 *
 * These changes are either USB devices or busses being added or removed.
 */
void qti_charge_register_notify(struct notifier_block *nb)
{
	blocking_notifier_chain_register(&qti_charge_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(qti_charge_register_notify);

/**
 * qti_charge_unregister_notify - unregister a notifier callback
 * @nb: pointer to the notifier block for the callback events.
 *
 * usb_register_notify() must have been previously called for this function
 * to work properly.
 */
void qti_charge_unregister_notify(struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&qti_charge_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(qti_charge_unregister_notify);


void qti_charge_notify_device_charge(void)
{
	blocking_notifier_call_chain(&qti_charge_notifier_list, QTI_POWER_SUPPLY_CHARGED, NULL);
}
EXPORT_SYMBOL(qti_charge_notify_device_charge);

void qti_charge_notify_device_not_charge(void)
{
	blocking_notifier_call_chain(&qti_charge_notifier_list,	QTI_POWER_SUPPLY_UNCHARGED, NULL);
}
EXPORT_SYMBOL(qti_charge_notify_device_not_charge);
