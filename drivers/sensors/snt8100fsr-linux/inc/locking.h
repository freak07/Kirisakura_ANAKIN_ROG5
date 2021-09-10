#include <linux/delay.h>
//#include <thread.h> 
//#include <synch.h> 

#ifndef LOCKING_H
#define LOCKING_H

#define MUTEX_LOCK(X) while (1 != mutex_trylock(X)) {PRINT_DEBUG("lock failed"); msleep(1);}
//#define MUTEX_LOCK(X) while (0 != mutex_lock_interruptible(X)) {PRINT_INFO("lock failed");}
//#define MUTEX_LOCK(X) mutex_lock(X)

#endif
