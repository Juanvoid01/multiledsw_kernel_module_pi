# Multiledsw kernel module

Kernel module driver for controlling led and button devices on a raspberry pi.



#### key-points :

* User interaction via /proc/modcontrol

* User - Device interaction via /dev/ledsw_xxx

* SMP-safe implementation

* Use of spin_lock_irqsave in an interrupt enviroment.
 
* Linux Device Model : use of Misc devices
