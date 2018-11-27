#ifndef DRIVERS_I2C_H_
#define DRIVERS_I2C_H_

#include <util/dlist.h>
#include <framework/mod/options.h>
#include <config/embox/driver/i2c.h>

#define I2C_BUS_MAX   \
	OPTION_MODULE_GET(embox__driver__i2c, NUMBER, i2c_bus_max)

#define MAX_I2C_BUS_NAME    8

struct i2c_bus {
	int id;
	char name[MAX_I2C_BUS_NAME];

	void *adapter_priv;

	struct dlist_head i2c_bus_list;
};

extern int i2c_bus_register(void *adapter_priv, int id, const char *bus_name);

extern int i2c_bus_unregister(int bus_id);

extern struct i2c_bus *i2c_bus_get(int id);

#endif
