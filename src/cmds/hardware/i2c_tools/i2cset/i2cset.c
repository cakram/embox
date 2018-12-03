/**
 * @file
 *
 * @date Nov 16, 2018
 * @author Anton Bondarev
 */

#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>

#include <drivers/i2c/i2c.h>

static void print_usage(void) {
	printf("Usage: i2cdump -h\n");
	printf("Usage: i2cset [-y] i2cbus chip-address data-address [value] ... [mode]\n");
}

static void print_error(void) {
	printf("Wrong parameters\n");
}

int main(int argc, char **argv) {
	int opt;
	int i = 1;
	long busn = 0;
	long chip_addr = 0;

	while (-1 != (opt = getopt(argc, argv, "hy"))) {
		switch (opt) {
		case 'y':
			break;
		case '?':
		case 'h':
			print_usage();
			return 0;
		default:
			print_error();
			return 0;
		}
	}

	busn = strtol(argv[i++], NULL, 0);
	chip_addr = strtol(argv[i], NULL, 0);
	printf ("bus(%d) chip(%x)\n", (int) busn, (unsigned )chip_addr);
	return 0;
}
