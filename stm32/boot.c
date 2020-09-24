#include <string.h>

static inline void copy_data() {
	extern char _sdata, _edata, _sidata;

	memcpy(&_sdata, &_sidata, &_edata - &_sdata);
}

static inline void clear_bss() {
	extern char _sbss, _ebss;

	memset(&_sbss, 0, &_ebss - &_sbss);
}

void reset() {
	extern void boot();
	extern void __libc_init_array();

	copy_data();
	clear_bss();

	__libc_init_array();

	boot();

	while (1) {
		__asm__ __volatile__("NOP");
	}
}
