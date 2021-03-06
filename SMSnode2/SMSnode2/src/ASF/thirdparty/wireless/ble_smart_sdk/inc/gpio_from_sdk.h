#ifndef GPIO_FROM_SDK_H_INCLUDED
#define GPIO_FROM_SDK_H_INCLUDED

#include <stdint.h>


/**
 * \brief SAMB11 available GPIO's.
 *
 * List of GPIO's available.
 */
enum {
	LPGPIO_0	= 0,
	LPGPIO_1,
	LPGPIO_2,
	LPGPIO_3,
	LPGPIO_4,
	LPGPIO_5,
	LPGPIO_6,
	LPGPIO_7,
	LPGPIO_8,
	LPGPIO_9,
	LPGPIO_10,
	LPGPIO_11,
	LPGPIO_12,
	LPGPIO_13,
	LPGPIO_14,
	LPGPIO_15,
	LPGPIO_16,
	LPGPIO_17,
	LPGPIO_18,
	LPGPIO_19,
	LPGPIO_20,
#ifdef CHIPVERSION_B0
	LPGPIO_21,
	LPGPIO_22,
	LPGPIO_23,
	LPGPIO_24,
#endif
	LPGPIO_MAX,
};	

#ifdef CHIPVERSION_B0
	#define GPIO0_COMBINED_VECTOR_TABLE_INDEX		39
	#define GPIO1_COMBINED_VECTOR_TABLE_INDEX		40
	#define GPIO2_COMBINED_VECTOR_TABLE_INDEX		41
#else
	#define GPIO0_COMBINED_VECTOR_TABLE_INDEX		23
	#define GPIO1_COMBINED_VECTOR_TABLE_INDEX		24
#endif	//CHIPVERSION_B0

// PINMUM VALUES for GPIO
#define PINMUX_VAL_0			0			// Default GPIO functionality
#define PINMUX_VAL_1			1			// Megamux
#define PINMUX_VAL_2			2
#define PINMUX_VAL_3			3
#define PINMUX_VAL_4			4
#define PINMUX_VAL_5			5

typedef union {
	uint16_t val;
	struct {
		uint8_t muxval : 8;
		uint8_t pinnum : 8;
	} bit_pinumx;
} pinumx;

typedef union {
	uint16_t port_info;
	struct {
		uint16_t gpio_num:8;
		uint16_t available:1;
		uint16_t configured:1;
		uint16_t pack:6;
	}bit;
}port;
extern port port_list[LPGPIO_MAX];

#define PINNUM_OFFSET		8

#endif	//GPIO_H_INCLUDED

