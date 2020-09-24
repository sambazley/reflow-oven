#include <stm32f0xx.h>

#define LED_B 0
#define LED_G 1
#define RELAY 3

#define CLK 5
#define MISO 6
#define CS 7

static volatile struct {
	volatile uint8_t OC : 1;
	volatile uint8_t SCG : 1;
	volatile uint8_t SCV : 1;
	volatile uint8_t _1 : 1;
	volatile uint16_t ref_temp : 12;
	volatile uint8_t fault : 1;
	volatile uint8_t _2 : 1;
	volatile uint16_t thermo_temp : 14;
} __attribute__((packed)) thermo;

static void openocd_send_str(const char *str) {
#ifdef DEBUG
	__asm__("mov r0, #0x04;"
			"mov r1, %[msg];"
			"bkpt #0xAB"
			:
			: [msg] "r" (str)
			: "r0", "r1", "memory");
#else
	(void) str;
#endif
}

static void spi_setup() {
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	SPI1->CR1 &= ~SPI_CR1_CPHA;
	SPI1->CR1 |= SPI_CR1_SSI | SPI_CR1_SSM;
	SPI1->CR1 |= SPI_CR1_MSTR;
	SPI1->CR1 |= SPI_CR1_BR_2;
	SPI1->CR2 &= ~SPI_CR2_DS_3;
	SPI1->CR2 |= SPI_CR2_FRXTH;
	SPI1->CR1 |= SPI_CR1_RXONLY;
}

static void timer_setup() {
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;

	TIM14->PSC = 7999;
	TIM14->ARR = 500; //500 ms
	TIM14->CNT = 0;

	TIM14->DIER |= TIM_DIER_UIE;

	TIM14->CR1 |= TIM_CR1_CEN;

	NVIC_EnableIRQ(TIM14_IRQn);
}

static void spi_thermo_recv() {
	GPIOA->ODR &= ~(1 << CS);

	while (SPI1->SR & SPI_SR_RXNE) {
		volatile char x = SPI1->DR;
		(void) x;
	}

	SPI1->CR1 |= SPI_CR1_SPE;

	for (int i = 0; i < 4; i++) {
		if (i == 3) {
			SPI1->CR1 &= ~SPI_CR1_SPE;
		}

		while (!(SPI1->SR & SPI_SR_RXNE)) {
			__NOP();
		}

		((volatile uint8_t *) &thermo)[3 - i] = SPI1->DR;
	}

	GPIOA->ODR |= (1 << CS);

	while (SPI1->SR & SPI_SR_BSY) {
		__NOP();
	}

	while (SPI1->SR & SPI_SR_FRLVL) {
		volatile char x = SPI1->DR;
		(void) x;
	}
}

#define SETUP_TIME 4

void TIM14_Irq() {
	struct TempPoint {
		int end_time, end_temp;
	} temp_data [] = {
		{0, 0},
		{90, 90},
		{180, 130},
		{210, 138},
		{240, 165},
		{270, 138},
		{390, 0}
	}, *prev = 0, *next = 0;

	static int blink = 0;
	static int progress = 0;

	spi_thermo_recv();

	GPIOA->ODR |= (1 << LED_G) | (1 << LED_B);

	int fault = 0;
	if (thermo.OC) {
		fault = 1;
		openocd_send_str("Open circuit\n");
	}
	if (thermo.SCG) {
		fault = 1;
		openocd_send_str("Short circuit to Vcc\n");
	}
	if (thermo.SCV) {
		fault = 1;
		openocd_send_str("Short circuit to GND\n");
	}
	if (thermo._1) {
		fault = 1;
		openocd_send_str("Expected D3 = 0\n");
	}
	if (thermo._2) {
		fault = 1;
		openocd_send_str("Expected D17 = 0\n");
	}

	if (!fault && (thermo.fault || thermo.thermo_temp == 0x1FFF)) {
		fault = 1;
		openocd_send_str("Unknown fault\n");
	}

	if (fault) {
		if (blink ^= 1) {
			GPIOA->ODR &= ~((1 << LED_B) | (1 << LED_G));
		}

		progress = 0;

		GPIOA->ODR &= ~(1 << RELAY);

		goto reset_timer;
	}

	if (progress < SETUP_TIME) {
		progress++;
		blink = 0;

		GPIOA->ODR &= ~((1 << LED_B) | (1 << LED_G));

		goto reset_timer;
	}

	for (unsigned int i = 0; i < sizeof(temp_data) / sizeof(struct TempPoint); i++) {
		if (temp_data[i].end_time * 2 + SETUP_TIME > progress) {
			prev = &temp_data[i - 1];
			next = &temp_data[i];
			break;
		}
	}

	if (!next) {
		GPIOA->ODR &= ~((1 << RELAY) | (1 << LED_G));
		goto reset_timer;
	}

	int time_elapsed = progress - SETUP_TIME - prev->end_time * 2;
	int duration = next->end_time * 2 - prev->end_time * 2;
	int temp_diff = next->end_temp - prev->end_temp;
	int target_temp = prev->end_temp + temp_diff * time_elapsed / duration;

	if ((thermo.thermo_temp >> 2) < target_temp) {
		GPIOA->ODR |= (1 << RELAY);
	} else {
		GPIOA->ODR &= ~(1 << RELAY);
	}

	if (!(blink ^= 1)) {
		GPIOA->ODR &= ~(1 << LED_B);
	}

	progress++;

reset_timer:
	TIM14->SR &= ~TIM_SR_UIF;
	TIM14->CNT = 0;
	TIM14->CR1 |= TIM_CR1_CEN;
}

void boot() {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= (1 << (2 * LED_B))
		| (1 << (2 * LED_G))
		| (1 << (2 * RELAY))
		| (1 << (2 * CS));

	GPIOA->MODER |= (2 << (2 * CLK))
		| (2 << (2 * MISO));

	GPIOA->OTYPER |= (1 << LED_G) | (1 << LED_B); //open drain

	GPIOA->ODR = 0;

	spi_setup();
	timer_setup();

	while (1) {
		__NOP();
	}
}
