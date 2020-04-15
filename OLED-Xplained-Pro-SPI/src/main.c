#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

// Configuracoes do botão 1
#define BUT1_PIO           PIOD
#define BUT1_PIO_ID        ID_PIOD
#define BUT1_PIO_IDX       28
#define BUT1_PIO_IDX_MASK  (1u << BUT1_PIO_IDX)

// Configuracoes do botão 2
#define BUT2_PIO           PIOC
#define BUT2_PIO_ID        ID_PIOC
#define BUT2_PIO_IDX       31
#define BUT2_PIO_IDX_MASK  (1u << BUT2_PIO_IDX)

// Configuracoes do botão 3
#define BUT3_PIO		   PIOA
#define BUT3_PIO_ID		   ID_PIOA
#define BUT3_PIO_IDX	   19
#define BUT3_PIO_IDX_MASK  (1u << BUT3_PIO_IDX)

// LED da placa
#define LED_PLACA_PIO_ID ID_PIOC
#define LED_PLACA_PIO PIOC
#define LED_PLACA_PIN 8
#define LED_PLACA_IDX_MASK (1 << LED_PLACA_PIN)

// Configuracoes do LED 1
#define LED1_PIO           PIOA
#define LED1_PIO_ID        ID_PIOA
#define LED1_PIO_IDX       0
#define LED1_PIO_IDX_MASK  (1u << LED1_PIO_IDX)

// Configuracoes do LED 2
#define LED2_PIO           PIOC
#define LED2_PIO_ID        ID_PIOC
#define LED2_PIO_IDX       30
#define LED2_PIO_IDX_MASK  (1u << LED2_PIO_IDX)

// Configuracoes do LED 3
#define LED3_PIO           PIOB
#define LED3_PIO_ID        ID_PIOB
#define LED3_PIO_IDX       2
#define LED3_PIO_IDX_MASK  (1u << LED3_PIO_IDX)

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/
volatile char button1_flag = 0;
volatile char button2_flag = 0;
volatile char button3_flag = 0;

void init(void);

/************************/
/* Handlers             */
/************************/
void button1_handler(void) {
	button1_flag = 1;
}

void button2_handler(void) {
	button2_flag = 1;
}

void button3_handler(void) {
	button3_flag = 1;
}

/************************************************************************/
/* Funções                                                              */
/************************************************************************/

// Função de inicialização do uC
void init(void) {
	// Initialize the board clock
	sysclk_init();

	// Desativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	// LED PLACA
	pmc_enable_periph_clk(LED_PLACA_PIO_ID);
	pio_configure(LED_PLACA_PIO, PIO_OUTPUT_0, LED_PLACA_IDX_MASK, PIO_DEFAULT);
	pio_set(LED_PLACA_PIO, LED_PLACA_IDX_MASK);
	
	// Ativa PIOs necessários
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);
	pmc_enable_periph_clk(LED1_PIO_ID);
	pmc_enable_periph_clk(LED2_PIO_ID);
	pmc_enable_periph_clk(LED3_PIO_ID);
	
	// Inicializa LEDs como saída
	pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, 0, 0, 0);
	pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK, 0, 0, 0);
	pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK, 0, 0, 0);
	
	//Inicializando os handlers
	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_FALL_EDGE, button1_handler);
	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_IDX_MASK, PIO_IT_FALL_EDGE, button2_handler);
	pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_IDX_MASK, PIO_IT_FALL_EDGE, button3_handler);
	
	//Interrupts
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);
	
	//NVIC
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_EnableIRQ(BUT3_PIO_ID);
}

void pin_toggle(Pio *pio, uint32_t mask) {
	if (pio_get_output_data_status(pio, mask)) {
		pio_clear(pio, mask);
		} else {
		pio_set(pio, mask);
	}
}

int main (void)
{
	board_init();
	sysclk_init();
	delay_init();
	
	//inicializa as flags
	button1_flag = 0;
	button2_flag = 0;
    button3_flag = 0;
	
	int flash_LED1 = 0;
	int flash_LED2 = 0;
	int flash_LED3 = 0;

  // Init OLED
	gfx_mono_ssd1306_init();
  
  // Escreve na tela um circulo e um texto
	gfx_mono_draw_filled_circle(20, 16, 16, GFX_PIXEL_SET, GFX_WHOLE);
    gfx_mono_draw_string("mundo", 50,16, &sysfont);

  /* Insert application code here, after the board has been initialized. */
	while(1) {
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		
		if (button1_flag) {
			flash_LED1 = !flash_LED1;
			button1_flag = 0;
		}
		
		if (button2_flag) {
			flash_LED2 = !flash_LED2;
			button2_flag = 0;
		}
		
		if (button3_flag) {
			flash_LED3 = !flash_LED3;
			button3_flag = 0;
		}
	}
}
