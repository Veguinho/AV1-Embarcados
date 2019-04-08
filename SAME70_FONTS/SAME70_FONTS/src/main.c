/*
 * main.c
 *
 * Created: 05/03/2019 18:00:58
 *  Author: eduardo
 */ 

#include <asf.h>
#include "tfont.h"
#include "rtt.h"
#include "rtc.h"
#include "sourcecodepro_28.h"
#include "calibri_36.h"
#include "arial_72.h"

#define YEAR        2018
#define MOUNTH      3
#define DAY         19
#define WEEK        12
#define HOUR        0
#define MINUTE      0
#define SECOND      0

uint32_t hour, minuto, seg;


#define LED_PIO       PIOC
#define LED_PIO_ID    ID_PIOC
#define LED_IDX       8u
#define LED_IDX_MASK  (1u << LED_IDX)

//botao da placa
#define BUT_PIO			PIOA
#define BUT_PIO_ID		10
#define BUT_PIO_IDX		11
#define BUT_PIO_IDX_MASK (1u << BUT_PIO_IDX)

//butao 3 oled
#define EBUT3_PIO PIOC //sei la EXT 3 PC31
#define EBUT3_PIO_ID 12 // piod ID
#define EBUT3_PIO_IDX 31
#define EBUT3_PIO_IDX_MASK (1u << EBUT3_PIO_IDX)

struct ili9488_opt_t g_ili9488_display_opt;

volatile Bool f_rtt_alarme = false;
volatile Bool but_flag;

int counter_pulsos = 0;
int velocidade = 0;
int distancia = 0;
char bufferVelocidade[32];
char bufferDistancia[32];
char bufferHour[32];
char bufferMinutes[32];
char bufferSeconds[32];


/************************************************************************/
/* prototypes                                                           */
/************************************************************************/
void pin_toggle(Pio *pio, uint32_t mask);
void io_init(void);
static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses);
void font_draw_text(tFont *font, const char *text, int x, int y, int spacing);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
void RTC_init(void);

/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/

void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {  }

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, 300);
		
		//calcula a velocidade angular baseado nos pulsos
		if(counter_pulsos != 0){
			velocidade = 2 * 3.1415 * counter_pulsos / 4;
			distancia += 2 * 3.1415 * 0.325 * counter_pulsos;
			
		}
		else{
			velocidade = 0;
		}
		sprintf(bufferVelocidade, "%d", velocidade);
		sprintf(bufferDistancia, "%d", distancia);
		font_draw_text(&sourcecodepro_28, "PROGRAMA", 50, 50, 1);
		font_draw_text(&calibri_36, "VELOCIDADE:", 50, 100, 1);
		font_draw_text(&calibri_36, bufferVelocidade, 50, 150, 2);
		font_draw_text(&calibri_36, "DISTANCIA:", 50, 200, 1);
		font_draw_text(&calibri_36, bufferDistancia, 50, 250, 1);

		counter_pulsos = 0;
		f_rtt_alarme = true;                  // flag RTT alarme
	}
}


void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	uint32_t hour, minute, second;
	/*
	*  Verifica por qual motivo entrou
	*  na interrupcao, se foi por segundo
	*  ou Alarm
	*/
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {

		rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
		
		rtc_get_time(RTC, &hour, &minute, &second);
		
		ili9488_draw_filled_rectangle(50, 300, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
		font_draw_text(&calibri_36, "TEMPO:", 50, 300, 1);
		
	
		sprintf(bufferHour, "%d", hour);
		sprintf(bufferMinutes, "%d", minute);
		sprintf(bufferSeconds, "%d", second);


		font_draw_text(&calibri_36, bufferHour, 50, 350, 1);
		font_draw_text(&calibri_36, ":", 75, 350, 1);
		font_draw_text(&calibri_36, bufferMinutes, 100, 350, 1);
		font_draw_text(&calibri_36, ":", 125, 350, 1);
		font_draw_text(&calibri_36, bufferSeconds, 150, 350, 1);
			
		if(second>=59){
			second = 0;
			minute += 1;
			rtc_set_time_alarm(RTC, 1, hour, 1, minute, 1, second+1);
		}
		if(minute>=60 && second>=59){
			minute = 0;
			hour += 1;
		}
		else{
			rtc_set_time_alarm(RTC, 1, hour, 1, minute, 1, second+1);
		}

	}
	
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
	
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/


void but_callback(void)
{
	but_flag = true;
}

void pulsou(){
	but_flag = false;
}



void io_init(void){
	/* led */
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);
	
	// Inicializa clock do periférico PIO responsavel pelo botao
	pmc_enable_periph_clk(EBUT3_PIO_ID);

	// Configura PIO para lidar com o pino do botão como entrada
	// com pull-up
	pio_configure(EBUT3_PIO, PIO_INPUT, EBUT3_PIO_IDX_MASK, PIO_PULLUP);

	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(EBUT3_PIO,
	EBUT3_PIO_ID,
	EBUT3_PIO_IDX_MASK,
	PIO_IT_RISE_EDGE,
	but_callback);
	
	 // Ativa interrupção
	 pio_enable_interrupt(EBUT3_PIO, EBUT3_PIO_IDX_MASK);

	 // Configura NVIC para receber interrupcoes do PIO do botao
	 // com prioridade 4 (quanto mais próximo de 0 maior)
	 NVIC_EnableIRQ(EBUT3_PIO_ID);
	 NVIC_SetPriority(EBUT3_PIO_ID, 4); // Prioridade 4
}

static float get_time_rtt(){
	uint ul_previous_time = rtt_read_timer_value(RTT);
}

static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
	uint32_t ul_previous_time;

	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	
	/* Configure RTT for a 1 second tick interrupt */
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));
	
	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);


	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
}

void RTC_init(){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(RTC, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(RTC, YEAR, MOUNTH, DAY, WEEK);
	rtc_set_time(RTC, HOUR, MINUTE, SECOND);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(RTC_IRQn);
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_SetPriority(RTC_IRQn, 0);
	NVIC_EnableIRQ(RTC_IRQn);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(RTC,  RTC_IER_ALREN);

}

void configure_lcd(void){
	/* Initialize display parameter */
	g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
	g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
	g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
	g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);

	/* Initialize LCD */
	ili9488_init(&g_ili9488_display_opt);
	ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
	
}


void font_draw_text(tFont *font, const char *text, int x, int y, int spacing) {
	char *p = text;
	while(*p != NULL) {
		char letter = *p;
		int letter_offset = letter - font->start_char;
		if(letter <= font->end_char) {
			tChar *current_char = font->chars + letter_offset;
			ili9488_draw_pixmap(x, y, current_char->image->width, current_char->image->height, current_char->image->data);
			x += current_char->image->width + spacing;
		}
		p++;
	}	
}


int main(void) {
	
	// Desliga watchdog
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	// Inicializa RTT com IRQ no alarme.
	f_rtt_alarme = true;
	RTC_init();
	io_init();
	board_init();
	sysclk_init();	
	configure_lcd();
	
	rtc_set_time_alarm(RTC, 1, HOUR, 1, MINUTE, 1, SECOND+1);
	
	font_draw_text(&sourcecodepro_28, "PROGRAMA", 50, 50, 1);
	font_draw_text(&calibri_36, "VELOCIDADE:", 50, 100, 1);
	//font_draw_text(&calibri_36, "102456", 50, 200, 2);
	font_draw_text(&calibri_36, "DISTANCIA:", 50, 200, 1);
	
	
	
	while(1) {
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		
		if (f_rtt_alarme){
			uint16_t pllPreScale = (int) (((float) 32768) / 2.0);
			uint32_t irqRTTvalue  = 8;
 
	  
			// reinicia RTT para gerar um novo IRQ
			RTT_init(pllPreScale, irqRTTvalue);  
      
			/*
			* caso queira ler o valor atual do RTT, basta usar a funcao
			*   rtt_read_timer_value() */
			f_rtt_alarme = false;
		}
		if (but_flag){
			counter_pulsos++;
			pulsou();
		}
	}
}