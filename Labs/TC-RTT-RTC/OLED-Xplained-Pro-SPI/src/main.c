#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/************************************************************************/
/* defines                                                              */
/************************************************************************/

typedef struct  {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t seccond;
} calendar;

// LEDs

// LED do SAME70
#define LED_PIO PIOC						// periferico que controla o LED
#define LED_PIO_ID ID_PIOC					// ID do periférico PIOC (controla LED)
#define LED_PIN 8							// ID do LED no PIO
#define LED_IDX_MASK (1 << LED_PIN)			// Mascara para CONTROLARMOS o LED

// LED1
#define LED1_PIO PIOA
#define LED1_PIO_ID ID_PIOA
#define LED1_PIN 0
#define LED1_IDX_MASK (1u << LED1_PIN)

// LED2
#define LED2_PIO PIOC
#define LED2_PIO_ID ID_PIOC
#define LED2_PIN 30
#define LED2_IDX_MASK (1u << LED2_PIN)

// LED3
#define LED3_PIO PIOB
#define LED3_PIO_ID ID_PIOB
#define LED3_PIN 2
#define LED3_IDX_MASK (1 << LED3_PIN)

/************************************************************************/
/* VAR globais                                                          */
/************************************************************************/

volatile char flag_tc0 = 0;
volatile char flag_tc1 = 0;
volatile Bool f_rtt_alarme = false;
volatile char flag_rtc = 0;

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

// LEDs
void LED_init(int estado);
void OLED_init(int estado);

// TC
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
void pisca_led1(int n, int t);

// RTT
void pin_toggle(Pio *pio, uint32_t mask);
void io_init(void);
static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses);

// RTC
void pisca_led(int n, int t);
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);

/************************************************************************/
/* Handlers                                                             */
/************************************************************************/

/**
*  Handle Interrupcao botao 1
*/
static void Button1_Handler(uint32_t id, uint32_t mask)
{
}

/**
*  Interrupt handler for TC1 interrupt.
*/
void TC1_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc0 = 1;
}

void TC4_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC1, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc1 = 1;
}

void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {  }

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		pin_toggle(LED2_PIO, LED2_IDX_MASK);    // BLINK Led
		f_rtt_alarme = true;                  // flag RTT alarme
	}
}

void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

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
      flag_rtc = 1;
	}
	
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}


/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/

/*
 * @Brief Pisca LED1
 */
void pisca_led1(int n, int t){
  for (int i=0;i<n;i++){
    pio_clear(LED1_PIO, LED1_IDX_MASK);
    delay_ms(t);
    pio_set(LED1_PIO, LED1_IDX_MASK);
    delay_ms(t);
  }
}

/*
 * @Brief Pisca LED3
 */
void pisca_led3(int n, int t){
  for (int i=0;i<n;i++){
    pio_clear(LED3_PIO, LED3_IDX_MASK);
    delay_ms(t);
    pio_set(LED3_PIO, LED3_IDX_MASK);
    delay_ms(t);
  }
}



/**
* @Brief Inicializa o pino do LED
*/
void LED_init(int estado){
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_set_output(LED_PIO, LED_IDX_MASK, estado, 0, 0);
};

void OLED_init(int estado){
	// LED1
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_set_output(LED1_PIO, LED1_IDX_MASK, estado, 0, 0);

	// LED2
	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_set_output(LED2_PIO, LED2_IDX_MASK, estado, 0, 0);
	
	// LED3
	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_set_output(LED3_PIO, LED3_IDX_MASK, estado, 0, 0);
};

/*
 * @Brief Pisca LED da placa
 */
void pisca_led(int n, int t){
  for (int i=0;i<n;i++){
    pio_clear(LED_PIO, LED_IDX_MASK);
    delay_ms(t);
    pio_set(LED_PIO, LED_IDX_MASK);
    delay_ms(t);
  }
}

/**
* Configura TimerCounter (TC) para gerar uma interrupcao no canal (ID_TC e TC_CHANNEL)
* na taxa de especificada em freq.
*/
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	/* O TimerCounter é meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrupçcão no TC canal 0 */
	/* Interrupção no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}

void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

static float get_time_rtt(){
	uint ul_previous_time = rtt_read_timer_value(RTT);
}

static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
	uint32_t ul_previous_time;

	/* Configure RTT for a 1 second tick interrupt */
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));
	
	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);

	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
}

/**
* Configura o RTC para funcionar com interrupcao de alarme
*/
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.seccond);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 0);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/

int main (void)
{
	sysclk_init();
	
	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;

	/* Configura Leds */
	LED_init(0);
	OLED_init(1);
	
	// Init OLED
	gfx_mono_ssd1306_init();
	
	// Define hora atual
	uint32_t hour, mins, secs;
	rtc_get_time(RTC, &hour, &mins, &secs);
	
	char *cur_time = (char*)malloc(8 * sizeof(char));
	sprintf(cur_time, "%d:%d:%d", hour-3, mins, secs);
	
	// Escreve na tela a hora atual
	gfx_mono_draw_string(cur_time, 50,16, &sysfont);
	
	/** Configura timer TC0, canal 1 */
	TC_init(TC0, ID_TC1, 1, 4);
	
	/** Configura timer TC1, canal 1 */
	TC_init(TC1, ID_TC4, 1, 5);
	
	// Inicializa RTT com IRQ no alarme.
	f_rtt_alarme = true;

	/** Configura RTC */	
	calendar rtc_initial = {2018, 3, 19, 12, 15, 45 ,1};
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN | RTC_IER_SECEN);

	/* configura alarme do RTC */
	rtc_set_date_alarm(RTC, 1, rtc_initial.month, 1, rtc_initial.day);
	rtc_set_time_alarm(RTC, 1, rtc_initial.hour, 1, rtc_initial.minute, 1, rtc_initial.seccond + 20);
	
	while(1) {
		if(flag_tc0){
			pisca_led1(1,10);
			flag_tc0 = 0;
		}
		if(flag_tc1){
			pisca_led3(1,10);
			flag_tc1 = 0;
		}
		if (f_rtt_alarme){
			/*
			* IRQ apos 4s -> 8*0.5
			*/
		    uint16_t pllPreScale = (int) (((float) 32768) / 4.0);
		    uint32_t irqRTTvalue = 8;
      
		    // reinicia RTT para gerar um novo IRQ
		    RTT_init(pllPreScale, irqRTTvalue);         
      
		    f_rtt_alarme = false;
		}
		if(flag_rtc){
			pisca_led(5, 200);
			flag_rtc = 0;
		}
	}
	return 0;
}
