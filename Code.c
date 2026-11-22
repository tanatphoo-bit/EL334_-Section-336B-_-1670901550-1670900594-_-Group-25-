/*
* EL334 Mini-Project: Stopwatch on FRDM-K64F + Multifunctional Shield
*
* ===== QUICK FIND  (press Ctrl+F and type the tag, e.g. [S1]) =====
*  [S1]     button S1 (PTB3)  : start / stop   -> INTERRUPT
*           init_gpio(), init_button_irq(), PORTB_IRQHandler()
*  [S2]     button S2 (PTB10) : reset to 0     -> polling
*           init_gpio(), check_buttons()
*  [S3]     button S3 (PTB11) : play melody    -> polling
*           init_gpio(), check_buttons()
*  [7SEG]   7-segment display : show seconds
*           display_7seg(), send_595(), shift_byte(), display_off()
*  [TIMER]  PIT timer         : 1 second tick  -> INTERRUPT
*           init_pit(), PIT0_IRQHandler(), delay_us()
*  [LED]    LED D4 (PTD0) blinks, LED D3 (PTD2) = running
*           PIT0_IRQHandler(), update_leds()
*  [BUZZER] buzzer (PTA1)     : beep + melody
*           beep(), play_note(), play_melody(), handle_events()
* ===================================================================
*
*  - PIT0 timer interrupt : count 1 second
*  - S1 (PTB3) interrupt  : start / stop
*  - S2 (PTB10)           : reset seconds to 0
*  - S3 (PTB11)           : play melody manually
*  - LED D4 (PTD0)        : blink every second while running
*  - LED D3 (PTD2)        : ON while running
*  - Buzzer (PTA1)        : short beep on S1, melody every 10 seconds
*  - 7-Segment            : show seconds via 74HC595
*                           (DATA=PTC12, CLK=PTC3, LATCH=PTB23)
*
* Buzzer note: the buzzer transistor is powered from 5V but the MCU pin
* only gives 3.3V, so driving the pin HIGH does not fully turn it off.
* To keep it silent we RELEASE the pin (input). To sound it we drive the
* pin LOW (output).
*/
#include <MK64F12.h>
#include <stdint.h>
#define MASK(x)         (1UL << (x))
#define MELODY_AT_SEC   10      /* play melody every 10 seconds */
/* 1 = use the 7-segment display, 0 = skip it (for testing the rest) */
#define ENABLE_7SEG     1
/* ---------- pin macros ---------- */
/* [LED] LEDs on the shield are active-low (ON when pin = 0) */
#define LED_D4_ON()      (PTD->PCOR = MASK(0))
#define LED_D4_OFF()     (PTD->PSOR = MASK(0))
#define LED_D4_TOGGLE()  (PTD->PTOR = MASK(0))
#define LED_D3_ON()      (PTD->PCOR = MASK(2))
#define LED_D3_OFF()     (PTD->PSOR = MASK(2))
/* [BUZZER] ON = drive pin LOW, OFF = release pin (input, not driven) */
#define BUZZER_ON()      do { PTA->PCOR = MASK(1); PTA->PDDR |= MASK(1); } while (0)
#define BUZZER_OFF()     (PTA->PDDR &= ~MASK(1))
/* [7SEG] 74HC595 pins: DATA=PTC12, CLK=PTC3, LATCH=PTB23 */
#define DATA_HIGH()      (PTC->PSOR = MASK(12))
#define DATA_LOW()       (PTC->PCOR = MASK(12))
#define CLK_HIGH()       (PTC->PSOR = MASK(3))
#define CLK_LOW()        (PTC->PCOR = MASK(3))
#define LATCH_HIGH()     (PTB->PSOR = MASK(23))
#define LATCH_LOW()      (PTB->PCOR = MASK(23))
/* ---------- shared variables (changed inside interrupts -> volatile) ---------- */
static volatile uint32_t seconds     = 0;
static volatile uint8_t  running     = 0;
static volatile uint8_t  beep_flag   = 0;
static volatile uint8_t  melody_flag = 0;
static volatile uint32_t ms_ticks    = 0;
static volatile uint32_t last_s1_ms  = 0;
static uint32_t bus_clock_hz;
/* Segment patterns (common anode: 0 = segment ON) and digit select */
static const uint8_t SEG[10] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99,
                               0x92, 0x82, 0xF8, 0x80, 0x90};
static const uint8_t SEL[4]  = {0xF1, 0xF2, 0xF4, 0xF8};
/* Melody: note frequency (Hz) and length (ms) */
static const uint16_t NOTE_FREQ[] = {262, 330, 392, 523};
static const uint16_t NOTE_MS[]   = {250, 250, 250, 500};
/* ---------- prototypes ---------- */
void init_clock_info(void);
void init_gpio(void);
void init_systick(void);
void init_pit(void);
void init_button_irq(void);
void delay_us(uint32_t us);
void check_buttons(void);
void handle_events(void);
void update_leds(void);
void beep(void);
void play_note(uint32_t freq, uint32_t ms);
void play_melody(void);
void shift_byte(uint8_t value);
void send_595(uint8_t seg, uint8_t sel);
void display_off(void);
void display_7seg(uint32_t value);
/* ---------- main ---------- */
int main(void) {
   init_clock_info();
   init_gpio();
   init_systick();
   init_pit();
   init_button_irq();
   while (1) {
       check_buttons();            /* [S2] reset, [S3] melody */
       handle_events();            /* [BUZZER] beep / melody requests */
       update_leds();              /* [LED] D3 shows running state */
#if ENABLE_7SEG
       display_7seg(seconds);      /* [7SEG] refresh 7-segment */
#endif
   }
}
/* ---------- initialisation ---------- */
/* Work out the bus clock, because PIT counts with the bus clock */
void init_clock_info(void) {
   SystemCoreClockUpdate();
   bus_clock_hz = SystemCoreClock /
       (((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV2_MASK) >> SIM_CLKDIV1_OUTDIV2_SHIFT) + 1);
}
void init_gpio(void) {
   /* enable clock to Port A, B, C, D */
   SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK |
                 SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK;
   /* [S1][S2][S3] buttons: GPIO + internal pull-up (so the pin never floats) */
   PORTB->PCR[3]  = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;  /* [S1] button S1 = PTB3  */
   PORTB->PCR[10] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;  /* [S2] button S2 = PTB10 */
   PORTB->PCR[11] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;  /* [S3] button S3 = PTB11 */
   /* outputs: GPIO */
   PORTB->PCR[23] = PORT_PCR_MUX(1);   /* [7SEG] 595 latch */
   PORTC->PCR[3]  = PORT_PCR_MUX(1);   /* [7SEG] 595 clock */
   PORTC->PCR[12] = PORT_PCR_MUX(1);   /* [7SEG] 595 data  */
   PORTA->PCR[1]  = PORT_PCR_MUX(1);   /* [BUZZER] buzzer */
   PORTD->PCR[0]  = PORT_PCR_MUX(1);   /* [LED] LED D4 */
   PORTD->PCR[2]  = PORT_PCR_MUX(1);   /* [LED] LED D3 */
   /* safe start levels: LEDs off, buzzer released (silent) */
   PTD->PSOR = MASK(0) | MASK(2);
   BUZZER_OFF();
   /* directions (buzzer pin PTA1 is NOT set as output here) */
   PTB->PDDR &= ~(MASK(3) | MASK(10) | MASK(11));  /* [S1][S2][S3] buttons = input */
   PTB->PDDR |= MASK(23);                          /* [7SEG] latch = output */
   PTC->PDDR |= MASK(3) | MASK(12);                /* [7SEG] clock, data = output */
   PTD->PDDR |= MASK(0) | MASK(2);                 /* [LED] LEDs = output */
}
/* SysTick gives a 1 ms tick, used to ignore button bounce */
void init_systick(void) {
   SysTick_Config(SystemCoreClock / 1000);
}
/* [TIMER] PIT channel 0: interrupt every 1 second. Channel 1: blocking delay. */
void init_pit(void) {
   SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;       /* clock to PIT */
   PIT->MCR = 0;                           /* enable PIT module */
   PIT->CHANNEL[0].LDVAL = bus_clock_hz - 1;   /* 1 second */
   PIT->CHANNEL[0].TFLG  = PIT_TFLG_TIF_MASK;  /* clear old flag */
   PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;
   NVIC_EnableIRQ(PIT0_IRQn);
}
/* [S1] S1 (PTB3): INTERRUPT on falling edge (button pressed = 0) */
void init_button_irq(void) {
   PORTB->PCR[3] |= PORT_PCR_IRQC(0xA);    /* 1010 = falling edge */
   PORTB->ISFR = MASK(3);                  /* clear any old flag */
   NVIC_EnableIRQ(PORTB_IRQn);
}
/* ---------- interrupt handlers ---------- */
void SysTick_Handler(void) {
   ms_ticks++;
}
/* [TIMER][LED] runs by itself every 1 second (Timer INTERRUPT) */
void PIT0_IRQHandler(void) {
   PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;   /* clear flag */
   if (running) {
       seconds++;
       LED_D4_TOGGLE();
       if (seconds % MELODY_AT_SEC == 0) {     /* 10, 20, 30, ... */
           melody_flag = 1;
       }
   }
}
/* [S1] runs by itself when S1 is pressed (button INTERRUPT) */
void PORTB_IRQHandler(void) {
   if (PORTB->ISFR & MASK(3)) {
       PORTB->ISFR = MASK(3);                  /* clear flag (write 1) */
       if ((ms_ticks - last_s1_ms) > 200) {    /* ignore bounce */
           last_s1_ms = ms_ticks;
           running = !running;                 /* start / stop */
           beep_flag = 1;
       }
   }
}
/* ---------- helper functions ---------- */
/* [TIMER] Blocking delay using PIT channel 1 */
void delay_us(uint32_t us) {
   PIT->CHANNEL[1].LDVAL = (uint32_t)(((uint64_t)us * bus_clock_hz) / 1000000UL) - 1;
   PIT->CHANNEL[1].TFLG  = PIT_TFLG_TIF_MASK;
   PIT->CHANNEL[1].TCTRL = PIT_TCTRL_TEN_MASK;
   while (!(PIT->CHANNEL[1].TFLG & PIT_TFLG_TIF_MASK)) { }
   PIT->CHANNEL[1].TCTRL = 0;
}
/* [S2][S3] polling: main() keeps reading these two pins in its loop */
void check_buttons(void) {
   static uint8_t s3_prev = 1;
   if (!(PTB->PDIR & MASK(10))) {          /* [S2] S2 pressed -> reset seconds to 0 */
       seconds = 0;
   }
   uint8_t s3 = (PTB->PDIR & MASK(11)) ? 1 : 0;
   if (s3_prev && !s3) {                   /* [S3] S3 just pressed -> play melody */
       melody_flag = 1;
   }
   s3_prev = s3;
}
/* [BUZZER] play beep / melody when the flags are set */
void handle_events(void) {
   if (beep_flag) {
       beep_flag = 0;
       beep();
   }
   if (melody_flag) {
       melody_flag = 0;
       play_melody();
   }
}
/* [LED] D3 ON while running, D3 and D4 OFF when stopped */
void update_leds(void) {
   if (running) {
       LED_D3_ON();
   } else {
       LED_D3_OFF();
       LED_D4_OFF();
   }
}
/* ---------- [BUZZER] buzzer ---------- */
/* Make a tone by switching the buzzer on and off at the note frequency */
void play_note(uint32_t freq, uint32_t ms) {
   uint32_t half_us = 500000UL / freq;
   uint32_t cycles  = (freq * ms) / 1000UL;
   while (cycles--) {
       BUZZER_ON();
       delay_us(half_us);
       BUZZER_OFF();
       delay_us(half_us);
   }
   BUZZER_OFF();                           /* always finish silent */
}
void beep(void) {
   display_off();
   play_note(2000, 40);
}
void play_melody(void) {
   display_off();
   for (int i = 0; i < 4; i++) {
       play_note(NOTE_FREQ[i], NOTE_MS[i]);
       delay_us(50000);                    /* short gap between notes */
   }
}
/* ---------- [7SEG] 7-segment via 74HC595 ---------- */
/* Send 8 bits, MSB first */
void shift_byte(uint8_t value) {
   for (int i = 7; i >= 0; i--) {
       CLK_LOW();
       if (value & MASK(i)) DATA_HIGH(); else DATA_LOW();
       CLK_HIGH();
   }
}
/* First byte = segments, second byte = which digit, then latch */
void send_595(uint8_t seg, uint8_t sel) {
   LATCH_LOW();
   shift_byte(seg);
   shift_byte(sel);
   LATCH_HIGH();
}
void display_off(void) {
#if ENABLE_7SEG
   send_595(0xFF, 0xF0);
#endif
}
/* [7SEG] Show a number 0-9999 by scanning the 4 digits one at a time */
void display_7seg(uint32_t value) {
   uint32_t v = value % 10000;
   uint8_t digit[4];
   digit[0] = (uint8_t)(v / 1000);
   digit[1] = (uint8_t)((v / 100) % 10);
   digit[2] = (uint8_t)((v / 10) % 10);
   digit[3] = (uint8_t)(v % 10);
   for (int p = 0; p < 4; p++) {
       send_595(SEG[digit[p]], SEL[p]);
       delay_us(1000);                     /* 1 ms per digit */
   }
}