/**
 **********************************************************************
 * @file    main.c
 * @mainpage
 * @author  Andrej Barc, Ondrej Barton Brno University of Technology, Czech Republic
 * @version V1.0
 * @date    Nov 20, 2018
 * @brief   Measuring distance by ultrasonic senzor.
 * @copyright (c) 2018 Barc, Barton, MIT License
 **********************************************************************
 */


/* Includes ------------------------------------------------------------------*/
 #include <stdbool.h>                  /* bool type */
 #include "settings.h"                 /* F_CPU definition */
 #include <avr/interrupt.h>
 #include <avr/io.h>
 #include <string.h>
 #include <stdlib.h>                   /* dtostrf() function */
 #include "lcd.h"
 #include <util/delay.h>



 /* Function prototypes -------------------------------------------------------*/
 /**
 *  @brief Initialize output pin, Timer/Counter1.
 */
 void setup(void);
 /**
 *  @brief Triggers ultrasonic senzor.
  */
 void trigger_sensor();
 /**
 *  @brief Activate buzzer according to distance.
  */
 void buzzer(unsigned int dist);

 /*  Globals ----------------------------------------------------------------- */
 /**
   * @var Define whether counting is in progress.
   */
 bool timer_started = false;
 /**
   * @var Sets variable on zero.
   */
 uint16_t timer = 0;

 /* Functions -----------------------------------------------------------------*/
 /**
   * @brief Main function.
   */
 int main(void)
 {
     char lcd_string[13];            /* xxx.yyyyy cm\0 -> 13 chars */

     /* Initializations */
     setup();

     /* Enables interrupts by setting the global interrupt mask */
     sei();

     /* Forever loop */
     while (1)
     {
         /* Send trigger signal to sensor on pin 13 (PB5) */
         trigger_sensor();

         /**
           * @details Describe float distance function.
           * distance computation -> computed as time till echo pin changes value (countet in clock pulse)
           * -> counted pulses * prescaler (one pulse counted per 256 cpu pulses) / F_CPU (pulses/s)
           * 10^6 (we want it in microseconds) / 58.2 (according to formula)
           */
         float distance = timer * 256 / (F_CPU / 1000000) / 58.2;

         /* Convert float data type to string data type. Function defined in stdlib.h */
         /* char * dtostrf(double __value, signed char __width, unsigned char __prec, char * str) */
         dtostrf(distance, 3, 5, lcd_string);

         /* Set cursor to specified position */
         lcd_gotoxy(0,1);
         /* Display converted string to display */
         lcd_puts(lcd_string);
         /* Set cursor to specified position */
         lcd_gotoxy(9,1);
         /* Display character at current cursor position */
         lcd_puts("cm");

         /* inversion distance */
         unsigned int inverse_distance = distance < 50 ? (5 - (uint8_t) distance/10) : 0;

         /* Reset LEDs */
         PORTC &= ~0x1f;
         /**
           * @details Set LEDs on.
           * set LEDs based on distance (every 10 cm turns on one led, starting from green, 1. lights on <2, 10) cm
           * +1 and -1 coz we need to light the right one and all leds before
           * for example 42cm -> 1 << 42/10 -> 100000 -> 100000 - 1 = 011111
           */
         PORTC = (1 << inverse_distance) - 1;

         buzzer(inverse_distance);

         _delay_ms(100);
     }

     return 0;

 }

 void buzzer(unsigned int dist)
 {

     for (int j=0; j < dist; j++)
      {
         PORTD |= _BV(PD3);
         for (int i=0; i < 500/dist; i++)
         {
             _delay_ms(1);
         }
         PORTD &= ~_BV(PD3);
         for (int i=0; i < 500/dist; i++)
         {
             _delay_ms(1);
         }
      }
 }

 /**
   * @brief Setup all peripherals.
   */
 void trigger_sensor()
 {
     /* Set low level */
     PORTB &= ~_BV(PB5);
     _delay_ms(2);

     /* Set high level */
     PORTB |= _BV(PB5);
     /**
     * @brief Trigger pulse.
     * @warning Must be greater than 10us.
     */
     _delay_us(11);


     /* Set low level */
     PORTB &= ~_BV(PB5);
     _delay_ms(100);
 }


 void setup(void)
 {
     _delay_ms(1000);
     /* Enable Pin Change Interrupt 0 */
     PCICR |= _BV(PCIE0);
     /* Enable pin change interrupt at pin 11 */
     PCMSK0 |= _BV(PB3);

     /* Set output pin 13 (PB5) - trigger */
     DDRB |= _BV(PB5);
     /* Set input pin 11 (PB3) - echo */
     DDRB &= ~_BV(PB3);
     /* Set output pins A0-A4 - LEDs */
     DDRC = 0x1f;
     /* Set output pin 3 (PD3) - buzzer */
     DDRD |= _BV(PD3);

     _delay_ms(100);

     /* Initialize display and select type of cursor */
     lcd_init(LCD_DISP_ON);

     /* Display string without auto linefeed */
     lcd_puts("Distance:");

 }

 ISR(PCINT0_vect)
 {
     if (!timer_started)
     {
         /* Start counting */
         timer_started = true;

         /* Timer/Counter1: select clock */
         /* Clock prescaler 256 => overflows every 1 s */
         TCCR1B |= _BV(2);
         TCNT1L = 0;
         TCNT1H = 0;                       /* Clearing counter */
     }
     else
     {
         /* 16 bit value, divided to two 8 bit registers HIGH and LOW */
         timer = TCNT1L;                   /* LOW = last 8 bits */
         timer += TCNT1H << 8;             /* Count LOW with High shifted value and write to the variable timer */

         /* Stop counting */
         timer_started = false;
     }
 }
 /* END OF FILE ****************************************************************/