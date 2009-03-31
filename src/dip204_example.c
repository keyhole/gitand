/* This source file is part of the ATMEL AVR32-SoftwareFramework-AT32UC3A-1.2.2ES Release */

/*This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief LCD DIP204 example driver for EVK1100 board.
 *
 * This file provides a useful example for the LCD DIP204 on SPI interface.
 * Press PB0 to see the full set of available chars on the LCD
 * Press PB1 to decrease the backlight power of the LCD
 * Press PB2 to increase the backlight power of the LCD
 * Use Joystick to see arrows displayed on the LCD
 * Press Joystick to return to the idle screen
 *
 * - Compiler:           IAR EWAVR32 and GNU GCC for AVR32
 * - Supported devices:  All AVR32 devices with : SPI and PWM
 * - AppNote:
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support and FAQ: http://support.atmel.no/
 *
 *****************************************************************************/

/*! \page License
 * Copyright (c) 2007, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*! \mainpage
 * \section intro Introduction
 * This documents data structures, functions, variables, defines, enums, and
 * typedefs in the software. <BR>It also gives an example of the usage of the
 * DIP204 LCD on EVK1100. \n \n
 * <b>Example's operating mode:</b>
 * <ul>
 * <li>A default message is displayed on the 4 lines of the LCD
 * <li>Press PB0 to see the full set of available chars on the LCD
 * <li>Press PB1 to decrease the backlight power of the LCD
 * <li>Press PB2 to increase the backlight power of the LCD
 * <li>Use the joystick to see arrows displayed on the LCD
 * <li>Press the joystick to see a circle displayed on the LCD and to return to the
 *     idle screen (displaying the default message)
 *
 * </ul>
 *
 * \section compinfo Compilation Info
 * This software was written for the GNU GCC for AVR32 and IAR Systems compiler
 * for AVR32. Other compilers may or may not work.
 *
 * \section deviceinfo Device Info
 * All AVR32UC devices with an SPI module can be used. This example has been tested
 * with the following setup:
 *- EVK1100 evaluation kit
 *
 * \section setupinfo Setup Information
 * CPU speed: <i> 12 MHz </i>
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com/products/AVR32/">Atmel AVR32</A>.\n
 * Support and FAQ: http://support.atmel.no/
 */

/*------------------------------------------------------
  --------------------- INCLUDES   ---------------------
  ------------------------------------------------------*/

#include "board.h"
#include "compiler.h"
#include "dip204.h"
#include "intc.h"
#include "gpio.h"
#include "pm.h"
#include "cycle_counter.h"
#include "spi.h"
#include <avr32/io.h>
#include "adc.h"
#include "tc.h"
#include "cycle_counter.h"
#include "flashc.h"
#include "usart.h"


/*------------------------------------------------------
  -------------------- DEFINITIONS ---------------------
  ------------------------------------------------------*/

// Connection of the temperature sensor
#  define EXAMPLE_ADC_TEMPERATURE_CHANNEL     0
#  define EXAMPLE_ADC_TEMPERATURE_PIN         AVR32_ADC_AD_0_PIN
#  define EXAMPLE_ADC_TEMPERATURE_FUNCTION    AVR32_ADC_AD_0_FUNCTION
// Connection of the light sensor
#  define EXAMPLE_ADC_LIGHT_CHANNEL           2
#  define EXAMPLE_ADC_LIGHT_PIN               AVR32_ADC_AD_2_PIN
#  define EXAMPLE_ADC_LIGHT_FUNCTION          AVR32_ADC_AD_2_FUNCTION
// Connection of the potentiometer
#  define EXAMPLE_ADC_POTENTIOMETER_CHANNEL   1
#  define EXAMPLE_ADC_POTENTIOMETER_PIN       AVR32_ADC_AD_1_PIN
#  define EXAMPLE_ADC_POTENTIOMETER_FUNCTION  AVR32_ADC_AD_1_FUNCTION

#define GPIO_PIN_EXAMPLE_1  LED2_GPIO
#define GPIO_PIN_EXAMPLE_2  LED3_GPIO
#define GPIO_PIN_EXAMPLE_3  GPIO_PUSH_BUTTON_0


#define FPBA    FOSC0
#define TC_CHANNEL    0
#define USE_ADC_8_BITS 1



/*! define the push button to see available char map on LCD */
#define GPIO_CHARSET            GPIO_PUSH_BUTTON_0

/*! define the push button to decrease back light power */
#define GPIO_BACKLIGHT_MINUS    GPIO_PUSH_BUTTON_1

/*! define the push button to increase back light power */
#define GPIO_BACKLIGHT_PLUS     GPIO_PUSH_BUTTON_2

/*! flag set when joystick display starts to signal main function to clear this display */
unsigned short display;

/*! flag set when time out starts and cleared when timeout occurs */
volatile unsigned char TimeOut = 0;

unsigned int sam = 0;
volatile static int print_sec = 1;

volatile U32 tc_tick = 0;

volatile avr32_pm_t* pm = &AVR32_PM;

unsigned int menu = 1;
unsigned int first = 1;
unsigned int teszt = 0;
unsigned int joy_pressed = 0;
unsigned int clksel = 1;
unsigned int i = 0;


/*------------------------------------------------------
  --------------------- INTERRUPTS ---------------------
  ------------------------------------------------------*/

/*!
 * \brief interrupt handler for compare IT.
 */
#if __GNUC__
__attribute__((__interrupt__))
#elif __ICCAVR32__
__interrupt
#endif
static void compare_irq_handler(void)
{
  TimeOut = 1;
  Set_sys_compare(0);
}


/*! \brief TC interrupt.
 */
#if __GNUC__
__attribute__((__interrupt__))
#elif __ICCAVR32__
#pragma handler = AVR32_TC_IRQ_GROUP, 1
__interrupt
#endif
static void tc_irq(void)
{
  // Increment the ms seconds counter
  tc_tick++;

  // Clear the interrupt flag. This is a side effect of reading the TC SR.
  tc_read_sr(&AVR32_TC, TC_CHANNEL);

  // specify that an interrupt has been raised
  print_sec = 1;

}

/*!
 * \brief The Push Buttons interrupt handler.
 */
#if __GNUC__
__attribute__((__interrupt__))
#elif __ICCAVR32__
__interrupt
#endif
static void dip204_example_PB_int_handler(void)
{


  /* display all available chars */
  if (gpio_get_pin_interrupt_flag(GPIO_CHARSET))
  {
//
     gpio_clear_pin_interrupt_flag(GPIO_CHARSET);
  }




/* increase backlight power */
 if (gpio_get_pin_interrupt_flag(GPIO_BACKLIGHT_PLUS))
  {
    	dip204_set_backlight(backlight_power_increase);
    	/* allow new interrupt : clear the IFR flag */
    	gpio_clear_pin_interrupt_flag(GPIO_BACKLIGHT_PLUS);
  }






/* decrease backlight power */
  if (gpio_get_pin_interrupt_flag(GPIO_BACKLIGHT_MINUS))
  {
    dip204_set_backlight(backlight_power_decrease);
    /* allow new interrupt : clear the IFR flag */
    gpio_clear_pin_interrupt_flag(GPIO_BACKLIGHT_MINUS);
  }



}

/*!
 * \brief The joystick interrupt handler.
 */
#if __GNUC__
__attribute__((__interrupt__))
#elif __ICCAVR32__
__interrupt
#endif
static void dip204_example_Joy_int_handler(void)
{



if (gpio_get_pin_interrupt_flag(GPIO_JOYSTICK_UP))
  {
	gpio_enable_pin_glitch_filter(GPIO_JOYSTICK_UP);
	//dip204_set_backlight(backlight_power_increase);
    clksel++;
	clksel = min(clksel,3);

	if (clksel==1)
	{
		  pm_cksel(pm,0,0,0,0,0,0); //(pm,pbadiv,pbasel,pbbdiv,pbbsel,hsbdiv,hsbsel)

	}

	if (clksel==2)
	{
		  pm_cksel(pm,1,0,0,0,0,0); //(pm,pbadiv,pbasel,pbbdiv,pbbsel,hsbdiv,hsbsel)
	}

	if (clksel==3)
	{
		  pm_cksel(pm,1,1,0,0,0,0); //(pm,pbadiv,pbasel,pbbdiv,pbbsel,hsbdiv,hsbsel)



	}
	gpio_clear_pin_interrupt_flag(GPIO_JOYSTICK_UP);
  }




  if (gpio_get_pin_interrupt_flag(GPIO_JOYSTICK_DOWN))
  {
	  gpio_enable_pin_glitch_filter(GPIO_JOYSTICK_DOWN);
	  //dip204_set_backlight(backlight_power_decrease);
	  clksel--;
	  clksel = max(clksel,1);

	  if (clksel==1)
	  	{
	  		  pm_cksel(pm,0,0,0,0,0,0); //(pm,pbadiv,pbasel,pbbdiv,pbbsel,hsbdiv,hsbsel)

	  	}

	  	if (clksel==2)
	  	{
	  		  pm_cksel(pm,1,0,0,0,0,0); //(pm,pbadiv,pbasel,pbbdiv,pbbsel,hsbdiv,hsbsel)
	  	}

	  if (clksel==3)
	  {
		  	 pm_cksel(pm,1,1,0,0,0,0); //(pm,pbadiv,pbasel,pbbdiv,pbbsel,hsbdiv,hsbsel)

	  }
	  gpio_clear_pin_interrupt_flag(GPIO_JOYSTICK_DOWN);
  }





  if (gpio_get_pin_interrupt_flag(GPIO_JOYSTICK_LEFT))
  {
	  gpio_enable_pin_glitch_filter(GPIO_JOYSTICK_LEFT);

	  menu--;
	  menu = max(menu,1);
	  first=1;
    gpio_clear_pin_interrupt_flag(GPIO_JOYSTICK_LEFT);
  }





  if (gpio_get_pin_interrupt_flag(GPIO_JOYSTICK_RIGHT))
  {
	  gpio_enable_pin_glitch_filter(GPIO_JOYSTICK_RIGHT);

	  menu++;
	  menu = min(menu,9);
	  first=1;
    gpio_clear_pin_interrupt_flag(GPIO_JOYSTICK_RIGHT);
  }






  if (gpio_get_pin_interrupt_flag(GPIO_JOYSTICK_PUSH))
  {


		joy_pressed = 1;
		gpio_clear_pin_interrupt_flag(GPIO_JOYSTICK_PUSH);
  }


}


/*!
 * \brief function to configure push button to generate IT upon rising edge
 */
void dip204_example_configure_push_buttons_IT(void)
{
  gpio_enable_pin_interrupt(GPIO_CHARSET , GPIO_RISING_EDGE);

  gpio_enable_pin_interrupt(GPIO_BACKLIGHT_PLUS , GPIO_RISING_EDGE);

  gpio_enable_pin_interrupt(GPIO_BACKLIGHT_MINUS , GPIO_RISING_EDGE);

  /* Disable all interrupts */
  Disable_global_interrupt();
  /* register PB0 handler on level 1 */
  INTC_register_interrupt( &dip204_example_PB_int_handler, AVR32_GPIO_IRQ_0 + (GPIO_BACKLIGHT_PLUS/8), AVR32_INTC_INT1);
  INTC_register_interrupt( &dip204_example_PB_int_handler, AVR32_GPIO_IRQ_0 + (GPIO_BACKLIGHT_MINUS/8), AVR32_INTC_INT1);
  INTC_register_interrupt( &dip204_example_PB_int_handler, AVR32_GPIO_IRQ_0 + (GPIO_CHARSET/8), AVR32_INTC_INT1);
  /* Enable all interrupts */
  Enable_global_interrupt();
}


/*!
 * \brief function to configure joystick to generate IT upon falling edge
 */
void dip204_example_configure_joystick_IT(void)
{
  gpio_enable_pin_interrupt(GPIO_JOYSTICK_UP , GPIO_FALLING_EDGE);
  gpio_enable_pin_interrupt(GPIO_JOYSTICK_DOWN , GPIO_FALLING_EDGE);
  gpio_enable_pin_interrupt(GPIO_JOYSTICK_RIGHT , GPIO_FALLING_EDGE);
  gpio_enable_pin_interrupt(GPIO_JOYSTICK_PUSH , GPIO_FALLING_EDGE);
  gpio_enable_pin_interrupt(GPIO_JOYSTICK_LEFT , GPIO_FALLING_EDGE);

  /* Disable all interrupts */
  Disable_global_interrupt();
  /* register PB0 handler on level 1 */
  INTC_register_interrupt( &dip204_example_Joy_int_handler, AVR32_GPIO_IRQ_0 + (GPIO_JOYSTICK_UP/8), AVR32_INTC_INT1);
  INTC_register_interrupt( &dip204_example_Joy_int_handler, AVR32_GPIO_IRQ_0 + (GPIO_JOYSTICK_DOWN/8), AVR32_INTC_INT1);
  INTC_register_interrupt( &dip204_example_Joy_int_handler, AVR32_GPIO_IRQ_0 + (GPIO_JOYSTICK_RIGHT/8), AVR32_INTC_INT1);
  INTC_register_interrupt( &dip204_example_Joy_int_handler, AVR32_GPIO_IRQ_0 + (GPIO_JOYSTICK_LEFT/8), AVR32_INTC_INT1);
  INTC_register_interrupt( &dip204_example_Joy_int_handler, AVR32_GPIO_IRQ_0 + (GPIO_JOYSTICK_PUSH/8), AVR32_INTC_INT1);
  /* Enable all interrupts */
  Enable_global_interrupt();
}


/*------------------------------------------------------
  --------------------- FUNCTIONS  ---------------------
  ------------------------------------------------------*/


/*!
 * \brief Software delay
 */
void delay_ms(unsigned short time_ms) {
unsigned long u32CountVal,u32CompareVal;

  TimeOut = 0;
  u32CountVal = Get_sys_count();

  u32CompareVal = u32CountVal  + ((unsigned long)time_ms * (FOSC0 / 1000)); // WARNING: MUST FIT IN 32bits.

  Set_sys_compare(u32CompareVal); // GO

  //  The previous COMPARE write succeeded.
  // Loop until the COUNT&COMPARE match triggers.
  while (!TimeOut);
}

/**

 * Ansi C "itoa" based on Kernighan & Ritchie's "Ansi C":

 */
void strreverse(char* begin, char* end) {

	char aux;

	while(end>begin)

		aux=*end, *end--=*begin, *begin++=aux;

}

void itoa(int value, char* str, int base) {

	static char num[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";

	char* wstr=str;

	int sign;



	// Validate base

	if (base<2 || base>35){ *wstr='\0'; return; }



	// Take care of sign

	if ((sign=value) < 0) value = -value;



	// Conversion. Number is reversed.

	do *wstr++ = num[value%base]; while(value/=base);

	if(sign<0) *wstr++='-';

	*wstr='\0';



	// Reverse string

	strreverse(str,wstr-1);

}

int add(int a, int b)
{
  return a + b;
}




void pllinitialize(void)
{

  volatile avr32_pm_t* pm = &AVR32_PM;
  pm_switch_to_osc0(pm, FOSC0, OSC0_STARTUP);
  pm_pll_setup(pm, 0, 10, 1, 0, 16);
  pm_pll_set_option(pm, 0, 1, 1, 0);
  pm_pll_enable(pm,0);
  pm_wait_for_pll0_locked(pm) ;
  pm_gc_setup(pm, 0, 1, 0, 0, 0); //GCLK0,PLL,PLL0,DivNo,div/0
  pm_gc_enable(pm, 0);
  pm_cksel(pm,0,0,0,0,0,0); 	//PBA,PBB,HSB=CPU
								//PBA=00 -> 12MHz
								//PBA=01 -> 24MHz
								//PBA=11 -> 48MHz

  flashc_set_wait_state(1);

  gpio_enable_module_pin(AVR32_PM_GCLK_0_0_PIN, AVR32_PM_GCLK_0_0_FUNCTION);
  pm_switch_to_clock(pm, AVR32_PM_MCCTRL_MCSEL_PLL0);

}


/*------------------------------------------------------
  ---------------------    MAIN    ---------------------
  ------------------------------------------------------*/

/*!
 * \brief main function : do init and loop (poll if configured so)
 */
int main(void)

{
  char ertek1[10];
  char ertek2[10];
  char ertek3[10];
  char ertek4[10];
  char ertek5[10];
  char ertek6[10];
  char ertek7[10];

  unsigned int fel = 1;

  static const gpio_map_t DIP204_SPI_GPIO_MAP =
  	{
    		{DIP204_SPI_SCK_PIN,  DIP204_SPI_SCK_FUNCTION },  // SPI Clock.
    		{DIP204_SPI_MISO_PIN, DIP204_SPI_MISO_FUNCTION},  // MISO.
    		{DIP204_SPI_MOSI_PIN, DIP204_SPI_MOSI_FUNCTION},  // MOSI.
    		{DIP204_SPI_NPCS_PIN, DIP204_SPI_NPCS_FUNCTION}   // Chip Select NPCS.
  	};

  static const gpio_map_t ADC_GPIO_MAP =
  	{
		{EXAMPLE_ADC_TEMPERATURE_PIN, EXAMPLE_ADC_TEMPERATURE_FUNCTION},
   		{EXAMPLE_ADC_LIGHT_PIN, EXAMPLE_ADC_LIGHT_FUNCTION},
   		{EXAMPLE_ADC_POTENTIOMETER_PIN, EXAMPLE_ADC_POTENTIOMETER_FUNCTION}
  	};


  volatile avr32_adc_t *adc = &AVR32_ADC; // ADC IP registers address
  volatile avr32_tc_t *tc = &AVR32_TC;

  //Options for waveform genration.
  static const tc_waveform_opt_t WAVEFORM_OPT =
  {
    .channel  = TC_CHANNEL,                        // Channel selection.

    .bswtrg   = TC_EVT_EFFECT_NOOP,                // Software trigger effect on TIOB.
    .beevt    = TC_EVT_EFFECT_NOOP,                // External event effect on TIOB.
    .bcpc     = TC_EVT_EFFECT_NOOP,                // RC compare effect on TIOB.
    .bcpb     = TC_EVT_EFFECT_NOOP,                // RB compare effect on TIOB.

    .aswtrg   = TC_EVT_EFFECT_NOOP,                // Software trigger effect on TIOA.
    .aeevt    = TC_EVT_EFFECT_NOOP,                // External event effect on TIOA.
    .acpc     = TC_EVT_EFFECT_NOOP,                // RC compare effect on TIOA: toggle.
    .acpa     = TC_EVT_EFFECT_NOOP,                // RA compare effect on TIOA: toggle (oth possibs are non,set,clear).

    .wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,// Waveform selection: Up mode with auto trigger(reset) on RC compare.
    .enetrg   = FALSE,                             // External event trigger enable.
    .eevt     = 0,                                 // External event selection.
    .eevtedg  = TC_SEL_NO_EDGE,                    // External event edge selection.
    .cpcdis   = FALSE,                             // Counter disable when RC compare.
    .cpcstop  = FALSE,                             // Counter clock stopped with RC compare.

    .burst    = FALSE,                             // Burst signal selection.
    .clki     = FALSE,                             // Clock inversion.
    .tcclks   = TC_CLOCK_SOURCE_TC2                // Internal source clock 2 - connected to PBA/4
  };

  static const tc_interrupt_t TC_INTERRUPT =
  {
    .etrgs = 0,
    .ldrbs = 0,
    .ldras = 0,
    .cpcs  = 1,
    .cpbs  = 0,
    .cpas  = 0,
    .lovrs = 0,
    .covfs = 0
  };


  signed short adc_value_temp = 1;
  signed short adc_value_light = 1;
  signed short adc_value_pot = 1;

  unsigned short adc_channel_temp = EXAMPLE_ADC_TEMPERATURE_CHANNEL;
  unsigned short adc_channel_light = EXAMPLE_ADC_LIGHT_CHANNEL;
  unsigned short adc_channel_pot = EXAMPLE_ADC_POTENTIOMETER_CHANNEL;

  U32 adatok;

  //volatile avr32_gpio_port_t *gpio_port;
  //volatile avr32_spi_t *spi;
  //volatile avr32_pwm_t *pwm;

  typedef union
  {
    unsigned long                 gcctrl;
    avr32_pm_gcctrl_t             GCCTRL;
  } u_avr32_pm_gcctrl_t;






   U32 u32CountVal;
   U32 u32CountValLast;


  // Switch the CPU main clock to oscillator 0

   pllinitialize();
   //pm_switch_to_osc0(&AVR32_PM, FOSC0, OSC0_STARTUP);




  Disable_global_interrupt();  // Disable all interrupts.
  INTC_init_interrupts();  // init the interrupts

  // Register the compare interrupt handler to the interrupt controller.
  // compare_irq_handler is the interrupt handler to register.
  // AVR32_CORE_COMPARE_IRQ is the IRQ of the interrupt handler to register.
  // AVR32_INTC_INT0 is the interrupt priority level to assign to the group of this IRQ.
  // void INTC_register_interrupt(__int_handler handler, unsigned int irq, unsigned int int_lev);
  INTC_register_interrupt(&compare_irq_handler, AVR32_CORE_COMPARE_IRQ, AVR32_INTC_INT0);
 // Register the RTC interrupt handler to the interrupt controller.
  INTC_register_interrupt(&tc_irq, AVR32_TC_IRQ0, AVR32_INTC_INT0);


  Enable_global_interrupt();  // Enable all interrupts.

  tc_init_waveform(tc, &WAVEFORM_OPT);         // Initialize the timer/counter waveform.
 // Set the compare triggers.
  // Remember TC counter is 16-bits, so counting second is not possible with fPBA = 12 MHz.
  // We configure it to count ms.
  // We want: (1/(FPBA/4)) * RC = 1000 Hz => RC = (FPBA/4) / 1000 = 3000 to get an interrupt every 1ms
  tc_write_rc(tc, TC_CHANNEL, (FPBA/4)/1000);  // Set RC value.
  tc_configure_interrupts(tc, TC_CHANNEL, &TC_INTERRUPT);
  tc_start(tc, TC_CHANNEL);  // Start the timer/counter.


  // add the spi options driver structure for the LCD DIP204
  spi_options_t spiOptions =
  {
    .reg          = DIP204_SPI_NPCS,
    .baudrate     = 1000000,
    .bits         = 8,
    .spck_delay   = 0,
    .trans_delay  = 0,
    .stay_act     = 1,
    .spi_mode     = 0,
    .fdiv         = 0,
    .modfdis      = 1
  };


  // Assign and enable GPIO pins to the ADC function.
  gpio_enable_module(ADC_GPIO_MAP, sizeof(ADC_GPIO_MAP) / sizeof(ADC_GPIO_MAP[0]));
  adc_configure(adc);// configure ADC

  gpio_enable_module(DIP204_SPI_GPIO_MAP,
                     sizeof(DIP204_SPI_GPIO_MAP) / sizeof(DIP204_SPI_GPIO_MAP[0]));// Assign I/Os to SPI
  spi_initMaster(DIP204_SPI, &spiOptions);  // Initialize as master
  spi_selectionMode(DIP204_SPI, 0, 0, 0);  // Set selection mode: variable_ps, pcs_decode, delay
  spi_enable(DIP204_SPI);  // Enable SPI
  spi_setupChipReg(DIP204_SPI, &spiOptions, FOSC0);  // setup chip registers

  dip204_example_configure_push_buttons_IT();  // configure local push buttons
  dip204_example_configure_joystick_IT();  // configure local joystick

  dip204_init(backlight_PWM, TRUE);  /* initialize LCD */

  gpio_set_gpio_pin(AVR32_PIN_PB21);

  /* do a loop */


while (1)
{
	if (menu == 1)
	{
		//sam++;
		//dip204_set_backlight_adc(1023-adc_value_light); //A változó megvilágításhoz//IF vége

		if ((print_sec) && (!(tc_tick%1000)))
		{
			dip204_clear_display();
			gpio_tgl_gpio_pin(AVR32_PIN_PB21);
			gpio_tgl_gpio_pin(AVR32_PIN_PB22);
			print_sec = 0;

			//itoa(sam/1000,ertek4,10);
			//dip204_set_cursor_position(1,1);
			//dip204_write_string("Cnt: ");
			//dip204_write_string(ertek4);
			//dip204_write_string(" kCPS  ");

			dip204_set_cursor_position(1,1);
			dip204_write_string("JOY u/d to clk");

			//itoa(menu,ertek6,10);
			//dip204_set_cursor_position(16,1);
			//dip204_write_string(ertek6);
			dip204_set_cursor_position(1,2);
			dip204_write_string("Ticks: ");
			u32CountVal = Get_sys_count();
			itoa(1*(u32CountVal-u32CountValLast),ertek5,10);
			u32CountValLast = u32CountVal;
			dip204_write_string(ertek5);
			dip204_write_string(" Hz");
			dip204_set_cursor_position(1,3);
			dip204_write_string(".Temp....Pot....LDR.");

			itoa(adc_value_temp,ertek1,10);
			dip204_set_cursor_position(2,4);
			dip204_write_string(ertek1);

			itoa(adc_value_pot,ertek2,10);
			dip204_set_cursor_position(10,4);
			dip204_write_string(ertek2);

			itoa(adc_value_light,ertek3,10);
			dip204_set_cursor_position(17,4);
			dip204_write_string(ertek3);

			dip204_hide_cursor();
			sam=0;


				adc_enable(adc,adc_channel_temp);
				adc_start(adc);
				adc_value_temp = adc_get_value(adc, adc_channel_temp);
				adc_disable(adc,adc_channel_temp);
				//----------------------------------------
				adc_enable(adc,adc_channel_pot);
				adc_start(adc);
				adc_value_pot = adc_get_value(adc, adc_channel_pot);
				adc_disable(adc,adc_channel_pot);
				//----------------------------------------
				adc_enable(adc,adc_channel_light);
				adc_start(adc);
				adc_value_light = adc_get_value(adc, adc_channel_light);
				adc_disable(adc,adc_channel_light);


		}		//IF vége




	} //IF (menu) vége

	if (menu == 2)
		{
							if (first)
							{
							dip204_clear_display();
							//dip204_set_cursor_position(7,1);
							//dip204_write_string("Counter");
							//dip204_set_cursor_position(7,2);
							//dip204_write_string("value");
							dip204_set_cursor_position(1,1);
							dip204_write_string("last reset: ");
							adatok = pm->rcause ;	//PM regiszter kiolvasás ilyen módon! + deklaráció fent!
							//itoa(caus,ertek7,10);
							if (adatok==1){dip204_write_string("POR");}
							if (adatok==2){dip204_write_string("BOR");}
							if (adatok==4){dip204_write_string("EXT");}
							if (adatok==8){dip204_write_string("WDT");}

							//adatok = gpio_port->pvr >> (21 & 0x1F);


							adatok = AVR32_FLASHC.fcr;	//FLASHC regiszter kiolvasása + deklaráció fent!
							dip204_set_cursor_position(1,2);
							itoa(adatok,ertek7,10);
							dip204_write_string("FLASHC.FCR: ");
							dip204_write_string(ertek7);

							adatok = AVR32_FLASHC.fsr;	//FLASHC regiszter kiolvasása + deklaráció fent!
							dip204_set_cursor_position(1,3);
							itoa(adatok,ertek7,10);
							dip204_write_string("FLASHC.FSR: ");
							dip204_write_string(ertek7);

							//adatok = usart->fidi;

							//adatok = (spi->sr & 0xFF) >> 4;		//SPI SR 8 bitje a 4-12 tartományban
							//adatok = (tc->channel[0].cmr & 0xFFFF); //Timer-counter 0-ás csatorna mode reg alsó 16 bitje

							adatok = (pm->mcctrl & 0x3);
							dip204_set_cursor_position(1,4);
							itoa(adatok,ertek7,2);
							dip204_write_string("MCSEL: ");
							dip204_write_string(ertek7);

							}
							first = 0;






		} //IF (menu) vége

	if (menu == 3)
			{
							if (first)
							{
							dip204_clear_display();
							dip204_set_cursor_position(4,1);
							dip204_write_string("Press joy key");
							dip204_set_cursor_position(4,2);
							dip204_write_string("to start...");
							}
							first = 0;
							teszt = 1;		//Gombkezelő rutinnak info..
							gpio_clr_gpio_pin(AVR32_PIN_PB21);
							gpio_set_gpio_pin(AVR32_PIN_PB22);

							if (joy_pressed == 1)
							{
								while (1)
								  {
									  //gpio_port->ovrt  = 1 << (AVR32_PIN_PB21 & 0x1F); // Toggle the I/O line.
									  //gpio_port->oders = 1 << (AVR32_PIN_PB21 & 0x1F);
									AVR32_GPIO.port[AVR32_PIN_PB21 >> 5].ovrt = 1 << (AVR32_PIN_PB21 & 0x1F);
									AVR32_GPIO.port[AVR32_PIN_PB22 >> 5].ovrt = 1 << (AVR32_PIN_PB22 & 0x1F);
								  }

							}


			} //IF (menu) vége

	if (menu == 4)
				{
							if (first)
							{
							dip204_clear_display();
							dip204_set_cursor_position(7,1);
							dip204_write_string("Counter");
							dip204_set_cursor_position(7,2);
							dip204_write_string("value");
							}
							first = 0;


					sam++;

					if ((print_sec) && (!(tc_tick%100)))
					{
						gpio_tgl_gpio_pin(AVR32_PIN_PB21);
						gpio_tgl_gpio_pin(AVR32_PIN_PB22);
						print_sec = 0;

						itoa(sam,ertek4,10);
						dip204_set_cursor_position(1,3);
						dip204_write_string(ertek4);

						dip204_set_cursor_position(16,1);
						itoa(menu,ertek6,10);
						dip204_write_string(ertek6);

					}		//IF vége



				} //IF (menu) vége

	if (menu == 5)
				{
					if (first)
					{
					dip204_clear_display();
					dip204_set_cursor_position(7,1);
					dip204_write_string("LED fade");
					dip204_set_cursor_position(5,2);
					dip204_write_string("by ADC value.. ");
					}
					first = 0;

					sam++;

					if (sam < adc_value_pot)
						{gpio_set_gpio_pin(AVR32_PIN_PB21);
						 gpio_clr_gpio_pin(AVR32_PIN_PB22);}

					if (sam >= adc_value_pot)
						{gpio_set_gpio_pin(AVR32_PIN_PB22);
						 gpio_clr_gpio_pin(AVR32_PIN_PB21);}

					if (sam > 1024)
						{

							adc_enable(adc,adc_channel_pot);
							adc_start(adc);
							adc_value_pot = adc_get_value(adc, adc_channel_pot);
							//adc_disable(adc,adc_channel_pot);
							itoa(adc_value_pot,ertek2,10);
							dip204_set_cursor_position(10,4);
							dip204_write_string("     ");
							dip204_set_cursor_position(10,4);
							dip204_write_string(ertek2);
							sam=0;
						}



				} //IF (menu) vége


	if (menu == 6)
					{
						if (first)
						{
						dip204_clear_display();
						dip204_set_cursor_position(7,1);
						dip204_write_string("LED fade");
						dip204_set_cursor_position(5,2);
						dip204_write_string("automatic...");
						i=0;
						fel = 1;
						}
						first = 0;

						sam++;

						if (sam < i)
							{gpio_set_gpio_pin(AVR32_PIN_PB21);
							 gpio_clr_gpio_pin(AVR32_PIN_PB22);
							}

						if (sam >= i)
							{gpio_set_gpio_pin(AVR32_PIN_PB22);
							 gpio_clr_gpio_pin(AVR32_PIN_PB21);
							}

						if (sam > 200)
							{

								if (fel == 1) {i++;}
								if (fel == 0) {i--;}

								itoa(i,ertek2,10);
								dip204_set_cursor_position(10,4);
								dip204_write_string("     ");
								dip204_set_cursor_position(10,4);
								dip204_write_string(ertek2);
								sam=0;
								if (i==201) {fel = 0;}
								if (i==0) {fel = 1;}
							}



					} //IF (menu) vége



	if (menu == 7)
				{
		if (first)
									{
									dip204_clear_display();
									dip204_set_cursor_position(1,1);
									dip204_write_string("CLK.SRC: ");
									if ((pm->mcctrl & 0x3) == 0) {dip204_write_string("Slow");}
									if ((pm->mcctrl & 0x3) == 1) {dip204_write_string("Osc0");}
									if ((pm->mcctrl & 0x3) == 2) {dip204_write_string("PLL0");}

									dip204_set_cursor_position(1,2);
									dip204_write_string("Osc0:    ");
									if (((pm->mcctrl & 0x1) >> 2) == 0) {dip204_write_string("Disabled");}
									if (((pm->mcctrl & 0x1) >> 2) == 1) {dip204_write_string("Enabled");}

									dip204_set_cursor_position(1,3);
									dip204_write_string("Osc1:    ");
									if (((pm->mcctrl & 0x1) >> 3) == 0) {dip204_write_string("Disabled");}
									if (((pm->mcctrl & 0x1) >> 3) == 1) {dip204_write_string("Enabled");}
									}
									first = 0;

									dip204_set_cursor_position(1,4);
									dip204_write_string("Osc0Xtl: ");
									if ((pm->oscctrl0 & 0x7) == 0) {dip204_write_string("Ext.clock");}
									if ((pm->oscctrl0 & 0x7) == 4) {dip204_write_string("XTAL.G0");}
									if ((pm->oscctrl0 & 0x7) == 5) {dip204_write_string("XTAL.G1");}
									if ((pm->oscctrl0 & 0x7) == 6) {dip204_write_string("XTAL.G2");}
									if ((pm->oscctrl0 & 0x7) == 7) {dip204_write_string("XTAL.G3");}




				} //IF (menu) vége


	if (menu == 8)
					{
			if (first)
										{
									dip204_clear_display();
									dip204_set_cursor_position(1,1);
									dip204_write_string("GEN.SC0: ");


									if ((pm->gcctrl[0] & 0x03) == 0) {dip204_write_string("Osc0");}
									if ((pm->gcctrl[0] & 0x03) == 1) {dip204_write_string("Osc1");}
									if ((pm->gcctrl[0] & 0x03) == 2) {dip204_write_string("PLL0");}
									if ((pm->gcctrl[0] & 0x03) == 3) {dip204_write_string("PLL1");}

									dip204_set_cursor_position(1,2);
									dip204_write_string("CEN:     ");
									if (((pm->gcctrl[0] & 0x01)>>2) == 0) {dip204_write_string("Stopped");}
									if (((pm->gcctrl[0] & 0x01)>>2) == 1) {dip204_write_string("Running");}

									dip204_set_cursor_position(1,3);
									dip204_write_string("DIV:     ");
									if (((pm->gcctrl[0] & 0x01)>>4) == 0) {dip204_write_string("Undivided");}
									if (((pm->gcctrl[0] & 0x01)>>4) == 1) {dip204_write_string("Divided");}

									dip204_set_cursor_position(1,4);
									adatok = ((pm->gcctrl[0] & 0xFFFF) >>8);
									itoa(adatok,ertek7,16);
									dip204_write_string("DIV.VAL: ");
									dip204_write_string(ertek7);

										}
										first = 0;



					} //IF (menu) vége


	/*if (menu == 9)
						{
				if (first)
											{
										dip204_clear_display();
										dip204_set_cursor_position(1,1);
										dip204_write_string("OscCtrl: ");


										if (((pm->oscctrl0 & 0x07)>>8) == 0) {dip204_write_string("0     cy");}
										if (((pm->oscctrl0 & 0x07)>>8) == 1) {dip204_write_string("64    cy");}
										if (((pm->oscctrl0 & 0x07)>>8) == 2) {dip204_write_string("128   cy");}
										if (((pm->oscctrl0 & 0x07)>>8) == 3) {dip204_write_string("2048  cy");}
										if (((pm->oscctrl0 & 0x07)>>8) == 4) {dip204_write_string("4096  cy");}
										if (((pm->oscctrl0 & 0x07)>>8) == 5) {dip204_write_string("8192  cy");}
										if (((pm->oscctrl0 & 0x07)>>8) == 6) {dip204_write_string("16384 cy");}

											}
											first = 0;



						} //IF (menu) vége   */





} //while vége

} //MAIN vége


