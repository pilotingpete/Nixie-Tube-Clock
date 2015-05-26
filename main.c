/*
 ~~~~~~~~~~~~~~~~~~~******************** Program Description ********************~~~~~~~~~~~~~~~~~~~
 
	Program Title:	Nixie Clock
	Author:			Pete Mills
	Email:			mills.pete@gmail.com
	Website:        petemills.blogspot.com
	Version:		1.0
	Filename:		main.c
	License:		CC BY-NC-SA 3.0 ( Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported )
 
    Clock Description:
    
    This nixie clock is settable using a USB serial interface.  For details please refer to the user
    manual at the above website.
    
    There is a PC app to simplify the setting of the clock.
    
    This clock provides a mechanism for xtal calibration so that it can keep time with it's own clock.
    It also has AC zero cross hardware and software to allow timekeeping with the incoming AC power line frequency.
    
    This clock requires no external RTC and instead relies upon software to keep track of time, date,
    leap year, day of the week, etc. 
    
    This clock keeps a running average of the ambient temperature with a DS18b20 sensor.
    
    The clock can calculate the frequency of the AC power line and display it.
    
    The boost converter for the HV DC supply is driven by the ATMega328.
    
 
    Software:
    This software may borrow from some of the example code listed below:
    http://homepage.hispeed.ch/peterfleury/doxygen/avr-gcc-libraries/group__pfleury__uart.html
    http://teslabs.com/openplayer/docs/docs/other/ds18b20_pre1.pdf
    https://github.com/ColinBrosseau/AVR-UART-Parse-exemple
 
	Hardware Description:
	
	Processor:	ATMega328
	F_CPU:		16384000 Hz
	
	Schematic at petemills.blogspot.com
	
 ~~~~~~~~~~~~~~~~~~~******************** Program Description ********************~~~~~~~~~~~~~~~~~~~
 
 
 
 
 
 ~~~~~~~~~~~~~~~~~~~~~******************** Fuse Settings ********************~~~~~~~~~~~~~~~~~~~~~~~
 
	Default		Description												Fuse Name / Setting
	
	CHANGED		External Crystal Osc.; 16K CK / 14 CK + 65 mS;			[ CKSEL = 1111 SUT = 11 ]
	UNCHANGED	Clock output on PORTB0; 								[ CKOUT = 0 ]
	CHANGED		Divide clock by 8 internally; 							[ CKDIV8 = 0 ]
	UNCHANGED	Boot Reset vector Enabled (default address=$0000); 		[ BOOTRST = 0 ]
	UNCHANGED	Boot Flash section size = 2048 words;					[ BOOTSZ = 00 ]
	UNCHANGED	Preserve EEPROM memory through the Chip Erase cycle;	[ EESAVE=0 ]
	UNCHANGED	Watch-dog Timer always on; 								[ WDTON = 0 ]
	UNCHANGED	Serial program downloading (SPI) enabled; 				[ SPIEN = 0 ]
	UNCHANGED	Debug Wire enable; 										[ DWEN = 0 ]
	UNCHANGED	Reset Disabled (Enable PC6 as i/o pin); 				[ RSTDISBL = 0 ]
	CHANGED		Brown-out detection level at VCC = 2.7v					[ BODLEVEL = 101 ]
	
	AVRDUDE_FUSES = -U lfuse:w:0xff:m -U hfuse:w:0xd9:m -U efuse:w:0xfd:m
 
 ~~~~~~~~~~~~~~~~~~~~~******************** Fuse Settings ********************~~~~~~~~~~~~~~~~~~~~~~~
 */





//~~~~~~~~~~~~~~~~~~~~~~******************** Includes ********************~~~~~~~~~~~~~~~~~~~~~~~~~

#include <stdlib.h>
#include <avr/io.h>
#define __DELAY_BACKWARD_COMPATIBLE__	// To allow passing of variables into delay functions.
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <string.h>

#include "uart.h"
#include "main.h"
#include "ds18b20.h"
#include "ds18b20.c"

//~~~~~~~~~~~~~~~~~~~~~~******************** Includes ********************~~~~~~~~~~~~~~~~~~~~~~~~~





//~~~~~~~~~~~~~~~~~~~~~******************** Definitions ********************~~~~~~~~~~~~~~~~~~~~~~~

#define SW_1 			PD2		// Generic pushbutton switch.
#define SER				PD3		// Shift registers.
#define SRCLK			PD4		// Shift registers.
#define RCLK			PD5		// Shift registers.
#define F_BOOST			PD6		// Boost converter drive signal.
#define LED 			PD7		// An LED for indication.



#define	AC_FREQ			PB0

// Analog inputs.
#define PWR_OK			PC0		// adc_raw[0]
#define HV_FEEDBACK		PC1		// adc_raw[1]
#define BATT_VOLTAGE 	PC2		// adc_raw[2]
#define	RECTIFIED_AC	PC3		// adc_raw[3]

#define TEMPERATURE_ARRAY_SIZE  6

#define UART_BAUD_RATE  57600

#define STRING_LENGTH   100		// UART parsing.
#define CHAR_NEWLINE    '\n'
#define CHAR_RETURN     '\r'
#define RETURN_NEWLINE  "\r\n"

#define FIRST_ADC_INPUT 0		// lowest ADC channel to sample
#define ADC_CHANNELS 	4		// number of ADC channels to sample
#define ADC_VREF_TYPE 	64		// AVCC with external capacitor at AREF pin

#define AC_FRQ_FILT_SEC	60		// Number of seconds to collect zero crossings.  These zero crossings
								// will then be averaged anc computed to a frequency stored in acFrequency.
								
#define AC_HERTZ		60		// number of AC cycles per second.  60 Hz (typ) North America, 50 Hz (typ) Europe/Asia

//~~~~~~~~~~~~~~~~~~~~~******************** Definitions ********************~~~~~~~~~~~~~~~~~~~~~~~





//~~~~~~~~~~~~~~~~~~~******************** Global Variables ********************~~~~~~~~~~~~~~~~~~~~

const int SERIAL_NUMBER = 100;  	// Start at 100.


uint8_t uartStringIndex = 0; 
unsigned char uartString[STRING_LENGTH];

char myUartString[STRING_LENGTH];   // A working copy of uartString.

char printBuffer[ 100 ];            // For UART print

uint16_t serialInInteger = 0;       // The parsed value of an serial input command.

uint8_t hours   = 10;				// these keep track of the current date and time.
uint8_t minutes = 10;
uint8_t day     = 1;
uint8_t month   = 2;
uint16_t year   = 2015;
volatile uint8_t seconds = 30;

volatile uint32_t mscal = 0;	// F_CPU / (F_CPU * PPM_ERROR); if mscal == 0, disable adjustment
uint32_t EEMEM ee_mscal = 0;	// For storing to eeprom.


uint8_t prevSeconds = 0;		// For firing a function every x seconds.
uint8_t prevMinutes = 0;		// For firing a function every x minutes.

uint8_t miltime = 1;        			// '1' to display 24-hour format, '0' to display 12-hour format
uint8_t EEMEM ee_miltime = 1;			// for storing miltime to eeprom.
uint8_t showtherm = 0;      			// '1' to display the temperature, '0' to have no temperature display
uint8_t EEMEM ee_showtherm = 0;			// For eeprom storage.
uint8_t showfreq = 0;      				// '1' to display the AC line frequency, '0' to have no frequency display
uint8_t EEMEM ee_showfreq = 0;			// For eeprom storage.
uint8_t showdate = 0;					// '1' to show the date on the nixies every so often.  '0' to disable.
uint8_t EEMEM ee_showdate = 0;			// For eeprom storage.
uint8_t celsius = 1;        			// '1' to display temperature on the celsius scale, '0' for fahrenheit
uint8_t EEMEM ee_celsius = 1;			// For eeprom storage.
uint8_t doecho = 1;        				// '1' to echo back on the UART
uint8_t EEMEM ee_doecho = 1;			// For eeprom storage.
volatile uint8_t acclock = 1;       	// '1' to use the AC power line frequency as a timebase.  '0' to use internal xtal
uint8_t EEMEM ee_acclock = 1;			// For eeprom storage.
uint8_t xtalisfast = 1;     			// '1' if the crystal is running faster than "real time", 0 if slower
uint8_t EEMEM ee_xtalisfast = 1;		// For eeprom Storage.
uint8_t toggleled = 1;     				// '1' to toggle the onboard LED at 1 Hz. "0" for LED OFF
uint8_t EEMEM ee_toggleled = 1;			// For eeprom Storage.

uint32_t nixiesleepstart = 0;			// Value for shutting down the display on weekday nights.  24 hour format, no colon.
										// Setting to 0 disables the feature.  A value of 1300 means 1:00 PM
uint32_t EEMEM ee_nixiesleepstart = 0;	// For eeprom storage.
uint32_t nixiesleepend = 0;				// This sets the time to turn on the nixies after being off since nixiesleepstart
										// The same 24 hr, no colon time format applies.  
										// Does not verify that the end time is after the start time.  You do.
uint32_t EEMEM ee_nixiesleepend = 0;	// Storing to eeprom.


double celsiusTemperature = 99.0;    		// The temperature read from the DS18b20 in celsius
double averageCelsiusTemperature = 99.0;	// FIR filter for temperature.
double arraySum = 0;						// FIR filter for temperature.

double movingAverageArray[ TEMPERATURE_ARRAY_SIZE ];

volatile uint8_t boostConverterPwm = 0;		// Current PWM value for the boost converter.
volatile uint8_t boostHighPwm = 80;			// High PWM value for the boost converter.
volatile uint8_t boostLowPwm = 30;			// Low PWM value for the boost converter.

volatile uint8_t adc_raw[ADC_CHANNELS];		// For storing the raw ADC counts of conversions.

volatile uint16_t acZeroCrossCounter = 0;	// For calculating the AC power line frequency.
volatile uint16_t acZeroCrossSnapshot = 0;	// For evaluating the contents at a specific time point.
volatile double acFrequency = 0.0;			// For calculating the AC power line frequency.
volatile uint8_t acFreqFlag = 0;			// When set to "1" the contents of acFrequency contain the 
											// total number of zero crossings since the last measure.

uint8_t cathodePoisonFlag = 0;				// Flag to tell the app to run the anti-poison routine.

//~~~~~~~~~~~~~~~~~~~******************** Global Variables ********************~~~~~~~~~~~~~~~~~~~~




//~~~~~~~~~~~~~~~~~~~~~~~~~******************** MAIN ********************~~~~~~~~~~~~~~~~~~~~~~~~~~

int main(void)
{
    cli();
    ioInit();
    adcInit();
    boostConverterInit();
    rtcInit();
    uartInit();
    extInterruptInit();
    readEepromToVars();  
    sei();
    
    uartWelcome();


    while( 1 )
    {
    	
    	updateAcFrequency();			// Every AC_FRQ_FILT_SEC seconds, compute the average.
    	updateTemperatureFilter( 20 );	// FIR averaging filter for the DS18b20.
        updateNixieOutputState();		// Turns ON or OFF the nixie tubes based on input power and sleep window.
      	checkSerialIn();				// Is there new serial data available?
		updateDateTime();				// Keep track of the current date and time.
      
        // If the time has changed, update the display.
        if( seconds != prevSeconds)
        {
            nixiePrintTime( 1 );	// Display the current time on the nixie tubes and toggle the decimal.   
            prevSeconds = seconds;
            
        }


        if( minutes != prevMinutes)
        {
        	cathodePoisonFlag = 1;		// Set every minute.  Cleared in checkCathodePoison(). 
            
            serialPrintDateTime();
            serialPrintTemperature();
            serialPrintAcFrequency();
            uart_puts(RETURN_NEWLINE);

            
            // On the nixie.
            if( showdate ){
            	nixiePrintDate();
            	_delay_ms( 5000 );
            }
            
            // On the nixie.
           	if( showtherm ){
           		nixiePrintTemperature();
           		_delay_ms( 5000 );
           	}
           	
           	// On the nixie.
           	if( showfreq ){
           		nixiePrintAcFrequency();
           		_delay_ms( 5000 );
           	}
           
            prevMinutes = minutes;   
        }
        
        checkCathodePoison();			// do the antipoison routine if ready.
        
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~******************** MAIN ********************~~~~~~~~~~~~~~~~~~~~~~~~~~




void ioInit(void)
{
    //~~~~~~~~~~~~~~~~~~******************** Pin Configuration ********************~~~~~~~~~~~~~~~~~~~~
    
    // outputs
    
    DDRD |= ( ( 1 << SW_1 ) | ( 1 << SER ) | ( 1 << SRCLK ) | ( 1 << RCLK ) | ( 1 << F_BOOST ) | ( 1 << LED ) );	// set  to "1" for output
    PORTD &= ~( ( 1 << SW_1 ) | ( 1 << SER ) | ( 1 << SRCLK ) | ( 1 << RCLK ) |( 1 << F_BOOST ) | ( 1 << LED ) );	// set the outputs low
    
    
    // inputs
    
    DDRD &= ~( 1 << SW_1 );	// set pin to 0 for input
    PORTD |= ( 1 << SW_1 );	// enable internal pullup
    
    DDRB &= ~( 1 << AC_FREQ );	// set pin to 0 for input
    PORTB |= ( 1 << AC_FREQ );	// enable internal pullup
    
    DDRC &= ~( ( 1 << PWR_OK ) | ( 1 << HV_FEEDBACK ) | ( 1 << BATT_VOLTAGE ) | ( 1 << RECTIFIED_AC ) ); 	
    
    //~~~~~~~~~~~~~~~~~~******************** Pin Configuration ********************~~~~~~~~~~~~~~~~~~~~
}





void extInterruptInit( void )
{
	//~~~~~~~~~~~~~~~~~~************** External Interrupt Configuration ************~~~~~~~~~~~~~~~~~~~~

    PCICR = 0x01; 	// Allow pin change interrupts.
    PCMSK0 = 0x01; 	// Enable interrupts specifically for pin PCINT0 
    
    //~~~~~~~~~~~~~~~~~~************** External Interrupt Configuration ************~~~~~~~~~~~~~~~~~~~~
}





void rtcInit( void )
{
	//~~~~~~~~~~~~~~~~~~***************** RTC Timer Configuration *****************~~~~~~~~~~~~~~~~~~~~

    OCR1AH = 0;                 // Interrupt at 1.000 kHz, 0.001 S.
    OCR1AL = 63;                // Decimal 63, one less than 64000/1000 because 0 and top are counted.
    TIMSK1 |= ( 1 << OCIE1A );  // enable OCR1A match interrupt
    TCCR1B |= (( 1 << WGM12 ) | ( 1 << CS12 ));	// start timer, CTC mode OCR1A match, ck/div by 256 for 64000 Hz
    
    //~~~~~~~~~~~~~~~~~~***************** RTC Timer Configuration *****************~~~~~~~~~~~~~~~~~~~~
}





void adcInit( void )
{
	//~~~~~~~~~~~~~~~~~~******************** ADC Configuration ********************~~~~~~~~~~~~~~~~~~~~

  	// ADC clock prescale of 1/128 for 128 kHz
  	ADCSRA = ( ( 1 << ADEN ) | ( 1 << ADATE ) | ( 1 << ADIE ) | ( 1 << ADPS2 ) | ( 1 << ADPS1 ) | ( 1 << ADPS0 ) );
  	
  	ADCSRB |= (1<<ADTS2);
  	
  	// Disable digital input buffers to save power.
  	DIDR0 = ( ( 1 << ADC0D ) | ( 1 << ADC1D ) | ( 1 << ADC2D ) | ( 1 << ADC3D ) );

    //~~~~~~~~~~~~~~~~~~******************** ADC Configuration ********************~~~~~~~~~~~~~~~~~~~~
}




void boostConverterInit( void )
{
	//~~~~~~~~~~~~~~~~~~************** Boost Converter Configuration **************~~~~~~~~~~~~~~~~~~~~

	// PWM Frequency: 64 kHz
	
	// Timer/Counter0: channel: A, clear on compare match ( non-inverting ), Fast PWM, TOP = 0xFF
	TCCR0A |= ((1<<COM0A1) | (1<<WGM01) | (1<<WGM00));
	TCCR0B |= ( 1 << CS00 );	// internal clock as source, no prescale, start the timer

    //~~~~~~~~~~~~~~~~~~************** Boost Converter Configuration **************~~~~~~~~~~~~~~~~~~~~
}





//~~~~~~~~~~~~~~~~~~~~~~******************** Functions ********************~~~~~~~~~~~~~~~~~~~~~~~~

// ADC interrupt service routine.
// This is called at the completion of each ADC read cycle.
// At the end of the ISR, the next conversion is started and we exit.
// When that conversion is complete, this function is called again.
ISR(ADC_vect) 
{
static unsigned char input_index=0;

// Read the AD conversion result
   adc_raw[input_index]=ADCH;
   
// Select next ADC input
   if (++input_index >= ADC_CHANNELS)
      {
      input_index=0;
      }

   ADMUX=(FIRST_ADC_INPUT | ADC_VREF_TYPE | ( 1 << ADLAR ) ) + input_index; //and left adjust

// Start the AD conversion
   ADCSRA |= (1<<ADSC);

} 



// External interrupt connected to the AC optocoupler.
// Interrupt triggers on rising and falling edges, so double your AC_HERTZ.
// This function will only be called so long as there is an AC input waveform.
// For that reason, we do not need to code for when we lose AC power.  We just have
// to say, if this function is called and we want to use the AC frequency as a timebase
// then increment our seconds.  In ISR( TIMER1_COMPA_vect ) we have to have a bit more logic.
ISR (PCINT0_vect)
{
	static uint8_t timebaseCounter = 0;		// For timekeeping
	
	timebaseCounter++;	// Counting AC cycles.
	
	if( acclock == 1 ){
	
		if( timebaseCounter > ( AC_HERTZ * 2 ) - 1 ){
		
			seconds++;	// Increment the main seconds counter.
			
			timebaseCounter = 0;
		}
		
	}
	
	acZeroCrossCounter++;	// For measuring the AC frequency - not used for timekeeping
	
}





void toggle_led()
{
    PORTD ^= ( 1 << LED );
}





void blip_led()
{
    PORTD |= ( 1 << LED );	// on
    _delay_ms(3);
    PORTD &= ~( 1 << LED );	// off
}





int is_switch_pressed( char port, char pin, int ms_debounce, int ms_block )
{
    
    if ( !( port & ( 1 << pin ) ) )
    {
        //_delay_ms( ms_debounce );
        if ( !( port & ( 1 << pin ) ) ) return 1;
    }
    
    return 0;
}




uint32_t splitString(char aString[100])
{
    
    char *splitChar;
    char subString[100];
    
    // find the equal symbol.  Var to the left, value to the right
    splitChar = strchr( aString, '=');
    
    strcpy(subString, splitChar + 1);
    
    return atol( subString );
}




void copy_command ()
{
    // Move a copy of uartString to myuartString
    memmove(myUartString, uartString, sizeof( myUartString ) );
    // Empty the uartString.
    memset(uartString, 0, sizeof( myUartString ) );
}




uint8_t inRange( uint32_t inputValue, uint32_t upperBound, uint32_t lowerBound )
{
	if( inputValue >= lowerBound && inputValue <= upperBound )
	{
		return 1;
	}
	
	return 0;
}





void process_command()
{
    if(strcasestr(myUartString,"hours") != NULL){
        if(strcasestr(myUartString,"?") != NULL){
            print_value("Hours", hours);
        }else{
        	if(strcasestr(myUartString,"=") != NULL){
        		if( inRange( splitString(myUartString), 23, 0 ) )
            		hours = splitString(myUartString);
            }
        }
    }
    
    else if(strcasestr(myUartString,"minutes") != NULL){
        if(strcasestr(myUartString,"?") != NULL){
            print_value("Minutes", minutes);
        }else{
        	if(strcasestr(myUartString,"=") != NULL){
        		if( inRange( splitString(myUartString), 59, 0 ) )
            		minutes = splitString(myUartString);
            }
        }
    }
    
    else if(strcasestr(myUartString,"seconds") != NULL){
        if(strcasestr(myUartString,"?") != NULL){
            print_value("Seconds", seconds);
        }else{
        	if(strcasestr(myUartString,"=") != NULL){
        		if( inRange( splitString(myUartString), 59, 0 ) )
            		seconds = splitString(myUartString);
            }
        }
    }
    
    else if(strcasestr(myUartString,"mscal") != NULL){
        if(strcasestr(myUartString,"?") != NULL){
            print_value("mS Calibration", mscal);
        }else{
        	if(strcasestr(myUartString,"=") != NULL){
        		if( inRange( splitString(myUartString), 4294967295, 0 ) )
            		mscal = splitString(myUartString);
            }
        }
    }
    
    else if(strcasestr(myUartString,"miltime") != NULL){
        if(strcasestr(myUartString,"?") != NULL){
            print_value("MIL Time", miltime);
        }else{
        	if(strcasestr(myUartString,"=") != NULL){
        		if( inRange( splitString(myUartString), 1, 0 ) )
            		miltime = splitString(myUartString);
            }
        }
    }
    
    else if(strcasestr(myUartString,"year") != NULL){
        if(strcasestr(myUartString,"?") != NULL){
            print_value("Year", year);
        }else{
        	if(strcasestr(myUartString,"=") != NULL){
        		if( inRange( splitString(myUartString), 2525, 0 ) )
            		year = splitString(myUartString);
            }
        }
    }
    
    else if(strcasestr(myUartString,"month") != NULL){
        if(strcasestr(myUartString,"?") != NULL){
            print_value("Month", month);
        }else{
        	if(strcasestr(myUartString,"=") != NULL){
        		if( inRange( splitString(myUartString), 12, 0 ) )
            		month = splitString(myUartString);
            }
        }
    }
    
    else if(strcasestr(myUartString,"day") != NULL){
        if(strcasestr(myUartString,"?") != NULL){
            print_value("Day", day);
        }else{
        	if(strcasestr(myUartString,"=") != NULL){
        		if( inRange( splitString(myUartString), 31, 0 ) )
            		day = splitString(myUartString);
            }
        }
    }
    
    else if(strcasestr(myUartString,"celsius") != NULL){
        if(strcasestr(myUartString,"?") != NULL){
            print_value("Celsius", celsius);
        }else{
        	if(strcasestr(myUartString,"=") != NULL){
        		if( inRange( splitString(myUartString), 1, 0 ) )
            		celsius = splitString(myUartString);
            }
        }
    }
    
    else if(strcasestr(myUartString,"doecho") != NULL){
        if(strcasestr(myUartString,"?") != NULL){
            print_value("Do Echo", doecho);
        }else{
        	if(strcasestr(myUartString,"=") != NULL){
        		if( inRange( splitString(myUartString), 1, 0 ) )
            		doecho = splitString(myUartString);
            }
        }
    }
    
    else if(strcasestr(myUartString,"acclock") != NULL){
        if(strcasestr(myUartString,"?") != NULL){
            print_value("AC Clock", acclock);
        }else{
        	if(strcasestr(myUartString,"=") != NULL){
        		if( inRange( splitString(myUartString), 1, 0 ) )
            		acclock = splitString(myUartString);
            }
        }
    }
    
    else if(strcasestr(myUartString,"xtalisfast") != NULL){
        if(strcasestr(myUartString,"?") != NULL){
            print_value("xtalisfast", xtalisfast);
        }else{
        	if(strcasestr(myUartString,"=") != NULL){
        		if( inRange( splitString(myUartString), 1, 0 ) )
            		xtalisfast = splitString(myUartString);
            }
        }
    }
    
	else if(strcasestr(myUartString,"showtherm") != NULL){
        if(strcasestr(myUartString,"?") != NULL){
            print_value("Show Therm", showtherm);
        }else{
        	if(strcasestr(myUartString,"=") != NULL){
        		if( inRange( splitString(myUartString), 1, 0 ) )
            		showtherm = splitString(myUartString);
            }
        }
    }
    
    else if(strcasestr(myUartString,"showfreq") != NULL){
        if(strcasestr(myUartString,"?") != NULL){
            print_value("Show Freq", showfreq);
        }else{
        	if(strcasestr(myUartString,"=") != NULL){
        		if( inRange( splitString(myUartString), 1, 0 ) )
            		showfreq = splitString(myUartString);
            }
        }
    }
	
	else if(strcasestr(myUartString,"showdate") != NULL){
        if(strcasestr(myUartString,"?") != NULL){
            print_value("Show Date", showdate);
        }else{
        	if(strcasestr(myUartString,"=") != NULL){
        		if( inRange( splitString(myUartString), 1, 0 ) )
            		showdate = splitString(myUartString);
            }
        }
    }
    
    else if(strcasestr(myUartString,"toggleled")!= NULL){
        if(strcasestr(myUartString,"?") != NULL){
            print_value("Toggle LED", toggleled);
        }else{
        	if(strcasestr(myUartString,"=") != NULL){
        		if( inRange( splitString(myUartString), 1, 0 ) )
            		toggleled = splitString(myUartString);
            }
        }
    }
    
    else if(strcasestr(myUartString,"nixiesleepstart") != NULL){
        if(strcasestr(myUartString,"?") != NULL){
            print_value("Nixie Sleep Start Time", nixiesleepstart);
        }else{
        	if(strcasestr(myUartString,"=") != NULL){
        		if( inRange( splitString(myUartString), 2358, 0 ) )
            		nixiesleepstart = splitString(myUartString);
            }
        }
    }
    
    else if(strcasestr(myUartString,"nixiesleepend") != NULL){
        if(strcasestr(myUartString,"?") != NULL){
            print_value("Nixie Sleep End Time", nixiesleepend);
        }else{
        	if(strcasestr(myUartString,"=") != NULL){
        		if( inRange( splitString(myUartString), 2359, nixiesleepstart + 1 ) )
            		nixiesleepend = splitString(myUartString);
            }
        }
    }
    
    
    else if(strcasestr(myUartString,"getall") != NULL){
    	
    	print_csv( ",", hours );					
    	print_csv( ",", minutes );					
    	print_csv( ",", seconds );					
    	print_csv( ",", mscal );					
    	print_csv( ",", miltime );					
    	print_csv( ",", year );						
    	print_csv( ",", month );					
    	print_csv( ",", day );						
    	print_csv( ",", celsius );					
    	print_csv( ",", doecho );					
    	print_csv( ",", xtalisfast );				
    	print_csv( ",", showtherm );				
    	print_csv( ",", showdate );					
    	print_csv( ",", toggleled );				
    	print_csv( ",", nixiesleepstart );			
    	print_csv( ",", nixiesleepend );				
    	print_csv( ",", SERIAL_NUMBER );			
    	
    	// PWR_OK
    	double Volts = adc_raw[0] * 0.019375;
        dtostrf(Volts , 10, 2, printBuffer);
        uart_puts(printBuffer);
        uart_puts(",");
        
        // HV_Feedback
        Volts = adc_raw[1] * 1.9375;
        dtostrf(Volts , 10, 2, printBuffer);
        uart_puts(printBuffer);
        uart_puts(",");
        
        // Battery Voltage
        Volts = adc_raw[2] * 0.019375;
        dtostrf(Volts , 10, 2, printBuffer);
        uart_puts(printBuffer);
        uart_puts(",");
        
        // Rectified AC voltage
        Volts = adc_raw[3] * 0.11;
        dtostrf(Volts , 10, 2, printBuffer);
        uart_puts(printBuffer);
    	
    }
    
    else if( strcasestr( myUartString, "help" ) != NULL ){
    	// print instructions
    	uart_puts("Command");			uart_puts("\t\t");	uart_puts("Set/Read");	uart_puts("\t");	uart_puts("Low Lim");				uart_puts("\t\t\t");	uart_puts("High Lim");		uart_puts( RETURN_NEWLINE );	
    	uart_puts("hours");				uart_puts("\t\t");	uart_puts("=/?");		uart_puts("\t\t");	uart_puts("0");						uart_puts("\t\t\t");	uart_puts("23");			uart_puts( RETURN_NEWLINE );
    	uart_puts("minutes");			uart_puts("\t\t");	uart_puts("=/?");		uart_puts("\t\t");	uart_puts("0");						uart_puts("\t\t\t");	uart_puts("59");			uart_puts( RETURN_NEWLINE );
    	uart_puts("seconds");			uart_puts("\t\t");	uart_puts("=/?");		uart_puts("\t\t");	uart_puts("0");						uart_puts("\t\t\t");	uart_puts("59");			uart_puts( RETURN_NEWLINE );
    	uart_puts("mscal");				uart_puts("\t\t");	uart_puts("=/?");		uart_puts("\t\t");	uart_puts("0");						uart_puts("\t\t\t");	uart_puts("4294967295");	uart_puts( RETURN_NEWLINE );
    	uart_puts("miltime");			uart_puts("\t\t");	uart_puts("=/?");		uart_puts("\t\t");	uart_puts("0");						uart_puts("\t\t\t");	uart_puts("1");				uart_puts( RETURN_NEWLINE );
    	uart_puts("year");				uart_puts("\t\t");	uart_puts("=/?");		uart_puts("\t\t");	uart_puts("0");						uart_puts("\t\t\t");	uart_puts("2525");			uart_puts( RETURN_NEWLINE );
    	uart_puts("month");				uart_puts("\t\t");	uart_puts("=/?");		uart_puts("\t\t");	uart_puts("0");						uart_puts("\t\t\t");	uart_puts("12");			uart_puts( RETURN_NEWLINE );
    	uart_puts("day");				uart_puts("\t\t");	uart_puts("=/?");		uart_puts("\t\t");	uart_puts("0");						uart_puts("\t\t\t");	uart_puts("31");			uart_puts( RETURN_NEWLINE );
    	uart_puts("celsius");			uart_puts("\t\t");	uart_puts("=/?");		uart_puts("\t\t");	uart_puts("0");						uart_puts("\t\t\t");	uart_puts("1");				uart_puts( RETURN_NEWLINE );
    	uart_puts("doecho");			uart_puts("\t\t");	uart_puts("=/?");		uart_puts("\t\t");	uart_puts("0");						uart_puts("\t\t\t");	uart_puts("1");				uart_puts( RETURN_NEWLINE );
    	uart_puts("accclock");			uart_puts("\t");	uart_puts("=/?");		uart_puts("\t\t");	uart_puts("0");						uart_puts("\t\t\t");	uart_puts("1");				uart_puts( RETURN_NEWLINE );
    	uart_puts("xtalisfast");		uart_puts("\t");	uart_puts("=/?");		uart_puts("\t\t");	uart_puts("0");						uart_puts("\t\t\t");	uart_puts("1");				uart_puts( RETURN_NEWLINE );
    	uart_puts("showtherm");			uart_puts("\t");	uart_puts("=/?");		uart_puts("\t\t");	uart_puts("0");						uart_puts("\t\t\t");	uart_puts("1");				uart_puts( RETURN_NEWLINE );
    	uart_puts("showfreq");			uart_puts("\t");	uart_puts("=/?");		uart_puts("\t\t");	uart_puts("0");						uart_puts("\t\t\t");	uart_puts("1");				uart_puts( RETURN_NEWLINE );
    	uart_puts("showdate");			uart_puts("\t");	uart_puts("=/?");		uart_puts("\t\t");	uart_puts("0");						uart_puts("\t\t\t");	uart_puts("1");				uart_puts( RETURN_NEWLINE );
    	uart_puts("toggleled");			uart_puts("\t");	uart_puts("=/?");		uart_puts("\t\t");	uart_puts("0");						uart_puts("\t\t\t");	uart_puts("1");				uart_puts( RETURN_NEWLINE );
    	uart_puts("nixiesleepstart");	uart_puts("\t");	uart_puts("=/?");		uart_puts("\t\t");	uart_puts("0");						uart_puts("\t\t\t");	uart_puts("2358");			uart_puts( RETURN_NEWLINE );
    	uart_puts("nixiesleepend");		uart_puts("\t");	uart_puts("=/?");		uart_puts("\t\t");	uart_puts("nixiesleepstart + 1");	uart_puts("\t");		uart_puts("2359");			uart_puts( RETURN_NEWLINE );
    	uart_puts("getall");			uart_puts("\t\t");	uart_puts("[ENTER]");	uart_puts( RETURN_NEWLINE );
    	uart_puts("ramtoeeprom");		uart_puts("\t");	uart_puts("[ENTER]");	uart_puts( RETURN_NEWLINE );
    	uart_puts("poke");				uart_puts("\t\t");	uart_puts("[ENTER]");	uart_puts( RETURN_NEWLINE );
    	uart_puts("pwrok");				uart_puts("\t\t");	uart_puts("[ENTER]");	uart_puts( RETURN_NEWLINE );
    	uart_puts("hvfeedback");		uart_puts("\t");	uart_puts("[ENTER]");	uart_puts( RETURN_NEWLINE );
    	uart_puts("battvoltage");		uart_puts("\t");	uart_puts("[ENTER]");	uart_puts( RETURN_NEWLINE );
    	uart_puts("rectifiedac");		uart_puts("\t");	uart_puts("[ENTER]");	uart_puts( RETURN_NEWLINE );
    }
    
    else if(strcasestr(myUartString,"ramtoeeprom") != NULL){
    	updateVarsToEeprom();   
		uart_puts( "EEPROM Storage Complete");
    }
    
    else if(strstr(myUartString,"poke") != NULL){
        
        uart_puts( "Hello!");
        uart_puts( RETURN_NEWLINE );
        uart_puts( "I'm Serial Number: ");
        uart_puts( itoa( SERIAL_NUMBER, printBuffer, 10 ) );
        
    }
    
    else if(strstr(myUartString,"pwrok") != NULL){
        // ~0.019375 V/count
        double pwrokVolts = adc_raw[0] * 0.019375;
        dtostrf(pwrokVolts , 10, 2, printBuffer);
        uart_puts(printBuffer);
    }
    
    else if(strstr(myUartString,"hvfeedback") != NULL){
        // ~1.9375 V/count
        double hvVolts = adc_raw[1] * 1.9375;
        dtostrf(hvVolts , 10, 2, printBuffer);
        uart_puts(printBuffer);
    }
    
    else if(strstr(myUartString,"battvoltage") != NULL){
        // ~0.019375 V/count
        double battVolts = adc_raw[2] * 0.019375;
        dtostrf(battVolts , 10, 2, printBuffer);
        uart_puts(printBuffer);
    }
    
    else if(strstr(myUartString,"rectifiedac") != NULL){
    	// 1K / 1K + 4K7 = 1K / 5K7 = ~0.175438 
    	// 0.175 ^-1 = 5.7 
        // ~0.11 V/count
        double rectVolts = adc_raw[3] * 0.11;
        dtostrf(rectVolts , 10, 2, printBuffer);
        uart_puts(printBuffer);
    }
    uart_puts( RETURN_NEWLINE );
}





void print_value (char *id, uint32_t value)
{
    ultoa(value, printBuffer, 10);
    uart_puts(id);
    uart_puts(": ");
    uart_puts(printBuffer);
    //uart_puts(RETURN_NEWLINE);
}



void print_csv( char *id, uint32_t value )
{
    ultoa(value, printBuffer, 10);

    //uart_puts(": ");
    uart_puts(printBuffer);
        uart_puts(id);
    //uart_puts(RETURN_NEWLINE);
}




void uart_ok()
{
    uart_puts("OK");
    uart_puts(RETURN_NEWLINE);
}




void checkSerialIn()
{
    
    unsigned int aChar = uart_getc();
    
    if ( aChar & UART_NO_DATA )
    {
        
         // No data available from UART

    }
    else
    {

        /*
         * new data available from UART
         * check for Frame or Overrun error
         */
        if ( aChar & UART_FRAME_ERROR )
        {
            /* Framing Error detected, i.e no stop bit detected */
            uart_puts_P("UART Frame Error: ");
        }
        if ( aChar & UART_OVERRUN_ERROR )
        {
            /*
             * Overrun, a character already present in the UART UDR register was
             * not read by the interrupt handler before the next character arrived,
             * one or more received characters have been dropped
             */
            uart_puts_P("UART Overrun Error: ");
        }
        if ( aChar & UART_BUFFER_OVERFLOW )
        {
            /*
             * We are not reading the receive buffer fast enough,
             * one or more received character have been dropped
             */
            uart_puts_P("Buffer overflow error: ");
        }
        /*
         * send received character back
         */
        
    // Add aChar to the growing string.
    uartString[uartStringIndex] = aChar;
    
    // Keep going until we see a CHAR_RETURN.
    if (uartString[uartStringIndex] == CHAR_RETURN) {
        // Reset to 0, ready to go again
        uartStringIndex = 0;
        uart_puts(RETURN_NEWLINE);
        
        copy_command();
        process_command();
        }
    else {
        uartStringIndex++;
    }
    
    if( doecho ){
    	uart_putc( (unsigned char)aChar ); //echo
    }
        
    }
}




void uartWelcome( void )
{

    uart_puts("//~~~~~~~~~~~~************ Nixie Clock Instructions ************~~~~~~~~~~~~~~");
    uart_puts(RETURN_NEWLINE);
    uart_puts("ATMega328 Nixie Clock v1.0");
    uart_puts(RETURN_NEWLINE);
    uart_puts("Check petemills.blogspot.com for more info");
    uart_puts(RETURN_NEWLINE);
    uart_puts(RETURN_NEWLINE);
    uart_puts("Usage Example:");
    uart_puts(RETURN_NEWLINE);
    uart_puts("Type 'HOURS=10' to set the hours to '10'.");
    uart_puts(RETURN_NEWLINE);
    uart_puts("Type 'MINUTES=26' to set the minutes to '26'.");
    uart_puts(RETURN_NEWLINE);
    uart_puts("Further parameter names can be found in the user manual.");
    uart_puts(RETURN_NEWLINE);
    uart_puts("Visit the URL above to download a clock calibration app and user manual.");
    uart_puts(RETURN_NEWLINE);
    uart_puts("//~~~~~~~~~~~~************ Nixie Clock Instructions ************~~~~~~~~~~~~~~");
}




void uartInit( void )
{
        uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );
}





// This interrupt is called at 1kHz
// Here we are keeping track of milliseconds and adjusting them slightly to get 1 clock second
// to equal 1 real world second very closely.
// Set via serial comms,  mscal = F_CPU / (F_CPU * PPM_ERROR), then set fast or slow.
ISR( TIMER1_COMPA_vect )
{
    static uint16_t milliSeconds = 0;		// mS value for timekeeping 1000mS/1S
    static uint32_t clockCalCounter = 0;	// counting up the milliseconds to mscal
    const uint16_t MS_IN_SEC = 1000;		// 1000mS/1S
    static uint16_t acFreqFiltCtr = 0;		// this is incremented until it == AC_FRQ_FILT_SEC
    milliSeconds++;
    clockCalCounter++;
    
    
    if( milliSeconds >= MS_IN_SEC )
    {
    	if( acclock == 0 || acPowerOk() == 0 ){	// If we are meant to use the internal xtal, not power line frequency for timekeeping.
    											// Or, if we loose AC power, and we are in AC clock mode we should step in and keep time here.
        	seconds++;          // increment seconds
        }
        
        	milliSeconds = 0;   // reset milliseconds
        
        	if( toggleled )		// Clearly toggling the LED is only available when using the internal xtal.
        		toggle_led();	// Toggle the LED if allowed.
        	acFreqFiltCtr++;	// Increment the frequency filter counter.	
    }
    
    // For measuring the power line frequency.
	if( acFreqFiltCtr >= AC_FRQ_FILT_SEC )
	{
		acZeroCrossSnapshot = acZeroCrossCounter;	// Grab the current number of zero crossings since the last.
		acZeroCrossCounter = 0;				// Reset the number of zero crossings.
		acFreqFlag = 1;						// Signal that a computation is ready to be made.
		acFreqFiltCtr = 0;					// Reset the measurement interval counter.
	}

    

    // Only adjust the time if mscal is not 0, this way we can disable the
    // adjustment during the calibration procedure.
    if( mscal != 0 )
    {
        if( xtalisfast == 1 )
        {
            if( ( clockCalCounter >= mscal ) && ( milliSeconds > 1 ))
            {
                milliSeconds--;
                clockCalCounter = clockCalCounter - mscal;
            }
        }
        else
        {
            // milliseconds must be less than 999 to avoid missing an adjustment.
            // eg if milliseconds were to be 999 and we increment it here to 1000
            // the next ISR call will make it 1001 and reset to zero just as if it
            // would for 1000 and the adjustment would be effectively canceled out.
            if( ( clockCalCounter >= mscal ) && ( milliSeconds < MS_IN_SEC - 1 ) )
            {
                milliSeconds++;
         
                // it may be that clock_cal_counter > than mscal in which case
                // I want to count the tick towards the next adjustment
                // should always be 1 or 0
                clockCalCounter = clockCalCounter - mscal;
            }
        }
    }
    
    updateBoostConverter();	// Updated here for determinism.
}




void updateDateTime( void )
{
    uint8_t numDaysInMonth[] = {0,31,28,31,30,31,30,31,31,30,31,30,31};
    
    if( numDaysInMonth[2] == 28 && isLeapYear() )
    {
        numDaysInMonth[2] = 29;
    }
    else if( numDaysInMonth[2] == 29 )
    {
        numDaysInMonth[2] = 28;
    }
    
    
    if( seconds > 59 )
    {
        minutes++;		// increment minutes
        seconds = 0;	// reset seconds
    }
    
    if( minutes > 59 )
    {
        hours++;		// increment hours
        minutes = 0; 	// reset minutes
    }
    
    if( hours > 23 )
    {
        day++;          // increment day
        hours = 0;		// reset hours
    }
    
    if( day > numDaysInMonth[ month ] )
    {
        month++;        // increment the month
        day = 1;        // reset the day counter to the first
    }
    
    if( month > 12 )
    {
        year++;         // increment the year
        month = 1;      // reset the month to january
    }
    
    
}




void serialPrintDateTime( void )
{
    //if( isLeapYear() )
    //    uart_puts("L ");
    // Print the date in YYYY.MM.DD format
    uart_puts( itoa( year, printBuffer, 10 ) );
    uart_putc('-');
    
    uart_puts( itoa( month, printBuffer, 10 ) );
    uart_putc('-');
    
    uart_puts( itoa( day, printBuffer, 10 ) );
    
    uart_puts(", ");
    
    
    if( miltime == 0  && hours > 12 )
    {
        // Display the hours in 12 hour format.
        uart_puts( itoa( hours - 12, printBuffer, 10 ) );
    }
    else if( miltime == 0 && hours == 0 )
    {
        // Don't display '0' hours in 12-hour format, display "12" instead.
        uart_puts( itoa( 12, printBuffer, 10 ) );
    }
    else
    {
        // Otherwise, everything is cool, just print the hours.
        uart_puts( itoa( hours, printBuffer, 10 ) );
    }
    uart_putc(':');
        
    uart_puts( itoa( minutes, printBuffer, 10 ) );
    uart_putc(':');
        
    uart_puts( itoa( seconds, printBuffer, 10 ) );
        
    //uart_puts(RETURN_NEWLINE);
    uart_putc( ',');

}




uint8_t isLeapYear( void )
{
    if( year % 4 != 0 )
    {
        return 0;   // not a leap year
    }
    else if( year % 100 != 0 )
    {
        return 1;   // it is a leap year
    }
    else if( year % 400 )
    {
        return 0;   // it is not a leap year
    }
    else
    {
        return 1;   // it is a leap year
    }
}




uint8_t dayOfWeek(uint8_t day, uint8_t month, uint16_t year)
{
   // http://en.wikipedia.org/wiki/Zeller%27s_congruence
   // returns 0 for monday thru 6 for sunday.
   
	if( month < 3 ){
		month += 12;
		year--;
   }
   return ( ( 13 * month + 3 ) / 5 + day + year + ( year / 4 ) - ( year / 100 ) + ( year / 400 ) ) % 7;
}




double readTemperature( void )
{
    celsiusTemperature = ds18b20_gettemp();
    
    return celsiusTemperature;
}




void updateBoostConverter( void )
{

    // The input to HV_FEEDBACK is a voltage divider with 470k0 and 4k7 resistors.
    // Therefore the voltage present will be about 10% of the boost converter output.
    // 175 V DC on the boost converter will be about 1.75 V DC input to the ADC
    
    // The ADC results are continuously stored as 8-bit results in array location adc_raw[1]
    // 5 v / 8 bits = ~0.019685 V/count, or 50.8 counts/V
    
    // ex. 50 V on the boost converter is 0.5 V to the ADC.  0.5 V in counts is 25.4
    //    175 V on the boost converter is 1.75V to the ADC.  1.75V in counts is 88.9
    
    uint16_t myVoltage = adc_raw[1];	// HV_FEEDBACK
    
	if( myVoltage > 89 )
	{
		boostConverterPwm = boostLowPwm;
		//boostConverterPwm--;
	}
	else 
	{
		boostConverterPwm = boostHighPwm;
		//boostConverterPwm++;
	}
	
	OCR0A = boostConverterPwm;        

}






void updateTemperatureFilter( uint8_t moduloSeconds )
{
	if( seconds % moduloSeconds == 0 )
    {
            
        // FIR
            
        for( int i = 0; i < TEMPERATURE_ARRAY_SIZE; i++ )
        {
            if( i < TEMPERATURE_ARRAY_SIZE - 1 )
            {
                movingAverageArray[ i ] = movingAverageArray[ i + 1 ];
            }
            else
            {
                movingAverageArray[ i ] = readTemperature();
            }
                
        }
            
        arraySum = 0;
            
        for( int i = 0; i < TEMPERATURE_ARRAY_SIZE; i++ )
        {
            arraySum += movingAverageArray[ i ];
        }
            
		averageCelsiusTemperature = arraySum / TEMPERATURE_ARRAY_SIZE;       
    }
}




void serialPrintTemperature( void )
{
	if( celsius == 1 )
        {
            //dtostrf(readTemperature(), 10, 3, printBuffer);
            dtostrf(averageCelsiusTemperature, 10, 3, printBuffer);

            //uart_puts(", Temperature: ");
            //uart_putc( ',');
            uart_puts(printBuffer); //uart_puts( RETURN_NEWLINE );
        }
        else
        {
            //double tempF = ( readTemperature() * 1.8 ) + 32;
            double tempF = ( averageCelsiusTemperature * 1.8 ) + 32;
            dtostrf(tempF , 10, 3, printBuffer);
            //uart_puts(", Temperature: ");
            //uart_putc(',');
            uart_puts(printBuffer); //uart_puts( RETURN_NEWLINE );
    }
    
    uart_putc( ',');
}





void serialPrintAcFrequency( void )
{
	dtostrf(acFrequency, 10, 3, printBuffer);
	uart_puts(printBuffer); //uart_puts( RETURN_NEWLINE );
	uart_putc( ',');	
}







void readEepromToVars( void )
{
	
	mscal = eeprom_read_dword( &ee_mscal );	
	nixiesleepstart = eeprom_read_dword( &ee_nixiesleepstart );
	nixiesleepend = eeprom_read_dword( &ee_nixiesleepend );
	
	miltime = eeprom_read_byte( &ee_miltime );
	showtherm = eeprom_read_byte( &ee_showtherm );
	showfreq = eeprom_read_byte( &ee_showfreq );
	showdate = eeprom_read_byte( &ee_showdate );
	celsius = eeprom_read_byte( &ee_celsius );
	doecho = eeprom_read_byte( &ee_doecho );
	acclock = eeprom_read_byte( &ee_acclock );
	xtalisfast = eeprom_read_byte( &ee_xtalisfast );
	toggleled = eeprom_read_byte( &ee_toggleled );

}





// Update will only write data to eeprom if the new value is different from the stored value.
void updateVarsToEeprom( void )
{

	eeprom_update_dword( &ee_mscal, mscal );
	eeprom_update_dword( &ee_nixiesleepstart, nixiesleepstart );
	eeprom_update_dword( &ee_nixiesleepend, nixiesleepend );
	
	eeprom_update_byte( &ee_miltime, miltime );
	eeprom_update_byte( &ee_showtherm, showtherm );
	eeprom_update_byte( &ee_showfreq, showfreq );
	eeprom_update_byte( &ee_showdate, showdate );
	eeprom_update_byte( &ee_celsius, celsius );
	eeprom_update_byte( &ee_doecho, doecho );
	eeprom_update_byte( &ee_acclock, acclock );
	eeprom_update_byte( &ee_xtalisfast, xtalisfast );
	eeprom_update_byte( &ee_toggleled, toggleled );

}




uint8_t acPowerOk( void )
{
	if( adc_raw[0] < 51 ){	// If PWR_OK < ~2 V
		return 0;
	}else{
		return 1;
	}

}




void updateNixieOutputState( void )
{
	uint16_t aTimeToCompare = 0;
	
	// Check to see if we have lost AC power.
	
	
	if( acPowerOk() == 0 )			// If we have lost AC power.
	{
		//
		turnOnOffBoostConverter( 0 );	// Turn off the boost converter.
		return;						// Leave.
	}
	

		// Check to see if the nixies will go to sleep at night.
		if( nixiesleepstart != 0 )
		{
			// We aren't really comparing time here, just some integers.
			// This is because 10 AM is stored as 10000 in the sleep time vars.
			// This is OK, as it works out mathematically the same.
			
			aTimeToCompare = ( hours * 100 ) + minutes;
			
			// If the turn back ON time is the next day, it could be a smaller number than the turn OFF time.
			// If the turn back ON time is the same day, it could be a larger  number than the turn OFF time.
			// This matters due to the logic of checking if you are "inside" or "outside" the time window.
			
			// For turn ON/OFF times that are on the same day, the turn OFF time is smaller than the turn ON time.
			if( nixiesleepend >= nixiesleepstart ){
				// Check to see if the time is in the sleep window.
				if( aTimeToCompare >= nixiesleepstart && aTimeToCompare < nixiesleepend )
				{
					turnOnOffBoostConverter( 0 );	// Turn off the boost converter.
					return;
				}
			}else{	// For turn ON/OFF times that are on different days, the turn OFF time is larger than the turn ON time.
				// Check to see if the time is in the sleep window.
				if( aTimeToCompare >= nixiesleepstart || aTimeToCompare < nixiesleepend ) 
				{
					turnOnOffBoostConverter( 0 );	// Turn off the boost converter.
					return;
				}
			}
		}
		
	// Otherwise, the voltage is good, and the nixie sleep window is either not now or is inactive.
	turnOnOffBoostConverter( 1 );	// Turn on the boost converter.
	
}


// Very important to use this function to turn ON or OFF the boost converter.  Read below to find out why.
// Send 1 to turn ON, send 0 ( or anything else ) to turn OFF
void turnOnOffBoostConverter( uint8_t state )
{
	if( state ){
		TCCR0A |= ( 1 << COM0A1 );	// Connect the output pin to the PWM controller.
	}
	else{
		TCCR0A &= ~( 1 << COM0A1 );	// Disconnect the output pin from the PWM controller.
		PORTD &= ~( 1 << F_BOOST );	// Set the output pin low, otherwise you will have a direct short 
	}								// thru the inductor and burn thru 5 fuses trying to figure it out. 
									// I have been told that the default state is low, so setting low may
}									// be redundant.  I leave it here as I have not tested this.





void updateAcFrequency( void )
{
	if( acFreqFlag )	// acFreqFlag is set in ISR( TIMER1_COMPA_vect ) and cleared here.
	{
		// If acFreqFlag == 1, then acZeroCrossSnapshot contains the total number of AC waveform 
		// "zero crossings" in AC_FRQ_FILT_SEC seconds.  Here we just need to compute the average and
		// stuff the result into acFrequency.
		// Then, in out main app: The contents of acFrequency is the frequency of the AC line input.
		
		acFrequency = ( acZeroCrossSnapshot / ( AC_FRQ_FILT_SEC * 60.0 ) ) * 60.0;
		
		// Since a pin change interrupt counts rising and falling edges, there are twice as many events.
		acFrequency /= 2.0;
		
		acFreqFlag = 0;	// Reset the counter.
	}
	
}



// Read about cathode poisoning on the web.  
// Basically we just want to make sure each nixie element gets some burn time
// otherwise the unused elements will underperform and get dim or dark spots.
void antiCathodePoisonPattern( uint16_t delay )
{		
		// Count up.
		for( int i = 0; i < 10; i++ ){
			for( int j = 0; j < 6; j++ ){
				sendIntegerToNixie( i, 1 );
			}
			
			shiftRegisterSetOutput();
			_delay_ms( delay );
		}
		
		// Count down.
		for( int i = 8; i >= 1; i-- ){
			for( int j = 0; j < 6; j++ ){
				sendIntegerToNixie( i, 1 );
			}
			
			shiftRegisterSetOutput();
			_delay_ms( delay );
		}
}


// See antiCathodePoisonPattern() comments.
void checkCathodePoison( void )
{
		// Anti-poison the cathodes every 30 minutes.
        if( minutes == 30 || minutes == 0){
        	if( cathodePoisonFlag == 1 ){
        		antiCathodePoisonPattern( 250 );
        		antiCathodePoisonPattern( 225 );
        		antiCathodePoisonPattern( 200 );
        		antiCathodePoisonPattern( 175 );
        		antiCathodePoisonPattern( 150 );
        		antiCathodePoisonPattern( 125 );
        		antiCathodePoisonPattern( 100 );
        		antiCathodePoisonPattern( 75 );
        		antiCathodePoisonPattern( 50 );
        		antiCathodePoisonPattern( 25 );
        		antiCathodePoisonPattern( 10 );
        		antiCathodePoisonPattern( 10 );
        		antiCathodePoisonPattern( 5 );
        		cathodePoisonFlag = 0;				// Reset the counter.
        	}
        }	

}



void shiftRegisterPulse( void )
{

	PORTD |= ( 1 << SRCLK );		// set the serial clock line high.
	//_delay_us( 1 );							// No delay
    PORTD &= ~( 1 << SRCLK );	// Set the serial Clock line low.

}




void shiftRegisterSetOutput( void )
{

	PORTD |= ( 1 << RCLK );		// set the store output line high.
	//_delay_us(1);							// No delay, might need a nop or 2.
    PORTD &= ~( 1 << RCLK );	// Set the store output line low.
    //_delay_us(1);							// No delay, might need a nop or 2.
}





void shiftByteOut( uint8_t byteToShift )
{
	PORTD &= ~( 1 << RCLK );		// Hold low while transmitting.
	
	for( uint8_t i = 0; i < 8; i++ ) 
	{
		// If the MSB is high.
		if ( byteToShift & _BV( 7 - i ) ){
		
            PORTD |= ( 1 << SER );	// set the output high
        
        } else {
        
            PORTD &= ~( 1 << SER );	// Set the output low.
        
        }
        
        shiftRegisterPulse();
    }
    
    PORTD |= ( 1 << RCLK );	

}




void sendIntegerToNixie( uint8_t intToSend, uint8_t doDecimal )
{
	// Sends an int thru a shift register, thru a HV BCD driver and to a Nixie tube.
	// doDecimal passed in will turn the decimal point inside the nixie ON if 1, off if 0.
/*
//~~~~~~~~~~~~~~~~~~~~~~*********** Nixie Digits to BCD to Byte ***********~~~~~~~~~~~~~~~~~~~~~~~~

Nixie Digit		  K155ID BCD		Byte for shift register
				A	B	C	D
_____________________________________________________________				
		0		L	L	L	L		0b00000000	
		1		L	L	L	H		0b00010000
		2		L	L	H	L		0b00100000
		3		L	L	H	H		0b00110000		
		4		L	H	L	L		0b01000000
		5		L	H	L	H		0b01010000
		6		L	H	H	L		0b01100000
		7		L	H	H	H		0b01110000
		8		H	L	L	L		0b10000000
		9		H	L	L	H		0b10010000
  Decimal			N/A				0b00001000	// & this with the digit above to turn the decimal ON.

//~~~~~~~~~~~~~~~~~~~~~~*********** Nixie Digits to BCD to Byte ***********~~~~~~~~~~~~~~~~~~~~~~~~
*/

switch ( intToSend ){
	
	case 0:
		if( doDecimal ){
			shiftByteOut( 0b00000000 | 0b00010000 );
		}else{
			shiftByteOut( 0b00000000 );
		}
	break;
	
	
	case 1:
	if( doDecimal ){
		shiftByteOut( 0b00000001 | 0b00010000 );
	}else{
		shiftByteOut( 0b00000001 );
	}
	break;
	

	case 2:
	if( doDecimal ){
		shiftByteOut( 0b00000010 | 0b00010000 );
	}else{
		shiftByteOut( 0b00000010 );
	}
	break;
	
	
	case 3:
	if( doDecimal ){
		shiftByteOut( 0b00000011 | 0b00010000 );
	}else{
		shiftByteOut( 0b00000011 );
	}
	break;
	
	
	case 4:
	if( doDecimal ){
		shiftByteOut( 0b00000100 | 0b00010000 );
	}else{
		shiftByteOut( 0b00000100 );
	}
	break;
	
	
	case 5:
	if( doDecimal ){
		shiftByteOut( 0b00000101 | 0b00010000 );
	}else{
		shiftByteOut( 0b00000101 );
	}
	break;
	
	
	case 6:
	if( doDecimal ){
		shiftByteOut( 0b00000110 | 0b00010000 );
	}else{
		shiftByteOut( 0b00000110 );
	}
	break;
	
	
	case 7:
	if( doDecimal ){
		shiftByteOut( 0b00000111 | 0b00010000 );
	}else{
		shiftByteOut( 0b00000111 );
	}
	break;
	
	
	case 8:
	if( doDecimal ){
		shiftByteOut( 0b00001000 | 0b00010000 );
	}else{
		shiftByteOut( 0b00001000 );
	}
	break;
	
	
	case 9:
	if( doDecimal ){
		shiftByteOut( 0b00001001 | 0b00010000 );
	}else{
		shiftByteOut( 0b00001001 );
	}
	break;

	}
}




void nixiePrintTemperature( void )
{
	// We have 6 nixie tubes.  Let's let the left most digit be "0" for positive temperaures
	// and "1" for negative.  Let's do 2 decimal places, and that leaves 3 places for the
	// whole number temperature.

	//averageCelsiusTemperature = -104.23;	// For testing.
	
	uint8_t isNegative = 0;		// 0 for positive temperatures, 1 for negative
	
	// Determine if the temperature is negative.
	// If so, we will set the leftmost nixie to "1".
	if( averageCelsiusTemperature >= 0 ){
		isNegative = 0;
	}else{
		isNegative = 1;
	}
	
	// Get the current temperature reading.
	double absValTemperature = averageCelsiusTemperature;
	
	// And if negative, convert to absolute value of the temperature reading.
	if( averageCelsiusTemperature < 0 )
		absValTemperature *= -1; 
	
		
	// Convert to Fahrenheit if necessary. 
	if( celsius == 0 )	
    {
    	absValTemperature = ( absValTemperature * 1.8 ) + 32;
    }
   
    
    // The temperature reading to the left of the decimal point. 
	uint8_t intPartTemperature = absValTemperature;	
		
		
	// Temperature reading to the right of the decimal point.						
	uint8_t decimalPartTemperature = ( absValTemperature - intPartTemperature ) * 100;
            
            
	uint8_t ones = 0;		// For outputting the digits to the nixies in order.
	uint8_t tens = 0;
	uint8_t hundreds = 0;
				
	// starting from the rightmost, decimal portion of the temperature.
	ones = decimalPartTemperature % 10;	// Grab the specific digits.
	tens = decimalPartTemperature / 10;		
	
	sendIntegerToNixie( ones, 0 );		// Display the digits.
	sendIntegerToNixie( tens, 1 );		// Display the decimal point.
	
	
	// And now the integer portion.
    ones = intPartTemperature % 10;   
    intPartTemperature /= 10;
    tens = intPartTemperature % 10;
    intPartTemperature /= 10;
    hundreds = intPartTemperature %10;		
	
	sendIntegerToNixie( ones, 0 );		// Display the digits.
	sendIntegerToNixie( tens, 0 );		
	sendIntegerToNixie( hundreds, 0 );
	
	sendIntegerToNixie( isNegative, 0 );	// Is the temperature negative?

}




void nixiePrintTime( uint8_t doToggleDecimal )
{
	int myDecimal = 1;
	
	if( doToggleDecimal ){			// Toggle the nixie Decimal point.
		if( seconds % 2 == 0 ){
			myDecimal = 1;
		}
		else{
			myDecimal = 0;
			}
		}else{
			myDecimal = 1;	// Else, they are steady ON.	
	}
	
	
	// Seconds
	// It can happen that nixiePrintTime() is called before updateDateTime() has had a change to roll over the seconds.
	// In which case seconds here can be >= 60.  This does not effect the determinism or accuracy of the clock.  
	// However, it may be disturbing to some people to see 60 seconds displayed on a clock when it should be 0::59.
	// We will adjust the 10's of seconds below, the one's of seconds will take care of themselves.
	 
	uint8_t tens = seconds;
	uint8_t ones = tens;
				
	tens /= 10;		// Get the "ten's" of seconds.
	if( tens > 5 )
		tens = 0;	// Restrict the display to 0::59 seconds. ( Read above that this does not affect accuracy of timekeeping. )
	ones %= 10;		// Get the "one's" of seconds
	
	sendIntegerToNixie( ones, 0 );
	sendIntegerToNixie( tens, myDecimal );
	
	
	// Minutes
	tens = minutes;
	ones = tens;
	
	tens /= 10;		// Get the "ten's" of minutes.
	ones %= 10;		// Get the "one's" of minutes
	
	sendIntegerToNixie( ones, 0 );
	sendIntegerToNixie( tens, myDecimal );
	
	
	// Hours
	uint8_t myHours = hours;
	
	if( miltime == 0 && hours > 12 )
		myHours -= 12;
		
	tens = myHours;
	ones = tens;
	
	tens /= 10;		// Get the "ten's" of hours.
	ones %= 10;		// Get the "one's" of hours.
	
	sendIntegerToNixie( ones, 0 );
	sendIntegerToNixie( tens, 0 );

}




void nixiePrintDate( void )
{
	// Printing the date in YY.MM.DD format to the nixies
	
	// Print the day.
	uint8_t ones = day;
	uint8_t tens = ones;
				
	tens /= 10;		// Get the "ten's" of days.
	ones %= 10;		// Get the "one's" of days.
	
	sendIntegerToNixie( ones, 0 );
	sendIntegerToNixie( tens, 1 );
	
	
	// Print the month.
	 ones = month;
	 tens = ones;
				
	tens /= 10;		// Get the "ten's" of months.
	ones %= 10;		// Get the "one's" of months.
	
	sendIntegerToNixie( ones, 0 );
	sendIntegerToNixie( tens, 1 );

	
	// Print the Year.
	uint16_t myYear = year;
	
	ones = myYear % 10;   
    myYear /= 10;
    tens = myYear % 10;
    
	sendIntegerToNixie( ones, 0 );		// Display the digits.
	sendIntegerToNixie( tens, 0 );		
	
}






void nixiePrintAcFrequency( void )
{
	uint32_t myFrequency = acFrequency * 1000;	// Shift everything over 3 decimal places.
	uint8_t intToSend = 0;						// The extracted digit to send to the nixie.
	
	// Loop 5 times because we have 5 digits to display.
	for( int i = 0; i < 5; i++ )
	{
		uint8_t myDecimal = 0;
		
		if( i == 2 ){		// put a decimal point in the right spot.
			myDecimal = 1;
		}else{
			myDecimal = 0;
		}
		
		intToSend = myFrequency % 10;				// Extract the rightmost digit.
		myFrequency /= 10;							// Shift right.
		sendIntegerToNixie( intToSend, myDecimal );	// And print it to the nixie tubes.
	}
	
	sendIntegerToNixie( 0, 0 );		//  Send an extra "0" to shift everything "right" one.

}

//~~~~~~~~~~~~~~~~~~~~~~******************** Functions ********************~~~~~~~~~~~~~~~~~~~~~~~~

