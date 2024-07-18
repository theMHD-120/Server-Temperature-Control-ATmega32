#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

/*
    ||| In the name of ALLAH |||
    ----------------------------
    Seyed Mahdi Mahdavi Mortazavi
    Std. number: 40030490
    ----------------------------
    Microprocessors:
    Assignement: Final Project
    >>> Server Temperature Control System with Atmega32
    >>> Slave source file
*/

#define BAUD 9600
#define F_CPU 8000000L

#define MOTOR1_PIN PB3
#define MOTOR2_PIN PD5
#define MOTOR3_PIN PD4

uint8_t duty_cycle = 0;
char correct_password[6] = "138*1";

// Function Prototypes -----------------------------------------------------------------------
void SPI_init();
void init_ADC(void);
void init_ports(void);
void init_TimerCounter(void);
uint8_t get_temperature(void);
unsigned char SPI_Receive(void);
char check_password(char *password);
void SPI_Transmit(unsigned char data);
void motor_control(uint8_t temperature);
void set_motor_duty_cycle(uint8_t temp);

// Main function -----------------------------------------------------------------------------
int main(void)
{
  init_ports();        // initialize the ports
  SPI_init();          // initialize the SPI unit
  init_ADC();          // initialize the ADC
  init_TimerCounter(); // initialize the Timer/Counter

  // Set motor pins as outputs
  DDRB |= (1 << MOTOR1_PIN);
  DDRD |= (1 << MOTOR2_PIN) | (1 << MOTOR3_PIN);

  while (1)
  {
    char received_password[6];
    for (uint8_t i = 0; i < 5; i++)
      received_password[i] = SPI_Receive();
    received_password[5] = '\0'; // Null-terminate the string

    char pass_status = check_password(received_password);
    if (pass_status)
    {
      while (1)
      {
        sei(); // Enable global interrupts
        uint8_t temperature = get_temperature();
        sei(); // Enable global interrupts
        char sel_mode = SPI_Receive();
        sei();               // Enable global interrupts
        if (sel_mode == '1') // start transmit
        {
          sei(); // Enable global interrupts
          temperature = get_temperature();
          sei(); // Enable global interrupts
          SPI_Transmit(temperature);
          sei();         // Enable global interrupts
          _delay_ms(10); // small delay to ensure synchronization
        }

        sei(); // Enable global interrupts
        motor_control(temperature);
      }
    }
  }
}

// Interrupt Service Routines ----------------------------------------------------------------
ISR(ADC_vect)
{
  init_ports();                            // initialize the ports
  init_ADC();                              // initialize the ADC
  init_TimerCounter();                     // initialize the Timer/Counter
  uint8_t temperature = get_temperature(); // get temperature from ADC
  set_motor_duty_cycle(temperature);       // update duty cycle based on temperature
}

// Used funtions -----------------------------------------------------------------------------
void init_ports(void)
{
  DDRA = 0x00;
  PORTA = 0x00;
  // All pull-up resistors are disabled
  // Port A (PA0) is used to get input of LM35 sensor

  DDRB = 0x48;
  PORTB = 0x00;
  // PB3 as input of motor-1 (OC0)
  // SPI unit pins: PB7 -> SCK (in) | PB6 -> MISO (out) | PB5 -> MOSI (in) | PB4 -> ~SS (in)

  DDRD = 0x30;
  PORTD = 0x00;
  // PD5 as input of motor-2 (OC1A)
  // PD4 as input of motor-3 (OC1B)
  // PD1: TxD pin (to master) / PD0: RxD pin (from master)
}

/**
 * @brief Initializes the SPI interface for Slave mode.
 */
void SPI_init()
{
  // Set MISO as output
  DDRB |= (1 << PB6);

  // Enable SPI
  SPCR = (1 << SPE);

  // Clear SPI interrupt flag by reading SPSR and SPDR
  uint8_t dummy = SPSR;
  dummy = SPDR;
}

/**
 * @brief Initializes the ADC for temperature sensor.
 */
void init_ADC(void)
{
  // Select Vref=Avcc
  ADMUX |= (1 << REFS0);
  ADMUX &= ~(1 << REFS1);
  ADC = 0; // initializing value (output) of ADC

  // ADCSRA --> ADC Control and Status Register A
  // Optionally prescaling (like master): set ADC prescaler to 8 (8 MHz / 8 = 1 MHz)
  ADCSRA |= (1 << ADPS1) | (1 << ADPS0);
  ADCSRA |= ~(1 << ADPS2);

  ADCSRA |= (1 << ADIF) | (1 << ADIE); // clear ADIF by writing one to it (ready for next interrupt)
  ADCSRA |= (1 << ADEN);               // Enable ADC
}

/**
 * @brief Initializes the Timer/Counter for motors.
 */
void init_TimerCounter(void)
{
  TCCR0 |= (1 << COM01);   // Clear (non-inverting) OC0 on compare match
  TCCR1A |= (1 << COM1A1); // Clear (non-inverting) OC1A on compare match
  TCCR1A |= (1 << COM1B1); // Clear (non-inverting) OC1B on compare match

  // OC0
  TCCR0 |= (1 << WGM01) | (1 << WGM00);             // 8-bit fast PWM
  TCCR0 |= (0 << CS02) | (1 << CS01) | (0 << CS00); // TC clock prescaling to 8 (1 MHz)

  // OC1A
  TCCR1A |= (0 << WGM11) | (1 << WGM10); // 8-bit fast PWM

  // OC1B
  TCCR1B |= (0 << WGM13) | (1 << WGM12);             // 8-bit fast PWM
  TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10); // TC clock prescaling to 8 (1 MHz)
}

/**
 * @brief Receives a byte of data via SPI.
 *
 * @return The received data byte.
 */
unsigned char SPI_Receive(void)
{
  // Transmit dummy data to initiate SPI clock
  SPDR = 0xFF;
  // Wait for reception complete (SPIF is set)
  while (!(SPSR & (1 << SPIF)))
    ;
  // Return received data from SPDR
  return SPDR;
}

/**
 * @brief Checks the received password and communicates with the Master unit.
 *
 * @param password: The received password to be checked.
 */
char check_password(char *password)
{
  // Clear SPI buffers
  uint8_t dummy = SPSR;
  dummy = SPDR;

  if (strcmp(password, correct_password) == 0)
  {
    SPI_Transmit('1'); // password correct
    _delay_ms(10);     // small delay to ensure synchronization
    return 1;
  }

  SPI_Transmit('0'); // Password incorrect
  _delay_ms(10);     // small delay to ensure synchronization
  return 0;
}

/**
 * @brief Transmits data over SPI.
 *
 * @param data The data byte to be transmitted.
 */
void SPI_Transmit(unsigned char data)
{
  // Load data into the buffer
  SPDR = data;

  // Wait until transmission complete
  while (!(SPSR & (1 << SPIF)))
    ;
  _delay_us(10); // Small delay to ensure synchronization
}

/**
 * @brief Gets the current temperature from the LM35 sensor.
 *
 * @return The temperature in degrees Celsius (digital value).
 */
uint8_t get_temperature(void)
{
  ADC = 0;
  ADMUX &= 0xF0;
  ADMUX |= 0x00;
  // Clear the previous channel selection
  // ADMUX --> MUX4..0 is zero --> Single-ended ADC;
  // Select ADC channel 0 (assuming the LM35 is connected to PA0 (ADC0))

  ADCSRA |= (1 << ADSC); // start the conversion
  while (ADCSRA & (1 << ADSC))
    ;                    // wait for conversion to complete
  ADCSRA |= (1 << ADIF); // clear ADIF by writing one to it (ready for next interrupt)

  uint16_t adc_value = ADC;
  uint8_t temperature = (adc_value * 5.0 * 100) / 1024; // adc_value * 100 --> for millivoltage
  // Calculate temperature in Celsius (LM35 gives 10mV/°C and ADC resolution is 5V/1024)

  //  Input (analog) voltage    output digital voltage (output of ADC) or ADC bit resolution
  // ----------------------- = -------------------------------------------------------------
  //     Vref (5 volt)               2^(n) = 1024 (from 0 to 1023 -> 10 bits for ADC)

  return temperature;
}

/**
 * @brief Sets duty cycle and detects the failed motor (doesn't word  :*).
 *
 * @param temperature: The current temperature value.
 */
void motor_control(uint8_t temperature)
{
  // Set duty cycle based on temperature
  set_motor_duty_cycle(temperature);

  char master_alert = SPI_Receive();
  if (master_alert == '1')
  {
    if (((PORTB >> PORTB6) & 1) == 0)
      SPI_Transmit('1');
    else if (((PORTD >> PORTD5) & 1) == 0)
      SPI_Transmit('2');
    else if (((PORTD >> PORTD4) & 1) == 0)
      SPI_Transmit('3');
    else
      SPI_Transmit('0');
  }
}

/**
 * @brief Sets the PWM duty cycle for the specified motor
 * >>> 10% per 10°C
 */
void set_motor_duty_cycle(uint8_t temp)
{
  if (temp < 10)
    duty_cycle = 0;
  else if (temp >= 10 && temp < 20)
    duty_cycle = 10;
  else if (temp >= 20 && temp < 30)
    duty_cycle = 20;
  else if (temp >= 30 && temp < 40)
    duty_cycle = 30;
  else if (temp >= 40 && temp < 50)
    duty_cycle = 40;
  else if (temp >= 50 && temp < 60)
    duty_cycle = 50;
  else if (temp >= 60 && temp < 70)
    duty_cycle = 60;
  else if (temp >= 70 && temp < 80)
    duty_cycle = 70;
  else if (temp >= 80 && temp < 90)
    duty_cycle = 80;
  else if (temp >= 90 && temp < 100)
    duty_cycle = 90;
  else
    duty_cycle = 100;

  OCR0 = (duty_cycle * 255) / 100;  // Set PWM duty cycle for Motor 1
  OCR1A = (duty_cycle * 255) / 100; // Set PWM duty cycle for Motor 2
  OCR1B = (duty_cycle * 255) / 100; // Set PWM duty cycle for Motor 3
}