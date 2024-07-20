#include "LCD.h"
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
    >>> Server Temperature Control (STC) System with Atmega32
    >>> Master source file
*/

#define BAUD 9600
#define F_CPU 8000000L

#define GREEN_LED_PIN PD7
#define YELLOW_LED_PIN PD6
#define RED_LED_PIN PD5

// Function Prototypes -----------------------------------------------------------------------
void SPI_Init();
void init_ports(void);
char select_mode(void);
void start_message(void);
char keypad_getkey(void);
void LCD_String(char *str);
void LCD_init_display(void);
void get_and_trans_pass(void);
void wait_for_pass_faults(void);
unsigned char SPI_Receive(void);
uint8_t call_of_duty(uint8_t temp);
void SPI_Transmit(unsigned char data);
void check_system_status(uint16_t overall_duty);

// Main function -----------------------------------------------------------------------------
int main(void)
{
  init_ports();  // initialize the ports
  SPI_Init();    // initialize the SPI unit
  init_LCD();    // initialize the LCD
  _delay_ms(10); // wait for 100 milliseconds ...

  LCD_cmd(0x0F);                                                             // make blinking cursor visible
  start_message();                                                           // start meggage and welcome
  DDRD |= (1 << GREEN_LED_PIN) | (1 << YELLOW_LED_PIN) | (1 << RED_LED_PIN); // set LED pins as outputs

  char number_of_pass_faults = 0;
  while (1)
  {
    get_and_trans_pass(); // get, display on LCD and send password to slave
    LCD_init_display();

    unsigned char pass_status = SPI_Receive();
    if (pass_status == '1')
    {
      number_of_pass_faults = 0;
      LCD_String("Access Granted! ");
      LCD_String("> Selecet mode:");
      _delay_ms(100); // wait for 1 second
      LCD_init_display();

      while (1) // enter to the STC system
      {
        char mode = select_mode();

        // Receive temperature data and calculate the duty cycle
        uint8_t temperature = SPI_Receive();
        uint8_t duty_cycle = call_of_duty(temperature);
        uint16_t overall_duty = duty_cycle * 3;

        char buffer[16]; // string number
        if (mode)
        {
          // Display motor status and temperature
          sprintf(buffer, "Motor 1: %d%%", duty_cycle);
          LCD_String(buffer);

          _delay_ms(300); // wait for 3 seconds
          LCD_init_display();

          sprintf(buffer, "Motor 2: %d%%     ", duty_cycle);
          LCD_String(buffer);

          sprintf(buffer, "Motor 3: %d%%", duty_cycle);
          LCD_String(buffer);

          _delay_ms(300); // wait for 3 seconds
          LCD_init_display();

          sprintf(buffer, "Overall: %d%%", overall_duty);
          LCD_String(buffer);
        }
        else
        {
          sprintf(buffer, "%d Celsius dgre", temperature);
          LCD_String(buffer);
        }

        check_system_status(overall_duty);
        _delay_ms(300); // wait for 3 seconds
        LCD_init_display();
      }
    }
    else
    {
      if (number_of_pass_faults == 2)
      {
        wait_for_pass_faults();
        number_of_pass_faults = 0;
      }
      else
      {
        number_of_pass_faults++;
        LCD_String("Access Denied!");
        _delay_ms(100); // wait for 1 second
      }

      LCD_init_display();
    }
  }
}

// General funtions --------------------------------------------------------------------------
/**
 * @brief Displays a string on the LCD.
 *
 * @param str: The string to be displayed.
 */
void LCD_String(char *str)
{
  while (*str)
    LCD_write(*str++);
}

void LCD_init_display()
{
  LCD_cmd(0x01); // clear LCD screen
  LCD_cmd(0x80); // move cursor to the first row
}

// Used funtions -----------------------------------------------------------------------------
void init_ports(void)
{
  DDRA = 0xFF; // port A is used to send output data to LCD

  DDRB = 0xB0;
  PORTB = 0x0E;
  // PB3, PB2, PB1 as inputs of keypad
  // Rows of keypad as inputs of keypad (micro to keypad)
  // SPI unit pins: PB7 -> SCK (out) | PB6 -> MISO (in) | PB5 -> MOSI (out) | PB4 -> ~SS (out)

  DDRC = 0xF7;
  // columns of keypad as outputs of keypad (keypad to micro)
  // Ports C.0, C.1, C.2 are used for controlling signals of LCD
  // Ports C.7, C.6, C.5, C.4 are used as inputs of keypad (port C.3 is unused)

  DDRD = 0xE0; // PD7, PD6, PD5 --> inputs of LED
}

/**
 * @brief Initializes the SPI interface for Master mode.
 */
void SPI_Init()
{
  // Set MOSI, SCK, and SS as output, MISO as input
  DDRB |= (1 << PB5) | (1 << PB7) | (1 << PB4);
  DDRB &= ~(1 << PB6);

  // Enable SPI, Set as Master
  // Prescaler: Fosc/8 = 1 MHz, Enable Interrupts
  SPCR = (1 << SPI2X) | (0 << SPR1) | (1 << SPR0);
  SPCR = (1 << SPE) | (1 << MSTR);

  // Clear SPI interrupt flag by reading SPSR and SPDR
  uint8_t dummy = SPSR;
  dummy = SPDR;
}

void start_message(void)
{
  LCD_init_display();
  LCD_String(">> STC System <<"); // Server Temperature Control (STC)
  LCD_String(">>  Welcome!  <<");
  _delay_ms(70); // wait for 700 milliseconds
  LCD_init_display();
}

/**
 * @brief Gets the password from user, displays it on LCD and sends it to slave.
 */
void get_and_trans_pass()
{
  // Clear SPI buffers
  uint8_t dummy = SPSR;
  dummy = SPDR;

  // Get password from user and display on LCD
  LCD_String("Enter Password: >");
  char password[6];

  for (uint8_t i = 0; i < 5; i++)
  {
    char string_keypad[1];
    password[i] = keypad_getkey();
    if (password[i] >= '0')
    {
      sprintf(string_keypad, "%d", password[i] - '0');
      LCD_String(string_keypad);
    }
    else if (password[i] - '0' == -13)
      LCD_write('#');
    else
      LCD_write('*');
  }
  password[5] = '\0'; // null-terminate the string

  // Send password to slave
  for (uint8_t i = 0; i < 5; i++)
  {
    SPI_Transmit(password[i]);
    _delay_ms(10); // small delay to ensure synchronization
  }
}

/**
 * @brief Scans the keypad for a key press.
 *
 * @return The character corresponding to the key press.
 */
char keypad_getkey(void)
{
  char keys[4][4] = {
      {'1', '2', '3'},
      {'4', '5', '6'},
      {'7', '8', '9'},
      {'*', '0', '#'}};

  while (1)
  {
    for (uint8_t row = 0; row < 4; row++)
    {
      PORTC = ~(1 << (row + 4)); // Ground one row at a time
      for (uint8_t col = 0; col < 3; col++)
      {
        if (!(PINB & (1 << (col + 1)))) // Check for key press
        {
          _delay_ms(20); // Debounce delay
          while (!(PINB & (1 << (col + 1))))
            ; // Wait for key release
          return keys[row][col];
        }
      }
    }
  }
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
 * @brief Temperature to duty cycle.
 *
 * @param Received temperature from Slave.
 *
 * @return The duty cycle of Slave motors.
 */
uint8_t call_of_duty(uint8_t temp)
{
  uint8_t duty_cycle = 0; //

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

  return duty_cycle;
}

/**
 * @brief After three consecutive incorrect password entries, locks user 30 seconds.
 */
void wait_for_pass_faults(void)
{
  char string_counter[2] = "";

  LCD_init_display();
  LCD_String("3 wrong tries!  ");
  LCD_String("Wait, 30 seconds");
  _delay_ms(100); // wait for 1 second

  for (int i = 30; i > 0; i--)
  {
    LCD_init_display();
    sprintf(string_counter, "%d", i);
    for (int j = 0; j < 2; j++)
      LCD_write(string_counter[j]);
    _delay_ms(20); // Why less than 1 second? for some intermediate delays
  }
}

/**
 * @brief Select checking mode (motor status | temperature).
 *
 * @return Selected mode (1 --> motor status | 0 --> temperature)
 */
char select_mode(void)
{
  LCD_String("1) Motor status ");
  LCD_String("2) Temperature  ");
  char sel_mode = keypad_getkey();
  LCD_init_display();

  if (sel_mode == '1')
  {
    LCD_String("> Motor status: ");
    SPI_Transmit('1'); // alert to slave
    _delay_ms(10);     // small delay to ensure synchronization
    return 1;
  }

  LCD_String("> Temperature:  ");
  SPI_Transmit('1'); // alert to slave
  _delay_ms(10);     // small delay to ensure synchronization
  return 0;
}

/**
 * @brief Checks the overall system status and updates the LEDs.
 *
 * @param overall_duty: The overall duty cycle of the motors.
 */
void check_system_status(uint16_t overall_duty)
{
  if (overall_duty < 90)
  {
    PORTD |= (1 << YELLOW_LED_PIN);                        // Turn on yellow LED
    PORTD &= ~((1 << GREEN_LED_PIN) | (1 << RED_LED_PIN)); // Turn off green and red LEDs
  }
  else if (overall_duty >= 150)
  {
    PORTD |= (1 << RED_LED_PIN);                              // Turn on red LED
    PORTD &= ~((1 << GREEN_LED_PIN) | (1 << YELLOW_LED_PIN)); // Turn off green and yellow LEDs
  }
  else
  {
    PORTD |= (1 << GREEN_LED_PIN);                          // Turn on green LED
    PORTD &= ~((1 << YELLOW_LED_PIN) | (1 << RED_LED_PIN)); // Turn off yellow and red LEDs
  }
}