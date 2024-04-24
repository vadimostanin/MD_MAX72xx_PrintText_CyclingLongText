// Use the MD_MAX72XX library to Print some text on the display
//
// Demonstrates the use of the library to print text.
//
// User can enter text on the serial monitor and this will display as a
// message on the display.

#include <MD_MAX72xx.h>
#include <SPI.h>
#include "font_ukr.h"
#include <EEPROM.h>

#define PRINT_ENABLE
#ifdef PRINT_ENABLE
  #define PRINT(s, v) { Serial.print(s); Serial.println(v); }
  #define PRINT3(s, v, j) { Serial.print(s); Serial.print(v); Serial.println(j); }
#else
  #define PRINT(s, v)
  #define PRINT3(s, v, j)
#endif

uint32_t delay_text_movement_millis_last = 0;
uint32_t delay_text_movement = 80;

// Define the number of devices we have in the chain and the hardware interface
// NOTE: These pin numbers will probably not work with your hardware and may
// need to be adapted
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_REAL_DEVICES (4)
#define MAX_VIRTUAL_DEVICES (100)
#define MAX_DEVICES (MAX_REAL_DEVICES+MAX_VIRTUAL_DEVICES)

#define CLK_PIN   SCK  // or SCK
#define DATA_PIN  MOSI  // or MOSI
#define CS_PIN    SS  // or SS

// SPI hardware interface
MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);
// Arbitrary pins
//MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);

// Text parameters
#define CHAR_SPACING  1 // pixels between characters

// Global message buffers shared by Serial and Scrolling functions
#define BUF_SIZE  (2000)
#define _(x) ((uint16_t)x)
// uint16_t message[BUF_SIZE] = {(uint16_t)'H',(uint16_t)'e', (uint16_t)'l', (uint16_t)'l', (uint16_t)'o', (uint16_t)'!', 0};//
// uint16_t message[BUF_SIZE] = {
// _('М'), _('А'), _('М'), _('О'), _('-'), _('І'), _('Р'), _('А'), _(','), _(' '), 
// _('Т'), _('А'), _('Т'), _('О'), _('-'), _('В'), _('А'), _('Д'), _('И'), _('М'), _(','), _(' '), 
// _('Є'), _('Г'), _('О'), _('Р'), _('-'), _('С'), _('О'), _('Н'), _('І'), _('К'), _(','), _(' '), 
// _('С'), _('У'), _('П'), _('Е'), _('Р'), _('-'), _('С'), _('О'), _('Ф'), _('І'), _('Я'), 0};//
uint16_t message[BUF_SIZE] = {0};//МАМО-ІРА, ТАТО-ВАДИМ, ЄГОР-СОНІК, СУПЕР-СОФІЯ
// uint16_t message[BUF_SIZE] = {
// _('М'), _('А'), _('М'), _('О'), _('-'), _('І'), _('Р'), _('А'), _(' '),
// _('О'), _('С'), _('Т'), _('А'), _('Н'), _('І'), _('Н'), _('А'), _(','),
// _('Т'), _('А'), _('Т'), _('О'), _('-'), _('В'), _('А'), _('Д'), _('И'), _('М'), _(' '), 
// _('О'), _('С'), _('Т'), _('А'), _('Н'), _('І'), _('Н'), _(','),
// _('Є'), _('Г'), _('О'), _('Р'), _('-'), _('С'), _('О'), _('Н'), _('І'), _('К'), _(' '),
// _('О'), _('С'), _('Т'), _('А'), _('Н'), _('І'), _('Н'), _(','),
// _('С'), _('У'), _('П'), _('Е'), _('Р'), _('-'), _('С'), _('О'), _('Ф'), _('І'), _('Я'), _(' '),
// _('О'), _('С'), _('Т'), _('А'), _('Н'), _('І'), _('Н'), _('А'), 0};//
// МАМО-ІРА ОСТАНІНА, ТАТО-ВАДИМ ОСТАНІН, ЄГОР-СОНІК ОСТАНІН, СУПЕР-СОФІЯ ОСТАНІНА
bool newMessageAvailable = true;

bool getUkranianChar(uint16_t ch, uint8_t chars[COL_SIZE], uint8_t *len)
{
  uint16_t ch_cyrillic_start = 0xd080;
  uint16_t ch_cyrillic_end = 0xd197;
  PRINT("ascii=", ch);
  if(ch < ch_cyrillic_start || ch > ch_cyrillic_end)
  {
    PRINT("Not Ukranian", "");
    return false;
  }
  memcpy(chars, vocabulary_ukranian[ch - ch_cyrillic_start], COL_SIZE);
  *len = vocabulary_ukranian[ch - ch_cyrillic_start][COL_SIZE];
  PRINT("Ukranian found","");
  return true;
}

void readSerial(bool &textChanged, uint16_t &newMessageSize)
{
  static uint8_t	putIndex = 0;
  textChanged = false;

  if (Serial.available())
  {
    memset(message, 0, sizeof(message));
  }

  while (Serial.available())
  {
    uint8_t serial_read = (uint8_t)Serial.read();
    message[putIndex] = serial_read;
    if(message[putIndex] == 208)
    {
      message[putIndex] <<= 8;
      while (!Serial.available());
      serial_read = (uint8_t)Serial.read();
      message[putIndex] |= serial_read;
      ++putIndex;

      newMessageSize = putIndex;
      textChanged = true;
    }
    else if ((message[putIndex] == '\n') || (putIndex >= BUF_SIZE-3))  // end of message character or full buffer
    {
      newMessageSize = putIndex;
      textChanged = true;

      // put in a message separator and end the string
      message[putIndex] = '\0';
      // restart the index for next filling spree and flag we have a message waiting
      putIndex = 0;
    }
    else
    {
      // Just save the next char in next location
      message[putIndex++];
      newMessageSize = putIndex;
      textChanged = true;
    }
  }
}

void printText(uint8_t modStart, uint8_t modEnd, uint16_t *pMsg)
// Print the text string to the LED matrix modules specified.
// Message area is padded with blank columns after printing.
{
  uint8_t   state = 0;
  uint8_t   curLen = 0;
  uint8_t  showLen = 0;
  uint8_t   cBuf[COL_SIZE] = {0};
  int16_t   col = ((modEnd + 1) * COL_SIZE) - 1;


  do     // finite state machine to print the characters in the space available
  {
    switch(state)
    {
      case 0: // Load the next character from the font table
        // if we reached end of message, reset the message pointer
        if (*pMsg == '\0')
        {
          showLen = col - (modEnd * COL_SIZE);  // padding characters
          state = 2;
          break;
        }

        if(!getUkranianChar(*pMsg, cBuf, &showLen))
        {
          // retrieve the next character form the font file
          showLen = mx.getChar((char)*pMsg, sizeof(cBuf)/sizeof(cBuf[0]), cBuf);
        }
        pMsg++;
        PRINT("showLen=", showLen);
        for(int char_i = 0 ; char_i < sizeof(cBuf) ; ++char_i)
        {
          PRINT(cBuf[char_i], ", ");
        }
        curLen = 0;
        state++;
        // !! deliberately fall through to next state to start displaying

      case 1: // display the next part of the character
        mx.setColumn(col--, cBuf[curLen++]);

        // done with font character, now display the space between chars
        if (curLen == showLen)
        {
          showLen = CHAR_SPACING;
          state = 2;
        }
        break;

      case 2: // initialize state for displaying empty columns
        curLen = 0;
        state++;
        // fall through

      case 3:	// display inter-character spacing or end of message padding (blank columns)
        mx.setColumn(col--, 0);
        curLen++;
        if (curLen == showLen)
          state = 0;
        break;

      default:
        col = -1;   // this definitely ends the do loop
    }
  } while (col >= (modStart * COL_SIZE));
}

void eeprom_message_read()
{
  unsigned char eeprom_value = 0;
  uint16_t message_bytes_size = 0;

  EEPROM.begin(2);

  EEPROM.get(0, eeprom_value);
  message_bytes_size = eeprom_value << 8;
  EEPROM.get(1, eeprom_value);
  message_bytes_size |= eeprom_value;
  if(0xffff != message_bytes_size)
  {
    memset(message, 0, sizeof(message));

    EEPROM.begin(2 + message_bytes_size);

    const uint16_t message_utf8_size = message_bytes_size / sizeof(message[0]);
    for(int message_i = 0 ; message_i < message_utf8_size ; ++message_i)
    {
      EEPROM.get(message_i*2 + 2, eeprom_value);
      message[message_i] = eeprom_value << 8;
      EEPROM.get(message_i*2 + 2 + 1, eeprom_value);
      message[message_i] |= eeprom_value;
    }
    newMessageAvailable = true;
  }
}

void eeprom_message_write(const uint16_t *str, uint16_t newMessageSize)
{
  const unsigned short message_utf8_size = newMessageSize;
  const unsigned short message_bytes_size = message_utf8_size * sizeof(message[0]);
  unsigned char eeprom_value = 0; 
  EEPROM.begin(2 + message_bytes_size);
  EEPROM.put(0, message_bytes_size>>8);
  EEPROM.put(1, message_bytes_size);
  for(int message_i = 0 ; message_i < message_utf8_size ; ++message_i)
  {
    eeprom_value = message[message_i] >> 8;
    EEPROM.put(message_i*2 + 2, eeprom_value);
    eeprom_value = message[message_i];
    EEPROM.put(message_i*2 + 2 + 1, eeprom_value);
  }
  EEPROM.commit();
}

void handle_text_movement()
{
  PRINT3(__func__,":", __LINE__);
  // Shift over and insert the new column
  mx.control(0, MAX_DEVICES-1, MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
  PRINT3(__func__,":", __LINE__);
  mx.transform(MD_MAX72XX::TSL);
  PRINT3(__func__,":", __LINE__);
  mx.control(0, MAX_DEVICES-1, MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
  PRINT3(__func__,":", __LINE__);
  yield();
  bool empty_panel = true;
  for(uint col_i = 0 ; col_i < COL_SIZE * MAX_REAL_DEVICES ; ++col_i)
  {
    if(0 != mx.getColumn(col_i / COL_SIZE, col_i % COL_SIZE))
    {
      empty_panel = false;
      break;
    }
  }
  if(empty_panel)
  {
    PRINT("Panel is empty", "");
    mx.control(0, MAX_DEVICES, MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
    for(int column_i = 0 ; column_i < COL_SIZE * (MAX_DEVICES); ++column_i)
    {
      if((column_i % COL_SIZE * 200) == 0)
      {
        yield();
      }
      mx.transform(MD_MAX72XX::TSL);
      for(uint col_i = 0 ; col_i < COL_SIZE * MAX_REAL_DEVICES ; ++col_i)
      {
        if(0 != mx.getColumn(col_i / COL_SIZE, col_i % COL_SIZE))
        {
          empty_panel = false;
          break;
        }
      }
      if(!empty_panel)
      {
        mx.transform(MD_MAX72XX::TSR);
        break;
      }
    }
    mx.control(0, MAX_DEVICES, MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
    PRINT("Panel is not empty", "");
  }
}

void handle_add_new_text()
{
  mx.control(0, MAX_DEVICES-1, MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
    mx.control(0, MAX_DEVICES-1, MD_MAX72XX::WRAPAROUND, MD_MAX72XX::OFF);
    
    printText(0, MAX_DEVICES-1, message);
    yield();
    mx.control(0, MAX_DEVICES-1, MD_MAX72XX::WRAPAROUND, MD_MAX72XX::ON);
    yield();
    for(int column_i = 0 ; column_i < COL_SIZE * (MAX_DEVICES); ++column_i)
    {
      if((column_i % COL_SIZE * 200) == 0)
      {
        yield();
      }
      mx.transform(MD_MAX72XX::TSR);
    }
  yield();
  mx.control(0, MAX_DEVICES, MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
}

uint get_message_length()
{
  for(uint char_i = 0 ; char_i < BUF_SIZE ; ++char_i)
  {
    if(0 == message[char_i])
    {
      return char_i;
    }
  }
  return BUF_SIZE - 1;
}

void setup()
{
  Serial.begin(74880);
  mx.begin();

  eeprom_message_read();
  mx.control(0, MAX_DEVICES-1, MD_MAX72XX::INTENSITY, 2);
  mx.control(0, 0, MD_MAX72XX::INTENSITY, 2 + 3);

  delay_text_movement_millis_last = millis();
}
void loop()
{
  bool messageChanged = false;
  uint16_t newMessageSize = 0;
  readSerial(messageChanged, newMessageSize);
  if(messageChanged)
  {
    eeprom_message_write(message, get_message_length());
    newMessageAvailable = true;
  }

  if (newMessageAvailable)
  {
    PRINT3(__func__,":", __LINE__);
    yield();
    handle_add_new_text();
    yield();
    PRINT3(__func__,":", __LINE__);
    newMessageAvailable = false;
  }

  if(millis() - delay_text_movement_millis_last > delay_text_movement)
  {
    yield();
    PRINT3(__func__,":", __LINE__);
    handle_text_movement();
    PRINT(__func__, __LINE__);
    yield();
    delay_text_movement_millis_last = millis();
  }
}
