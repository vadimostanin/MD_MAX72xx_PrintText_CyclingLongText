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
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Pinger.h>

#define PRINT_ENABLE
#ifdef PRINT_ENABLE
  #define PRINT(s, v) { Serial.print(s); Serial.println(v); }
  #define PRINT3(s, v, j) { Serial.print(s); Serial.print(v); Serial.println(j); }
#else
  #define PRINT(s, v)
  #define PRINT3(s, v, j)
#endif
const char* ssid     = "laptop_ext";
const char* password = "iZ27St#R ";
const char* mqtt_server = "192.168.1.79";
const int mqtt_port = 1883; // Порт для подключения к серверу MQTT
const char *mqtt_user = "mqtt_user";
const char *mqtt_pass = "q1w2e3r4";

uint32_t delay_text_movement_millis_last = 0;
uint32_t delay_text_movement = 80;
uint32_t delay_wifi_connection_first_millis_last = 0;
WiFiClient wclient;
void mqtt_callback(char* topic, byte* payload, unsigned int length);
PubSubClient client(mqtt_server, mqtt_port, mqtt_callback, wclient);
Pinger pinger;
// Define the number of devices we have in the chain and the hardware interface
// NOTE: These pin numbers will probably not work with your hardware and may
// need to be adapted
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_REAL_DEVICES (4)
#define MAX_VIRTUAL_DEVICES (68)
#define MAX_DEVICES (MAX_REAL_DEVICES + MAX_VIRTUAL_DEVICES)

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
#define BUF_SIZE  (1000)
#define _(x) ((uint16_t)x)
// uint16_t message[BUF_SIZE] = {(uint16_t)'H',(uint16_t)'e', (uint16_t)'l', (uint16_t)'l', (uint16_t)'o', (uint16_t)'!', 0};//
// uint16_t message[BUF_SIZE] = {(uint16_t)'А',(uint16_t)'Б', (uint16_t)'В', (uint16_t)'Г', (uint16_t)'Д', (uint16_t)'Е', 0};//
// uint16_t message[BUF_SIZE] = {
//   (uint16_t)'А',(uint16_t)'A',
//   (uint16_t)'В',(uint16_t)'B',
//   (uint16_t)'Е',(uint16_t)'E',
//   (uint16_t)'И',(uint16_t)'N',
//   (uint16_t)'К',(uint16_t)'K',
//   (uint16_t)'М',(uint16_t)'M',
//   (uint16_t)'Н',(uint16_t)'H',
//   (uint16_t)'О',(uint16_t)'O',
//   (uint16_t)'Р',(uint16_t)'P',
//   (uint16_t)'С',(uint16_t)'C',
//   (uint16_t)'Т',(uint16_t)'T',
//   (uint16_t)'Х',(uint16_t)'X',
//   (uint16_t)'Я',(uint16_t)'R',
//   0};//
// uint16_t message[BUF_SIZE] = {
// _('М'), _('А'), _('М'), _('О'), _('-'), _('І'), _('Р'), _('А'), _(','), _(' '), 
// _('Т'), _('А'), _('Т'), _('О'), _('-'), _('В'), _('А'), _('Д'), _('И'), _('М'), _(','), _(' '), 
// _('Є'), _('Г'), _('О'), _('Р'), _('-'), _('С'), _('О'), _('Н'), _('І'), _('К'), _(','), _(' '), 
// _('С'), _('У'), _('П'), _('Е'), _('Р'), _('-'), _('С'), _('О'), _('Ф'), _('І'), _('Я'), 0};//
//uint16_t message[BUF_SIZE] = {0};//МАМО-ІРА, ТАТО-ВАДИМ, ЄГОР-СОНІК, СУПЕР-СОФІЯ

uint16_t message[BUF_SIZE] = {
_('М'), _('А'), _('М'), _('О'), _('-'), _('І'), _('Р'), _('А'), _(' '),
_('О'), _('С'), _('Т'), _('А'), _('Н'), _('І'), _('Н'), _('А'), _(','), _(' '), 
_('Т'), _('А'), _('Т'), _('О'), _('-'), _('В'), _('А'), _('Д'), _('И'), _('М'), _(' '), 
_('О'), _('С'), _('Т'), _('А'), _('Н'), _('І'), _('Н'), _(','), _(' '),
_('Є'), _('Г'), _('О'), _('Р'), _('-'), _('С'), _('О'), _('Н'), _('І'), _('К'), _(' '),
_('О'), _('С'), _('Т'), _('А'), _('Н'), _('І'), _('Н'), _(','), _(' '),
_('С'), _('У'), _('П'), _('Е'), _('Р'), _('-'), _('С'), _('О'), _('Ф'), _('І'), _('Я'), _(' '),
_('О'), _('С'), _('Т'), _('А'), _('Н'), _('І'), _('Н'), _('А'), _(','), _(' '),
_('К'), _('И'), _('Ї'), _('В'), _(','), _('В'), _('У'), _('Л'), _('И'), _('Ц'), _('Я'), _(' '), 
_('Д'), _('Е'), _('М'), _('І'), _('Ї'), _('В'), _('С'), _('Ь'), _('К'), _('А'), _(' '), _('1'), _('6'), 0};

// МАМО-ІРА ОСТАНІНА, ТАТО-ВАДИМ ОСТАНІН, ЄГОР-СОНІК ОСТАНІН, СУПЕР-СОФІЯ ОСТАНІНА
// МАМО-ІРА ОСТАНІНА, ТАТО-ВАДИМ ОСТАНІН, ЄГОР-СОНІК ОСТАНІН, СУПЕР-СОФІЯ ОСТАНІНА, КИЇВ, ВУЛИЦЯ ДЕМІЇВСЬКА 16
bool newMessageAvailable = true;

void mqtt_callback(char* topic, byte* payload, unsigned int length)
{
  char tempPayload[1024] = {0};
  PRINT(topic, ""); // выводим в сериал порт название топика
  PRINT(" => length=", length); // выводим в сериал порт значение полученных данных

  memcpy(tempPayload, payload, length);

  if(String(topic) == "homeassistant/light/MAX7219/delay/set") // проверяем из нужного ли нам топика пришли данные
  {
    const int delay = atoi(tempPayload); // преобразуем полученные данные в тип integer
    PRINT("delay=", delay);
    delay_text_movement = delay;
  }
  else if(String(topic) == "homeassistant/light/MAX7219/delay/increase") // проверяем из нужного ли нам топика пришли данные
  {
    const int delay_delta = atoi(tempPayload); // преобразуем полученные данные в тип integer
    PRINT("delay_delta=", delay_delta);
    delay_text_movement -= delay_delta;
  }
  else if(String(topic) == "homeassistant/light/MAX7219/delay/decrease") // проверяем из нужного ли нам топика пришли данные
  {
    const int delay_delta = atoi(tempPayload); // преобразуем полученные данные в тип integer
    PRINT("delay_delta=", delay_delta);
    delay_text_movement += delay_delta;
  }
  else if(String(topic) == "homeassistant/light/MAX7219/text/set") // проверяем из нужного ли нам топика пришли данные
  {
    uint16_t putIndex = 0;
    memset(message, 0, sizeof(message));
    for(uint16_t char_i = 0 ; char_i < length ; ++char_i)
    {
      uint8_t serial_read = (uint8_t)tempPayload[char_i];
      message[putIndex] = serial_read;
      if(serial_read == 208)
      {
        message[putIndex] <<= 8;
        ++char_i;
        serial_read = (uint8_t)tempPayload[char_i];
        message[putIndex] |= serial_read;
        ++putIndex;
      }
      else if ((message[putIndex] == '\n') || (putIndex >= BUF_SIZE-3))  // end of message character or full buffer
      {
        // put in a message separator and end the string
        message[putIndex] = '\0';
        // restart the index for next filling spree and flag we have a message waiting
        putIndex = 0;
      }
      else
      {
        // Just save the next char in next location
        message[putIndex++];
      }
    }
    newMessageAvailable = true;
  }
}

void handle_wifi_connection_first_check()
{
  static bool message_has_ip = false;
  char ip_str[16] = {0};
  PRINT3(__func__, ":", __LINE__);
  int start = millis();
  if(WiFi.status() != WL_CONNECTED)
  {
    PRINT(".", "");
  }
  else if(false == message_has_ip)
  {
    PRINT("WiFi connected: IP address: ", WiFi.localIP());
    const uint message_len = get_message_length();
    IPAddress ip = WiFi.localIP();
    sprintf(ip_str, ", IP: %d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    const uint ip_len = strlen(ip_str);
    uint16_t *pMessage = message + message_len - 1;
    PRINT("message_len=", message_len);
    // *pMessage = ',';
    // *(++pMessage) = ' ';
    // for(uint char_i = 0; char_i < ip_len ; ++char_i)
    // {
    //   uint16_t wide_char = ip_str[char_i];
    //   *(++pMessage) = wide_char;
    // }
    PRINT("get_message_length()=", get_message_length());
    message_has_ip = true;

    // for(uint char_i = 0 ; message[char_i] != 0 ; ++char_i)
    // {
    //   PRINT("char_i=", char_i);
    //   PRINT("char=", message[char_i]);
    // }


    // newMessageAvailable = true;
  }
  int end = millis();
  PRINT("end - start=", end - start);
}

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
  uint16_t	putIndex = 0;
  textChanged = false;

  if (Serial.available())
  {
    memset(message, 0, sizeof(message));
  }

  while (Serial.available())
  {
    uint8_t serial_read = (uint8_t)Serial.read();
    message[putIndex] = serial_read;
    PRINT("serial_read=", serial_read);
    if(serial_read == 208)
    {
      message[putIndex] <<= 8;
      while (!Serial.available());
      serial_read = (uint8_t)Serial.read();
      PRINT("serial_read=", serial_read);
      message[putIndex] |= serial_read;
      PRINT("message[putIndex]=", message[putIndex]);
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

void eeprom_message_read()
{
  unsigned char eeprom_value = 0;
  uint16_t message_bytes_size = 0;
  constexpr unsigned int epprom_data_offset = 4;

  EEPROM.begin(epprom_data_offset);

  if('A' != EEPROM[0] || 'B' != EEPROM[1])
  {
    newMessageAvailable = true;
    return;
  }
  eeprom_value = EEPROM[2];
  message_bytes_size = eeprom_value << 8;
  eeprom_value = EEPROM[3];
  message_bytes_size |= eeprom_value;
  if(0xffff != message_bytes_size)
  {
    memset(message, 0, sizeof(message));

    EEPROM.begin(epprom_data_offset + message_bytes_size);

    const uint16_t message_utf8_size = message_bytes_size / sizeof(message[0]);
    for(int message_i = 0 ; message_i < message_utf8_size ; ++message_i)
    {
      eeprom_value = EEPROM[message_i*2 + epprom_data_offset];
      message[message_i] = eeprom_value << 8;
      eeprom_value = EEPROM[message_i*2 + epprom_data_offset + 1];
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
  constexpr unsigned int epprom_data_offset = 4;
  EEPROM.begin(4 + message_bytes_size);
  EEPROM[0] = 'A';//magic numbers
  EEPROM[1] = 'B';
  EEPROM[2] = message_bytes_size >> 8;
  EEPROM[3] = message_bytes_size;
  for(int message_i = 0 ; message_i < message_utf8_size ; ++message_i)
  {
    eeprom_value = message[message_i] >> 8;
    EEPROM[message_i*2 + epprom_data_offset] = eeprom_value;
    eeprom_value = message[message_i];
    EEPROM[message_i*2 + epprom_data_offset + 1] = eeprom_value;
  }
  EEPROM.commit();
}

void printText(uint8_t modStart, uint8_t modEnd, uint16_t *pMsg)
// Print the text string to the LED matrix modules specified.
// Message area is padded with blank columns after printing.
{
  uint16_t   state = 0;
  uint16_t   curLen = 0;
  uint8_t   showLen = 0;
  uint8_t   cBuf[COL_SIZE] = {0};
  int16_t   col = ((modEnd + 1) * COL_SIZE) - 1;


  do     // finite state machine to print the characters in the space available
  {
    switch(state)
    {
      case 0: // Load the next character from the font table
      // PRINT3(__func__,":", __LINE__);
        // if we reached end of message, reset the message pointer
        PRINT("*pMsg=", *pMsg);
        if (*pMsg == '\0')
        {
          // PRINT3(__func__,":", __LINE__);
          showLen = col - (modEnd * COL_SIZE);  // padding characters
          state = 2;
          return;
          break;
        }
        // PRINT3(__func__,":", __LINE__);
        if(!getUkranianChar(*pMsg, cBuf, &showLen))
        {
          // retrieve the next character form the font file
          showLen = mx.getChar((char)*pMsg, sizeof(cBuf)/sizeof(cBuf[0]), cBuf);
          // PRINT("showLen=", showLen);
        }
        // PRINT("showLen=", showLen);
        
        pMsg++;
        // PRINT("*pMsg=", *pMsg);
        // for(int char_i = 0 ; char_i < sizeof(cBuf) ; ++char_i)
        // {
        //   PRINT(cBuf[char_i], ", ");
        // }
        curLen = 0;
        state++;
        // !! deliberately fall through to next state to start displaying

      case 1: // display the next part of the character
        mx.setColumn(col--, cBuf[curLen++]);
        // PRINT("curLen=", curLen);

        // done with font character, now display the space between chars
        if (curLen == showLen)
        {
          showLen = CHAR_SPACING;
          // PRINT("showLen=", showLen);
          // PRINT3(__func__,":", __LINE__);
          state = 2;
        }
        break;

      case 2: // initialize state for displaying empty columns
      // PRINT3(__func__,":", __LINE__);
        curLen = 0;
        state++;
        // fall through

      case 3:	// display inter-character spacing or end of message padding (blank columns)
      // PRINT3(__func__,":", __LINE__);
        mx.setColumn(col--, 0);
        curLen++;
        // PRINT("curLen=", curLen);
        // PRINT("showLen=", showLen);
        if (curLen == showLen)
        {
          // PRINT3(__func__,":", __LINE__);
          state = 0;
        }
        break;

      default:
      // PRINT3(__func__,":", __LINE__);
        col = -1;   // this definitely ends the do loop
    }
  } while (col >= (modStart * COL_SIZE));
}

void handle_text_movement()
{
  // Shift over and insert the new column
  mx.control(0, MAX_DEVICES-1, MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
  mx.transform(MD_MAX72XX::TSR);
  mx.control(0, MAX_DEVICES-1, MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
  yield();
}

void handle_add_new_text()
{
  mx.control(0, MAX_DEVICES-1, MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
  mx.control(0, MAX_DEVICES-1, MD_MAX72XX::WRAPAROUND, MD_MAX72XX::OFF);
  PRINT3(__func__,":", __LINE__);
  printText(0, MAX_DEVICES-1, message);
  PRINT3(__func__,":", __LINE__);
  yield();
  mx.control(0, MAX_DEVICES-1, MD_MAX72XX::WRAPAROUND, MD_MAX72XX::ON);
  yield();
  for(int column_i = 0 ; column_i < COL_SIZE * (MAX_REAL_DEVICES); ++column_i)
  {
    if((column_i % COL_SIZE * 200) == 0)
    {
      yield();
    }
    mx.transform(MD_MAX72XX::TSR);
  }
  mx.transform(MD_MAX72XX::TFUD);
  mx.transform(MD_MAX72XX::TFLR);
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

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  pinger.OnReceive([](const PingerResponse& response)
  {
    if (response.ReceivedResponse)
    {
      if (!client.connected())
      {
        PRINT("Connecting to MQTT server", "");
        if (client.connect("homeassistant/light/MAX7219", mqtt_user, mqtt_pass))
        {
          PRINT("Connected to MQTT server", "");
          client.subscribe("homeassistant/light/MAX7219/delay/set");
          client.subscribe("homeassistant/light/MAX7219/delay/increase");
          client.subscribe("homeassistant/light/MAX7219/delay/decrease");
          client.subscribe("homeassistant/light/MAX7219/text/set");
        } else {
          PRINT("Could not connect to MQTT server", "");
        }
      }
    }
    else
    {
      PRINT("Request timed out.", "");
    }

    // Return true to continue the ping sequence.
    // If current event returns false, the ping sequence is interrupted.
    return true;
  });

  delay_text_movement_millis_last = millis();
  delay_wifi_connection_first_millis_last = delay_text_movement_millis_last;
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
    static uint32_t offsetX = 0;
    yield();
    handle_text_movement();
    yield();
    delay_text_movement_millis_last = millis();
    ++offsetX;
  }

  if(millis() - delay_wifi_connection_first_millis_last > 10000)
  {
    PRINT3(__func__,":", __LINE__);
    handle_wifi_connection_first_check();
    if(WiFi.status() == WL_CONNECTED)
    {
      if (!client.connected())
      {
        if(pinger.Ping(mqtt_server) == false)
        {
          PRINT("Error during ping command.", "");
        }
      }
    }
    PRINT3(__func__,":", __LINE__);
    delay_wifi_connection_first_millis_last = millis();
  }

  if (client.connected())
  {
    client.loop();
  }
}

void yield() {
  // ваш код
}