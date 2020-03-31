/**
 ******************************************************************************
 * @file    X_NUCLEO_S2915A1_HelloWorld.ino
 * @author  SRA
 * @version V1.0.0
 * @date    12 March 2020
 * @brief   Arduino test application for the STMicrolectronics X-NUCLEO-S2915A1
 *          Sub-1 GHz 915 MHz RF expansion board based on S2-LP module.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#include "SPI.h"
#include "M95640R.h"
#include "S2LP.h"

#define SerialPort Serial

SPIClass *devSPI;
M95640R *myM95640R;
S2LP *myS2LP;
volatile uint8_t receive_packet = 0;
const int buttonPin = PC13; // set buttonPin to digital pin PC13 */
int pushButtonState = LOW;

static uint8_t send_buf[FIFO_SIZE] ={'S','2','L','P',' ','H','E','L','L','O',' ','W','O','R','L','D',' ','P','2','P',' ','D','E','M','O'};
static uint8_t read_buf[FIFO_SIZE] ={0};

void callback_func(void);
void recv_data(void);
void blink_led(void);
uint8_t eeprom_identification(void);
void read_eeprom_content(uint32_t *s_frequency, uint32_t *s_RfXtalFrequency, RangeExtType *s_RfRangeExtender);
uint32_t get_frequency_band(uint8_t s_RfModuleBand);


/* Setup ---------------------------------------------------------------------*/

void setup() {
  uint32_t s_frequency = 868000000;
  uint32_t s_RfXtalFrequency = 50000000;
  PAInfo_t paInfo;

  memset(&paInfo, 0, sizeof(PAInfo_t));

  paInfo.paSignalCSD_MCU = A0;
  paInfo.paSignalCPS_MCU = A2;
  paInfo.paSignalCTX_MCU = A3;

  // Initialize serial for output.
  SerialPort.begin(115200);

  // Initialize Led.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize Button
  pinMode(buttonPin, INPUT);
  pushButtonState = (digitalRead(buttonPin)) ?  LOW : HIGH;

  // Put S2-LP in Shutdown
  pinMode(D7, OUTPUT);
  digitalWrite(D7, HIGH);

  // Initialize SPI
  devSPI = new SPIClass(D11, D12, D3);
  devSPI->begin();

  // Initialize M95640-R
  myM95640R = new M95640R(devSPI, D5);
  myM95640R->begin();

  // Read X-NUCLEO-S2915A1 EEPROM
  if(eeprom_identification())
  {
    SerialPort.println("EEPROM present");
    read_eeprom_content(&s_frequency, &s_RfXtalFrequency, &paInfo.paRfRangeExtender);
  } else
  {
    SerialPort.println("EEPROM not present");
  }

  // Initialize S2-LP
  myS2LP = new S2LP(devSPI, A1, D7, A5, s_frequency, s_RfXtalFrequency, paInfo);
  myS2LP->begin();
  myS2LP->attachS2LPReceive(callback_func);
}

/* Loop ----------------------------------------------------------------------*/

void loop() {
  if(digitalRead(buttonPin) == pushButtonState)
  {
    /* Debouncing */
    HAL_Delay(50);

    /* Wait until the button is released */
    while (digitalRead(buttonPin) == pushButtonState);

    /* Debouncing */
    HAL_Delay(50);

    if(!myS2LP->send(send_buf, (strlen((char *)send_buf) + 1), 0x44, true))
    {
      /* Blink LED */
      blink_led();

      /* Print message */
      SerialPort.print("Transmitted ");
      SerialPort.print((strlen((char *)send_buf) + 1));
      SerialPort.println(" bytes successfully");
    } else
    {
      SerialPort.println("Error in transmission");
    }
  }

  if(receive_packet)
  {
    receive_packet = 0;
    recv_data();

    /* Blink LED */
    blink_led();
  }
}

void recv_data(void)
{
  uint8_t data_size = myS2LP->getRecvPayloadLen();

  myS2LP->read(read_buf, data_size);

  SerialPort.print("Received packet (size of ");
  SerialPort.print(data_size);
  SerialPort.print(" bytes): ");
  SerialPort.println((char *)read_buf);
}

void callback_func(void)
{
  receive_packet = 1;
}

void blink_led(void)
{
  digitalWrite(LED_BUILTIN, HIGH);
  delay(10);
  digitalWrite(LED_BUILTIN, LOW);
  delay(10);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(10);
  digitalWrite(LED_BUILTIN, LOW);
}

uint8_t eeprom_identification(void)
{
  uint8_t status=0;

  status = myM95640R->EepromStatus();

  if((status&0xF0) == EEPROM_STATUS_SRWD) {
    /* If it is EEPROM_STATUS_SRWD => OK, the EEPROM is present and ready to work */
    status=1;
  }
  else
  {
    myM95640R->EepromWriteEnable();
    delay(10);
    /* Else the bit may be not set (first time we see this EEPROM), try to set it*/
    status = myM95640R->EepromSetSrwd();
    delay(10);
    /*check again*/
    status = myM95640R->EepromStatus();

    if((status&0xF0) == EEPROM_STATUS_SRWD) { // 0xF0 mask [SRWD 0 0 0]
      /* If it is EEPROM_STATUS_SRWD => OK, the EEPROM is present and ready to work */
      status=1;
    }
    else
    {
      /* Else no EEPROM is present */
      status = 0;
    }
  }

  return status;
}

void read_eeprom_content(uint32_t *s_frequency, uint32_t *s_RfXtalFrequency, RangeExtType *s_RfRangeExtender)
{
  float foffset = 0;
  uint8_t tmpBuffer[32];
  uint8_t s_RfModuleBand = 0;
  int32_t xtal_comp_value = 0;

  /* Read the EEPROM */
  myM95640R->EepromRead(0x0000, 32, tmpBuffer);

  /* Data in EEPROM is not valid ... */
  if(tmpBuffer[0]==0 || tmpBuffer[0]==0xFF) {
    *s_RfXtalFrequency = 50000000;

    /* If EEPROM fails, set no EXT_PA by default */
    *s_RfRangeExtender = RANGE_EXT_NONE;

    return;
  }

  switch(tmpBuffer[1]) {
  case 0:
    *s_RfXtalFrequency = 24000000;
    break;
  case 1:
    *s_RfXtalFrequency = 25000000;
    break;
  case 2:
    *s_RfXtalFrequency = 26000000;
    break;
  case 3:
    *s_RfXtalFrequency = 48000000;
    break;
  case 4:
    *s_RfXtalFrequency = 50000000;
    break;
  case 5:
    *s_RfXtalFrequency = 52000000;
    break;
  default:
    *s_RfXtalFrequency = 50000000;
    break;
  }

  s_RfModuleBand = tmpBuffer[3];

  myM95640R->EepromRead(0x0021,4,tmpBuffer);

  for(uint8_t i=0;i<4;i++)
  {
    ((uint8_t*)&foffset)[i]=tmpBuffer[3-i];
  }

  xtal_comp_value = 0;

  /* foffset is a value measured during manufacturing as follows:
  *
  * foffset = fnominal-fmeasured.
  * To compensate such value it should be reported to xtal freq
  * and then subtracted
  *
  */
  if (foffset != 0xFFFFFFFF) {
    uint32_t frequency = get_frequency_band(s_RfModuleBand);

    if (frequency != 0)
    {
	  uint32_t xtal_frequency = *s_RfXtalFrequency;

	  /* This is the value to be added to the xtal nominal value
	  to compensate the xtal offset */
	  xtal_comp_value = (int32_t) ((xtal_frequency*(-foffset))/frequency);

      *s_frequency = frequency;
    }
  }

  *s_RfXtalFrequency = *s_RfXtalFrequency + xtal_comp_value;

  *s_RfRangeExtender = (RangeExtType)tmpBuffer[5];
}

uint32_t get_frequency_band(uint8_t s_RfModuleBand)
{
  uint32_t frequency = 0;
  const uint32_t band_frequencies[] = {
    169000000,
    315000000,
    433000000,
    868000000,
    915000000,
    450000000
  };

  if (s_RfModuleBand < (sizeof(band_frequencies)/sizeof(uint32_t))) {
    frequency = band_frequencies[s_RfModuleBand];
  }

  return frequency;
}
