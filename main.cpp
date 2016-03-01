/*
 * main.cpp
 *
 *  Created on: 2013-05-11
 *      Author: Matthew Brigdan
 */

#include <Arduino.h>
#include "OneWire.h"
#include <avr/eeprom.h>

// Uncomment if you need it for C++ style new/delete and pointer protection
//#include "cppsupport.h"


uint16_t EEMEM overflowLimitEEPROM = 858;

#ifdef batteryheat
const int batteryHeatActivationTemp = -1;
const char batteryHeaterFET = PD2;
#endif

const char rpi_reset = PD6;
const char pi_power = PD7;
const char LED = PD3;
const char sensor_enable = PB2;
const char ext_temp = 9; //arduino pin number
const char int_temp = 8; //arduino pin number
const int tempNotConnectedReading = -16000; //Converts to -500C



inline void enable_pi_power();
inline void disable_pi_power();
inline void enable_LED();
inline void disable_LED();
inline void enable_4_20mASensors();
inline void disable_4_20mASensors();

inline void enable_pi_power()
{
	PORTD &= ~(1 << pi_power);
	DDRD |= (1 << pi_power);
	enable_LED();
	enable_4_20mASensors();
}

inline void disable_pi_power()
{
	DDRD &= ~(1 << pi_power);
	PORTD |= 1 << pi_power;
	disable_LED();
	disable_4_20mASensors();
}

inline void enable_LED()
{
	DDRD |= (1 << LED);
	PORTD |= (1 << LED);
}

inline void disable_LED()
{
	DDRD &= ~(1 << LED);
	PORTD &= ~(1 << LED);
}

inline void enable_4_20mASensors()
{
	DDRB |= (1 << sensor_enable);
	PORTB |= (1 << sensor_enable);
}

inline void disable_4_20mASensors()
{
	DDRB &= ~(1 << sensor_enable);
	PORTB &= ~(1 << sensor_enable);
}

#ifdef batteryheat
inline void enableBatteryHeat()
{
	DDRD |= (1 << batteryHeaterFET);
	PORTD |= (1 << batteryHeaterFET);
}

inline void disableBatteryHeat()
{
	DDRD &= ~(1 << batteryHeaterFET);
	PORTD &= ~(1 << batteryHeaterFET);
}
#endif

OneWire external(ext_temp); //1-wire temperature sensor external ds18b20
OneWire internal(int_temp); //1-wire temperature sensor internal ds18b20

//Timer1 overflow interrupt handler
//volatile bool wake_flag = false;
volatile bool wake_count_enable = true;
volatile uint16_t t1_overflows = 0;
volatile bool getBatteryTemp = false;
uint16_t overflowLimit = 858;
ISR(TIMER1_OVF_vect)
{
	if (wake_count_enable == true)
	{
		++t1_overflows;
	}

	TIFR1 = 0; //clear timer1 overflow flag

	if (t1_overflows >= overflowLimit)
	{
		//wake_flag=true;
		t1_overflows = 0;
		//wake_count_enable = false;
		enable_pi_power();
		enable_LED();
	}
#ifdef batteryheat
	else if ((t1_overflows % 10) == 0)
	{
		getBatteryTemp = true;
	}
#endif
}

void setup()
{
	enable_LED();
	delay(500);
	Serial.begin(9600);
	delay(1000);
	disable_LED();

	enable_pi_power();

	overflowLimit = eeprom_read_word(&overflowLimitEEPROM);
	if (overflowLimit > 20600 || overflowLimit < 143) //If recorded value is greater than 1 day or less than 10 minutes
	{
		overflowLimit = 858;
		eeprom_write_word(&overflowLimitEEPROM, overflowLimit);

	}

	//Enable pullups on DIOs (PD2->PD7)
	DDRD &= ~(1 << PD2 | 1 << PD4 | 1 << PD5 | 1 << PD6); //Data direction is input
	PORTD |= (1 << PD2 | 1 << PD4 | 1 << PD5 | 1 << PD6); //Enable pullup (output high)

	//Setup timer1
	TCCR1A = 0;
	TCCR1B = 0;
	TCNT1 = 0; //set value of timer1 to zero
	TIMSK1 |= (1 << TOIE1); //enable timer1 interrupt
	TCCR1B |= (1 << CS12) | (1 << CS10); //set divide by 1024 prescaler on timer1

	sei();
}

/*inline void wake_pi()
 {
 //set pin low and then drive it, (hopefully) waking the pi
 PORTD &= 0xFF & (0 << rpi_reset);
 DDRD |= (1 << rpi_reset);
 delay(50);
 DDRD &= 0xFF & (0 << rpi_reset);
 delay(50);
 }*/

void sendPressureReading();
int readBatteryTemp();

char serialRead[20];
unsigned short index = 0;
unsigned short count = 0;

void loop()
{
	while (Serial.available() > 0)
	{
		if (index < 19)
		{
			serialRead[index] = Serial.read();
			index++;
			serialRead[index] = '\0'; // Null terminate the string
		}
		else
		{
			Serial.read();
		}
	}

	if (strcmp(serialRead, "READV1") == 0)
	{
		enable_LED();
		sendPressureReading();
		disable_LED();
		index = count = 0;
		serialRead[0] = '\0';
	}
	else if (strcmp(serialRead, "SLEEP") == 0)
	{
		Serial.println("ACK");
		enable_LED();
		delay(20000); //20s wait to let the pi power down
		disable_pi_power();
		index = count = 0;
		serialRead[0] = '\0';
		wake_count_enable = true;
	}
	else if (strcmp(serialRead, "SETSLEEP") == 0)
	{
		overflowLimit = Serial.parseInt();
		eeprom_write_word(&overflowLimitEEPROM, overflowLimit);
		Serial.print("SLEEPSET");
		Serial.println(overflowLimit);
	}
	else if ((strncmp(serialRead, "READV1", index) == 0)
			|| (strncmp(serialRead, "SLEEP", index) == 0)
			|| (strncmp(serialRead, "SETSLEEP", index) == 0)) //first part matches
	{
		++count;
		if (count > 15)
		{
			index = count = 0;
			serialRead[0] = '\0';
		}
	}
	else
	{
		index = count = 0;
		serialRead[0] = '\0';
	}

#ifdef batteryheat
	if (getBatteryTemp == true)
	{
		getBatteryTemp = false;
		int temp = readBatteryTemp();
		if (temp <= batteryHeatActivationTemp)
		{
			enableBatteryHeat();
		}
		else
		{
			disableBatteryHeat();
		}
	}
#endif batteryheat

	/* Old Method used to wake the pi
	 if (wake_flag)
	 {
	 wake_flag = false;

	 wake_pi();
	 wake_pi();
	 wake_pi();
	 wake_pi();
	 }
	 */

	delay(50);

}

int main(void)
{
	init();

	setup();
	enable_pi_power();
	for (;;)
		loop();

	return 0;
}
#ifdef batteryheat
int readBatteryTemp()
{
	const int notConnected = 404; //Needs to be a high value to prevent the battery heater from turning on

	uint16_t tempReading = 0;
	byte signBit = 0;
	byte addr[8];
	external.reset_search();
	if (external.search(addr))
	{
		if (!external.search(addr)) //Get the second device on the bus
		{
			//Search failed, return a temperature that won't cause the battery heater to turn on
			return notConnected;
		}
		external.reset();
		external.select(addr);
		external.write(0x44); //start temperature conversion
		delay(750); //750ms delay to allow conversion to complete
		external.reset();
		external.select(addr);
		external.write(0xBE); // Read Scratchpad
		byte ii = 0;
		byte data[12];
		for (ii = 0; ii < 9; ii++)
		{
			data[ii] = external.read();
		}
		tempReading = (data[1] << 8) + data[0];
		signBit = data[1] & 0x80;
		if (signBit) // negative
		{
			tempReading = (tempReading ^ 0xffff) + 1; //two's complement conversion
		}
		return tempReading / 16;
	}
	else
	{
		return notConnected; //Didn't even find 1 device on the bus
	}
}
#endif

void sendPressureReading()
{

	//For 1-wire temp sensor:
	byte SignBitE = 0;
	bool searchFailE = false;
	uint16_t TReadingE = 0;
	byte iE = 0;
	byte presentE = 0;
	byte dataE[12];
	byte addrE[8];

	byte SignBitE2 = 0;
	uint16_t TReadingE2 = 0;
	byte iE2 = 0;
	byte addrE2[8];
	byte dataE2[12];
	bool searchFailE2 = false;

	external.reset_search();
	if (!external.search(addrE))
	{
		searchFailE = true; //couldn't find device on bus
	}
	if (OneWire::crc8(addrE, 7) != addrE[7])
	{
		searchFailE = true; //CRC error
	}
	if (addrE[0] != 0x28)
	{
		searchFailE = true; //device is wrong type
	}
	if (!external.search(addrE2))
	{
		searchFailE2 = true; //couldn't find device on bus
	}
	if (OneWire::crc8(addrE2, 7) != addrE2[7])
	{
		searchFailE2 = true; //CRC error
	}
	if (addrE2[0] != 0x28)
	{
		searchFailE2 = true; //device is wrong type
	}

	external.reset();
	external.select(addrE);
	external.write(0x44); //start conversion
	external.reset();
	external.select(addrE2);
	external.write(0x44); //start conversion

	//For 1-wire temp sensor:
	byte SignBitI = 0;
	bool searchFailI = false;
	uint16_t TReadingI = 0;
	byte iI = 0;
	byte presentI = 0;
	byte dataI[12];
	byte addrI[8];

	internal.reset_search();
	if (!internal.search(addrI))
	{
		searchFailI = true; //couldn't find device on bus
	}
	if (OneWire::crc8(addrI, 7) != addrI[7])
	{
		searchFailI = true; //CRC error
	}
	if (addrI[0] != 0x28)
	{
		searchFailI = true; //device is wrong type
	}

	internal.reset();
	internal.select(addrI);
	internal.write(0x44); //start conversion

	enable_LED();
	enable_4_20mASensors();
	delay(4000); //allow values to stabilise

	unsigned long analog0 = 0;
	for (unsigned int ii = 1; ii <= 1024; ++ii)
	{
		analog0 += analogRead(A0);
	}

	unsigned long analog1 = 0;
	for (unsigned int ii = 1; ii <= 1024; ++ii)
	{
		analog1 += analogRead(A1);
	}

	unsigned long analog2 = 0;
	for (unsigned int ii = 1; ii <= 1024; ++ii)
	{
		analog2 += analogRead(A2);
	}

	unsigned long analog3 = 0;
	for (unsigned int ii = 1; ii <= 1024; ++ii)
	{
		analog3 += analogRead(A3);
	}

	unsigned long analog4 = 0;
	for (unsigned int ii = 1; ii <= 1024; ++ii)
	{
		analog4 += analogRead(A4);
	}

	unsigned long analog5 = 0;
	for (unsigned int ii = 1; ii <= 1024; ++ii)
	{
		analog5 += analogRead(A5);
	}

	unsigned long analog6 = 0;
	for (unsigned int ii = 1; ii <= 1024; ++ii)
	{
		analog6 += analogRead(A6);
	}

	unsigned long analog7 = 0;
	for (unsigned int ii = 1; ii <= 1024; ++ii)
	{
		analog7 += analogRead(A7);
	}

	delay(750); //750ms + the time for all analog readings should guarantee conversion is complete
	external.reset();
	external.select(addrE);
	external.write(0xBE);         // Read Scratchpad
	for (iE = 0; iE < 9; iE++)
	{
		dataE[iE] = external.read();
	}
	TReadingE = (dataE[1] << 8) + dataE[0];
	SignBitE = dataE[1] & 0x80;
	if (SignBitE) // negative
	{
		TReadingE = (TReadingE ^ 0xffff) + 1; //two's complement conversion
	}

	external.reset();
	external.select(addrE2);
	external.write(0xBE);         // Read Scratchpad
	for (iE2 = 0; iE2 < 9; iE2++)
	{
		dataE2[iE2] = external.read();
	}
	TReadingE2 = (dataE2[1] << 8) + dataE2[0];
	SignBitE2 = dataE2[1] & 0x80;
	if (SignBitE2) // negative
	{
		TReadingE2 = (TReadingE2 ^ 0xffff) + 1; //two's complement conversion
	}

	internal.reset();
	internal.select(addrI);
	internal.write(0xBE);         // Read Scratchpad
	for (iI = 0; iI < 9; iI++)
	{
		dataI[iI] = internal.read();
	}
	TReadingI = (dataI[1] << 8) + dataI[0];
	SignBitI = dataI[1] & 0x80;
	if (SignBitI) // negative
	{
		TReadingI = (TReadingI ^ 0xffff) + 1; //two's complement conversion
	}

	disable_LED();
	disable_4_20mASensors();
	if (!searchFailE)
	{
		if (SignBitE)
		{
			Serial.print("-");
		}
		Serial.println(TReadingE); //Slot1
	}
	else
	{
		Serial.println(tempNotConnectedReading);
	}
	if (!searchFailE2)
	{
		if (SignBitE2)
		{
			Serial.print("-");
		}
		Serial.println(TReadingE2); //Slot2
	}
	else
	{
		Serial.println(tempNotConnectedReading);
	}
	if (!searchFailI)
	{
		if (SignBitI)
		{
			Serial.print("-");
		}
		Serial.println(TReadingI); //Slot3
	}
	else
	{
		Serial.println(tempNotConnectedReading);
	}
	Serial.println(analog0); //Slot4
	Serial.println(analog1); //Slot5
	Serial.println(analog2); //Slot6
	Serial.println(analog3); //Slot7
	Serial.println(analog4); //Slot8
	Serial.println(analog5); //Slot9
	Serial.println(analog6); //Slot10
	Serial.println(analog7); //Slot11
	Serial.println((PIND & (1 << PD2)) >> PD2); //Slot12
	Serial.println((PIND & (1 << PD4)) >> PD4); //Slot13
	Serial.println((PIND & (1 << PD5)) >> PD5); //Slot14
	Serial.println((PIND & (1 << PD6)) >> PD6); //Slot15
	delay(100);
}
