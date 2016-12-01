//#if defined(ARDUINO) && (ARDUINO >= 100)
//#include "Arduino.h"
//#else
//#include "WProgram.h" // shouldn't need this but arduino sometimes messes up and puts inside an #ifdef
//#endif // ARDUINO

#include <Time.h>
#include <Wire.h>
#include <SoftI2C.h>
#include <avr/wdt.h>
#include <SPI.h>
#include <EEPROM.h>

// EMON Stuff
#define EE_TOTAL_COST            0
#define EE_TOTAL_TIME            4
#define EE_TOTAL_KWH             8

// New LCD Code
#include <LiquidCrystal.h>
uint8_t RSPin = 2;
uint8_t RWPin = 3;
uint8_t ENPin = 4;
uint8_t D4Pin = 5;
uint8_t D5Pin = 6;
uint8_t D6Pin = 7;
uint8_t D7Pin = 8;
uint8_t BLPin = 9;

LiquidCrystal lcd(RSPin, RWPin, ENPin, D4Pin, D5Pin, D6Pin, D7Pin);

byte customChar[8] = {
  0b00100,
  0b01110,
  0b10001,
  0b10000,
  0b10000,
  0b10001,
  0b01110,
  0b00100
};

#include "EmonLib.h" // Include Emon Library
#define EMON_VT_PIN                   A1           // Analog voltage transformer reading pin
#define EMON_CT_PIN                   A0           // Analog current transformer reading pin
#define CURRENT_CALIBRATION   ((double)(16.9594594594595))   // Current calibration = CT Ratio / Burden resistance = 2000 / 110
#define VOLTAGE_CALIBRATION   234.26
#define EMON_POWERFACTOR      0.98           // Power factor - To be checked
//#define KWH_RATE              12.08

#define TOGGLEPIN                 10

#define EVSE_I2C_ADDR    0x44
#define EMON_I2C_ADDR    0x45

bool togglepinstatus, clearme, charging, chargeOn, chargeOff, chargeAbort, clearStats;

time_t totaltime;
float watts, kw, cost, totalcost, ikwh, kwh, totalkwh, tempfloat, amps, volts, pf, ap, rp;
EnergyMonitor g_EMON;
time_t m_ChargeStartTime;
time_t m_ElapsedChargeTime;
time_t m_ElapsedChargeTimePrev;
byte cmdarg[5];
int command;
int buttonState;             // the current reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long templong;
#define BTN_PRESS_SHORT 300  // ms
#define BTN_PRESS_LONG 1000 // ms
#define BTN_STATE_OFF   0
#define BTN_STATE_SHORT 1 // short press
#define BTN_STATE_LONG  2 // long press

char g_sTmp[64];
byte registerMap[32];
byte registerMapTemp[32];
int16_t result;
byte highlow[4];
unsigned long startSaveTime;

float kwh_rate;

SoftI2C i2c = SoftI2C(A2, A3);

// wdt_init turns off the watchdog timer after we use it
// to reboot
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));
void wdt_init(void)
{
  MCUSR = 0;
  wdt_disable();

  return;
}

void setup()
{
  wdt_disable();

  //  Serial.begin(9600);
  Wire.begin(EMON_I2C_ADDR);
  Wire.onReceive (receiveEvent);  // interrupt handler for incoming messages
  Wire.onRequest (requestEvent);  // interrupt handler for when data is wanted

  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  digitalWrite(A2, HIGH);
  digitalWrite(A3, HIGH);

  highlow[0] = EEPROM.read(0x00);
  highlow[1] = EEPROM.read(0x01);
  kwh_rate = word(highlow[0], highlow[1]) * 0.01;

  lcd.createChar(0, customChar);
  lcd.begin(16, 2);
  pinMode(BLPin, OUTPUT);
  backlight();
  lcd.display();
  lcd.setCursor(0, 0);
  lcd.print(" EVSE Power Mon ");
  lcd.setCursor(0, 1);
  if (i2c.startRead(B01010000, 1)) {
    lcd.print("  EEPROM Found  ");
  } else {
    lcd.print(" EEPROM Missing ");
  }
  delay(2000);
  lcd.setCursor(0, 1);
  if (eeprom_i2c_read(B01010000, 15) == 0x55) {
    lcd.print("EEPROM Check: OK");
  } else {
    lcd.print("EEPROM Check: NK");
  }
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Cost: "); dtostrf(kwh_rate, 4, 2, g_sTmp); lcd.print(g_sTmp); lcd.write((uint8_t)0); lcd.print("/kWh");
  delay(2000);
  lcd.clear();
  noBacklight();
  lcd.noDisplay();
  pinMode(TOGGLEPIN, INPUT_PULLUP);
  buttonState = BTN_STATE_OFF;

  g_EMON.current(EMON_CT_PIN, CURRENT_CALIBRATION);
  g_EMON.voltage(EMON_VT_PIN, VOLTAGE_CALIBRATION, 1.7);
  wdt_enable(WDTO_8S);

  //  chargeOn = 1; chargeOff = 0;
  volts = 1;
}

void loop()
{
  wdt_reset();

  if (chargeOff == 1) {
    chargeOff = 0;
    if (charging == 0) {
      lcd.clear();
      lcd.noDisplay();
      noBacklight();
    } else {
      charging = 0;

      eepromWriteFloat(EE_TOTAL_COST, totalcost + cost);
      eepromWriteLong(EE_TOTAL_TIME, totaltime + m_ElapsedChargeTime);
      eepromWriteFloat(EE_TOTAL_KWH, totalkwh + ikwh);
      lcd.clear();
      lcd.print("    Charging    ");
      lcd.setCursor(0, 1);
      lcd.print("   Data Saved");
      delay(2000);
      lcd.clear();
      lcd.noDisplay();
      noBacklight();
      watts = kwh = ikwh = kw = cost = m_ChargeStartTime = m_ElapsedChargeTime = totalcost = totaltime = totalkwh = 0;
      togglepinstatus = LOW;
    }
  }

  if (chargeOn == 1) {
    chargeOn = 0;
    if (volts == 1) {
      volts = 120.50;
    } else if (volts == 2) {
      volts = 240.50;
    } else {
      volts = 0;
    }
    charging = 1;
    kw = 0;
    cost = 0;
    watts = 0;
    totalcost = eepromReadFloat(EE_TOTAL_COST);
    totaltime = eepromReadLong(EE_TOTAL_TIME);
    totalkwh = eepromReadFloat(EE_TOTAL_KWH);
    m_ChargeStartTime = now();
    backlight();
    lcd.display();
  }

  if (chargeAbort == 1) {
    if (charging == 1) {
      chargeAbort = 0;
      charging = 0;
      eepromWriteFloat(EE_TOTAL_COST, totalcost + cost);
      eepromWriteLong(EE_TOTAL_TIME, totaltime + m_ElapsedChargeTime);
      eepromWriteFloat(EE_TOTAL_KWH, totalkwh + ikwh);
      watts = kwh = ikwh = kw = cost = m_ChargeStartTime = m_ElapsedChargeTime = totalcost = totaltime = totalkwh = 0;
      lcd.clear();
      lcd.print("  Charging has  ");
      lcd.setCursor(0, 1);
      lcd.print("  been Aborted  ");
      delay(2000);
      lcd.clear();
      lcd.noDisplay();
      noBacklight();
    }
  }

  if (clearStats == 1)
    resetstats();

  ////  g_EMON.calcIrms(1480);
  ////  amps = g_EMON.calcIrms(1480);

  if (charging == 1) {
    //    g_EMON.calcVI(20, 2000);        // Calculate all. No.of half wavelengths (crossings), time-out
    g_EMON.calcVI(20, 500);        // Calculate all. No.of half wavelengths (crossings), time-out

    ////    amps = g_EMON.calcIrms(1480);
    ////
    amps = g_EMON.Irms;
    //    volts = g_EMON.Vrms;
    pf = g_EMON.powerFactor;
    ap = g_EMON.apparentPower;
    rp = g_EMON.realPower;
    ////
    m_ElapsedChargeTimePrev = m_ElapsedChargeTime;
    m_ElapsedChargeTime = now() - m_ChargeStartTime;
    if (m_ElapsedChargeTime != m_ElapsedChargeTimePrev) {
      ////      watts = (g_EMON.Irms * volts * EMON_POWERFACTOR);
      watts = (amps * volts * EMON_POWERFACTOR);

      kw += (watts / 1000);
      if (m_ElapsedChargeTime == 0) {
        ikwh = 0;
      } else {
        kwh = (((watts / 1000) * 1.0) * 0.000277777777778);
        ikwh += kwh;
      }

      //      cost = (((watts / 1000) / 3600) * kwh_rate * 0.01) * m_ElapsedChargeTime;
      cost = ikwh * kwh_rate * 0.01;
    }

    unsigned long checkSaveTime = millis();
    if (checkSaveTime - startSaveTime > 1800000000) {
      eepromWriteFloat(EE_TOTAL_COST, totalcost + cost);
      eepromWriteLong(EE_TOTAL_TIME, totaltime + m_ElapsedChargeTime);
      eepromWriteFloat(EE_TOTAL_KWH, totalkwh + ikwh);
      startSaveTime = millis();
    }

    read_button();

    if (shortPress()) {
      togglepinstatus = !togglepinstatus;
      clearme = 1;
    }

    if (longPress()) {
      resetstats();
    }

    if (togglepinstatus == LOW) {
      if (clearme == 1) {
        lcd.clear();
        clearme = 0;
      }
      lcd.home();
      dtostrf(volts, 4, 1, g_sTmp); lcd.print(g_sTmp); lcd.print("v  ");
      //      dtostrf(g_EMON.Vrms, 4, 1, g_sTmp); lcd.print(g_sTmp); lcd.print("v  ");
      lcd.setCursor(7, 0);
      dtostrf(kw, 7, 1, g_sTmp);
      lcd.print(g_sTmp); lcd.setCursor(14, 0); lcd.print("kW");
      lcd.setCursor(0, 1);
      dtostrf(g_EMON.Irms, 5, 2, g_sTmp);
      lcd.print(g_sTmp); lcd.print("A  ");
      lcd.setCursor(9, 1);
      lcd.print("$"); dtostrf(cost, 4, 4, g_sTmp); lcd.print(g_sTmp);
    } else {
      if (clearme == 1) {
        lcd.clear();
        clearme = 0;
      }
      lcd.home();
      dtostrf(ikwh, 5, 2, g_sTmp);
      lcd.print(g_sTmp); lcd.setCursor(5, 0); lcd.print("kWh");
      lcd.setCursor(9, 0);
      lcd.print("$"); dtostrf(totalcost + cost, 6, 2, g_sTmp); lcd.print(g_sTmp);
      lcd.setCursor(0, 1);
      sprintf(g_sTmp, "%08u", totaltime + m_ElapsedChargeTime);
      lcd.print(g_sTmp);
      lcd.setCursor(8, 1);
      dtostrf(totalkwh + ikwh, 5, 0, g_sTmp);
      lcd.print(g_sTmp); lcd.setCursor(13, 1); lcd.print("kWh");
    }
  } else {
    amps = 0;

    read_button();

    if (shortPress()) {
      totalcost = eepromReadFloat(EE_TOTAL_COST);
      totaltime = eepromReadLong(EE_TOTAL_TIME);
      totalkwh = eepromReadFloat(EE_TOTAL_KWH);
      delay(1000);
      backlight();
      lcd.display();
      lcd.clear();
      lcd.print("Totals: ");
      lcd.setCursor(9, 0);
      lcd.print("$"); dtostrf(totalcost, 6, 2, g_sTmp); lcd.print(g_sTmp);
      lcd.setCursor(0, 1);
      sprintf(g_sTmp, "%08u", totaltime);
      lcd.print(g_sTmp);
      lcd.setCursor(8, 1);
      dtostrf(totalkwh, 5, 0, g_sTmp);
      lcd.print(g_sTmp); lcd.setCursor(13, 1); lcd.print("kWh");
      delay(5000);
      lcd.clear();
      lcd.noDisplay();
      noBacklight();
    }

    if (longPress()) {
      resetstats();

      totalcost = eepromReadFloat(EE_TOTAL_COST);
      totaltime = eepromReadLong(EE_TOTAL_TIME);
      totalkwh = eepromReadFloat(EE_TOTAL_KWH);
      delay(1000);
      backlight();
      lcd.display();
      lcd.clear();
      lcd.print("Totals: ");
      lcd.setCursor(9, 0);
      lcd.print("$"); dtostrf(totalcost, 6, 2, g_sTmp); lcd.print(g_sTmp);
      lcd.setCursor(0, 1);
      sprintf(g_sTmp, "%08u", totaltime);
      lcd.print(g_sTmp);
      lcd.setCursor(8, 1);
      dtostrf(totalkwh, 5, 0, g_sTmp);
      lcd.print(g_sTmp); lcd.setCursor(13, 1); lcd.print("kWh");
      delay(5000);
      lcd.clear();
      lcd.noDisplay();
      noBacklight();
    }

  }
}

void resetstats()
{
  for (int i = 0; i < 15; i++) {
    eeprom_i2c_write(B01010000, i, 0x00);
    delay(10);
  }

  totalcost = 0;
  totaltime = 0;
  totalkwh = 0;
  clearStats = 0;
  eeprom_i2c_write(B01010000, 15, 0x55);
  delay(10);
}

void resetMe() {  // for periodic resets to be sure nothing clogs it up
  backlight();
  lcd.display();
  lcd.clear();
  lcd.print(" EMON Rebooting ");
  delay(1000);
  wdt_enable(WDTO_15MS);
  for (;;)
  {
  }
}

void read_button()
{
  uint8_t sample = digitalRead(TOGGLEPIN) ? 0 : 1;

  if (!sample && (buttonState == BTN_STATE_LONG) && !lastDebounceTime) {
    buttonState = BTN_STATE_OFF;
  }

  if ((buttonState == BTN_STATE_OFF) || ((buttonState == BTN_STATE_SHORT) && lastDebounceTime)) {
    if (sample) {
      if (!lastDebounceTime && (buttonState == BTN_STATE_OFF)) {
        lastDebounceTime = millis();
      }
      unsigned long delta = millis() - lastDebounceTime;

      if (buttonState == BTN_STATE_OFF) {
        if (delta >= BTN_PRESS_SHORT) {
          buttonState = BTN_STATE_SHORT;
        }
      }
      else if (buttonState == BTN_STATE_SHORT) {
        if (delta >= BTN_PRESS_LONG) {
          buttonState = BTN_STATE_LONG;
        }
      }
    }
    else { //!sample
      lastDebounceTime = 0;
    }
  }
}

uint8_t shortPress()
{
  if ((buttonState == BTN_STATE_SHORT) && !lastDebounceTime) {
    buttonState = BTN_STATE_OFF;
    return 1;
  }
  else {
    return 0;
  }
}

uint8_t longPress()
{
  if ((buttonState == BTN_STATE_LONG) && lastDebounceTime) {
    lastDebounceTime = 0;
    return 1;
  }
  else {
    return 0;
  }
}

void eepromWriteFloat(int address, float value)
{
  union u_tag {
    byte b[4];
    float fval;
  } u;
  u.fval = value;

  eeprom_i2c_write(B01010000, address, u.b[0]);
  delay(20);
  eeprom_i2c_write(B01010000, address + 1, u.b[1]);
  delay(20);
  eeprom_i2c_write(B01010000, address + 2, u.b[2]);
  delay(20);
  eeprom_i2c_write(B01010000, address + 3, u.b[3]);
  delay(20);
}

float eepromReadFloat(int address)
{
  union u_tag {
    byte b[4];
    float fval;
  } u;
  u.b[0] = eeprom_i2c_read(B01010000, address);
  u.b[1] = eeprom_i2c_read(B01010000, address + 1);
  u.b[2] = eeprom_i2c_read(B01010000, address + 2);
  u.b[3] = eeprom_i2c_read(B01010000, address + 3);
  return u.fval;
}

unsigned long eepromReadLong(int address)
{
  union u_tag {
    byte b[4];
    unsigned long ulval;
  } u;
  u.b[0] = eeprom_i2c_read(B01010000, address);
  u.b[1] = eeprom_i2c_read(B01010000, address + 1);
  u.b[2] = eeprom_i2c_read(B01010000, address + 2);
  u.b[3] = eeprom_i2c_read(B01010000, address + 3);
  return u.ulval;
}

void eepromWriteLong(int address, unsigned long value)
{
  union u_tag {
    byte b[4];
    unsigned long ulval;
  } u;
  u.ulval = value;

  eeprom_i2c_write(B01010000, address, u.b[0]);
  delay(20);
  eeprom_i2c_write(B01010000, address + 1, u.b[1]);
  delay(20);
  eeprom_i2c_write(B01010000, address + 2, u.b[2]);
  delay(20);
  eeprom_i2c_write(B01010000, address + 3, u.b[3]);
  delay(20);
}

void eeprom_i2c_write(byte address, byte from_addr, byte data)
{
  i2c.startWrite(address);
  i2c.write(from_addr);
  i2c.write(data);
  i2c.endWrite();
}

byte eeprom_i2c_read(int address, int from_addr)
{
  i2c.startWrite(address);
  i2c.write(from_addr);
  i2c.endWrite();

  i2c.startRead(address, 1);
  //  if (i2c.available() > 0) {
  return i2c.read();
  //  } else {
  //    return 0xFF;
  //  }
}

void receiveEvent (int howMany)
{
  command = Wire.read ();  // remember command for when we get request
  for (int i = 1; i <= howMany; i++) {
    cmdarg[i] = Wire.read ();
  }
} // end of receiveEvent

void requestEvent ()
{
  //  Serial.println(command);
  switch (command) {
    case 'a':
      Wire.write('A');
      //        Serial.println("Abort Signal Received");
      chargeAbort = 1; chargeOn = 0; chargeOff = 0;
      break;
    case 'A':
      if (charging == 1) {
        result = (float)g_EMON.apparentPower * 100;
        highlow[0] = highByte(result);
        highlow[1] = lowByte(result);
      } else {
        result = 0;
        highlow[0] = 0;
        highlow[1] = 0;
      }
      Wire.write(highlow, 2);
      break;
    case 'b':
      Wire.write('B');
      //        Serial.println("Reset Signal Received");
      //      resetMe();
      backlight();
      lcd.display();
      lcd.clear();
      lcd.print(" EMON Rebooting ");
      delay(1000);
      wdt_enable(WDTO_15MS);
      for (;;)
      {
      }
      break;
    case 'c':
      Wire.write('C');
      //        Serial.println("Charging Signal Received");
      chargeOn = 1; chargeOff = 0;
      break;
    case 'C':
      if (charging == 1) {
        result = (float)g_EMON.Irms * 100;
        highlow[0] = highByte(result);
        highlow[1] = lowByte(result);
      } else {
        result = 0;
        highlow[0] = 0;
        highlow[1] = 0;
      }
      Wire.write(highlow, 2);
      break;
    case 'G':
      if (charging == 1) {
        result = (float)g_EMON.Irms * 100;
        highlow[0] = highByte(result);
        highlow[1] = lowByte(result);
        result = (float)volts * 100;
        //        result = (float)g_EMON.Vrms * 100;
        highlow[2] = highByte(result);
        highlow[3] = lowByte(result);
      } else {
        result = 0;
        highlow[0] = 0;
        highlow[1] = 0;
        highlow[2] = 0;
        highlow[3] = 0;
      }
      Wire.write(highlow, 4);
      break;
    case 'i':
      //        Serial.println("Sending EEPROM Status");
      if (eeprom_i2c_read(B01010000, 15) == 0x55) {
        Wire.write('a');
      } else {
        Wire.write('n');
      }
      break;
    case 'k':
      if (charging == 1) {
        result = (float)ikwh * 100;
        highlow[0] = highByte(result);
        highlow[1] = lowByte(result);
      } else {
        result = 0;
        highlow[0] = 0;
        highlow[1] = 0;
      }
      Wire.write(highlow, 2);
      break;
    case 'K':
      if (charging == 1) {
        result = (float)totalkwh + (float)ikwh;
        highlow[0] = highByte(result);
        highlow[1] = lowByte(result);
      } else {
        result = (float)eepromReadFloat(EE_TOTAL_KWH);
        highlow[0] = highByte(result);
        highlow[1] = lowByte(result);
      }
      Wire.write(highlow, 2);
      break;
    case 'n':
      Wire.write('N');
      //        Serial.println("Charging Off Signal Received");
      chargeOff = 1; chargeOn = 0;
      break;
    case 'o':
      if (charging == 1) {
        result = (float)cost * 100;
        highlow[0] = highByte(result);
        highlow[1] = lowByte(result);
      } else {
        result = 0;
        highlow[0] = 0;
        highlow[1] = 0;
      }
      Wire.write(highlow, 2);
      break;
    case 'O':
      if (charging == 1) {
        result = (float)totalcost + (float)cost;
        highlow[0] = highByte(result);
        highlow[1] = lowByte(result);
      } else {
        result = (float)eepromReadFloat(EE_TOTAL_COST);
        highlow[0] = highByte(result);
        highlow[1] = lowByte(result);
      }
      Wire.write(highlow, 2);
      break;
    case 'p':
      if (charging == 1) {
        result = (float)g_EMON.powerFactor * 100;
        highlow[0] = highByte(result);
        highlow[1] = lowByte(result);
      } else {
        result = 0;
        highlow[0] = 0;
        highlow[1] = 0;
      }
      Wire.write(highlow, 2);
      break;
    case 'P':
      if (charging == 1) {
        result = (float)g_EMON.realPower * 100;
        highlow[0] = highByte(result);
        highlow[1] = lowByte(result);
      } else {
        result = 0;
        highlow[0] = 0;
        highlow[1] = 0;
      }
      Wire.write(highlow, 2);
      break;
    case 'r':
      EEPROM.update(0x00, cmdarg[1]);
      EEPROM.update(0x01, cmdarg[2]);
      break;
    case 'R':
      highlow[0] = EEPROM.read(0x00);
      highlow[1] = EEPROM.read(0x01);
      Wire.write(highlow, 2);
      break;
    case 's':
      //        Serial.println("Sending Live Data");
      storeData_Live();
      for (int c = 0; c < (32 - 1); c++)
      {
        registerMap[c] = registerMapTemp[c];
      }
      Wire.write(registerMap, 32);
      break;
    case 'S':
      /*
        if (charging == 1) {
             amps = (float)g_EMON.Irms * 100;
              volts = (float)g_EMON.Vrms * 100;
              pf = (float)g_EMON.powerFactor * 100;
              ap = (float)g_EMON.apparentPower * 100;
              rp = (float)g_EMON.realPower * 100;
              highlow[0] = highByte(result);
              highlow[1] = lowByte(result);
              Wire.write(highlow, 2);
        } else {
        result = 0;
        highlow[0] = 0;
        highlow[1] = 0;
        Wire.write(highlow, 2);
        }*/
      break;
    case 't':
      if (charging == 1) {
        highlow[0] = m_ElapsedChargeTime >> 24;
        highlow[1] = m_ElapsedChargeTime >> 16;
        highlow[2] = m_ElapsedChargeTime >> 8;
        highlow[3] = m_ElapsedChargeTime;
      } else {
        result = 0;
        highlow[0] = 0;
        highlow[1] = 0;
        highlow[2] = 0;
        highlow[3] = 0;
      }
      Wire.write(highlow, 4);
      break;
    case 'T':
      if (charging == 1) {
        highlow[0] = totaltime >> 24;
        highlow[1] = totaltime >> 16;
        highlow[2] = totaltime >> 8;
        highlow[3] = totaltime;
      } else {
        uint32_t longresult;
        longresult = eepromReadLong(EE_TOTAL_TIME);
        highlow[0] = longresult >> 24;
        highlow[1] = longresult >> 16;
        highlow[2] = longresult >> 8;
        highlow[3] = longresult;
      }
      Wire.write(highlow, 4);
      break;
    case 'v':
      //        Serial.println("Sending Saved Data");
      storeData_Saved();
      for (int c = 0; c < (32 - 1); c++)
      {
        registerMap[c] = registerMapTemp[c];
      }
      Wire.write(registerMap, 32);
      break;
    case 'V':
      if (charging == 1) {
        result = volts * 100;
        //        result = (float)g_EMON.Vrms * 100;
        highlow[0] = highByte(result);
        highlow[1] = lowByte(result);
      } else {
        result = 0;
        highlow[0] = 0;
        highlow[1] = 0;
      }
      Wire.write(highlow, 2);
      break;
    case 'z':
      Wire.write('z');
      clearStats = 1;
      break;
  }
}


// Store Live Stats for Sending to EVSE
void storeData_Live()
{
  byte * bytePointer;  //we declare a pointer as type byte
  byte arrayIndex = 0; //we need to keep track of where we are storing data in the array

  totalcost = eepromReadFloat(EE_TOTAL_COST);
  //  Serial.println(totalcost);
  totaltime = eepromReadLong(EE_TOTAL_TIME);
  //  Serial.println(totaltime);
  totalkwh = eepromReadFloat(EE_TOTAL_KWH);
  //  Serial.println(totalkwh);

  //  amps = g_EMON.calcIrms(1480);
  bytePointer = (byte*)&amps;////
  ////  bytePointer = (byte*)&g_EMON.Irms;

  //  for (int i = 3; i > -1; i--)
  for (int i = 0; i < 4; i++)
  {
    registerMapTemp[arrayIndex] = bytePointer[i];  //increment pointer to store each byte
    arrayIndex++;
  }

  bytePointer = (byte*)&kw;
  //  for (int i = 3; i > -1; i--)
  for (int i = 0; i < 4; i++)
  {
    registerMapTemp[arrayIndex] = bytePointer[i];  //increment pointer to store each byte
    arrayIndex++;
  }

  bytePointer = (byte*)&cost;
  //  for (int i = 3; i > -1; i--)
  for (int i = 0; i < 4; i++)
  {
    registerMapTemp[arrayIndex] = bytePointer[i];  //increment pointer to store each byte
    arrayIndex++;
  }

  bytePointer = (byte*)&ikwh;
  //  for (int i = 3; i > -1; i--)
  for (int i = 0; i < 4; i++)
  {
    registerMapTemp[arrayIndex] = bytePointer[i];  //increment pointer to store each byte
    arrayIndex++;
  }

  tempfloat = cost + totalcost;
  bytePointer = (byte*)&tempfloat;
  //  for (int i = 3; i > -1; i--)
  for (int i = 0; i < 4; i++)
  {
    registerMapTemp[arrayIndex] = bytePointer[i];  //increment pointer to store each byte
    arrayIndex++;
  }

  tempfloat = ikwh + totalkwh;
  bytePointer = (byte*)&tempfloat;
  //  for (int i = 3; i > -1; i--)
  for (int i = 0; i < 4; i++)
  {
    registerMapTemp[arrayIndex] = bytePointer[i];  //increment pointer to store each byte
    arrayIndex++;
  }

  templong = totaltime;
  bytePointer = (byte*)&templong;
  //  for (int i = 3; i > -1; i--)
  for (int i = 0; i < 4; i++)
  {
    registerMapTemp[arrayIndex] = bytePointer[i];  //increment pointer to store each byte
    arrayIndex++;
  }
}

// Store Saved Stats for Sending to EVSE
void storeData_Saved()
{
  byte * bytePointer;  //we declare a pointer as type byte
  byte arrayIndex = 0; //we need to keep track of where we are storing data in the array


  totalcost = eepromReadFloat(EE_TOTAL_COST);
  //  Serial.println(totalcost);
  totaltime = eepromReadLong(EE_TOTAL_TIME);
  //  Serial.println(totaltime);
  totalkwh = eepromReadFloat(EE_TOTAL_KWH);
  //  Serial.println(totalkwh);

  tempfloat = cost + totalcost;
  bytePointer = (byte*)&tempfloat;
  //  for (int i = 3; i > -1; i--)
  for (int i = 0; i < 4; i++)
  {
    registerMapTemp[arrayIndex] = bytePointer[i];  //increment pointer to store each byte
    arrayIndex++;
  }

  tempfloat = ikwh + totalkwh;
  bytePointer = (byte*)&tempfloat;
  //  for (int i = 3; i > -1; i--)
  for (int i = 0; i < 4; i++)
  {
    registerMapTemp[arrayIndex] = bytePointer[i];  //increment pointer to store each byte
    arrayIndex++;
  }

  templong = totaltime;
  bytePointer = (byte*)&templong;
  //  for (int i = 3; i > -1; i--)
  for (int i = 0; i < 4; i++)
  {
    registerMapTemp[arrayIndex] = bytePointer[i];  //increment pointer to store each byte
    arrayIndex++;
  }
}

void backlight()
{
  analogWrite(BLPin, 255);
}

void noBacklight()
{
  analogWrite(BLPin, 0);
}

