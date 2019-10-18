//Libraries
#include <LiquidCrystal_I2C.h>  //library for I2C LCD Module (20 char 4 lines)
#include <AltSoftSerial.h>      // for the connection to the FT-818ND
#include <RotaryEncoder.h>      // for the XIT control rotary encoder
#include <EEPROM.h>             // to store and retrieve last XIT

// Setup the I2C LCD Display
LiquidCrystal_I2C lcd(0x27, 20, 4);

// SoftwareSerial radioSerial(2, 3); // RX, TX
AltSoftSerial radioSerial;

// Setup a RoraryEncoder for pins A2 and A3:
RotaryEncoder encoder(A3, A2);

const int CYCLE_TIME = 20;

const int ADDRESS = 1;

const int DISPLAY_TIME  = 2000 / CYCLE_TIME;
const int SDR_POLL_TIME = 1000 / CYCLE_TIME;

const int SDR_FAIL_COUNT = 3;

long XIT ; // Xmitter Incremental Tuning

bool encoderTouched = false;

bool sdrConnected = false;
bool radioConnected = true;

unsigned long rxFreqHz = 0UL;
unsigned long txFreqHz = 0UL;
unsigned long ifFreqHz = 0UL;

char sdrBuffer[50];
int sdrIndex = 0;
int sdrCount = 0;
int sdrFails = 0;

byte radioBuffer[5];
int radioIndex = 0;
int radioCount = 0;

int displayCount = 0;

void setup()
{
  Serial.begin(57600, SERIAL_8N1); // open the serial port that will be connected with SDR Console at 57600, 8 bits, no parity, 1 stop bit

  radioSerial.begin(38400);// set the data rate for the SoftSerial port

  // For the rotary encoder that works on interrupts
  PCICR |= (1 << PCIE1);    // This enables Pin Change Interrupt 1 that covers the Analog input pins or Port C.
  PCMSK1 |= (1 << PCINT10) | (1 << PCINT11);  // This enables the interrupt for pin 2 and 3 of Port C.
  pinMode(A1, INPUT);    // For the Rotary encoder switch

  // Read last used XIT from EEPROM.
  EEPROMRead();
  encoder.setPosition(XIT / 10);

  // initialize the LCD Screen
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0); lcd.print("SDR Console - FT-817");
  lcd.setCursor(0, 1); lcd.print(" Es hail 2 / QO-100");
  lcd.setCursor(0, 2); lcd.print("Controller by G4KLX");
  lcd.setCursor(0, 3); lcd.print("      Ver 1.00");
  displayCount = 0;

  delay(1000);

  showConnections();
}

// The Interrupt Service Routine for Pin Change Interrupt 1
// This routine will only be called on any signal change on A2 and A3
ISR(PCINT1_vect)
{
  if (sdrConnected && radioConnected) {
    encoder.tick(); // just call tick() to check the state.
    int newPos = encoder.getPosition();
    XIT = newPos * 20;
    encoderTouched = true;
  }
}

void loop()
{
  if (Serial.available()) {
    int c = Serial.read();
    sdrBuffer[sdrIndex++] = c;

    if (c == ';') {
      if (!sdrConnected) {
        processID();
      } else {
        processFreq();
        if (calculateFreq()) {
          showFrequency();
		  setRadioFrequency();
        }
      }

      sdrIndex = 0;
    }
  }

  if (radioSerial.available()) {
    int c = radioSerial.read();
	radioBuffer[radioIndex++] = c;

    if (radioIndex == 5) {
      if (!radioConnected)
        processRadio();
    }

    radioIndex = 0;
  }

  sdrCount++;
  if (sdrCount >= SDR_POLL_TIME) {
    if (!sdrConnected) {
      Serial.write("ID;");
    } else if (radioConnected) {
      if (sdrFails > SDR_FAIL_COUNT) {
        sdrConnected = false;
		showConnections();
      } else {
        Serial.write("FA;");
        sdrFails++;
      }
    }

    sdrCount = 0;
  }
  
  radioCount++;
  if (radioCount >= RADIO_POLL_TIME) {
    if (!radioConnected)
      readRXStatus();

    radioCount = 0;
  }

  if (encoderTouched) {
    if (calculateFreq()) {
      showFrequency();
      setRadioFrequency();
	}
    encoderTouched = false;
  }

  if (displayCount > 0) {
    displayCount--;
	if (displayCount == 0)
      lcd.noBacklight();
  }

  if (sdrConnected && radioConnected) {
    if (digitalRead(A1) == LOW) {
      saveXIT();
	  showFrequency();
	}
  }

  delay(CYCLE_TIME);
}

void processID()
{
  if (sdrIndex == 6 && memcmp(sdrBuffer, "ID019;", 6) == 0) {
    sdrConnected = true;
    sdrFails = 0;
    showConnections();
  }
}

void processFreq()
{
  // if (sdrIndex == 14 && memcmp(sdrBuffer, "FA10489", 7) == 0) {
    // The RX frequency in Hertz minus the 10 GHz part.
    rxFreqHz = (sdrBuffer[8]  - '0') * 100000UL    + (sdrBuffer[9]  - '0') * 10000UL    + (sdrBuffer[10] - '0') * 1000UL +
               (sdrBuffer[11] - '0') * 100UL       + (sdrBuffer[12] - '0') * 10UL       + (sdrBuffer[13] - '0');

    if (sdrFails > 0)
      sdrFails--;
}

bool calculateFreq()
{
  txFreqHz = rxFreqHz + 2400000000UL - 500000UL;  

  unsigned long new IFFreqHz = rxFreqHz + XIT + 432000000UL - 500000UL;

  if (newIFFreqHz != ifFreqHz) {
    ifFreqHz = newIFFreqHz;
    return true;
  } else {
    return false;
  }
}

void showConnections()
{
  if (sdrConnected && radioConnected)
    return;

  lcd.clear();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("Waiting for:");

  int n = 1;

  if (!sdrConnected) {
    lcd.setCursor(0, n);
    lcd.print("  SDR Console");
    n++;
  }

  if (!radioConnected) {
    lcd.setCursor(0, n);
    lcd.print("  FT-817");
  }

  displayCount = DISPLAY_TIME;
}

void showFrequency()
{
  if (!sdrConnected || !radioConnected)
    return;

  char buf[20];

  lcd.clear();
  lcd.backlight();

  sprintf(buf, "RX:  10489lu Hz", rxFreqHz);

  lcd.setCursor(0, 0);
  lcd.print(buf);

  sprintf(buf, "TX:     %lu Hz", txFreqHz);

  lcd.setCursor(0, 1);
  lcd.print(buf);

  sprintf(buf, "IF:     %lu Hz", ifFreqHz);

  lcd.setCursor(0, 2);
  lcd.print(buf);

  if (XIT < 0L)
    sprintf(buf, "-%ld", -XIT);
  else
    sprintf(buf, "+%ld", XIT);

  lcd.setCursor(0, 3);
  lcd.print("XIT:   ");
  for (int i = 0; i < (9 - strlen(buf)); i++)
    lcd.write(' ');
  lcd.print(buf);
  lcd.print("  Hz");

  displayCount = DISPLAY_TIME;
}

void EEPROMWrite()
{
  byte four  = ((XIT >> 0)  & 0xFF);
  byte three = ((XIT >> 8)  & 0xFF);
  byte two   = ((XIT >> 16) & 0xFF);
  byte one   = ((XIT >> 24) & 0xFF);

  EEPROM.write(ADDRESS + 0, four);
  EEPROM.write(ADDRESS + 1, three);
  EEPROM.write(ADDRESS + 2, two);
  EEPROM.write(ADDRESS + 3, one);
}

void EEPROMRead()
{
  long four  = EEPROM.read(ADDRESS + 0);
  long three = EEPROM.read(ADDRESS + 1);
  long two   = EEPROM.read(ADDRESS + 2);
  long one   = EEPROM.read(ADDRESS + 3);

  XIT = ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

void saveXIT()
{
  EEPROMWrite();

  for (int i = 0; i <= 3; i++) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print ("####################");
    lcd.setCursor(0, 1); lcd.print ("#   XIT saved to   #");
    lcd.setCursor(0, 2); lcd.print ("#      EEPROM      #");
    lcd.setCursor(0, 3); lcd.print ("####################");
    delay(100);
  }
}

void unlockRadio()
{
  radioSerial.write(0x00); // DATA1 = null
  radioSerial.write(0x00); // DATA2 = null
  radioSerial.write(0x00); // DATA3 = null
  radioSerial.write(0x00); // DATA4 = null
  radioSerial.write(0x80); // DATA5 = 80 (LOCK OFF Opcode)
}

void readRXStatus()
{
  radioSerial.write(0x00); // DATA1 = null
  radioSerial.write(0x00); // DATA2 = null
  radioSerial.write(0x00); // DATA3 = null
  radioSerial.write(0x00); // DATA4 = null
  radioSerial.write(0xE7); // DATA5 = E7 (Read RX Status Opcode)
}

void setRadioFrequency()
{
  byte one   = (((ifFreqHZ / 100000000UL) & 0x0FUL) << 4) | (((ifFreqHZ / 10000000UL) & 0x0FUL) << 0);
  byte two   = (((ifFreqHZ / 1000000UL)   & 0x0FUL) << 4) | (((ifFreqHZ / 100000UL)   & 0x0FUL) << 0);
  byte three = (((ifFreqHZ / 10000UL)     & 0x0FUL) << 4) | (((ifFreqHZ / 1000UL)     & 0x0FUL) << 0);
  byte four  = (((ifFreqHZ / 100UL)       & 0x0FUL) << 4) | (((ifFreqHZ / 10UL)       & 0x0FUL) << 0);

  radioSerial.write(one);
  radioSerial.write(two);
  radioSerial.write(three);
  radioSerial.write(four);
  radioSerial.write(0x01); // DATA5 = 01 (Set Frequency Opcode)
}

bool processRadio()
{
  unsigned int checksum = 0U;
  for (int i = 0; i < 5; i++)
    checksum += radioBuffer[i];

  if (checksum > 0U) {
    radioConnected = true;
    return true
  } else {
    return false;
  }
}
