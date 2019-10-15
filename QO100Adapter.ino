//Libraries
#include <LiquidCrystal_I2C.h>  //library for I2C LCD Module (20 char 4 lines)
#include <AltSoftSerial.h>      // for the connection to the FT-818ND
#include <RotaryEncoder.h>      // for the XIT control rotary encoder
#include <EEPROM.h>             // to store and retrieve last XIT

// Setup the I2C LCD Display
LiquidCrystal_I2C lcd(0x27, 20, 4);

// SoftwareSerial Yaesu_Serial(2, 3); // RX, TX
AltSoftSerial Yaesu_Serial;

// Setup a RoraryEncoder for pins A2 and A3:
RotaryEncoder encoder(A3, A2);

long XIT ; // Xmitter Incremental Tuning

bool Encoder_was_touched = false;

bool sdrConnected = false;
bool radioConnected = true;

unsigned long rxFreqKHz = 0UL;
unsigned long txFreqKHz = 0UL;
unsigned long ifFreqKHz = 0UL;

char sdrBuffer[50];
int sdrIndex = 0;
int count = 0;

void setup()
{
  Serial.begin(57600, SERIAL_8N1); // open the serial port that will be connected with SDR Console at 57600, 8 bits, no parity, 1 stop bit

  Yaesu_Serial.begin(38400);// set the data rate for the SoftSerial port

  // For the rotary encoder that works on interrupts
  PCICR |= (1 << PCIE1);    // This enables Pin Change Interrupt 1 that covers the Analog input pins or Port C.
  PCMSK1 |= (1 << PCINT10) | (1 << PCINT11);  // This enables the interrupt for pin 2 and 3 of Port C.
  pinMode(A1, INPUT);    // For the Rotary encoder switch

  //Reading last used XIT from EEPROM.
  // XIT = EEPROM_Read(1); // Gets Xmitter Incremental Tuning stored in EEPROM
  encoder.setPosition(XIT / 10);  // for initiate the encoder value

  // initialize the LCD Screen
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0); lcd.print("SDR Console - FT-817");
  lcd.setCursor(0, 1); lcd.print(" Es hail 2 / QO-100");
  lcd.setCursor(0, 2); lcd.print("Controller by G4KLX");
  lcd.setCursor(0, 3); lcd.print("      Ver 1.00");
  delay(1000);

  showConnections();

  // Check_Connection_With_SDR_Console();// as it says
  // Check_Connection_With_FT818ND();
  // Set_SDR_RX_Frequency_to_Upper_Beacon();// as it says
  // Unlock_FT818ND(); //as it says
  // Set_FT818ND_Operating_Mode_to_USB();//as it says
}

// The Interrupt Service Routine for Pin Change Interrupt 1
// This routine will only be called on any signal change on A2 and A3
ISR(PCINT1_vect)
{
  encoder.tick(); // just call tick() to check the state.
  int newPos = encoder.getPosition();
  XIT = newPos * 20;
  Encoder_was_touched = true;
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
        calculateFreq();
        showFrequency();
      }

      sdrIndex = 0;
    }
  }

  count++;
  if (count >= 50) {
    if (!sdrConnected)
      Serial.write("ID;");
    else
      Serial.write("FA;");

    count = 0;
  }

  if (Encoder_was_touched) {
    calculateFreq();
    showFrequency();
    Encoder_was_touched = false;
  }

  delay(20);
}

void processID()
{
  if (sdrIndex == 6 && memcmp(sdrBuffer, "ID019;", 6) == 0) {
    sdrConnected = true;
    showConnections();
  }
}

void processFreq()
{
  // if (sdrIndex == 14 && memcmp(sdrBuffer, "FA10489", 7) == 0) {
    // The RX frequency in Hertz minus the 10 GHz part.
    rxFreqKHz = (sdrBuffer[8]  - '0') * 100000UL    + (sdrBuffer[9]  - '0') * 10000UL    + (sdrBuffer[10] - '0') * 1000UL +
                (sdrBuffer[11] - '0') * 100UL       + (sdrBuffer[12] - '0') * 10UL       + (sdrBuffer[13] - '0');
}

void calculateFreq()
{
    txFreqKHz = rxFreqKHz + 2400000000UL - 500000UL;  

    ifFreqKHz = rxFreqKHz + XIT + 432000000UL - 500000UL;
}

void showConnections()
{
  if (sdrConnected && radioConnected)
    return;

  lcd.clear();
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
}

void showFrequency()
{
  if (!sdrConnected || !radioConnected)
    return;

  char buf[20];

  lcd.clear();

  sprintf(buf, "%lu", rxFreqKHz);

  lcd.setCursor(0, 0);
  lcd.print("RX: 10489.");
  lcd.write(buf[0]); lcd.write(buf[1]); lcd.write(buf[2]);
  lcd.write(buf[3]); lcd.write(buf[4]); lcd.write(buf[5]);
  lcd.print(" MHz");

  sprintf(buf, "%lu", txFreqKHz);

  lcd.setCursor(0, 1);
  lcd.print("TX:  ");
  lcd.write(buf[0]); lcd.write(buf[1]); lcd.write(buf[2]); lcd.write(buf[3]);
  lcd.print(".");
  lcd.write(buf[4]); lcd.write(buf[5]); lcd.write(buf[6]);
  lcd.write(buf[7]); lcd.write(buf[8]); lcd.write(buf[9]);
  lcd.print(" MHz");

  sprintf(buf, "%lu", ifFreqKHz);

  lcd.setCursor(0, 2);
  lcd.print("IF:   ");
  lcd.write(buf[0]); lcd.write(buf[1]); lcd.write(buf[2]);
  lcd.print(".");
  lcd.write(buf[3]); lcd.write(buf[4]); lcd.write(buf[5]);
  lcd.write(buf[6]); lcd.write(buf[7]); lcd.write(buf[8]);
  lcd.print(" MHz");

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
}
