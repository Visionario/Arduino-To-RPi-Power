/* CongoroCar - Master Power System
    Using a Arduino Pro Mini 5V / 16Mhz
    Version: 1.0.0
    Autor:  Asdrubal Velasquez @Visionario
    email: visionario@gmail.com
    Date: 2016/05/26
    Last Update: 2016/06/09
    Description:
*/

// Define PINS
const byte CAR_KEY_STATE_PIN = 3;               // Key car state (ON/OFF) (INPUT)VIA INTERRUPT
const byte RPI_IS_LIVE = 2;                     // GPIO from Raspberry, (LOW = total OFF, HIGH= Running) (INPUT)
const byte MAIN_POWER_PIN = 4;                  // To Relay or Transistor (as switch) to turn on/off main power (OUTPUT)
const byte DO_RPI_POWEROFF_PIN = 5;             // GPIO on Raspberry to initiate a safe shutdown (OUTPUT)
const byte BUTTON_AUX_PIN = 6;                  // Auxiliar button (INPUT)
const byte LED_STATUS_PIN[2] = {7, 8};          // Status led for Warning (0=RED, 1=GREEN)

//Define Delays
const int PRE_POWERON_DELAY = 20000;            // Wait this time before to put MAIN_POWER_PIN HIGH, after CAR_KEY_STATE_PIN=HIGH (Starts RPI)
const int POST_POWERON_DELAY = 25000;           // After PRE_POWERON_DELAY, RPI must send RPI_IS_LIVE signal to HIGH, if not. RPI has problems to boot
const int POST_POWEROFF_DELAY = 2000;           // Wait for this time before to put MAIN_POWER_PIN LOW (cutoff energy. Raspberry lights still blinking after poweroff
const int POST_POWEROFF_DELAY_FORCE = 15000;    // This time is after POST_POWEROFF_DELAY it is only if RPI is hung and doesn't send RPI_IS_LIVE=OFF, THIS IS A CRITICAL ERROR AND MUST TO INFORM
const int PRE_POWEROFF_DELAY = 20000;           // Wait for this time before sends DO_RPI_POWEROFF_PIN signal to RPI, after CAR_KEY_STATE_PIN=LOW

//Warning Led variables
unsigned long LedStatusTimerON[2] = {1000UL, 1000UL}; // Time for led status is ON
unsigned long LedStatusTimerOFF[2] = {1500UL, 1500UL}; // Time for led status is OFF
boolean LedStatusState[2] = {LOW, LOW};             // Actual led status
unsigned long LedStatusTime2OFF[2] = {0, 0};        // Time left to turn OFF the status led
unsigned long LedStatusTime2ON[2] = {0, 0};         // Time left to turn ON the status led
const byte RED = 0;
const byte GREEN = 1;

//Power Status
enum PowerState {OFF_TOTAL, RPI_RUNNING, PRE_POWER_ON, RPI_WAITING_LIVE, RPI_WAITING_DEAD, PRE_POWER_OFF};
PowerState MainPowerStatus = OFF_TOTAL;

//Car key states
byte CarKeyState = LOW;
volatile boolean CarKeyStateChanged = false;

// millis for states
unsigned long PrePowerOnMillis = 0;
unsigned long PostPowerOnMillis = 0;
unsigned long PrePowerOffMillis = 0;
unsigned long PostPowerOffMillis = 0;
unsigned long PostPowerOffForcedMillis = 0;

unsigned long DEBUG_PREVIOUSMILLIS = 0;



/*
 * ************************ POWERON
*/
//It connects power to main system
void PowerON() {
  digitalWrite(MAIN_POWER_PIN, HIGH); // Connects power to RPI, activates Relay
  setNewPowerStatus(RPI_WAITING_LIVE); //Powering ON and waiting for RPI_IS_LIVE==HIGH
}

/*
 * *********************** POWEROFF
*/
//Cut off all the power
boolean PowerOFF(boolean force) {
  if ((digitalRead(RPI_IS_LIVE) == LOW) || (force)) { //waits for RPI send signal off or Force cut power
    digitalWrite(MAIN_POWER_PIN, LOW);
    digitalWrite(DO_RPI_POWEROFF_PIN, LOW); // Sends Poweroff signal to RPI
    setNewPowerStatus(OFF_TOTAL);
    return true;
  }
  return false;
}


/*
 * ******************* INTERRUPT CARKEYCHANGE
*/
// Interrupts triggered when Car Key state change
// It is very importat to use hardware debounce
void CarKeyChange() {
  CarKeyStateChanged = true;
}


/*
 * ***********************RESETMILLIS()
*/
void ResetMillis() {
  unsigned long CurrentMillis  = millis();
  PrePowerOnMillis = CurrentMillis;
  PostPowerOnMillis = CurrentMillis;
  PrePowerOffMillis = CurrentMillis;
  PostPowerOffMillis = CurrentMillis;
  PostPowerOffForcedMillis = CurrentMillis;
}



/*
 * ************************ STATUS LED
*/
// Set new blinking status (led)
void LedStatusTimer(byte led, unsigned long ON , unsigned long OFF ) {
  LedStatusTimerON[led] = ON;
  LedStatusTimerOFF[led] = OFF;
}

//Change led state from checkStatusLed()
void toggleLed(byte led) {
  digitalWrite(LED_STATUS_PIN[led], !LedStatusState[led]);
  LedStatusState[led] = !LedStatusState[led];
}

void checkStatusLed(byte led, unsigned long currentmillis) {
  if (LedStatusState[led]) {
    if ((currentmillis - LedStatusTime2OFF[led]) > LedStatusTimerON[led]) {
      LedStatusTime2ON[led] = currentmillis;
      toggleLed(led);
    }
  } else { //LedStatusState (LOW HERE)
    if ((currentmillis - LedStatusTime2ON[led]) > LedStatusTimerOFF[led]) {
      LedStatusTime2OFF[led] = currentmillis;
      toggleLed(led);
    }
  }

}


boolean setNewPowerStatus(PowerState state) {
  switch (state) {
    case OFF_TOTAL:
      LedStatusTimer(RED, 30, 1970);
      LedStatusTimer(GREEN, 5, 100000);
      break;
    case RPI_RUNNING:
      LedStatusTimer(RED, 5, 100000);
      LedStatusTimer(GREEN, 2000, 1000);
      break;
    case PRE_POWER_ON:
      LedStatusTimer(RED, 30, 500);
      LedStatusTimer(GREEN, 5, 100000);
      break;
    case RPI_WAITING_LIVE:
      LedStatusTimer(RED, 30, 100);
      LedStatusTimer(GREEN, 5, 100000);
      break;
    case RPI_WAITING_DEAD:
      LedStatusTimer(RED, 50, 50);
      LedStatusTimer(GREEN, 5, 100000);
      break;
    case PRE_POWER_OFF:
      LedStatusTimer(RED, 250, 250);
      LedStatusTimer(GREEN, 50, 50);
      break;
  }
  MainPowerStatus = state;
  ResetMillis();
}


/*
 * ************************ SETUP
*/
void setup() {
  pinMode(CAR_KEY_STATE_PIN, INPUT);
  pinMode(RPI_IS_LIVE, INPUT);
  pinMode(MAIN_POWER_PIN, OUTPUT);
  pinMode(DO_RPI_POWEROFF_PIN, OUTPUT);
  pinMode(BUTTON_AUX_PIN, INPUT);

  pinMode(LED_STATUS_PIN[0], OUTPUT);
  pinMode(LED_STATUS_PIN[1], OUTPUT);


  //Check actual state (this system is starting and dont know actual state)
  CarKeyState = digitalRead(CAR_KEY_STATE_PIN);
  if (CarKeyState == HIGH) {
    setNewPowerStatus(RPI_WAITING_LIVE); //Powering ON and waiting for RPI_IS_LIVE==HIGH
    digitalWrite(MAIN_POWER_PIN, HIGH); // Connects power to RPI, activates Relay
  } else {
    digitalWrite(MAIN_POWER_PIN, LOW); // cut off power to RPI
    setNewPowerStatus(OFF_TOTAL); // Waits for RPI_IS_LIVE=LOW
  }

  ResetMillis();

  attachInterrupt(digitalPinToInterrupt(CAR_KEY_STATE_PIN), CarKeyChange, CHANGE);
//  Serial.begin(9600);

}

/*
 * ************************* LOOP
*/
void loop() {
  unsigned long CurrentMillis  = millis();

  if ((CurrentMillis - DEBUG_PREVIOUSMILLIS) >= 1000) {
/*
    Serial.print("CurrentMillis ");
    Serial.println(CurrentMillis);

    Serial.print("MainPowerStatus ");
    switch (MainPowerStatus) {
      case OFF_TOTAL:
        Serial.println("OFF_TOTAL");
        break;
      case RPI_RUNNING:
        Serial.println("RPI_RUNNING");
        break;
      case PRE_POWER_ON:
        Serial.println("PRE_POWER_ON");
        break;
      case RPI_WAITING_LIVE:
        Serial.println("RPI_WAITING_LIVE");
        break;
      case RPI_WAITING_DEAD:
        Serial.println("RPI_WAITING_DEAD");
        break;
      case PRE_POWER_OFF:
        Serial.println("PRE_POWER_OFF");
        break;
    }
*/
    DEBUG_PREVIOUSMILLIS = CurrentMillis;
  }


  // CarKeyState has changed? (only changed by interrupt function)
  if (CarKeyStateChanged) {
    noInterrupts();

    CarKeyState = digitalRead(CAR_KEY_STATE_PIN);

    if (CarKeyState == HIGH) {
      switch (MainPowerStatus) {
        case OFF_TOTAL:
          setNewPowerStatus(PRE_POWER_ON);
          break;
        case PRE_POWER_OFF:
          setNewPowerStatus(RPI_RUNNING);
          break;

      }

    } else {
      switch (MainPowerStatus) {
        case RPI_RUNNING:
          setNewPowerStatus(PRE_POWER_OFF);
          break;
        case PRE_POWER_ON:
          setNewPowerStatus(OFF_TOTAL);
          break;
      }
    }
    CarKeyStateChanged = false;
  }



  //Test every Main Power Status
  switch (MainPowerStatus) {
    case OFF_TOTAL: //Power is OFF, do nothing
      // DO NOTHING
      break;

    case PRE_POWER_ON: //Triggered by interrupt by CarKeyChange() because car key was ON, waits for a safe boot, may be you want to switch off again
      if ((CurrentMillis - PrePowerOnMillis) >= PRE_POWERON_DELAY) {
        PowerON();
        PrePowerOnMillis = CurrentMillis;
      }
      break;

    case RPI_WAITING_LIVE: //Powering ON??? so waits for RPI_IS_LIVE==HIGH
      if (digitalRead(RPI_IS_LIVE) == HIGH) { // RPI is running perfect
        setNewPowerStatus(RPI_RUNNING); // Status = ON
      } else {
        if ((CurrentMillis - PostPowerOnMillis) >= POST_POWERON_DELAY) {
          // If RPI doesnt send RPI_IS_LIVE=HIGH then may be is hung or there is a problem with it
          // WHAT MUST I DO HERE?
          PostPowerOnMillis = CurrentMillis;
        }
      }
      break;

    case RPI_RUNNING: //System is ON, RPI is Running
      //Do nothing
      break;

    case PRE_POWER_OFF:  // Triggered by CarKeyChange() when switch off the car, prepare and waits for safe shutdown
      if ((CurrentMillis - PrePowerOffMillis) >= PRE_POWEROFF_DELAY) {
        digitalWrite(DO_RPI_POWEROFF_PIN, HIGH); // Sends Poweroff signal to RPI
        setNewPowerStatus(RPI_WAITING_DEAD); // Waits for RPI_IS_LIVE=LOW
      }
      break;

    case RPI_WAITING_DEAD:  // Poweroff signal was send to RPI, so, waits for RPI_IS_LIVE=LOW
      if ((CurrentMillis - PostPowerOffMillis) >= POST_POWEROFF_DELAY) {
        if (PowerOFF(false)) { //PwerOFF was normal (PI_IS_LIVE=LOW) was received
          PostPowerOffMillis = CurrentMillis;
        } else { //RPI did not sent PI_IS_LIVE=LOW, proceed with force power off
        }
      }
      if ((CurrentMillis - PostPowerOffForcedMillis) >= (POST_POWEROFF_DELAY + POST_POWEROFF_DELAY_FORCE)) { // Check is there is an error (doesnt receives RPI_IS_LIVE=LOW)
        PowerOFF(true);
        PostPowerOffForcedMillis = CurrentMillis;
      }
      break;
  }

  checkStatusLed(0, CurrentMillis);
  checkStatusLed(1, CurrentMillis);
  interrupts();
}

