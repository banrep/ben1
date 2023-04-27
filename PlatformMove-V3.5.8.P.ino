
#define VERSION "FastIII Platform Move Version 3.5.8.P 20210128"
#define COPYRIGHT "Copyright Ben Anrep 2020"
/*
   Platform Actuators now used
   3.5.4  PWM added
   3.5.5  add stop on legs
   3.5.6 second try stop on legs
   3.5.7 fixed stop on legs
   3.5.8 add delay relay on for dscope
   3.5.8.P Pins changed to below
                                //#define Error_PIN 9
                                  #define SwitchShelfExtend_PIN 38
                                  #define SwitchShelfRetract_PIN 39
                                  #define LampShelfMoving_PIN 40
*/
char cop[] = {0x43, 0x70, 0x72, 0x7c, 0x76, 0x6e, 0x6d, 0x68, 0x75, 0x22, 0x45, 0x69, 0x73, 0x26, 0x41, 0x6f, 0x74, 0x68, 0x74, 0x25, 0x38, 0x30, 0x33, 0x32, 0};

#include <TB6560.h>

#define LED_BUILTIN 13
#define LOG false
#define LOG2 true
#define SwitchCountLimit 10
#define SwitchShelfMaxCount 200000

// Limits
#define IN 101
#define OUT 102
#define EXTEND 103
#define RETRACT 104

//#define Error_PIN 9
#define SwitchShelfExtend_PIN 38
#define SwitchShelfRetract_PIN 39
#define LampShelfMoving_PIN 40

//Leg H-bridges
#define legMotorRightPin1 5
#define legMotorRightPin2 6
#define legMotorLeftPin1 7
#define legMotorLeftPin2 8
#define legMotorLeftPwmPin 0
#define legMotorRightPwmPin 0
//Shelf H-bridges
#define shelfMotorRightPin1 9
#define shelfMotorRightPin2 10
#define shelfMotorLeftPin1 11
#define shelfMotorLeftPin2 12
#define shelfMotorLeftPwmPin 44
#define shelfMotorRightPwmPin 45

#define dScopeRELAYPin 24 //for dscope
#define dScopeDelayTime 20000 // 20 secs

#define shelfMotorLeftPwmSpeed 255 //change these to make the motors move at different speeds
#define shelfMotorRightPwmSpeed 225

//Limit Switches
#define legLimitLeftIn_PIN A0
#define legLimitLeftOut_PIN A1
#define legLimitRightIn_PIN A2
#define legLimitRightOut_PIN A3
#define ShelfLimitLeftIn_PIN A4
#define ShelfLimitOut_PIN A5
#define ShelfLimitRightIn_PIN A6

//Logging
void logIt(char *str) {
  if (LOG) {
    Serial.print("Info: ");
    Serial.println(str);
  }
}

class Actuator {
  public:
    Actuator(int pin1, int pin2, int pwmPin, int pwm);
    void init(int pin1, int pin2);
    void extend();
    void retract();
    void sstop();
    void pwmSpeed(int pwm);
  private:
    int _pin1, _pin2, _pwmPin, _pwm;
};

Actuator::Actuator(int pin1, int pin2, int pwmPin, int pwm) {
  pinMode(pin1, OUTPUT);
  digitalWrite(pin1, HIGH); // High and High will be stopped so will Low and Low be
  pinMode(pin2, OUTPUT);
  digitalWrite(pin2, HIGH);
  if (pwmPin != 0) {
    pinMode(pwmPin, OUTPUT);
    analogWrite(pwmPin, pwm);
  }
  _pin1 = pin1;
  _pin2 = pin2;
  _pwmPin = pwmPin;
  _pwm = pwm;
}

void Actuator:: pwmSpeed(int pwm) {
  _pwm = pwm;
  if (_pwmPin != 0) {
    analogWrite(_pwmPin, pwm);
  }
}

void Actuator:: extend() {
  logIt("extend");
  digitalWrite(_pin1, LOW);
  digitalWrite(_pin2, HIGH);
}

void Actuator:: retract() {
  logIt("retract");
  digitalWrite(_pin1, HIGH);
  digitalWrite(_pin2, LOW);
}

void Actuator:: sstop() {
  //logIt("stop");
  digitalWrite(_pin1, HIGH);
  digitalWrite(_pin2, HIGH);
}

Actuator shelfLeft(shelfMotorLeftPin1, shelfMotorLeftPin2, shelfMotorLeftPwmPin, shelfMotorLeftPwmSpeed);
Actuator shelfRight(shelfMotorRightPin1, shelfMotorRightPin2, shelfMotorRightPwmPin, shelfMotorRightPwmSpeed);
Actuator legLeft(legMotorLeftPin1, legMotorLeftPin2, 0, 0);
Actuator legRight(legMotorRightPin1, legMotorRightPin2, 0, 0);

class Limit { //limit switch
  public:
    Limit::Limit(int pin1, int pin2); // pin for retract and extend switches
    boolean retract();
    boolean extend();
  private:
    int _pin1, _pin2;
};

Limit::Limit(int pin1, int pin2) {
  pinMode(pin1, INPUT_PULLUP); // use internal pullup switch. also using external and cap
  pinMode(pin2, INPUT_PULLUP);
  _pin1 = pin1;
  _pin2 = pin2;
}

boolean Limit:: retract() {
  return (digitalRead(_pin1) == LOW);
}

boolean Limit:: extend() {
  return (digitalRead(_pin2) == LOW);
}

//Legs Limit
Limit legLeftLimit(legLimitLeftIn_PIN, legLimitLeftOut_PIN);
Limit legRightLimit(legLimitRightIn_PIN, legLimitRightOut_PIN);
//Shelf Limits
Limit shelfLimitLeft(ShelfLimitLeftIn_PIN, ShelfLimitOut_PIN); //only one limit for OUT No left and right
Limit shelfLimitRight(ShelfLimitRightIn_PIN, ShelfLimitOut_PIN);

void initPins() {
  pinMode(SwitchShelfExtend_PIN, INPUT_PULLUP); //toggle switch
  pinMode(SwitchShelfRetract_PIN, INPUT_PULLUP);
  pinMode(LampShelfMoving_PIN, OUTPUT); //bleeper
}

//Buzzer on and off
#define ON true
#define OFF false
void LampShelfMoving2(boolean state) {
  // turn on and off the shelf moving / not ready lamp
  if (state) {
    digitalWrite(LampShelfMoving_PIN, HIGH);
  } else {
    digitalWrite(LampShelfMoving_PIN, LOW);
  }
}

//Buzzer
unsigned long lastToneTime = 0;
boolean movingState = false; // true is moving
boolean toneState = false; // true is buzzer on
void warningShelfMoving(boolean state) {
  //return;
  int t = 500;
  unsigned long currentToneTime = millis();
  movingState = state;
  if (state) {
    if ((currentToneTime - lastToneTime) > t) {
      lastToneTime = currentToneTime;
      currentToneTime = millis();
      if (toneState) {
        digitalWrite(LampShelfMoving_PIN, LOW);    // turn the LED off by making the voltage LOW
        toneState = false; // it was true now make it false true is on
        //logIt("led off");
      }
      else {
        digitalWrite(LampShelfMoving_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
        toneState = true; // it was false now flip it true is on
        //logIt("led on");
      }
    }
  } else {
    digitalWrite(LampShelfMoving_PIN, LOW);
  }
}

//LED Flash
unsigned long lastLEDBlinkTime = 0;
boolean blinkState = true; // true is on
void ledBlink(int t) {
  unsigned long currentLEDBlinkTime = millis();
  if ((currentLEDBlinkTime - lastLEDBlinkTime) > t) {
    lastLEDBlinkTime = currentLEDBlinkTime;
    currentLEDBlinkTime = millis();
    if (blinkState) {
      digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
      blinkState = false; // true is on
      //logIt("led off");
    }
    else {
      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
      blinkState = true; // true is on
      //logIt("led on");
    }
  }
}

//Switch Extend Out
#define SwitchDebounceTime 50 //mS
unsigned long ShelfExtendSwitchDebounceStartTime = 0;
boolean newSwitchShelfExtendState = false;
boolean switchShelfExtendState = false;
boolean switchShelfExtendTimerRunning = false;
boolean legsRetracted = legLeftLimit.retract() && legRightLimit.retract();

boolean getSwitchextExtendShelf() {
  unsigned long currentTime = millis();
  boolean newSwitchShelfExtendState = (digitalRead(SwitchShelfExtend_PIN) == LOW); // if its low then its on
  if (newSwitchShelfExtendState != switchShelfExtendState && !switchShelfExtendTimerRunning) { // if they are different and ttimer not running, this it first time in
    logIt("state change");
    ShelfExtendSwitchDebounceStartTime = currentTime; // start the timer
    switchShelfExtendTimerRunning = true; //say were timing now
  }
  if (switchShelfExtendTimerRunning && (currentTime > (ShelfExtendSwitchDebounceStartTime + SwitchDebounceTime))) { // timer is running, and we have timed out
    logIt("time is out");
    switchShelfExtendState = newSwitchShelfExtendState; // new state of switch
    switchShelfExtendTimerRunning = false; // stop the timer running
    legsRetracted = legLeftLimit.retract() && legRightLimit.retract(); // dont know why!!
  }
  /*
    if (switchShelfExtendState)
      logIt("extend true");
    else
      logIt("extend false");
    /**/
  return (switchShelfExtendState); // either its the original state or its the new state.
}

//Switch Retract In
unsigned long ShelfRetractSwitchDebounceStartTime = 0;
boolean newSwitchShelfRetractState = false;
boolean switchShelfRetractState = false;
boolean switchShelfRetractTimerRunning = false;

boolean getSwitchRetractShelf() {
  unsigned long currentTime = millis();
  boolean newSwitchShelfRetractState = (digitalRead(SwitchShelfRetract_PIN) == LOW);
  if (newSwitchShelfRetractState != switchShelfRetractState && !switchShelfRetractTimerRunning) {
    logIt("state change");
    ShelfRetractSwitchDebounceStartTime = currentTime;
    switchShelfRetractTimerRunning = true;
  }
  if (switchShelfRetractTimerRunning && (currentTime > (ShelfRetractSwitchDebounceStartTime + SwitchDebounceTime))) {
    logIt("time is out");
    switchShelfRetractState = newSwitchShelfRetractState;
    switchShelfRetractTimerRunning = false;
    legsRetracted = legLeftLimit.retract() && legRightLimit.retract();
  }
  /*
    if (switchShelfRetractState)
      logIt("retract true");
    else
      logIt("retract false");
    /**/
  return (switchShelfRetractState);
}

// Move the Platform in Both sides
void movePlatformIn() {
  boolean limtLeft = shelfLimitLeft.retract(); // get limit switch

  if (limtLeft) {
    shelfLeft.sstop(); //we have hit the limit so stop
  } else
  {
    shelfLeft.retract(); // start it up or just keep going
  }
  boolean limtRight = shelfLimitRight.retract();  // get limit switch


  if (limtRight) {
    shelfRight.sstop(); //we have hit the limit so stop
  } else
  {
    shelfRight.retract();  // get limit switch

  }
}

// Move the Platform out Both sides
void movePlatformOut() {
  boolean limt = shelfLimitLeft.extend(); // only one on limit extend both left and right will gibe the same thing
  if (limt) {
    shelfLeft.sstop(); //we have hit the limit so stop both
    shelfRight.sstop();
  } else
  {
    shelfLeft.extend(); // start it up or just keep going both
    shelfRight.extend();
  }
}

boolean legsLeftStopped, legsRightStopped;
// Move both legs
void retractLegs() {
  logIt("retract legs");

  if (legRightLimit.retract() || legsRightStopped) { //check the limit and if the whole move has ended
    legRight.sstop(); //we have hit the limit so stop
  } else {
    legRight.retract(); // start it up or just keep going
  }

  if (legLeftLimit.retract() || legsLeftStopped) { //check the limit and if the whole move has ended
    legLeft.sstop(); //we have hit the limit so stop
  } else {
    legLeft.retract(); // start it up or just keep going
  }
  //return (getLimitLeftLeg(IN) && getLimitRightLeg(IN));
}

void extendLegs() {
  logIt("extend legs");

  if (legRightLimit.extend() || legsRightStopped) { //check the limit and if the whole move has ended
    legRight.sstop();  //we have hit the limit so stop
  } else {
    legRight.extend(); // start it up or just keep going
  }

  if (legLeftLimit.extend() || legsLeftStopped) { //check the limit and if the whole move has ended
    legLeft.sstop();  //we have hit the limit so stop
  } else {
    legLeft.extend(); // start it up or just keep going
  }
  //return (getLimitLeftLeg(OUT) && getLimitRightLeg(OUT));
}

void copy() {
  for (unsigned char i = 0; i < strlen(cop); i++) {
    cop[i] = cop[i] - i % 7;
  }
}

void ledError(int n) {
  while (true) {
    for (int i; i < n; i++) {
      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(300);                       // wait for a second
      digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
      delay(300);   // wait for a second
    }
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);   // wait for a second
  }
}

/*
  pause when going off/in to out and out/off to in
  stop legs when shelf in
*/
boolean extendShelf, retractShelf;
void run1() {
  boolean extendShelfNew, retractShelfNew;
  legsRetracted = legLeftLimit.retract() && legRightLimit.retract();
  legsLeftStopped = true; // Stop legs moving
  legsRightStopped = true;
  Serial.println("Starting Loop");
  while (true) {
    warningShelfMoving(movingState);
    logSwitches();
    ledBlink(500);

    // Start to extend the shelf, we need to Raise legs, push out shelf, lower legs, then stop.
    extendShelfNew = getSwitchextExtendShelf();
    retractShelfNew = getSwitchRetractShelf();
    // check if changing state going off/in to out and out/off to in and pause and allow legs to move
    //if (((!extendShelf && !retractShelf) && (extendShelfNew || retractShelfNew)) ||
    //   ((extendShelf && retractShelfNew) || (retractShelf && extendShelfNew))) {

    if ((extendShelf != extendShelfNew || retractShelf != retractShelfNew) && (retractShelfNew || extendShelfNew)) { // if the state has changed and one or other is on, dont do it when they go off
      legsLeftStopped = false; // Allow legs moving
      legsRightStopped = false;
      logIt("sw change");
      delay(1000); // wait before we start
    }

    extendShelf = extendShelfNew; //make the new state the current one
    retractShelf = retractShelfNew;

    if (extendShelf && (shelfLimitLeft.extend() && legLeftLimit.extend() && legRightLimit.extend())) { //is the switch extend the shelf is out and legs out too
      // Got to end so stop everything
      legLeft.sstop();
      legRight.sstop();
      shelfLeft.sstop();
      shelfRight.sstop();
      legsLeftStopped = true; // Stop legs moving in the furure unless switch is changed
      legsRightStopped = true;
      if (!retractShelf && !extendShelf) //????
        warningShelfMoving(OFF);
      logIt("Done extending");
      //extendShelf = false;

    } else if (extendShelf && shelfLimitLeft.extend()) {
      // shelf is out so we can put the legs down
      shelfLeft.sstop();
      shelfRight.sstop();
      warningShelfMoving(ON);
      extendLegs();

    } else if (extendShelf && legsRetracted) {
      // legs are up so we want to push out the shelf
      legLeft.sstop();
      legRight.sstop();
      warningShelfMoving(ON);
      movePlatformOut();

    } else if (extendShelf) {
      // Make sure the legs are up before moving anything out
      warningShelfMoving(ON);
      logIt("extending");
      retractLegs();
      legsRetracted = legLeftLimit.retract() && legRightLimit.retract();
    }

    // Start to retract the shelf, we need to raise legs, pull in shelf.
    if (retractShelf && (shelfLimitLeft.retract() && shelfLimitRight.retract())) {
      //Got to end so stop everything
      legLeft.sstop();
      legRight.sstop();
      shelfLeft.sstop();
      shelfRight.sstop();
      legsLeftStopped = true; // Stop legs moving
      legsRightStopped = true;
      if (!retractShelf && !extendShelf) {
        warningShelfMoving(OFF);
        legsRetracted = legLeftLimit.retract() && legRightLimit.retract();
      }
      logIt("Done retracting");
      //retractShelf = false;

    } else if (retractShelf && legsRetracted) {
      // legs are up so move the platform in
      legLeft.sstop();
      legRight.sstop();
      warningShelfMoving(ON);
      movePlatformIn();

    } else if (retractShelf) {
      // lift up the legs
      warningShelfMoving(ON);
      logIt("retracting legs");
      legsRetracted = legLeftLimit.retract() && legRightLimit.retract();
      retractLegs();
    }
    if (extendShelf || retractShelf) {
      warningShelfMoving(ON);

    } else {
      if (!retractShelf && !extendShelf)
        warningShelfMoving(OFF);
      // stop everything but dont change the lamp
      legLeft.sstop();
      legRight.sstop();
      shelfLeft.sstop();
      shelfRight.sstop();
      legsLeftStopped = true; // Stop legs moving
      legsRightStopped = true;
    }
  }
}

#define ListLen 11
boolean switchList[2][ListLen + 1];
char *switchPrt[ListLen + 1];
void logSwitches() {
  boolean skipIt = true;
  if (!LOG2)
    return;
  switchList[0][0] = legLeftLimit.extend();
  switchList[0][1] = legRightLimit.extend();
  switchList[0][2] = shelfLimitLeft.extend();
  switchList[0][3] = legLeftLimit.retract();
  switchList[0][4] = legRightLimit.retract();
  switchList[0][5] = shelfLimitRight.retract();
  switchList[0][6] = shelfLimitLeft.retract();
  switchList[0][7] = legsRetracted;
  switchList[0][8] = extendShelf;
  switchList[0][9] = retractShelf;
  switchList[0][10] = legsLeftStopped;
  switchList[0][11] = legsRightStopped;
  for (int i = 0; i <= ListLen; i++) {
    if (switchList[0][i] != switchList[1][i]) {
      skipIt = false;
      break;
    }
  }
  if (skipIt) return;
  for (int i = 0; i <= ListLen; i++)
    switchList[1][i] = switchList[0][i];
  switchPrt[0] = "leftLegLimit.extend";
  switchPrt[1] = "rightLegLimit.extend";
  switchPrt[2] = "shelfLimit.extend";
  switchPrt[3] = "leftLegLimit.retract";
  switchPrt[4] = "rightLegLimit.retract";
  switchPrt[5] = "shelfLimitLeft.retract";
  switchPrt[6] = "shelfLimitRight.retract";
  switchPrt[7] = "legsRetracted";
  switchPrt[8] = "extendShelfSwitch";
  switchPrt[9] = "retractShelfSwitch";
  switchPrt[10] = "legsLeftStopped";
  switchPrt[11] = "legsRightStopped";
  for (int i = 0; i <= ListLen; i++) {
    if (switchList[0][i])
      Serial.print("True ");
    else
      Serial.print("False ");
    Serial.println(switchPrt[i]);
  }
  Serial.println("----------------------------------");
  Serial.println("");
}

void setup() {
  Serial.begin(19200);                    //Remember to set this same baud rate to the serial monitor
  initPins();
  // initLegMotors();
  //initShelfMotors();
  Serial.println();
  Serial.println();
  Serial.println(VERSION);
  Serial.println(COPYRIGHT);
  Serial.println();

  //delay for dscope startup
  digitalWrite(dScopeRELAYPin, LOW); // make sure relay is off before making output
  pinMode(dScopeRELAYPin, OUTPUT);
  digitalWrite(dScopeRELAYPin, LOW);
  Serial.print("Waiting ");
  Serial.print(dScopeDelayTime);
  Serial.print(" secconds before dscope on\n");
  delay(dScopeDelayTime); // currently 20 sec
  digitalWrite(dScopeRELAYPin, HIGH); // turn it on
  Serial.print("dscope on\n");

  
  //copy();
  extendShelf = false;
  retractShelf = false;
  //  interuptActive = true; // set the interupts up soon
  delay(500);
  //Serial.println("Testing");
  //testlegs();
  //Serial.println("Done");
}

void loop() {
  //testRIn();
  // put your main code here, to run repeatedly:
  Serial.println("Starting Please wait for power up");
  delay(5000); //wait for puwer to boards to come up
  run1();
  Serial.println("Error");
  while (true)
    ledError(2);
  exit(0);
}
