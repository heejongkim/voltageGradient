// this is a working version with master calibration capability, it can be used as master to control another units for parallel operation
// it also has function to calibrate slave with master calibration, so that the two units has very close calibration
// LCD display is enabled as well
// https://github.com/fmalpartida/New-LiquidCrystal <- header files should be added to arduino IDE via library manager (entire repo's zip file should work) -- hjk

#include <EEPROM.h>
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>


#define I2C_ADDR  0x27 // <<- Add your address here.  0x3F for test unit; 0x27 for production
#define Rs_pin 0
#define Rw_pin 1
#define En_pin 2
#define BACKLIGHT_PIN 3
#define D4_pin 4
#define D5_pin 5
#define D6_pin 6
#define D7_pin 7

#define RS485Transmit    LOW
#define RS485Receive     HIGH
#define RS485Pin 8
LiquidCrystal_I2C lcd(I2C_ADDR, En_pin, Rw_pin, Rs_pin, D4_pin, D5_pin, D6_pin, D7_pin);


const uint8_t  pushButton = A5; //this pin reads the input value of all the push buttons in the front
const uint8_t  powerStatusPin = A4;
const uint8_t  potPin = A0; //this is the big adjustment pot in the front
const uint8_t outputPin[4] = {10, 9, 11, 7};  //pin 10 for I, pin 9 for V, Pin 11 for OVP, pin 7 for Enable/Disable
const uint8_t readPin[3] = {A1, A2, A3};

#define sbi(x, y)  (x |= (1 << y))   /*Shift Register x position y*/

const uint8_t rpControlPin = 4;             //reverse polarity control pin, this is to be used with the reverse polarity box
uint8_t  rpDirection = 0 ;  //0 is reverse, 1 is normal
uint8_t reverseStatus = 0;
uint8_t reverseAutoStatus = 0 ;

uint8_t  resetZero = 0;       //when resetZero =1, the current read is re-zeroed
uint8_t  OutputStatus = 0;
uint8_t PowerStatus = 0;
uint8_t RMStatus = 0; // remote sense status = 1 means that remote sense is on, make sure to connect the remote sense to the negative end of load. failing to do so can be detrimental
uint8_t masterFlag = 0;
uint8_t OVtestStart = 0;
uint8_t OVTestStatus = 0;
uint8_t DVMStatus = 0;
uint8_t batterychargeStatus = 0;

uint8_t testParameter = 0;
uint8_t saveCalibrationStatus = 0;
uint8_t activeStatus = 1;
uint8_t calibrationType = 0;
uint8_t caliType[2]={0,0};

int n = 0;
long val;
long datV[3] = {0, 0, 0};

float loadValue = 0;
//float reverOffSet = 0.0;
float SetValue[3];
float ReadValue[3] = {0.000, 0.000, 0.000};

uint8_t samplingTime = 1; //milliseconds between data reads
float controlSum = 0.000;

float memoVals[3][6] = {{0.5, 1, 5, 0.5, 10, 20}, {1, 5, 5, 15, 20, 30}, {20, 20, 20, 125, 125, 125}};
//float memoVals[3][6] = {{0.5, 1, 1, 5, 5, 5}, {1, 5, 12, 13.8, 24, 48}, {20, 20, 20, 20, 27, 52}};
uint8_t multiplier[3] ={1, 1, 4};
int numberOfSlaves = 0;
int slaveOutput = 0;
uint8_t slaveID = 1;
float SlaveSetValue[6] = {1.000, 2.000, 2.500, 0, 0, 0};
float SlaveReadValue[3] = {0, 0, 0};
uint8_t slaveCC;
uint8_t slaveConnectionFlag = 20;
uint8_t slaveSetFlag[3] = {0, 0, 0};
uint8_t calibrationFlag = 0;
float SlaveCaliValue[2] = {0, 0};
int d = 1200;     //in microseconds
long controlTime;
uint8_t slaveReadFlag = 0;
float rawr[3];
float currentOffset[2] = {0.00, 0.00};

float c[2][15] = {{0.795, 3.093, 5.392, 7.670, 9.970, 13.000, 37.350, 61.700, 86.000, 110.400, 6.000, 1.000, 1.000, 4.000, 251.000 },{85.842, 305.136, 524.643, 743.962, 963.309, 99.713, 288.065, 476.781, 665.984, 855.811, 103.989, 300.404, 496.770, 694.275, 891.830}} ;
//float c[2][15] = {{0.309, 1.407, 2.507, 3.605, 4.704, 9.79, 94.500, 154.700, 214.900, 299.100, 6.000, 1.000, 1.000, 4.000, 251.000 },{61.914, 275.231, 487.969, 701.129, 913.929, 30.781, 307.655, 505.530, 707.006, 996.279, 30.053, 301.050, 495.140, 693.416, 979.515}} ;

uint8_t numberOfAverage = 100;

/* Configure digital pins 9 and 10 as 16-bit PWM OutputStatuss. */
void setupPWM16() {
  DDRB |= _BV(PB1) | _BV(PB2);        /* set pins as outputs */
  TCCR1A = _BV(COM1A1) | _BV(COM1B1)  /* non-inverting PWM */
           | _BV(WGM11);                   /* mode 14: fast PWM, TOP=ICR1 */
  TCCR1B = _BV(WGM13) | _BV(WGM12)
           | _BV(CS10);                    /* no prescaling */
  ICR1 = 0xffff;                      /* TOP counter value */

}

/* 16-bit version of analogWrite(). Works only on pins 9 and 10. */

void analogWrite16(uint8_t pin, uint16_t val) //Control function for PWM 9 & 10
{
  switch (pin) {
    case  9: OCR1A = val; break;
    case 10: OCR1B = val; break;
  }
}


void setup()
{

  pinMode(outputPin[2], OUTPUT);//PWM OV Control Pin
  analogWrite(outputPin[2], 254); // set OV to maximum
  setupPWM16();
  pinMode(powerStatusPin, INPUT);
  pinMode(outputPin[1], OUTPUT);//PWM Voltage Control Pin
  pinMode(outputPin[0], OUTPUT);//PWM Current Control Pin
  pinMode(outputPin[3], OUTPUT);//Output Enable/Disable
  pinMode(rpControlPin, OUTPUT);//control pin for reverse polarity
  digitalWrite(outputPin[3], LOW);//disable Output

  delay(300);
  pinMode(RS485Pin, OUTPUT); //RS485 send/receive pin
  digitalWrite(RS485Pin, RS485Receive);//Set RS485 to listen
  //analogReference(INTERNAL);
  analogReference(EXTERNAL);//choose internal 1.1V reference voltage;0:external;1:internal VCC;2:null;3:internal 1.1V
  delay(300);
  lcd.begin (20, 4); // <<-- our LCD is a 20x4, change for your LCD if needed
  // LCD Backlight ON
  lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
  lcd.setBacklight(HIGH);
  delay(300);
  checkPowerStatus();
  loadCalibrationData();        //load calibration data

  Serial.begin(9600);
  Serial1.begin(9600);
  delay(200);
  setOutput(0, 0.50); // set I to minimum
  setOutput(1, 1.0); // set V  to minimum
  n = 0;
}


//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
void loop()
{

 // if (calibrationFlag == 0) {
 //   if (numberOfSlaves > 0) {

 //     if (slaveReadFlag > 0) {
  //      slaveReadFlag = slaveReadFlag - 1;
  //      if (slaveReadFlag == 0) {
  //        Serial.println(F("Starting slave data feed."));
 //       }
 //     }
  //    actingAsMaster();
 //   }
 // }
 // else if ( calibrationFlag > 0) {
 //   SlaveCalibration(1);
 // }

  if (Serial.available() > 0) {
    checkMainSerialInput();
  }

  n++;

  //if (n == 5) {
  //  setOutput(2, c[0][9]);
  //}

 activeControl();

  LCDread();


  if (batterychargeStatus == 1) {
    autoBatteryCharger();
  }

 if (resetZero ==1) {resetCurrentOffset();
    resetZero = 0;}
 //if (testParameter >0) {Test(testParameter-1);
 //   testParameter = 0;}
 if (saveCalibrationStatus >0) {saveCalibration() ;
    saveCalibrationStatus = 0;}
    

 if (reverseStatus >0) {digitalWrite(rpControlPin, rpDirection);
    reverseStatus = 0;
    lcd.setCursor (10,0); 
    lcd.print(rpDirection);
    }

      
}



//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
void actingAsMaster() {
  if (masterFlag > 0 && PowerStatus == 2) {
    if (slaveCC / 4 == 0) {
      slaveOutput = 3;
    }

  }

  if (slaveReadFlag == 0 ) {
    RS485MasterSend(0, slaveOutput, 15);
    slaveOutput = masterFlag;
    masterDelay(40);
  }

  for (int x = 0; x < 2; x++) {
    if (abs(SlaveSetValue[3 + x] - SlaveSetValue[x]) > 0.01) {
      slaveSetFlag[x] = 1;
      Serial.print(F("resetting slave data"));
    }
    else {
      slaveSetFlag[x] = 0;
    }


    if (slaveSetFlag[x] == 1 ) {
      setSlaveOutput(x);
      delay(100);
    }
  }

}

void setSlaveOutput(int x) {
  //uint8_t f =0;
  // do {
  // f++;
  if (abs(SlaveSetValue[3 + x] - SlaveSetValue[x]) > 0.01 && x < 2) {
    RS485MasterSend(0, SlaveSetValue[x], x + 1);
    masterDelay(40);
    slaveReadFlag = 6;
    if (abs(SlaveSetValue[3 + x] - SlaveSetValue[x]) > 0.01) {
      Serial.println(F("Data congestion, setting slave output again."));
      Serial.print(SlaveSetValue[x], 3);
      Serial.print("\t"); 
      Serial.println(SlaveSetValue[x + 3], 3);
      //Serial.println(x);
      slaveSetFlag[x] = 1;

    }
    else {
      slaveSetFlag[x] = 0;
    }
  }
  //} while (slaveSetFlag[x]== 1 && f<6);


}

float potEntry(int x) {
  char* lcdEntry[9] = {"Current:", "Voltage:", "OV:", "A.M.:", "Min.:", "Start V", "End V", "Step In mS", "Cutoff Current"}; //start V and end V in Volts, step time in milliseconds, total 1000 steps, cutoff current in amps
  int rMode[9] = {0,1,1,1,1,1,1,1,0};
  int input1, input2;
  float amset = 0;
  int finestatus = 0;
  lcd.clear();
  lcd.setCursor (0, 0);
  lcd.print(F("Enter "));
  lcd.print(lcdEntry[x]);
  delay(200);
  int y = 0;
  do {

    input2 = analogRead(potPin)-1;

    if (finestatus == 0) {
      input1 = input2;
    }

    if (x == 3 || x == 4 || x == 7) {
      amset = input1 / 10.00 + input2 / 1000.00;
    }
    else  {
      amset = 3 * input1 / 100.00 * c[0][4 + 5 * rMode[x]] / 26.50 + 3 * input2 / 10000.00 * c[0][4 + 5 * rMode[x]] / 26.50;
      if (x<3) {SetValue[x] = amset;}
    }

    if (amset<0) {amset = 0;}
    lcd.setCursor (0, 2);
    lcd.print(amset, 3);
    lcd.setCursor (0, 3);
    lcd.print(F("COAR FINE      ENTR "));
    y = checkPushButton();
    if (y == 1) {
      finestatus = 0;
    }
    else if (y == 2)  {
      finestatus = 1;
    }
  } while (y != 4);

  lcd.clear();
  delay(200);
  if (x > 2 && x < 5) {
    setAM(x, amset);
  }
  if (x > 5) {
    return amset;
  }
}


void rampV() {          //start V and end V in Volts, step time in milliseconds, total 1000 steps, minimum delay for each step is 15 mS
  float nn[3];
  for (int i = 0; i < 3; i++) {
    nn[i] = potEntry(i + 5); 
  }
  long n1[5];
  n1[2] = (int) nn[2];
  n1[4] = (int) ((nn[2]-n1[2])*1000);
  //Serial.println(n1[2]);
  //Serial.println(n1[4]);
  for (int i = 0; i < 2; i++) {
    getSetValue(1, nn[i]);
    n1[i] = datV[1];
  }
  analogWrite16(outputPin[1], n1[0]);
  delay(1000);
  LCDread();
  digitalWrite(outputPin[3], HIGH);
  OutputStatus = 1;

  controlTime = millis();
  for (int y = 0; y < 1000; y++) {
    n1[3] = (int) ((n1[1] - n1[0]) / 1000.00 * y);
    analogWrite16(outputPin[1], n1[0] + n1[3]);

    readOutput(1, 10);
    panelInput();
    lcd.setCursor (0, 2);
    lcd.print(ReadValue[1], 3);
    //delayMicroseconds(1660);
    delay(n1[2]);
    delayMicroseconds(n1[4]);
    if (OutputStatus == 0) {
      y--;
    }
  }
  Serial.println(millis() - controlTime);
  analogWrite16(outputPin[1], n1[1]);
}



void setAM(int mtype, float ampmnts) {
  float factor = 1.000;

  //if (mtype == 3) {
 //   Serial.print(F("Amp.Minute = "));
 // }
//  else if (mtype == 4) {
 //   Serial.print(F("Duration (Minutes) = "));
 // }
  
  float targetam = ampmnts;
  Serial.println(targetam, 3);

  digitalWrite(outputPin[3], HIGH);
  OutputStatus = 1;
  long startT = millis();
  float amts = 0;
  do {
    n++;
    activeControl();
    LCDread();
    if (mtype == 3) {
      factor = ReadValue[0];
    }
    amts = amts + OutputStatus * factor * max((millis() - startT), 0) / 1000 / 60;

    startT = millis();
     if ((amts>targetam-factor/60) && (amts<targetam)) {
      delay((targetam - amts)/factor*1000);
      amts = targetam;
      //Serial.println((millis() - startT));
      }
    lcd.setCursor (10, 0);
    lcd.print(amts, 3);
    if (mtype == 3) {
      lcd.print(F("A.M."));
    }
    else {
      lcd.print(F("Min."));
    }

  }
  while (targetam > amts);

  digitalWrite(outputPin[3], LOW);
  OutputStatus = 0;
  //Serial.print(targetam, 3);
  // Serial.println(F(" Amp.Minute is Done."));
  //lcd.clear();
  delay(1000);
}









void RS485MasterSend(byte slaveID, float data, byte type) {
  data = data + 0.001;
  byte data1 = (int) (data * 2);
  byte data2 = lround((data * 2 - data1) * 255);
  if (slaveID < 16 && slaveID >= 0) {

    if (type < 16 && type > 0) {
      type = slaveID * 16 + type;
      long payload = data1 * 256 + data2 + type * 65536;
      digitalWrite(RS485Pin, RS485Transmit);//Set RS485 to send
      delayMicroseconds(d / 3);
      Serial1.print(payload);
      delayMicroseconds(6 * d);
      Serial1.print("$");
      delayMicroseconds(2 * d);
      digitalWrite(RS485Pin, RS485Receive);//Set RS485 to liste

    }
  }
}


void RS485MasterReceive() {


  if (Serial1.available() > 0) {
    delayMicroseconds(d * 12); //Waiting For Receive More Data
    while (Serial1.available() > 0) {

      uint16_t  type = 0;
      char incomingChar;
      float data;

      long payload = Serial1.parseInt();

      slaveID = payload / 65536;
      type = slaveID % 16;
      slaveID = slaveID / 16;

      //Serial.print("type is  ");
      //Serial.println(type);

      int slaveStatus = slaveID % 8;


      slaveID = slaveID / 8 + 1;

      //if ((slaveStatus0/8) == (slaveStatus/4+(slaveStatus%4)/2+slaveStatus%2)%2) {
      slaveCC = slaveStatus;
      //}



      if (slaveID > numberOfSlaves ) {
        serial1Flush();
        break;
      }

      payload = payload % 65536;
      byte data1 = payload / 256;
      byte data2 = payload % 256;
      data = data1 * 0.500 + data2 / 510.000;


      incomingChar = Serial1.read();
      if (incomingChar != '$') {
        serial1Flush();
        break;
      }

      // Serial.println(type);

      if (type == 0) {
        break;
      }
      else if (type < 3) {
        SlaveSetValue[3 + type - 1] = data;
        //Serial.print(slaveID);
        //Serial.print("\t"); 
        //Serial.println(data, 3);
      }
      else if (type < 6) {
        SlaveReadValue[type - 4] = data;
        slaveConnectionFlag = 20;
      }
      else if (type == 6 && data == 0) {
        calibrationFlag = 1;
      }
    }
  }
}


void SlaveCalibration(int z) {


}



void serial1Flush() {
  while (Serial1.available() > 0) {
    char t = Serial1.read();
    if (t == '$') {
      break;
    }
  }
}





//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$//


void readOutput(int x, int w) {
  long sum = 0;
  //RS485MasterReceive();

  if (w>20) {masterDelay(10);}

  for (int j = 0; j < w + 50; j++)
  {
    //val = analogRead(readPin[x]);
    sum = sum + analogRead(readPin[x]);
    delayMicroseconds(100);
    if (j == 49 ) {
      sum = 0;
    }
  }
  
  rawr[x] = 1.000 * sum / w;
  convert(x, 1, rawr[x]);


}


void loadCalibrationData() {

  int q = 5;    //5 calibration point
  int j;
  int o;
  float p=0.250;
  

  for (int i=0; i<3; i++) {
      if (i<2) {
       multiplier[i]=EEPROM.read(12 * q +i);}
  for (int z =0; z<2; z++) {
      if (z==0) { p =4.000/multiplier[i];}
      else {p = 0.250;}
  for (int x = 0; x <q ; x++) {
    j = EEPROM.read(x+i*q + z*3*q);
    o = EEPROM.read(x + i*q +z*3*q +6*q);
    c[z][x+i*q] = (j + o / 255.000)/p;
      }
     }
  }

 for (int n=0; n<2; n++) {
 convert(0, 1, c[0][11+n]);
 currentOffset[n] = ReadValue[0];
 if ((c[1][n * 5+1]-c[1][n * 5])/(c[1][n * 5+2]-c[1][n * 5+1])>1.2) {caliType[n]=1;}
 else {caliType[n] = 0;}
 }

  Serial.println(F("Data loaded."));
}

void saveCalibration() {
  int q = 5;    //5 calibration point

  int j;
  int o;

  float p=0.250;
  
  for (int i =0; i<2; i++) {
    int power = 1;
    power =(int) (c[0][i*5+4]/63.75);
    if (power>0) {
      multiplier[i]= power+1;}
  }


  for (int i=0; i<3; i++) {
  for (int z = 0; z<2; z++) {
       if (z==0) { p =4.000/multiplier[i];}
       else {p = 0.250;}
  for (int x = 0; x <q ; x++) {
      j = (int)(c[z][x+i*q]*p);
      o = lround((c[z][x+i*q]*p - j) * 255);
    EEPROM.write(x+i*q + z*3*q, j);
    EEPROM.write(x + i*q +z*3*q +6*q, o);
  }}
  if (i<2) {
    EEPROM.write(12*q +i , multiplier[i]);  }
  }
  
  Serial.println(F("Data saved."));
}


//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$//
void Calibration(int z, int calibrationtype)     {
  int pastActiveStatus = activeStatus;
  activeStatus = 0;
  uint8_t q = 5;
  int waitTime = 0;
  long sum = 0;
  float average[3] = {0.00, 0.00, 0.00};
  int se = 0;
  //for (int x = 0; x < q; x++) {
  //  c[0][x + q * z] = 0;
  //}
  analogWrite(outputPin[2], 254); // set OV to maximum
  analogWrite16(outputPin[1 - z], 16000);

  for (int x = 0; x < q; x++) {
    analogWrite16(outputPin[z], 8000 + x *12800 + 5200*calibrationtype*caliStep(x));

    for (int xx=0; xx<25; xx++) {
    activeControl();
    LCDread();
    if (xx==9) {
    c[0][x + q * z] = 0;
    
    if (z == 0) {
      Serial.print(F("Enter Current in A: "));
    }
    else {
      Serial.print(F("Enter Voltage in V: "));
    }
    while (c[0][x + q * z] < 0.01) {
      val = analogRead(readPin[z]);
      delay(200);
      if (Serial.available() > 0) {
        c[0][x + q * z] = Serial.parseFloat();
      }
    }
    Serial.println(c[0][x + q * z], 3);}
    
    if (xx>14) {
      for (int k=0; k<3; k++) {
        average[k] = average[k] + rawr[k]; }}
    }
    
    for (int jj = z; jj <= 2 * z; jj++) {
      
      if (average[jj] > 10) {
      c[1][x + q * jj] = 0.100 * average[jj];}
      else if (jj==2) {
        c[1][x + q * jj] = c[1][x + q ];
      }
      average[jj] = 0.00;
      Serial.println(c[1][x + q * jj], 3);
    }
  }
  

  Serial.println(F("Calibration Complete."));
  activeStatus = pastActiveStatus;
  for (int i=0; i<2; i++) {
  setOutput(i, SetValue[i]);}

}

void testRawRead(int pinval) {
      long sum = 0;
      long startTime = millis();
      for ( int j = 0; j < numberOfAverage * 5 + 5; j++)
      {
        delay(samplingTime);
        val = analogRead(readPin[pinval]);
        sum = sum + val;
        if (j == 4) {
          sum = 0;
        }
          Serial.print(millis()-startTime); 
          Serial.print("\t"); 
          Serial.println(val);
      }
      float average = 0.200 * sum / numberOfAverage;
          Serial.println(average, 3);
}

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$//
void setOutput(int x, float input) {


  float startVal = SlaveSetValue[x];
  getSetValue(x, input);
  float target = SetValue[x];
  float delta = 1.0;
  do {

    if (numberOfSlaves != 0) {
      if (x == 1) {
        delay(1000);
        if (target - startVal > delta) {
          input = startVal + delta;
        }
        else if (target - startVal < -delta) {
          input = startVal - delta;
        }
        else {
          input = target;
        }
      }
    }

    serial1Flush();
    getSetValue(x, input);
    LCDread();
    //Serial.print("setting output to  ");
    //Serial.println(SetValue[x], 3);

    if (numberOfSlaves > 0) {
      setSlaveOutput(x);
    }
    if (x < 2) {
      analogWrite16(outputPin[x], datV[x]);
      Serial.println();
      Serial.println(datV[x]);
    }
    else if (x == 2) {
      analogWrite(outputPin[x], datV[x]);
    }

    startVal = SetValue[x];

  } while (abs(SetValue[x] - target) > 0.005);

  slaveReadFlag = 6;
  n = 31;
  controlSum = 0;

}




//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
void activeControl() {
  long controlTime1;
  int x = 1;
  int acurracy = 3;
  
  controlTime = millis();

  checkPowerStatus();

  loadValue = 0;
  if (PowerStatus == 2 ) {
      x = 0; }
  readOutput(2, 4 * numberOfAverage);
  masterDelay(10);
  readOutput(1 - x, 4 * numberOfAverage);
  masterDelay(10);
  readOutput(x, 4 * numberOfAverage);
      //Serial.print(rawr[2], 3);
    //Serial.print("\t");
    Serial.print(rawr[0], 3);
    Serial.print("\t");
    Serial.print(rawr[1], 3);
    Serial.print("\t"); 
    ReadValue[0] = ReadValue[0] - currentOffset[rpDirection];
 
  masterDelay(10);
  loadValue =  ReadValue[x] + 2 * RMStatus * x * (ReadValue[2] - ReadValue[x]) ;

  if (PowerStatus > 0) {

  
    Serial.print(ReadValue[1-x], 3);
    Serial.print("\t"); 
    Serial.println(loadValue, 3);

    //Serial.print(SlaveReadValue[y][1-(slaveCC[y]%2)],3);}
    if (numberOfSlaves != 0) {
      Serial.print(SlaveReadValue[n % 2], 3);
    }
    //if (n%2==0) {Serial.println(" A");}
    //else {Serial.println(" V");}


    if (OutputStatus < 1 || n<11 ||activeStatus ==0) {
      controlSum = 0;
      masterDelay(10);
    }
    else {
      controlSum = loadValue + controlSum ;

      if ((n % 3) == 0) {

        controlSum  = controlSum / 3;
        //Serial.println(RMStatus);
        //Serial.print("\t"); 


        //Serial.println(controlSum  , 3);

        if (abs(loadValue - controlSum) < 0.2)  {

          if (abs((SetValue[x] - controlSum) * 1000) > acurracy) {


            if (abs(SetValue[x] - controlSum) > abs(SetValue[x] - loadValue)) {
              controlSum = SetValue[x] - loadValue;
            }
            else {
              controlSum = SetValue[x] - controlSum;
            }

            if (x == 0 || RMStatus  == 1) {
              controlSum = controlSum ;
            }

            //datV[x] = datV[x] + controlSum * 10000 / (c[0][5 * x + 3] - c[0][5 * x]);
            datV[x] = datV[x] + controlSum * (10000*(3-2*x)) / (c[0][5 * x + 3] - c[0][5 * x]);
            if (datV[x] < 0)    {
              datV[x] = 0;
            }
            if (datV[x] > 65535 )    {
              datV[x] = 65535;
            }
               // Serial.print(datV[x]);
               // Serial.print("\t");
           analogWrite16(outputPin[x], datV[x]);
            //Serial.print("\t"); 
            //Serial.println(datV[x]);
     
          }

        }

        controlSum = 0;
      }

    }

  }
  else if (PowerStatus == 0) {
    lcd.setCursor (0, 0);
    lcd.print(F("OV Protected."));
    controlTime1 = 500;
    do {
      masterDelay(10);
      controlTime1 = controlTime1 - 10;
    } while (controlTime1 > 0);
    digitalWrite(outputPin[3], LOW);
    OutputStatus = 0;
    }
    
  controlTime1 = millis();
  
  if ( controlTime1> controlTime) {
    controlTime = controlTime1 - controlTime;
        }
  else {
    controlTime = millis();
  }
  controlTime = 1000 - controlTime;

  do {
    masterDelay(10);
    controlTime = controlTime - 10;
  } while (controlTime > 0);

}

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
void getSetValue(int i, float input) {

  float ratio = 1.000;
  float cal1, cal2;
  if (i == 1 && RMStatus == 1) {
    float loadValue = 0;
    readOutput(0, numberOfAverage * 5);
    readOutput(1, numberOfAverage * 5);
    readOutput(2, numberOfAverage * 5);
    loadValue =  ReadValue[1] + 2 * (ReadValue[2] - ReadValue[1]) ;
    if (loadValue > 0.5) {
      ratio = ReadValue[1] / loadValue;
    }
   
  }

  input = input * ratio;
  convert(i, 0, input);


  if (masterFlag == 1 && i == 1) {
    SlaveSetValue[i] = SetValue[i] + 0.2;
  }
  else {
    SlaveSetValue[i] = SetValue[i];
  }


  slaveSetFlag[i] = 1;

}





void LCDread() {
  float showvalue=0;
  panelInput();
  // RS485MasterReceive();
  lcd.setCursor (0, 0); // go to start of 1st line
  if (DVMStatus == 1) {
    lcd.print(F("V="));
    lcd.print(ReadValue[2], 3);
    //lcd.print(F("V"));
  }
  //  if (OutputStatus>0){
  //   lcd.print(F("ON  "));}
  // else {lcd.print(F("OFF "));}

  if (RMStatus == 1) {
    lcd.print(F("RS ON"));
  }

  //    if(PowerStatus==2) {
  //     lcd.print(F("CC"));}
  //  else if(PowerStatus==1) {
  //     lcd.print(F("CV"));}
  //   else if(PowerStatus==0){
  //      lcd.print(F("OV"));}

  // lcd.setCursor (7,0);
  //  lcd.print(SlaveReadValue[0][n%2],3);
  //  lcd.setCursor (14,0);
  //   lcd.print(SlaveReadValue[1][n%2],3);
for (int i =0; i<2; i++) {
  lcd.setCursor (0, 1+i);
  //lcd.print(F("I= "));
  if (RMStatus == 1 && i ==1) {
    showvalue = loadValue;}
  else {
    showvalue = ReadValue[i]; }
  if (showvalue<0) {showvalue = 0.000;}
  lcd.print(showvalue, 3);
  lcd.print(F("  / "));
  lcd.print(SetValue[i], 3);
  lcd.setCursor (19, 1+i);
  if (i==0) {lcd.print(F("A"));}
  else {lcd.print(F("V"));}
}

  lcd.setCursor (0, 3); // go to start of 3rd line
  lcd.print(F("EDIT SAVE RECL "));
  if (OutputStatus == HIGH) {
    lcd.print(F("STOP"));
  }
  else {
    lcd.print(F("ENAB"));
  }

  //if (OutputStatus==HIGH){lcd.print(F("EDIT SAVE RECL STOP"));}
  //else {lcd.print(F("EDIT SAVE RECL ENAB"));}

}




void EditSetValue() {
  uint8_t nn[3] = {0, 0, 0};
  delay(500);
  int y = 0;
  
  lcd.clear();
  

  do {
    for (int i =0; i<3; i++) {
    lcd.setCursor (0, i);
    if (i==0) {lcd.print(F("I="));}
    else if (i==1) {lcd.print(F("V="));}
    else {lcd.print(F("OV="));}
    lcd.print(SetValue[i], 3);
    }
    lcd.setCursor (0, 3);
    lcd.print(F("SetI SetV SetOV ENTR"));
    //Serial.print(millis()%1000);
    //Serial.print("  ");
    y = checkPushButton();
    //Serial.println(millis()%1000);
    if (y > 0 && y < 4) {
      potEntry(y - 1);
      nn[y - 1] = 1;
      y = 0;
    }

  } while (y != 4);
  delay(300);
  lcd.clear();
  if (nn[2] != 0) {
    analogWrite(outputPin[2], 250);  // set OV to maximum
  }
  for (int z = 0; z < 3; z++) {
    if (nn[z] > 0) {
      setOutput(z, SetValue[z]);
    }
  }

}

void ChangOutputState() {
  lcd.clear();
  OutputStatus = 1 - OutputStatus;
  lcd.setCursor (0, 1);
  if (OutputStatus == HIGH) {
    digitalWrite(outputPin[3], HIGH);
    // go to start of 1st line
    lcd.print(F("Output Enabled."));
  }
  else {
    digitalWrite(outputPin[3], LOW);
    lcd.print(F("Output Disabled."));
  }
  delay(1000);
  lcd.clear();
}

void ChangOutputStateManual(int OutputStatus) { // for output enable/disable via USB communication 
  lcd.clear();
  //OutputStatus = 1 - OutputStatus;
  lcd.setCursor (0, 1);
  if (OutputStatus == HIGH) {
    digitalWrite(outputPin[3], HIGH);
    // go to start of 1st line
    lcd.print(F("Output Enabled."));
  }
  else {
    digitalWrite(outputPin[3], LOW);
    lcd.print(F("Output Disabled."));
  }
  delay(1000);
  lcd.clear();
}

void saveData1() {
  int w = 0;
  lcd.clear();
  int saveData2Status = 0;
  showPage2(w);

  delay(300);
  do {
    int y = checkPushButton();
    if (y > 0 && y < 4) {
      for (int i =0; i<3; i++) {
      memoVals[i][w * 3 + y - 1] = SetValue[i];}
      saveData2Status = 1;
      saveEntry(w * 3 + y - 1);
    }
    else if (y == 4) {
      w = 1 - w;
      delay(300);
      lcd.clear();
      showPage2(w);
    }

    delay(50);
  } while (saveData2Status == 0);

  lcd.clear();
  lcd.print(F("Data Saved"));
  delay(1000);
  lcd.clear();
}


void saveEntry(int x) {
  int q = 6;
  int i = 63 + x;
  float p;
       
  for (int m=0; m<3; m++) {
  p =4.000/multiplier[m];  
  int j = (int)(memoVals[m][x] * p );
  int o = lround((memoVals[m][x] * p - j) * 255);
  EEPROM.write(i+2*m*q, j);
  EEPROM.write(i + (2*m+1)*q, o);
  }
  
}


void showPage2(int w) {
  for (int z = 0; z < 3; z++) {
    readSavedPosition(w * 3 + z);
    lcd.setCursor (0, z);

    lcd.print(w * 3 + z);
    lcd.print(F(":"));
    lcd.print(memoVals[0][w * 3 + z], 3);
    lcd.print(F("A /"));
    lcd.print(memoVals[1][w * 3 + z], 3);
    lcd.print(F("V"));
    
  }
  lcd.setCursor (0, 3);
  if (w == 0) {
    lcd.print(F("SAV0 SAV1 SAV2 NEXT"));
  }
  else   {
    lcd.print(F("SAV3 SAV4 SAV5 BACK"));
  }
}


void saveDefaults() {
  for (int z = 0; z < 6; z++) {
    saveEntry(z);
  }
  Serial.println(F("Data saved"));
}



void readSavedPosition(int x) {
  int q = 6;
  int i = 63 + x;
  float p;
for (int m =0; m<3; m++) {
   p=4.000/multiplier[m];  
  int j = EEPROM.read(i+2*m*q);
  int o = EEPROM.read(i + (2*m+1)*q);
  memoVals[m][x] = (j  + o / 255.000)/p;
}

}


void recallMemory() {
  lcd.clear();
  int recallStatus = 0;
  int x = 0;
  readSavedPosition(x);
  delay(500);

  do {
    lcd.setCursor (18, 0);
    lcd.print(F("#"));
    lcd.print(x);   

    for (int i =0; i<3; i++) {
    lcd.setCursor (0, i);
    if (i==0) {lcd.print(F("I="));}
    else if (i==1) {lcd.print(F("V="));}
    else {lcd.print(F("OV="));}
    lcd.print(memoVals[i][x], 3);
    }
    
    lcd.setCursor (0, 3);
    lcd.print(F("EDIT NEXT PREV ENTR"));
    int y = checkPushButton();

    if (y == 1) {
      refreshSetValue(x);
      EditSetValue();
      recallStatus = 1;
    }
    if (y == 2) {
      x = (x + 1) % 6;
      lcd.clear();
      readSavedPosition(x);
      delay(300);
    }
    if (y == 3) {
      x = (x + 5) % 6;
      lcd.clear();
      readSavedPosition(x);
      delay(300);
    }
    if (y == 4) {
      lcd.clear();
      refreshSetValue(x);
      recallStatus = 1;
    }

  } while (recallStatus == 0);
  lcd.clear();
  delay(300);
  analogWrite(outputPin[2], 250);
  for (int w = 0; w < 3; w++) {
    setOutput(w, SetValue[w]);
    readOutput(w, numberOfAverage);
    delay((w % 2) * 3000);
  }

}

void refreshSetValue(int x) {
  for (int i =0; i <3; i++) {
  SetValue[i] = memoVals[i][x]; }
}


void checkMainSerialInput()
{

  char CMD, Fg;
  int commStatus = 1;
  float input ;
  delay(8);
  CMD = Serial.read();
  Fg = Serial.read();
  input  = Serial.parseFloat();
  if (Fg == '=') {
    Serial.println(input, 3);
    if (CMD == 'I' ) {
      setOutput(0, input); //Set I
    }
    if (CMD == 'V' ) {
      setOutput(1, input); //Set Voltage
    }
    if (CMD == 'G' ) {
      testRawRead(input); //Set Voltage
    }
    
    if (CMD == 'O' ) {
      setOutput(2, input); //Set OV level
    }
    if (CMD == 'M' ) {
      setAM(3, input); //Set Amp.Minute
    }
    if (CMD == 'Z' ) {                      //this test the linearility of the reading. Z=1 to test voltage reading, Z=0 to test Amperage reading, and Z=2 to test Remote sensing reading linearity
      int zz = (int) input;
      readingTest(zz);
    }
    if (CMD == 'C' ) {                      //this is for calibrating voltage & Current; 0 & 1 for current calibration; 2&3 for voltage calibration
      calibrationType = (int) input;
      Calibration(calibrationType/2, calibrationType%2 );
      caliType[calibrationType/2] = calibrationType%2;
    }
    if (CMD == 'R' ) {  // 1 - output enable, 0 - disable -- hjk
      ChangOutputStateManual(OutputStatus = (int) input );
    }

    
    if (CMD == 'P' ) {                      //this is to set output polarity normal or reversed, must use a reverse control box
     // rpDirection = (int) input;
     // digitalWrite(rpControlPin, rpDirection);
       rpDirection = (int) input; // 1- rpDirection;  // modified so that polarity can be explicitly directed -- hjk
       reverseStatus = 1;
    }
  }


  if (CMD == 'o') //Calibrate OV protection
  {
    testOVP();
  }

  if (CMD == 'L') //Load calibration data from EEPROM
  {
    loadCalibrationData();
  }

  if (CMD == 'W') //Save calibration data to EEPROM
  {
    saveCalibrationStatus = 1;

  }
  if (CMD == 'w') //Save defaults
  {
    saveDefaults();

  }


  if (CMD == 'v') //show
  {
    showCalibrationData();
  }
  
  if (CMD == 'E' ) {                      // reset zero of current reading
      resetZero = 1;
    }
}


void showCalibrationData() {
  Serial.println(F("c[0][]"));
  for (int x = 0; x < 15; x++) {

    Serial.print(c[0][x], 3);
    Serial.print(F(", "));
  }
  Serial.println();
  Serial.println(F("c[1][]"));
  for (int x = 0; x < 15; x++) {
    Serial.print(c[1][x], 3);
    Serial.print(F(", "));
  }
  Serial.println();
}



void readingTest(int z) {
  //this function tests the reading linearity, make sure to do it only after the power supply is warmed up for over 10 minutes

  int y = z;
  if (z == 2) {y = 1;}

  for (int x = 0; x < 200; x++)
  {
    analogWrite16(outputPin[y], datV[y] + x);    
    delay(1000);
    readOutput(z, numberOfAverage * 5);
    Serial.print(x);
    Serial.print("\t"); 
    Serial.println(ReadValue[z], 3);

  }
}




void autoBatteryCharger(){


  digitalWrite(outputPin[3], LOW);
  OutputStatus =0;
  lcd.clear();
  lcd.setCursor (0,0); 
  lcd.print(F("Connect Battery"));
  delay(2000);
  float nn[2]; 
  for (int i =0; i<2; i++) {
    nn[i] = potEntry(i*3+5);  //start V is the battery voltage below which the power supply will be connected to the battery, and end V is the battery voltage above which power supply will be disconnected
  }
  
  long startT = millis();
  float amts = 0;

  do{

  LCDread();
  amts =amts +  ReadValue[0]*max((millis() - startT), 0)/1000.0/60.000;
  if (ReadValue[1]<nn[0] && OutputStatus ==0 ) {digitalWrite(outputPin[3], HIGH);
    OutputStatus = 1 ;}
  else if ((ReadValue[0] < nn[1]) && OutputStatus ==1 ) {digitalWrite(outputPin[3], LOW);
    OutputStatus = 0 ;}
  startT = millis();
  lcd.setCursor (10,0); 
  if (amts<60) {lcd.print(amts,3);
  lcd.print(F("A.M."));}
  else {lcd.print(amts/60.000,3);
  lcd.print(F("A.H."));}
  
    n++;
  activeControl();
  
  } while (batterychargeStatus==1);

  digitalWrite(outputPin[3], LOW);
  OutputStatus = 0 ;
  lcd.clear();
  lcd.print(F("Charger Off"));
  delay(500);
  lcd.clear();
}


void testOVP() {
  int calitype = caliType[1];
  int upperlimit = 220+35*calitype;
  int lowerlimit = 33 - 15*calitype;
  digitalWrite(outputPin[3], HIGH);
  OutputStatus = 1 ;
  for (int i = 0; i < 2; i++) {
    analogWrite(outputPin[2], 254);
    do {
      checkPowerStatus();
      delay(1000);
    } while (PowerStatus == 0) ;

    setOutput(0, 1.0);
    setOutput(1, c[0][5 + i * 4]);
    delay(5000);

    int x = 0;
    do {
      Serial.println(lowerlimit + (upperlimit -lowerlimit)* i - x);
      analogWrite(outputPin[2], lowerlimit + (upperlimit -lowerlimit)* i - x);
      checkPowerStatus();
      delay(2000);
      x++;
    } while (PowerStatus != 0);
    c[0][10 + i * 4] = lowerlimit + (upperlimit -lowerlimit)* i - x+2;
  }
  Serial.println("OV Calibration Complete");
  analogWrite(outputPin[2], 254);

}


void convert( int x, int mtype, float input) {
  int y = x;
  if (x == 2 && mtype ==1) { y = 1;}
  float v1, v2, v3;
  int  j1 = 0;
  int calitype = caliType[x];
  int  jstatus = 0 ;


  
    do {
      j1++;
      if (input < c[mtype][x * 5 + j1] || j1 == 4) {
        if (mtype ==1) {v1 = c[0][y * 5 + j1] - c[0][y * 5 + j1 - 1];
            v2 = c[0][y * 5 + j1 - 1]; }
        else if (mtype ==0) {
          
          if (j1 ==1 || j1==4) {v1 = 12800.00+5200.00*calitype;}
          else {v1 = 12800.00;}

          v2 = 8000.00+ (j1-1)*12800.00+5200*calitype*caliStep(j1-1);}
          
        v1 = v1 / (c[mtype][x * 5 + j1] - c[mtype][x * 5 + j1 - 1]);
        v2 = v2 - v1 * c[mtype][x * 5 + j1 - 1];
        jstatus = 1;
      }
    } while ( jstatus != 1);
    
   v3 = v1 * input + v2; 
   if (v3 < 0)    { v3= 0;}
   
   if (mtype ==1) {ReadValue[x] = v3;}
   
   else if (mtype==0) { 
       if (v3> 65535) { v3 = 65535;}
       datV[x] = v3;
       SetValue[x] = (v3 - v2) / v1;
          if (x==2 ) {
          v1 = (c[0][14] - c[0][10])/(c[0][9] - c[0][5]);
          v2 = v1*(input - c[0][5])+c[0][10];
          if (v2>255.0) {datV[x] = 255;}
          else {datV[x]= lround(v2);}
          SetValue[x] = input;
          }
          if (SetValue[x]<0) {SetValue[x]=0;}
  }
}

void resetCurrentOffset() {            //resetting current zero level
  //reverOffSet = 2.000;
  for (int i =0; i<2; i++) {
  c[0][11+i] = 0.00;
  currentOffset[i] = 0.00;
  rpDirection = i;
  digitalWrite(rpControlPin, i);
  delay(10000);
  readOutput(0, 1000);
  c[0][11+i] = rawr[0];
  convert(0, 1, rawr[0]);
  currentOffset[i] = ReadValue[0];}
  
}

int caliStep (int x) {
  int cs = 0;
    if (x==0) { cs = - 1;}
    else if (x==4) {cs = 1;}
    else {cs = 0;}

    return cs;
}

void masterDelay(int x) {

  if (x > 5) {
    panelInput();
    //if (Serial1.available() > 0) {
    //  RS485MasterReceive();
   //   x = 0;
   // }
  }
  delay(x);
}

void panelInput() {

  int x = 0;
  x = checkPushButton();

  if (x == 1) {
    EditSetValue();
  }
  else if (x == 2) {
    saveData1();
  }
  else if (x == 3) {
    recallMemory();
  }
  else if (x == 4)  {
    ChangOutputState();
  }
  else if (x == 21)  {
    RMStatus = 1 - RMStatus;
  }
  else if (x == 20) {
    potEntry(3);
  }
  else if (x == 19) {
    potEntry(4);
  }
  else if (x == 18) {
    rampV();
  }
  else if (x == 17) {
    batterychargeStatus = 1 - batterychargeStatus;
  }
  else if (x == 16) {
    DVMStatus = 1 - DVMStatus;
  }
  else if (x == 15) {         //press M5 will reset the zero of current reading
   resetZero = 1;
    
  }

  else if (x == 14) {        //press M4 will save calibration data
    saveCalibrationStatus = 1;
  }

  else if (x == 12) {               //turning active control on and off
      activeStatus = 1- activeStatus;
      Serial.print(F("Active Control: "));
      Serial.println(activeStatus);
  }


  else if (x == 13) {               //press M3 will change polarity with the help of reverse polarity box
      rpDirection = 1- rpDirection;
      reverseStatus = 1;
  }


  
  if (x != 0 && x != 11) {
    lcd.clear();
    delay(300);
  }
}

int checkPushButton() {

  uint16_t valPushButton[2];
  valPushButton[0] = analogRead(pushButton);
  uint8_t nn = 0;
  do {
    delay(3);
    nn = 1 - nn;
    valPushButton[nn] = analogRead(pushButton);
  } while (abs(valPushButton[0] - valPushButton[1]) > 40);

  uint8_t pushButtonState = 0;
  if (valPushButton[0] > 920) {
    pushButtonState = 1;
  }
  else if (valPushButton[0] > 868) {
    pushButtonState = 2;
  }
  else if (valPushButton[0] > 816) {
    pushButtonState = 3;
  }
  else if (valPushButton[0] > 765) {
    pushButtonState = 4;
  }
  else if (valPushButton[0] > 714) {
    pushButtonState = 21;
  }
  else if (valPushButton[0] > 662) {
    pushButtonState = 20;
  }
  else if (valPushButton[0] > 610) {
    pushButtonState = 19;
  }
  else if (valPushButton[0] > 560) {
    pushButtonState = 18;
  }
  else if (valPushButton[0] > 508) {
    pushButtonState = 17;
  }
  else if (valPushButton[0] > 456) {
    pushButtonState = 16;
  }
  else if (valPushButton[0] > 405) {
    pushButtonState = 15;
  }
  else if (valPushButton[0] > 353) {
    pushButtonState = 14;
  }
  else if (valPushButton[0] > 301) {
    pushButtonState = 13;
  }
  else if (valPushButton[0] > 247) {
    pushButtonState = 12;
  }
  else if (valPushButton[0] > 150) {
    pushButtonState = 11;
  }
  else {
    pushButtonState = 0;
  }

  //if (pushButtonState > 0) {

  //  if (pushButtonState > 10) {
   //   Serial.print(F("M"));
   //   int pstatus = pushButtonState - 10;
   //   Serial.println(pstatus);
  //  }
 //   else {
 //     Serial.print(F("F"));
//      Serial.println(pushButtonState);
 //   }
 // }
  return pushButtonState;
}

void checkPowerStatus() {
  uint16_t valPS = 0;

  for (int x = 0; x < 50; x++) {
  valPS = valPS + analogRead(powerStatusPin);
  //Serial.println(valPS);
  }
  valPS = valPS/50;
  
  if (valPS > 400)      {
    PowerStatus = 2; //CC
  }
  else if (valPS > 100) {
    PowerStatus = 1; //CV

  }
  else if (valPS < 40) {
    PowerStatus = 0; //OV
  }
  //Serial.println(valPS);
 // Serial.println(PowerStatus);
}
