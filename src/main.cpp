#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <stdlib.h>


LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
int ledPin = 13; // choose the pin for the LED
int redInPin = 3;   // choose the input pin (for a pushbutton)
int blueInPin = 2;   // choose the input pin (for a pushbutton)

int buttonPin = A1; 

int val = 0;     // variable for reading the pin status
int lastRedButtonState = LOW;
int sensorPin = A0; // select the input pin for the potentiometer
int digitalPin=7; //D0 attach to pin7
float averageSensorValue = 0;
int sensorValue = 0;// variable to store the value coming from A0
bool digitalHallSensorValue=0;// variable to store the value coming from pin7

unsigned long loopCount = 0;
unsigned long currentLapStartTime = 0;

unsigned long  segmentsTime [100];
unsigned long previousSegmentTime = 0; 

unsigned long previousSegmentStopTime = 0;




unsigned long elapsedTimeSinceStart = 0;
unsigned long elapsedTimeSinceLastSegment = 0;
unsigned long totalRaceTime = 0;
unsigned long currentLapTime = 0;
unsigned long cumulatedLapsTime = 0;

int NB_OF_SEGMENTS = 3;
int segmentCounter = 0;
int lapSegmentCounter = 0;

unsigned long elapsedTimeSinceStop = 0;
unsigned long elapsedTimeSinceReset =0 ;
boolean raceStarted=false;
int leftButtonPushedCounter = 0;
int lapNb = 1;

File raceLogFile;


unsigned long getTotalRaceTime()
{return millis() - elapsedTimeSinceStart;}


void setup() {
 

 
  pinMode(digitalPin,INPUT);//set the state of D0 as INPUT

 
  
  pinMode(ledPin, OUTPUT);  // declare LED as output
  pinMode(redInPin, INPUT);    // declare pushbutton as input
  
  pinMode(blueInPin, INPUT);    // declare pushbutton as input
 

  lcd.init();  //initialize the lcd
  lcd.backlight();  //open the backlight
  lcd.setCursor ( 12, 0 );
  lcd.print("00:00.00");
  Serial.begin(9600);

 if (!SD.begin(53)) {
    //Serial.println("initialization failed!");
     lcd.setCursor ( 15, 3 );
  lcd.print("NO SD");
    while (1);
  }
  else{
 lcd.setCursor ( 15, 3 );
  lcd.print("   SD");}
}

String formatMilliseconds(unsigned long milliseconds)
{
  
  unsigned long millisec  = milliseconds % 1000;
  unsigned long tseconds = milliseconds / 1000;
 /* unsigned long tminutes = tseconds / 60;
  unsigned long seconds = tseconds % 60;*/

  /*char milliChr[12];
  char secChr[12];
  char minChr[12];
  sprintf(milliChr, "%d",millisec);
   sprintf(secChr, "%d",seconds);
  sprintf(minChr, "%d",tminutes);*/
  
char buf[12];
// sprintf(buf, "%d",milliseconds % 1000);
sprintf(buf, "%d",round(milliseconds/10) % 100);
  return (buf);
 
  }




String formatSeconds(unsigned long milliseconds)
{
  
  unsigned long millisec  = milliseconds % 1000;
  unsigned long tseconds = milliseconds / 1000;
 /* unsigned long tminutes = tseconds / 60;
  unsigned long seconds = tseconds % 60;*/

 /* char milliChr[12];
  char secChr[12];
  char minChr[12];
  sprintf(milliChr, "%d",millisec);
   sprintf(secChr, "%d",seconds);
  sprintf(minChr, "%d",tminutes);*/
  
char buf[12];
 sprintf(buf, "%02d", milliseconds / 1000 %60);

  return (buf);
  
  }

String formatMinutes(unsigned long milliseconds)
{
  
  unsigned long millisec  = milliseconds % 1000;
  unsigned long tseconds = milliseconds / 1000;
  unsigned long tminutes = tseconds / 60;
 /* unsigned long seconds = tseconds % 60;*/

  /*char milliChr[12];
  char secChr[12];
  char minChr[12];
  sprintf(milliChr, "%d",millisec);
   sprintf(secChr, "%d",seconds);
  sprintf(minChr, "%d",tminutes);*/
  
char buf[12];
 sprintf(buf, "%02d", tminutes);

  return (buf);
  
  }


  void lcdPrintTime(unsigned long timeToPrint, int yLcd, int xLcd)
  {
    
   
    lcd.setCursor ( yLcd, xLcd );
    lcd.print( formatMinutes(timeToPrint));
    lcd.print(":");
     lcd.setCursor ( yLcd+3, xLcd );
    lcd.print( formatSeconds(timeToPrint));
    lcd.print(".");
     lcd.setCursor ( yLcd+6, xLcd );
    lcd.print( formatMilliseconds(timeToPrint));
    
    }

void lcdPrintNewTime(unsigned long timeToPrint, int yLcd, int xLcd)
  {
    
   
    lcd.setCursor ( yLcd, xLcd );
    lcd.print("         ");
     lcd.setCursor ( yLcd, xLcd );
    lcd.print( formatMinutes(timeToPrint));
    lcd.print(":");
     lcd.setCursor ( yLcd+3, xLcd );
    lcd.print( formatSeconds(timeToPrint));
    lcd.print(".");
     lcd.setCursor ( yLcd+6, xLcd );
    lcd.print( formatMilliseconds(timeToPrint));
    
    }

  boolean isLeftButtonPushed()
  {
    if (analogRead(buttonPin)>600  && analogRead(buttonPin)<1200)
    {
      return true;}
    else 
    {return false;}
  }

    boolean isMiddleButtonPushed()
  {
    if (analogRead(buttonPin)>350  && analogRead(buttonPin)<500)
    {return true;}
    else 
    {return false;}
  }

      boolean isRightButtonPushed()
  {
    if (analogRead(buttonPin)>150  && analogRead(buttonPin)<300)
    {return true;}
    else 
    {return false;}
  } 

  boolean isNoButtonPushed()
  {
    if ( analogRead(buttonPin)<100)
    {return true;}
    else 
    {return false;}

  } 

void loop(){
//Test
   sensorValue = analogRead(sensorPin);
     lcd.setCursor ( 17, 3 );
    
     //loopCount ++;
     //averageSensorValue = (averageSensorValue +sensorValue) / loopCount;
     // lcd.print (sensorValue);
  
  val = digitalRead(redInPin);  // read input value

  int temp = analogRead(buttonPin);
//start modifs
  
  if (isNoButtonPushed()) { 
           // check if the input is LOW (button released)
    
    lastRedButtonState = LOW;

    
  } else {
    if (( isLeftButtonPushed()&&lastRedButtonState!=val&& leftButtonPushedCounter == 0 &&(elapsedTimeSinceStart ==0 || millis() < elapsedTimeSinceStart + 1)&& millis() > elapsedTimeSinceReset + 100)) //ensure that  the first millisecond of the pressure on the button trigger the timer
    {
     
    digitalWrite(ledPin, LOW);  // turn LED ON -- START RACING
    raceStarted = true;
    elapsedTimeSinceStart = millis();
    currentLapStartTime = millis();
    


    previousSegmentStopTime = elapsedTimeSinceStart;
    leftButtonPushedCounter = 1;
    lastRedButtonState = val;
    temp=0;
    }
    else if (isLeftButtonPushed()&& lastRedButtonState!=val&& leftButtonPushedCounter == 1&&  millis() > elapsedTimeSinceStart + 100) // 2nd action on the start button: pauses the timer -  'millis() > elapsedTimeSinceStart + 100' ==> ensures that the 
    {
      elapsedTimeSinceStop = millis();
       digitalWrite(ledPin, LOW);
      raceStarted=false;
       leftButtonPushedCounter = 2;
       lastRedButtonState = val;

      raceLogFile = SD.open("raceLog.csv", FILE_WRITE);
      int x = 1;
      for (unsigned long  segmentTime: segmentsTime)
      {
        if(segmentTime>0)
       { raceLogFile.println(segmentTime);
        x = x+1;
        }
      }
      raceLogFile.close();

      }

      else if (isLeftButtonPushed()&& lastRedButtonState!=val&& leftButtonPushedCounter == 2 && millis() > elapsedTimeSinceStop + 100) // 2nd action on the start button: pauses the timer -  'millis() > elapsedTimeSinceStart + 100' ==> ensures that the 
    {
       digitalWrite(ledPin, LOW);
      totalRaceTime =0;
      elapsedTimeSinceStart =0;
      elapsedTimeSinceReset =millis();
      leftButtonPushedCounter = 0;
      lcd.setCursor ( 12, 0 );
      lcd.print("00:00.00");
      lastRedButtonState = val;
      }

      else if (isMiddleButtonPushed())
      {
        Serial.println("MIDDLE Button Pushed");

      }
      else if (isRightButtonPushed())
      {
        Serial.println("RIGHT Button Pushed");

      }
   
  }

if(digitalRead(digitalPin) == 0&&millis()>previousSegmentStopTime+1000 && raceStarted==true)
{
  
unsigned long segmentTime = millis()-previousSegmentStopTime;
previousSegmentStopTime = millis();
  segmentsTime[segmentCounter] = segmentTime;
  elapsedTimeSinceLastSegment = millis() - elapsedTimeSinceLastSegment;
  lapSegmentCounter =lapSegmentCounter +1;
  segmentCounter ++;
  currentLapTime = currentLapTime + segmentTime;

  if (lapSegmentCounter > NB_OF_SEGMENTS) //means that the lap is finished
  {
    cumulatedLapsTime = cumulatedLapsTime+segmentsTime[3*lapNb-1]+segmentsTime[3*lapNb-2]+segmentsTime[3*lapNb-3];
    Serial.print("New Lap - nb of segments = ");
    Serial.println(lapSegmentCounter);
   Serial.print("New current lap start time = ");
    lapSegmentCounter = 1;
    lapNb = lapNb+1;
    
    currentLapStartTime = elapsedTimeSinceStart  -segmentTime;
    //Serial.print("current Lap Start Time: ");
    //Serial.println(currentLapStartTime);
    //Serial.print("current Lap  Time: ");
   // Serial.println(currentLapTime);
     lcd.setCursor(3,0); //erase the previous lap time
     lcd.print("         ");
     lcd.setCursor(0,1); //erase the previous segments times
     lcd.print("           ");
     lcd.setCursor(0,2); //erase the previous segments times
     lcd.print("           ");
     lcd.setCursor(0,3); //erase the previous segments times
     lcd.print("           ");


  }
  //Serial.println("-----");
  //Serial.print("millis: ");
  //Serial.println(millis());
  //Serial.print("Current lap start time: ");
 // Serial.println(currentLapStartTime);
   //Serial.print("Diff= ");
  //Serial.println(millis()-currentLapStartTime); //currentLapStartTime becomes incorrect
//Serial.println(segmentTime);
 
  
  lcd.setCursor ( 0, lapSegmentCounter );
  lcd.print( "S");
  lcd.print(lapSegmentCounter);
  lcd.print( " ");
  lcdPrintNewTime(segmentTime, 3, lapSegmentCounter);
  

   
  } 

  
  

  if (raceStarted==true)
  {
    totalRaceTime = millis() - elapsedTimeSinceStart;
     lcdPrintTime(totalRaceTime, 12, 0);
     lcd.setCursor(0,0);
     lcd.print("L");
     lcd.print(lapNb);
     
    lcdPrintTime(totalRaceTime-cumulatedLapsTime, 3, 0);


    
   }
}