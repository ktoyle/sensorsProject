#include <Arduino.h> 
#include <LiquidCrystal.h> // include the library code
#include <Servo.h>

//analog pins///////////////////////////////////////////////////////////////////

int potentiometer =0; //initialize analog pin 1 for potentiometer


//digital pins///////////////////////////////////////////////////////////////////////

int echoPin = 2;    // ECHO pin (ultrasonic sensor)
int trigPin = 3;    // TRIG pin (ultrasonic sensor)
int greenledpin=4; //initialize pin 4
int redledpin=5;// initialize pin 5
int buzzerPin=6;// select digital IO pin for the buzzer //9 or 10
int firstServo = 13; //first servor motor that has range sensor on top
int secondServo =22;//2nd servor motor that sweeps field when a player loses
int buttonpin=18; //button that starts the game

///Buzzer setup///////////////////////////////////////////////////////////////////
int buzzerFreq;
int buzzerPeriod;
int buzzerVal;


///LCD Pin set up/////////////////////////////////////////////////////////////////////////////
const int rs = 7, en = 8, d4 = 9, d5 = 10, d6 = 11, d7 = 12; // initialize the library by associating any needed LCD interface pin
LiquidCrystal lcd(rs, en, d4, d5, d6, d7); // with the arduino pin number it is connected to
int potVal = 0;

/// 1st servo variable set up//////////////////////////////////////////////////////////////////////
int firstMotorAngle;
int firstServoVal;// define val for 1st servo motor 
int firstServoAngle; //define val that holds angle for 1st servo motor
Servo myFirstServo; //defines 1st servo variable name

///2nd servo variable set up////////////////////////////////////////////////////////////
int secondMotorAngle;
int secondServoVal;// define val for 1st servo motor 
int secondServoAngle; //define val that holds angle for 1st servo motor
Servo mySecondServo; //defines 2nd servo variable name

///Sonic range sensor variable set up/////////////////////////////////////////////////////////////
float duration, distance_cm, distance_inch;// distance for the ultrasonic sensor
int prevDistance;
/// start button variable set up//////////////////////////////////////////////////////
int buttonVal;// define val of button

///game variable setup//////////////////////////////////////////////////////////////

int greenLight = 0;
int gameStart = 0;
int loseMode = 0;
int winMode = 0;
volatile int timeLeft = 60;
long greenTime;
long redTime;


volatile int toggleServo = 0;
volatile int servoTime = 0;
volatile int servoState = 0;
int ultrasonicFirstReading = 1;


void INT0_ISR(){ //interrupt for start of game (pin 18) button

  timeLeft = 60;
  gameStart = 1;
  loseMode = 0;
  winMode = 0;
  servoState = 180; //0-redlight, 180-greenlight
  greenLight = 1;
  servoTime = 0;
  int ultrasonicFirstReading = 1;
  greenTime = random(2,5); //initialize random green time duration
  redTime = random(2, 5);
 
	

}



void setup() {

  cli();

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  OCR1A = 15624 /1;  

  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (0 << CS11)  | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);

  sei();

    Serial.begin(9600);// set baud rate at 9600

    lcd.begin(16, 2); // set up the LCD's number of columns and rows:
    myFirstServo.attach(firstServo);
    mySecondServo.attach(secondServo);

    pinMode(buttonpin, INPUT); // Set the button pin as an input


    
    pinMode(buzzerPin,OUTPUT);// set digital IO pin pattern, OUTPUT to be output(pin 8)
    pinMode(greenledpin, OUTPUT);// set green LED pin as "output"
    pinMode(redledpin,OUTPUT);// set red LED pin as “output”

    pinMode(trigPin, OUTPUT); // Configure the trigger pin as an output for ultrasonic sensor
    pinMode(echoPin, INPUT);  // Configure the echo pin as an input ultrasonic sensor

  attachInterrupt(digitalPinToInterrupt(buttonpin), INT0_ISR, RISING);

}


void LEDs(int LED_switch){

  if (LED_switch == 1){
    digitalWrite(greenledpin, HIGH);
    digitalWrite(redledpin, LOW);
  } else{
    digitalWrite(redledpin, HIGH);
    digitalWrite(greenledpin, LOW);

    
  }

}

void buzzer_noise(int noiseMode){

    if(noiseMode == 1){

      tone(buzzerPin, 2000);
    
    }
     else{

      tone(buzzerPin, 4000);

    }
  
}

void Ultrasonic() {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);


  distance_cm = (duration) / 58.2;


  Serial.print("Distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");

  if(myFirstServo.read() == 0){

    if (ultrasonicFirstReading == 1){
      ultrasonicFirstReading = 0;
      prevDistance = distance_cm;
    }
    else{


      if(distance_cm > (prevDistance + 2) || distance_cm < (prevDistance -2 )){
        Serial.println("You lost!");
         loseMode  = 1;
      }
      else if(distance_cm < 4){
        Serial.println("You Win! :(");
        winMode = 1;
      }

       prevDistance = distance_cm;

      Serial.print("PREVIOUS Distance: ");
      Serial.println(prevDistance);
    }
  }

}

void LCD_screen(){// Display the time left on the LCD

  
  if (timeLeft == 0 || loseMode == 1){
    lcd.setCursor(0, 0);
    lcd.print("      Game      ");
    lcd.setCursor(0, 1);
    lcd.print("      Over      ");
  }
   else if(winMode == 1){
     lcd.setCursor(0, 0);
    lcd.print("      YOU      ");
    lcd.setCursor(0, 1);
    lcd.print("      WIN      ");
   }

   else if (timeLeft >= 0){
     lcd.setCursor(0, 0);
     lcd.print("Time Left");
     lcd.setCursor(0, 1);
     lcd.print(timeLeft);
     lcd.print(" Seconds      ");
   }
  
}


void servo1(int servoMode) {


  int currentAngle = myFirstServo.read(); // myservo1.read();
  myFirstServo.write(servoMode);

}

void servo2(int loseMode) {

  if(loseMode == 0 || winMode ==1){
   mySecondServo.write(0);
  }
  else{
    mySecondServo.write(180);
  }

}


ISR(TIMER1_COMPA_vect){// interrupt game clock countdown (pin 19)  button

  if(timeLeft == 0){
    timeLeft = 0;
    loseMode = 1;
  }
  else{
     timeLeft --;
  }

  servoTime++;


  if(servoState==0){ //redlight phase where ultrasonic sensor faces player


      if(servoTime == greenTime){
        greenLight = 1;
  
        servoState = 180;// angle after 3 sec go back to greenlight


        servoTime = 0;

        redTime = random(2,5); //generate next random red time duration
      }
  }
  else{//greenlight phase where is turned away from sensor

     

      if (servoTime == redTime){
           greenLight = 0;

           ultrasonicFirstReading = 1;


        servoState = 0; //angle after 2 sec go back to redlight
        servoTime = 0;

        greenTime = random(2,5); //generate next random green time duration
      }
  }


}



void loop() {


  if (gameStart == 1){

    servo2(loseMode);

    LCD_screen();

      if(loseMode== 0 && winMode == 0){ 

        servo1(servoState);
        LEDs(greenLight);
        buzzer_noise(greenLight);

        if(greenLight == 0){


        delay(300);  
        Ultrasonic();
        }
 
      }
    else{
   
    
    digitalWrite(redledpin, LOW);
    digitalWrite(greenledpin, LOW);
    noTone(buzzerPin);
  
  }
  }
 else{

  digitalWrite(redledpin, LOW);
  digitalWrite(greenledpin, LOW);
  noTone(buzzerPin); // Turn the buzzer off

 
}

}