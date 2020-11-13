/*
 * IV_Pump_Control
 * Author:Vikram Seshadri
 * 
 * Description:
 * Using a Relay Circuit to switch every minute (switch_time variable) between pump on and pump off 
 * Using a Push button to turn off the pump until next power cycle.
 * 
 */

#define switch_time 3000//1 min

const int flowpump = 10;    //Relay1
const int respump = 7;      //Relay 2
const int buttonPin = 2;    //Push Button

int buttonState = 0;
int counter = 0;            //no. of switches made

unsigned long currentMillis = 0;//Stores current/latest value of millis()
unsigned long prevMillis = 0;   //Stores currentMillis before increment

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

//  pinMode(flowpump, INPUT_PULLUP);
  pinMode(flowpump, OUTPUT);
  pinMode(respump, OUTPUT);
  

}

void loop() {
  // put your main code here, to run repeatedly:
//  buttonState = digitalRead(buttonPin);
//
//  if (buttonState == LOW) {
//    Active();
//  }
//  
PumpON_Flow();
delay (3000);
PumpOFF_Flow();
PumpON_Res();
delay (3000);
PumpOFF_Res;
Delay(3000);
  
}

void Active()
{
  currentMillis = millis();
  if(digitalRead(flowpump) == HIGH && digitalRead(respump) == LOW)
  {
    if (currentMillis - prevMillis >= switch_time)
    {
      counter = counter + 1;
      Serial.print("Sample #  ");
      Serial.println(counter);
      
      PumpON_Flow();
      delay(3000);//Time On
      PumpOFF_Flow();
      Serial.println("Pump OFF");
      prevMillis = currentMillis;
      
      Serial.print("Timer (sec) ");
      Serial.println(currentMillis/1000);
      delay(2000);
    }
  
    else
    {
      PumpOFF_Flow();
    }
  }
  else if(digitalRead(respump) == HIGH && digitalRead(flowpump) == LOW)
  {
   if (currentMillis - prevMillis >= switch_time)
    {
      counter = counter + 1;
      Serial.print("Sample #  ");
      Serial.println(counter);
      
      PumpON_Res();
      delay(3000);//Time On
      PumpOFF_Res();
      Serial.println("Pump OFF");
      prevMillis = currentMillis;
      
      Serial.print("Timer (sec) ");
      Serial.println(currentMillis/1000);
      delay(2000);
    }
  
    else
    {
      PumpOFF_Res();
    } 
  }
}

void PumpON_Flow() {
  //Activate Pump
  digitalWrite(flowpump, LOW); //Trigger ON
  Serial.println("Flow Pump ON");
}

void PumpOFF_Flow() {
  //Shuts off Pump
  digitalWrite(flowpump, HIGH); //Trigger OFF
}


void PumpON_Res() {
  //Activate Pump
  digitalWrite(flowpump, LOW); //Trigger ON
  Serial.println("Resevoir Pump ON");
}

void PumpOFF_Res() {
  //Shuts off Pump
  digitalWrite(respump, HIGH); //Trigger OFF
}
