#include <Wire.h>
#include <Adafruit_VCNL4010.h>
#include <ZumoShield.h>
Adafruit_VCNL4010 vcnl;
extern "C" { 
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}

ZumoMotors motors;

#define TCAADDR 0x70       //Het adres van de multiplexer

//Verander hieronder op welk adres je de sensoren hebt gezet
#define SENSORLINKS 2      
#define SENSORRECHTS 7
//#define MOTORLINKS 7
//#define MOTORRECHTS 8
#define SENSORLINKSVOOR 0
#define SENSORRECHTSVOOR 5

#define LEFTSPEED 150
#define RIGHTSPEED 150
#define HANDTIMING 300
#define VOLGAFSTAND 60
#define STOPAFSTAND 20
#define NAUWKEURIGHEID 10  //Nauwkeurigheid van de sensoren op een schaal van 10-100, dichter bij de 10 is nauwkeuriger maar kan valse metingen geven.

//Sorry maar ik maak toch echt een globale variabele voor de tijd.
unsigned long currentMillis = 0;

void startSensoren()
{
    while (!Serial);
    delay(1000);
    Wire.begin();
    Serial.println("\nTCAScanner ready!");
    delay(100);
    for (uint8_t t=0; t<8; t++) {
      SensorSelect(t);
      Serial.print("TCA Port #"); Serial.println(t);
 
      for (uint8_t addr = 0; addr<=127; addr++) {
        if (addr == TCAADDR) continue;
      
        uint8_t data;
        if (! twi_writeTo(addr, &data, 0, 1, 1)) {
           Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
        }
      }
    }
    Serial.println("\ndone");
}

 //Deze functie selecteert de sensor op de multiplexer voor je
void SensorSelect(uint8_t nummer) {
  if (nummer > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << nummer);
  Wire.endTransmission();  
}

//Deze functie returnt het gemiddelde van 2 scans voor stabiliteit. Heeft het sensornummer nodig
int SensorAfstand(int sensorNr){
  unsigned long int average = 0;
  SensorSelect(sensorNr);
  for(int i = 0; i<2; i++){
    average += vcnl.readProximity();
  }
  average = (average/2);
  return(average);
}

int ScanFront()
{
  static unsigned long handDetectieMillis = currentMillis;
  //De eerste keer dat deze functie wordt aangeroepen ijkt hij de waardes van de sensoren.
  static int sensorLinksVoorIjk = SensorAfstand(SENSORLINKSVOOR);
  static int sensorRechtsVoorIjk = SensorAfstand(SENSORRECHTSVOOR);
  //Maak een vlag voor de detectie van een hand.
  static int handDetectie = false;

  //Door de ijkwaarde van de sensorwaarde af te halen wordt de uiteindelijke waarde ~0 als er niks gedetecteerd word.
  int afstandLinksVoor = (SensorAfstand(SENSORLINKSVOOR)-sensorLinksVoorIjk);
  int afstandRechtsVoor = (SensorAfstand(SENSORRECHTSVOOR)-sensorRechtsVoorIjk);
  Serial.print("Proximity links: "); Serial.println(afstandLinksVoor);
  Serial.print("Proximity rechts: "); Serial.println(afstandRechtsVoor);

  
  /*long nauwkeurigheid = abs((afstandLinksVoor + afstandRechtsVoor)/2);
  Serial.print("Nauwkeurigheid voor remappen: "); Serial.println(nauwkeurigheid);
  nauwkeurigheid = map(nauwkeurigheid, 0, 5000, NAUWKEURIGHEID, 50); 
  nauwkeurigheid = (nauwkeurigheid * nauwkeurigheid); 
  Serial.print("Nauwkeurigheid na remappen: "); Serial.println(nauwkeurigheid);*/

  //Kijk of er een obstakel voor de auto is
  if((afstandLinksVoor >= NAUWKEURIGHEID) || (afstandRechtsVoor >= NAUWKEURIGHEID))
  {
    handDetectie = true;
    Serial.println("Hand gevonden!");
    handDetectieMillis = currentMillis;
  }
  else
  {
    if((currentMillis - handDetectieMillis) >= HANDTIMING)
    {
      //Serial.println("Geen hand gevonden");
      motors.setLeftSpeed(0);
      motors.setRightSpeed(0);
      handDetectie = false;
    }
  }

  if(handDetectie == true)
  {
    if((afstandRechtsVoor > VOLGAFSTAND) && (afstandLinksVoor > VOLGAFSTAND))
    {
      //Te dichtbij, rijd achteruit.
      motors.setLeftSpeed((-LEFTSPEED/1.5));
      motors.setRightSpeed((-RIGHTSPEED/1.5));
     // Serial.println("Rijd achteruit");
      return; 
    }
    if((afstandRechtsVoor < (STOPAFSTAND)) && (afstandLinksVoor < (STOPAFSTAND)))
    {
      //Te ver weg rijd dichterbij.
      motors.setLeftSpeed(LEFTSPEED/1.5);
      motors.setRightSpeed(RIGHTSPEED/1.5);
      //Serial.println("Rijd tot de volgafstand");
      return;
    }
    
    if(afstandRechtsVoor > afstandLinksVoor + NAUWKEURIGHEID){
      motors.setLeftSpeed(LEFTSPEED/1.5);
      motors.setRightSpeed((-RIGHTSPEED/1.5));
      Serial.println("Draai naar rechts");
      return;
    }
    
    if(afstandLinksVoor > afstandRechtsVoor + NAUWKEURIGHEID){
      motors.setLeftSpeed((-LEFTSPEED/1.5));
      motors.setRightSpeed(RIGHTSPEED/1.5);
      Serial.println("Draai naar links");
      return;
    }
    

    
    if(((afstandRechtsVoor < VOLGAFSTAND) && (afstandRechtsVoor > STOPAFSTAND)) || ((afstandLinksVoor < VOLGAFSTAND) && (afstandLinksVoor > STOPAFSTAND)))
    {
      motors.setLeftSpeed(0);
      motors.setRightSpeed(0);
      Serial.println("Stop met rijden"); 
    }


  }
}

int ScanSides()
{
  //De eerste keer dat deze functie wordt aangeroepen ijkt hij de waardes van de sensoren.
  static int sensorLinksIjk = SensorAfstand(SENSORLINKS);
  static int sensorRechtsIjk = SensorAfstand(SENSORRECHTS);
  static unsigned long lastMillisRechts = 0;
  static unsigned long lastMillisLinks = 0;
  static int hoogsteLinks = 0;
  static int hoogsteRechts = 0;
  static int draaiRechts = false;

  //Door de ijkwaarde van de sensorwaarde af te halen wordt de uiteindelijke waarde ~0 als er niks gedetecteerd word.
  int afstandLinks = (SensorAfstand(SENSORLINKS)-sensorLinksIjk);
  int afstandRechts = (SensorAfstand(SENSORRECHTS)-sensorRechtsIjk);
  
  //Serial.print("Proximity links: "); Serial.println(afstandLinks);
  //Serial.print("Proximity rechts: "); Serial.println(afstandRechts);
  
  if(afstandLinks > (afstandRechts + NAUWKEURIGHEID)){
    //hoogsteRechts = 0;
    //Hier stuur je de motor naar rechts aan, voor nu nog even een led.
    if(afstandLinks > hoogsteLinks){
      hoogsteLinks = afstandLinks;
      Serial.println("draai naar rechts");
          motors.setLeftSpeed(LEFTSPEED);
          motors.setRightSpeed(RIGHTSPEED/1.5);
    }
    else{
        motors.setLeftSpeed(LEFTSPEED);
        motors.setRightSpeed(RIGHTSPEED);
        hoogsteLinks = afstandLinks;
        Serial.println("rechtdoor 2");
    }
    return 2;
    
  }
  
  if(afstandRechts > (afstandLinks + NAUWKEURIGHEID)){
    //hoogsteLinks = 0;
    //Hier stuur je de motor naar links aan, voor nu nog even een led.
    if(afstandRechts > hoogsteRechts){
      hoogsteRechts = afstandRechts;
      Serial.println("draai naar links");
          motors.setLeftSpeed(LEFTSPEED/1.5);
          motors.setRightSpeed(RIGHTSPEED);
    }
    else{
        motors.setLeftSpeed(LEFTSPEED);
        motors.setRightSpeed(RIGHTSPEED);
        hoogsteRechts = afstandRechts;
        Serial.println("rechtdoor 1");
    }
    return 1;
  }
  
  else
  {
    motors.setLeftSpeed(LEFTSPEED);
    motors.setRightSpeed(RIGHTSPEED);
    Serial.println("rechtdoor");
  }

}

int DraaiRechts()
{
  static unsigned long draaiMillis = currentMillis;
  static int draaiTijd = (100000/RIGHTSPEED);
  static int timeSet = false;
  
  if(((currentMillis - draaiMillis) <= draaiTijd) && ((currentMillis - draaiMillis) > 0))
  {
    Serial.println("draaien");
    motors.setRightSpeed((-RIGHTSPEED));
    motors.setLeftSpeed(LEFTSPEED);
    timeSet = false;
  }
  else if(timeSet == false)
  {
    motors.setRightSpeed(0);
    motors.setLeftSpeed(0);
    draaiMillis = (currentMillis + 2000);
    timeSet = true;
  }
  
  
}

void Pathfinding()
{


  
}

void setup() 
{
  Serial.begin(9600);
  delay(1000);
  startSensoren();
  SensorSelect(SENSORLINKS);
  if (! vcnl.begin()){
    Serial.println("Sensor niet gevonden:");
    while (1);
  }
  Serial.println("VCNL4010 Linkerkant gevonden");
  SensorSelect(SENSORRECHTS);
  if (! vcnl.begin()){
    Serial.println("Sensor niet gevonden:");
    while (1);
  }
  Serial.println("VCNL4010 Rechterkant gevonden");

  //ledjes om te testen
  //pinMode(MOTORLINKS, OUTPUT);
  //pinMode(MOTORRECHTS, OUTPUT);
}



void loop() 
{
  currentMillis = millis();
  //motorActie 1 = links, 2 = rechts, 0 is niks of rechtdoor.
  //int motorActie = ScanSides();
  //DraaiRechts();
  int volgActie = ScanFront();
  //delay(100);
}
