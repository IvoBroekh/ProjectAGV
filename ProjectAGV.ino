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

#define LEFTSPEED 120
#define RIGHTSPEED 120
#define HANDTIMING 50
#define VOLGAFSTAND 20
#define NAUWKEURIGHEID 5  //Nauwkeurigheid van de sensoren op een schaal van 10-100, dichter bij de 10 is nauwkeuriger maar kan valse metingen geven.

//Sorry maar ik maak toch echt een globale variabele voor de tijd.
unsigned long currentMillis = 0;

void startSensoren()
{
    while (!Serial);
    delay(1000);
 
    Wire.begin();

    Serial.println("\nTCAScanner ready!");
    
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

//Deze functie returnt het gemiddelde van 3 scans voor stabiliteit. Heeft het sensornummer nodig
int SensorAfstand(int sensorNr){
  unsigned long int average = 0;
  SensorSelect(sensorNr);
  for(int i = 0; i<3; i++){
    average += vcnl.readProximity();
  }
  average = abs((average/3));
  //Serial.println(average);
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
  int nauwkeurigheid = abs((afstandLinksVoor + afstandRechtsVoor)/2);
  Serial.print("Nauwkeurigheid: "); Serial.println(nauwkeurigheid);
  nauwkeurigheid = map(nauwkeurigheid, 0, 2000, NAUWKEURIGHEID, 50); 
  nauwkeurigheid = (nauwkeurigheid * nauwkeurigheid); 
  Serial.print("Nauwkeurigheid: "); Serial.println(nauwkeurigheid);

  //Kijk of er een obstakel voor de auto is
  if((afstandLinksVoor >= 5) || (afstandRechtsVoor >= 5))
  {
    handDetectie = true;
    //Serial.println("Hand gevonden");
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
    if(afstandLinksVoor > afstandRechtsVoor + nauwkeurigheid){
      motors.setLeftSpeed((-LEFTSPEED));
      motors.setRightSpeed(RIGHTSPEED);
      //Serial.println("Draai naar links");
      return 2;
      
    }
    if(afstandRechtsVoor > afstandLinksVoor + nauwkeurigheid){
      motors.setLeftSpeed(LEFTSPEED);
      motors.setRightSpeed((-RIGHTSPEED));
      //Serial.println("Draai naar rechts");
      return 1;
    }
    
    if((afstandRechtsVoor > VOLGAFSTAND) || (afstandLinksVoor > VOLGAFSTAND))
    {
      motors.setLeftSpeed(0);
      motors.setRightSpeed(0);
      //Serial.println("Stop met rijden"); 
    }
    if((afstandRechtsVoor > (VOLGAFSTAND + nauwkeurigheid)) || (afstandLinksVoor > (VOLGAFSTAND + nauwkeurigheid)))
    {
      motors.setLeftSpeed((-LEFTSPEED));
      motors.setRightSpeed((-RIGHTSPEED));
      //Serial.println("Rijd achteruit"); 
    }
    if((afstandRechtsVoor < (VOLGAFSTAND)) || (afstandLinksVoor < (VOLGAFSTAND)))
    {
      motors.setLeftSpeed(LEFTSPEED);
      motors.setRightSpeed(RIGHTSPEED);
      //Serial.println("Rijd tot de volgafstand");
    }
  }
}

int ScanSides()
{
  //De eerste keer dat deze functie wordt aangeroepen ijkt hij de waardes van de sensoren.
  static int sensorLinksIjk = SensorAfstand(SENSORLINKS);
  static int sensorRechtsIjk = SensorAfstand(SENSORRECHTS);
  static int hoogsteLinks = 0;
  static int hoogsteRechts = 0;

  //Door de ijkwaarde van de sensorwaarde af te halen wordt de uiteindelijke waarde ~0 als er niks gedetecteerd word.
  int afstandLinks = abs(SensorAfstand(SENSORLINKS)-sensorLinksIjk);
  int afstandRechts = abs(SensorAfstand(SENSORRECHTS)-sensorRechtsIjk);
  
  Serial.print("Proximity links: "); Serial.println(afstandLinks);
  Serial.print("Proximity rechts: "); Serial.println(afstandRechts);
  
  if(afstandLinks > (afstandRechts + NAUWKEURIGHEID)){
    //Hier stuur je de motor naar rechts aan, voor nu nog even een led.
    if(afstandLinks > hoogsteLinks){
      hoogsteLinks = afstandLinks;
          motors.setLeftSpeed(LEFTSPEED);
          motors.setRightSpeed(RIGHTSPEED/1.5);
    }
    else{
        motors.setLeftSpeed(LEFTSPEED);
        motors.setRightSpeed(RIGHTSPEED);
        hoogsteLinks = afstandLinks;
    }
    return 2;
    
  }
  else if(afstandRechts > (afstandLinks + NAUWKEURIGHEID)){
    //Hier stuur je de motor naar links aan, voor nu nog even een led.
    if(afstandRechts > hoogsteRechts){
      hoogsteRechts = afstandRechts;
          motors.setLeftSpeed(LEFTSPEED/1.5);
          motors.setRightSpeed(RIGHTSPEED);
    }
    else{
        motors.setLeftSpeed(LEFTSPEED);
        motors.setRightSpeed(RIGHTSPEED);
        hoogsteRechts = afstandRechts;
    }
    return 1;
  }
  else{
    //Niet draaien maar weer rechtdoor rijden.
    motors.setLeftSpeed(LEFTSPEED);
    motors.setRightSpeed(RIGHTSPEED);
    return 0;
  }
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
  int motorActie = ScanSides();
 // int volgActie = ScanFront();
  //delay(100);
}
