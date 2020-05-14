#include <Wire.h>
#include <Adafruit_VCNL4010.h>
Adafruit_VCNL4010 vcnl;
extern "C" { 
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}

#define TCAADDR 0x70       //Het adres van de multiplexer

//Verander hieronder op welk adres je de sensoren hebt gezet
#define SENSORLINKS 1      
#define SENSORRECHTS 7
#define MOTORLINKS 7
#define MOTORRECHTS 8
#define SENSORLINKSVOOR 1
#define SENSORRECHTSVOOR 7
#define HANDTIMING 1000
#define VOLGAFSTAND 40

#define NAUWKEURIGHEID 10  //Nauwkeurigheid van de sensoren op een schaal van 10-100, dichter bij de 10 is nauwkeuriger maar kan valse metingen geven.

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
  average = average/3;
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

  //Kijk of er een obstakel voor de auto is
  if((afstandLinksVoor >= NAUWKEURIGHEID) || (afstandRechtsVoor >= NAUWKEURIGHEID))
  {
    handDetectie = true;
    Serial.println("Hand gevonden");
    handDetectieMillis = currentMillis;
  }
  else
  {
    if((currentMillis - handDetectieMillis) >= HANDTIMING)
    {
      Serial.println("Geen hand gevonden");
      handDetectie = false;
    }
  }

  if(handDetectie == true)
  {
    if(afstandLinksVoor > afstandRechtsVoor + NAUWKEURIGHEID){
      Serial.println("Draai naar links");
      return 2;
      
    }
    if(afstandRechtsVoor > afstandLinksVoor + NAUWKEURIGHEID){
      Serial.println("Draai naar rechts");
      return 1;
    }
    
    if((afstandRechtsVoor > VOLGAFSTAND) || (afstandLinksVoor > VOLGAFSTAND))
    {
      Serial.println("Stop met rijden"); 
    }
    else
    {
      Serial.println("Rijd tot de volgafstand");
      return 0;
    }
  }
}

int ScanSides()
{
  //De eerste keer dat deze functie wordt aangeroepen ijkt hij de waardes van de sensoren.
  static int sensorLinksIjk = SensorAfstand(SENSORLINKS);
  static int sensorRechtsIjk = SensorAfstand(SENSORRECHTS);

  //Door de ijkwaarde van de sensorwaarde af te halen wordt de uiteindelijke waarde ~0 als er niks gedetecteerd word.
  int afstandLinks = (SensorAfstand(SENSORLINKS)-sensorLinksIjk);
  int afstandRechts = (SensorAfstand(SENSORRECHTS)-sensorRechtsIjk);
  
  Serial.print("Proximity links: "); Serial.println(afstandLinks);
  Serial.print("Proximity rechts: "); Serial.println(afstandRechts);
  
  if(abs(afstandLinks) > (abs(afstandRechts) + NAUWKEURIGHEID)){
    //Hier stuur je de motor naar rechts aan, voor nu nog even een led.
    digitalWrite(MOTORLINKS, HIGH);
    digitalWrite(MOTORRECHTS, LOW);
    return 2;
    
  }
  else if(abs(afstandRechts) > (abs(afstandLinks) + NAUWKEURIGHEID)){
    //Hier stuur je de motor naar links aan, voor nu nog even een led.
    digitalWrite(MOTORLINKS, LOW);
    digitalWrite(MOTORRECHTS, HIGH);
    return 1;
  }
  else{
    //Niet draaien maar weer rechtdoor rijden.
    digitalWrite(MOTORLINKS, LOW);
    digitalWrite(MOTORRECHTS, LOW);
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
  Serial.println("VCNL4010 nummer 1 gevonden");
  SensorSelect(SENSORRECHTS);
  if (! vcnl.begin()){
    Serial.println("Sensor niet gevonden:");
    while (1);
  }
  Serial.println("VCNL4010 nummer 2 gevonden");

  //ledjes om te testen
  pinMode(MOTORLINKS, OUTPUT);
  pinMode(MOTORRECHTS, OUTPUT);
}



void loop() 
{
  currentMillis = millis();
  //motorActie 1 = links, 2 = rechts, 0 is niks of rechtdoor.
  int motorActie = ScanSides();
  int volgActie = ScanFront();
  delay(100);
}
