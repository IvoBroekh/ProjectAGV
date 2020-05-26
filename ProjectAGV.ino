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
#define HANDTIMING 200
#define VOLGAFSTAND 100
#define STOPAFSTAND 30
#define NAUWKEURIGHEID 10  //Nauwkeurigheid van de sensoren op een schaal van 10-100, dichter bij de 10 is nauwkeuriger maar kan valse metingen geven.

//Sorry maar ik maak toch echt een globale variabele voor de tijd.
unsigned long currentMillis = 0;

void StartSensoren()
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

int VolgModus()
{
  //Maak een vlag voor de detectie van een hand.
  static int handDetectie = false;
  static unsigned long handDetectieMillis = currentMillis;

  //Check de afstanden
  int afstandLinksVoor = Afstand(SENSORLINKSVOOR);
  int afstandRechtsVoor = Afstand(SENSORRECHTSVOOR);

  Serial.println(afstandLinksVoor);
  Serial.println(afstandRechtsVoor);
  //Kijk of er een hand voor de auto is
  //Als er geen hand meer wordt gedetecteert gedurende HANDTIMING dan stopt hij.
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
      motors.setLeftSpeed(0);
      motors.setRightSpeed(0);
      handDetectie = false;
    }
  }

  //Wat moet er gebeuren als er een hand is gevonden.
  if(handDetectie == true)
  {
    if((afstandRechtsVoor > VOLGAFSTAND) && (afstandLinksVoor > VOLGAFSTAND))
    {
      //Te dichtbij, rijd achteruit.
      motors.setLeftSpeed((-LEFTSPEED));
      motors.setRightSpeed((-RIGHTSPEED)); 
    }
    else if((afstandRechtsVoor < (STOPAFSTAND)) && (afstandLinksVoor < (STOPAFSTAND)))
    {
      //Te ver weg rijd dichterbij.
      motors.setLeftSpeed(LEFTSPEED);
      motors.setRightSpeed(RIGHTSPEED);
    }
    else if(afstandRechtsVoor > (afstandLinksVoor + NAUWKEURIGHEID)){
      //Draai naar rechts om de hand te volgen
      motors.setLeftSpeed(LEFTSPEED);
      motors.setRightSpeed((-RIGHTSPEED));
      return;
    }
    else if(afstandLinksVoor > (afstandRechtsVoor + NAUWKEURIGHEID)){
      //Draai naar links om de hand te volgen
      motors.setLeftSpeed((-LEFTSPEED));
      motors.setRightSpeed(RIGHTSPEED);
      return;
    }

    //Wanneer de auto zich bevind in het gebied tussen de Stopafstand en Volgafstand blijft deze stil staan
    //Dit is om heen en weer rijden te voorkomen
    if(((afstandRechtsVoor < VOLGAFSTAND) && (afstandRechtsVoor > STOPAFSTAND)) || ((afstandLinksVoor < VOLGAFSTAND) && (afstandLinksVoor > STOPAFSTAND)))
    {
      motors.setLeftSpeed(0);
      motors.setRightSpeed(0);
      Serial.println("Stop met rijden"); 
    }
  }
}

int Afstand(int sensor)
{
  //De eerste keer dat deze functie wordt aangeroepen ijkt hij de waardes van de sensoren.
  static int sensorLinksIjk = SensorAfstand(SENSORLINKS);
  static int sensorRechtsIjk = SensorAfstand(SENSORRECHTS);
  static int sensorLinksVoorIjk = SensorAfstand(SENSORLINKSVOOR);
  static int sensorRechtsVoorIjk = SensorAfstand(SENSORRECHTSVOOR);

  int afstand = 0;
  
  if(sensor == SENSORLINKS){
    //Door de ijkwaarde van de sensorwaarde af te halen wordt de uiteindelijke waarde ~0 als er niks gedetecteerd word.
    afstand = (SensorAfstand(sensor)-sensorLinksIjk);
  }
  else if(sensor == SENSORRECHTS){
    //Door de ijkwaarde van de sensorwaarde af te halen wordt de uiteindelijke waarde ~0 als er niks gedetecteerd word.
    afstand = (SensorAfstand(sensor)-sensorRechtsIjk);
  }
  else if(sensor == SENSORLINKSVOOR){
    afstand = (SensorAfstand(sensor)-sensorLinksVoorIjk);
  }
  else if(sensor == SENSORRECHTSVOOR){
    afstand = (SensorAfstand(sensor)-sensorRechtsVoorIjk);
  }
  return afstand;
}

int DraaiRechts(unsigned long draaiMillis)
{
  //static unsigned long draaiMillis = currentMillis;
  static int draaiTijd = 590;
  
  if((currentMillis - draaiMillis) <= draaiTijd)
  {
    Serial.println("Draai kort naar rechts");
    motors.setRightSpeed((-RIGHTSPEED));
    motors.setLeftSpeed(LEFTSPEED);
    return true;
  }
  else if((currentMillis - draaiMillis) <= 2000)
  {
    Serial.println("Rechtdoor");
    motors.setRightSpeed(RIGHTSPEED);
    motors.setLeftSpeed(LEFTSPEED);
    return true;
  }
  else if((currentMillis - draaiMillis) <= (2000+draaiTijd)){
    Serial.println("Draai nog een keer naar rechts");
    motors.setRightSpeed((-RIGHTSPEED));
    motors.setLeftSpeed(LEFTSPEED);
    return true;
  }
  else if((currentMillis - draaiMillis) <= 3200)
  {
    Serial.println("Rij weer het pad in");
    motors.setRightSpeed(RIGHTSPEED);
    motors.setLeftSpeed(LEFTSPEED);
    return true;
  }
  else{
    //Klaar met draai maneuvre
    return false;
  }
}

  
int Pathfinding()
{
  int afstandLinks = Afstand(SENSORLINKS);
  int afstandRechts = Afstand(SENSORRECHTS);
  static unsigned long draaiMillis = 0;
  static int draaien = false;
  static int inBocht = false;
  static int bocht = false;
  static int hoogsteLinks = 0;
  static int hoogsteRechts = 0;
  static int laagsteLinks = 0;
  static int laagsteRechts = 0;
  static int aantalBochten = 0;

  if(draaien == false && inBocht == false){
    if(afstandLinks > (afstandRechts + NAUWKEURIGHEID)){
      //hoogsteRechts = 0;
      //Hier stuur je de motor naar rechts aan, voor nu nog even een led.
      if(afstandLinks > hoogsteLinks){
        hoogsteLinks = afstandLinks;
        Serial.println("draai naar rechts");
            motors.setLeftSpeed(LEFTSPEED);
            motors.setRightSpeed(RIGHTSPEED/2.5);
      }
      else{
          motors.setLeftSpeed(LEFTSPEED);
          motors.setRightSpeed(RIGHTSPEED);
          hoogsteLinks = afstandLinks;
          Serial.println("Rechtdoor 2");
      }
    }
    else if(afstandRechts > (afstandLinks + NAUWKEURIGHEID)){
      //hoogsteLinks = 0;
      //Hier stuur je de motor naar links aan, voor nu nog even een led.
      if(afstandRechts > hoogsteRechts){
        hoogsteRechts = afstandRechts;
        Serial.println("draai naar links");
            motors.setLeftSpeed(LEFTSPEED/2.5);
            motors.setRightSpeed(RIGHTSPEED);
      }
      else{
          motors.setLeftSpeed(LEFTSPEED);
          motors.setRightSpeed(RIGHTSPEED);
          hoogsteRechts = afstandRechts;
          Serial.println("Rechtdoor 1");
      }
    }
    else if((afstandLinks < NAUWKEURIGHEID) || (afstandRechts < NAUWKEURIGHEID)){
      motors.setLeftSpeed(0);
      motors.setRightSpeed(0);
      Serial.println("STOP");
    }
    else{
      motors.setLeftSpeed(LEFTSPEED);
      motors.setRightSpeed(RIGHTSPEED);
      Serial.println("Rechtdoor");
    }
  }


  //Code om te kijken wanneer hij een hoek om moet
  if(((Afstand(SENSORLINKSVOOR) > 350) || (Afstand(SENSORRECHTSVOOR) > 350)) && (draaien == false)){
    Serial.println("We rijden tegen een muur we moeten draaien");
    laagsteLinks = afstandLinks;
    hoogsteLinks = 0;
    draaiMillis = currentMillis;
    draaien = true;
  }
  
  if(draaien == true){
    draaien = DraaiRechts(draaiMillis);  
    /*motors.setLeftSpeed(LEFTSPEED/1.5);
    motors.setRightSpeed(-RIGHTSPEED/1.5);
    if(afstandLinks < laagsteLinks){
        laagsteLinks = afstandLinks;
    }
    if(afstandLinks > (laagsteLinks + NAUWKEURIGHEID)){
      hoogsteLinks = afstandLinks;
    }
    if(afstandLinks <= (hoogsteLinks-3)){
      Serial.println("Ver genoeg gedraaid ga weer verder");
      motors.setLeftSpeed(LEFTSPEED);
      motors.setRightSpeed(RIGHTSPEED);
      aantalBochten++;
      inBocht = true;
      draaien = false;
      hoogsteLinks = 0;
      hoogsteRechts = 0;
    }
  }
  if(inBocht == true){
    if(bocht == false){
      motors.setLeftSpeed(LEFTSPEED);
      motors.setRightSpeed(RIGHTSPEED);
    }
    if(afstandRechts > hoogsteRechts){
      hoogsteRechts = afstandRechts;
    }
    if(afstandRechts < (hoogsteRechts - NAUWKEURIGHEID)){
      bocht = true;
      motors.setLeftSpeed(LEFTSPEED/1.5);
      motors.setRightSpeed(-RIGHTSPEED/1.5);
      delay(1000);
      bocht = false;
      inBocht = false;
    }*/
  }
  
  
}

void setup() 
{
  Serial.begin(9600);
  delay(1000);
  StartSensoren();
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
  SensorSelect(SENSORLINKSVOOR);
  if (! vcnl.begin()){
    Serial.println("Sensor niet gevonden:");
    while (1);
  }
  Serial.println("VCNL4010 Linksvoor gevonden");
  SensorSelect(SENSORRECHTSVOOR);
  if (! vcnl.begin()){
    Serial.println("Sensor niet gevonden:");
    while (1);
  }
  Serial.println("VCNL4010 Rechtsvoor gevonden");;
}



void loop() 
{
  currentMillis = millis();
  //motorActie 1 = links, 2 = rechts, 0 is niks of rechtdoor.
  //int motorActie = Pathfinding();
  int volgActie = VolgModus();
  //DraaiRechts();
  //delay(100);
}
