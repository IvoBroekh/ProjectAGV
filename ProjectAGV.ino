#include <Wire.h>
#include <Adafruit_VCNL4010.h>
#include <ZumoShield.h>
Adafruit_VCNL4010 vcnl;
extern "C" { 
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}
//hallo
//Aron test

ZumoMotors motors;
ZumoBuzzer buzzer;

#define TCAADDR 0x70       //Het adres van de multiplexer

//Verander hieronder op welk adres/pin je de sensoren hebt gezet
#define SENSORLINKS 2      
#define SENSORRECHTS 7
#define BOOMLINKS 11
#define BOOMRECHTS 5
#define SENSORLINKSVOOR 0
#define SENSORRECHTSVOOR 5

//Verschillende vaste waardes voor timingen, snelheden en afstanden.
#define LEFTSPEED 150
#define RIGHTSPEED 150
#define HANDTIMING 200
#define VOLGAFSTAND 300
#define STOPAFSTAND 150
#define NAUWKEURIGHEID 10  //Nauwkeurigheid van de sensoren op een schaal van 10-50, dichter bij de 10 is nauwkeuriger maar kan valse metingen geven.

//Sorry maar ik maak toch echt een globale variabele voor de tijd.
unsigned long currentMillis = 0;
volatile int noodstop = false;

//Deze functie initialiseert de sensoren
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

//Deze functie checkt of hij een boom ziet, zo ja returneert hij 1 anders 0.
//Ook moet je aangeven welke zijde je scant, links is 1 en rechts is 0
int KijkBoom(int zijde)
{
  static int boom = 0;
  //kant 1 is links
  if(zijde == 1){
  boom = digitalRead(BOOMLINKS);
  delay(1);
    if(boom == 1){
     Serial.print("Er is een boom links");Serial.println(boom);
      return boom;
    }
  }
  //kant 0 is rechts
  else if(zijde == 0){
  boom = digitalRead(BOOMRECHTS);
  delay(1);
    if(boom == 1){
     Serial.print("Er is een boom rechts");Serial.println(boom);
      return boom;
    }
  }
  return boom;
}

//Deze functie zorgt ervoor dat een hand gevolgd kan worden.
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
    else if((afstandRechtsVoor < (STOPAFSTAND)) && (afstandLinksVoor < (STOPAFSTAND)) && (afstandRechtsVoor > 50) && (afstandLinksVoor > 50))
    {
      //Te ver weg rijd dichterbij.
      motors.setLeftSpeed(LEFTSPEED);
      motors.setRightSpeed(RIGHTSPEED);
    }
    else if(afstandRechtsVoor > (afstandLinksVoor + NAUWKEURIGHEID*5)){
      //Draai naar rechts om de hand te volgen
      motors.setLeftSpeed(LEFTSPEED);
      motors.setRightSpeed((-RIGHTSPEED));
      return;
    }
    else if(afstandLinksVoor > (afstandRechtsVoor + NAUWKEURIGHEID*5)){
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

//Deze functie is gemaakt zodat het makkelijk is om een afstand van de proximity sensor op te vragen.
//Geef aan van welke sensor je het wilt weten en de functie returned de afstandswaarde.
int Afstand(int sensor)
{
  //De eerste keer dat deze functie wordt aangeroepen ijkt hij de waardes van de sensoren.
  //Omdat de belichting erg verschilt op verschillende locaties doet hij 1 meting en zet dit als nulpunt.
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
    //Door de ijkwaarde van de sensorwaarde af te halen wordt de uiteindelijke waarde ~0 als er niks gedetecteerd word.
    afstand = (SensorAfstand(sensor)-sensorLinksVoorIjk);
  }
  else if(sensor == SENSORRECHTSVOOR){
    //Door de ijkwaarde van de sensorwaarde af te halen wordt de uiteindelijke waarde ~0 als er niks gedetecteerd word.
    afstand = (SensorAfstand(sensor)-sensorRechtsVoorIjk);
  }
  return afstand;
}

//Wanneer deze functie word aangeroepen begint hij een cyclus om naar rechts te draaien.
int DraaiRechts(unsigned long draaiMillis)
{
  //Omdat het draaien op timing werkt moet de draaitijd aangepast worden elke keer dat de batterijen leeg raken of wanneer de motorsnelheid veranderd word.
  static int draaiTijd = 550;
  
  if((currentMillis - draaiMillis) <= draaiTijd)
  {
    //Draai 90 graden naar rechts
    motors.setRightSpeed((-RIGHTSPEED));
    motors.setLeftSpeed(LEFTSPEED);
    return true;
  }
  else if((currentMillis - draaiMillis) <= draaiTijd*3.5)
  {
    //Rij rechtdoor totdat je bij het volgende pad bent.
    motors.setRightSpeed(RIGHTSPEED);
    motors.setLeftSpeed(LEFTSPEED);
    return true;
  }
  else if((currentMillis - draaiMillis) <= draaiTijd*4.5){
    //Draai nogmaals 90 graden rechts
    motors.setRightSpeed((-RIGHTSPEED));
    motors.setLeftSpeed(LEFTSPEED);
    return true;
  }
  else if((currentMillis - draaiMillis) <= draaiTijd*5.5)
  {
    //Rij rechtdoor tot je weer in het pad bent.
    motors.setRightSpeed(RIGHTSPEED);
    motors.setLeftSpeed(LEFTSPEED);
    return true;
  }
  else{
    //Klaar met draai maneuvre
    return false;
  }
}

//Wanneer deze functie word aangeroepen begint hij een cyclus om naar links te draaien.
//In principe precies hetzelfde als DraaiRechts maar dan zijn de motorsnelheden omgedraaid
int DraaiLinks(unsigned long draaiMillis)
{
  static int draaiTijd = 550;
  
  if((currentMillis - draaiMillis) <= draaiTijd)
  {
    Serial.println("Draai kort naar links");
    motors.setRightSpeed(RIGHTSPEED);
    motors.setLeftSpeed((-LEFTSPEED));
    return true;
  }
  else if((currentMillis - draaiMillis) <= draaiTijd*3.5)
  {
    Serial.println("Rechtdoor");
    motors.setRightSpeed(RIGHTSPEED);
    motors.setLeftSpeed(LEFTSPEED);
    return true;
  }
  else if((currentMillis - draaiMillis) <= (draaiTijd*4.5)){
    Serial.println("Draai nog een keer naar links");
    motors.setRightSpeed(RIGHTSPEED);
    motors.setLeftSpeed((-LEFTSPEED));
    return true;
  }
  else if((currentMillis - draaiMillis) <= draaiTijd*5.5)
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


//Deze functie heeft veel veriabelen in is behoorlijk groot omdat hij alles regelt qua pathfinding.
//Ook regelt deze functie wanneer er gedraaid moet worden, en aan welke kant de agv moet kijken voor bomen.
int Pathfinding()
{
  int afstandLinks = Afstand(SENSORLINKS);
  int afstandRechts = Afstand(SENSORRECHTS);
  static unsigned long draaiMillis = 0;
  static int draaien = false;
  static int inBocht = false;
  static int bocht = false;
  static int kant = 0;
  static int hoogsteLinks = 0;
  static int hoogsteRechts = 0;
  static int laagsteLinks = 0;
  static int laagsteRechts = 0;
  static int aantalBochten = 0;
  static int boom = 0;
  static int boomSpotted = 0;

  
  if((draaien == false) && (inBocht == false) && (boom == false)){
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
          Serial.println("Rechtdoor 2");
      }
    }
    else if(afstandRechts > (afstandLinks + NAUWKEURIGHEID)){
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
          Serial.println("Rechtdoor 1");
      }
    }
    else if((afstandLinks < NAUWKEURIGHEID) || (afstandRechts < NAUWKEURIGHEID)){
      motors.setLeftSpeed(LEFTSPEED);
      motors.setRightSpeed(RIGHTSPEED);
      Serial.println("STOP");
    }
    else{
      motors.setLeftSpeed(LEFTSPEED);
      motors.setRightSpeed(RIGHTSPEED);
      Serial.println("Rechtdoor");
    }
  }

  //Check voor bomen als je niet aan het draaien of in een bocht bent.
  if((draaien == false)){
    boom = KijkBoom(kant);
    if(boom == 1 && boomSpotted == 0){
      motors.setLeftSpeed(LEFTSPEED);
      motors.setRightSpeed(RIGHTSPEED);
      boomSpotted = 1;
    }
    else if(boomSpotted == 1 && boom == 0){
      motors.setLeftSpeed(0);
      motors.setRightSpeed(0);
      buzzer.playNote(NOTE_E(4),500, 10);
      delay(1000);
      boomSpotted = 0;
    }
  }

  //Code om te kijken wanneer hij een hoek om moet
  if(((Afstand(SENSORLINKSVOOR) > 200) || (Afstand(SENSORRECHTSVOOR) > 200)) && (draaien == false)){
    Serial.println("We rijden tegen een muur we moeten draaien");
    hoogsteLinks = -1000;
    hoogsteRechts = -1000;
    draaiMillis = currentMillis;
    draaien = true;
    aantalBochten++;
    
    //Verander aan welke kant de sensoren moeten kijken voor de bomen
    if(kant == 0){
      kant = 1;
    }
    else if(kant == 1){
      kant = 0;
    }
    if(aantalBochten == 3){
      kant = -1;
    }
  }
  if(aantalBochten == 4){
    RijdTerug(draaiMillis);
  }
  
  if(draaien == true){
    if(aantalBochten == 1){
      draaien = DraaiRechts(draaiMillis);
    }
    else if(aantalBochten == 2){
      draaien = DraaiLinks(draaiMillis);  
    }
    else if(aantalBochten == 3){
      draaien = DraaiRechts(draaiMillis);
    }
    else if(aantalBochten == 5){
      if((currentMillis - draaiMillis) <= 550){
        Serial.println("Draai kort naar rechts");
        motors.setRightSpeed((-RIGHTSPEED));
        motors.setLeftSpeed(LEFTSPEED);
      }
      else{
        while(1);
      }
    }
  }
}

//Deze functie word aangeroepen wanneer de agv het einde van de route heeft bereikt om hem weer naar het begin te laten rijden.
void RijdTerug(unsigned long draaiMillis)
{
  
  int afstandLinks = Afstand(SENSORLINKS);
  static int hoogsteLinks = 0;
  static int draaiTijd = 550;
  
  if((currentMillis - draaiMillis) <= draaiTijd)
  {
    Serial.println("Draai kort naar rechts");
    motors.setRightSpeed((-RIGHTSPEED));
    motors.setLeftSpeed(LEFTSPEED);
  }
  else
  {
    Serial.println(afstandLinks);
    if(afstandLinks > 400){
      if(afstandLinks > hoogsteLinks){
        hoogsteLinks = afstandLinks;
        motors.setLeftSpeed(LEFTSPEED);
        motors.setRightSpeed(RIGHTSPEED/2);
        Serial.println("Draai naar rechts");
      }
       else{
         motors.setLeftSpeed(LEFTSPEED);
         motors.setRightSpeed(RIGHTSPEED);
         hoogsteLinks = afstandLinks;
       }
    }
    else if(afstandLinks < 390){
      motors.setLeftSpeed(LEFTSPEED/1.5);
      motors.setRightSpeed(RIGHTSPEED);
      Serial.println("Draai naar links");
    }
    else{
      motors.setLeftSpeed(LEFTSPEED);
      motors.setRightSpeed(RIGHTSPEED);
    }
  }
  
}

void Noodknop()
{
  delay(100);
  if(noodstop == false){
    noodstop = true;
    Serial.println("noodknop");
  }
  else if(noodstop == true){
    noodstop = false;
    Serial.println("ga verder");
  }
}

//Begin de Serial voor debuggen, zet pinmodes, start de sensoren, kijk of de sensoren het doen en ijk hierna de sensoren.
void setup() 
{
  Serial.begin(9600);
  delay(1000);
  pinMode(BOOMLINKS, INPUT);
  pinMode(BOOMRECHTS, INPUT);
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), Noodknop, FALLING);
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
  Afstand(SENSORLINKS);
  Afstand(SENSORRECHTS);
  Afstand(SENSORLINKSVOOR);
  Afstand(SENSORRECHTSVOOR);

  //Een klein startdeuntje gemaakt om te laten horen dat de sensoren zijn geijkt en dat de agv kan gaan rijden.
  buzzer.playNote(NOTE_E(4),500, 10);
  delay(2000);
  buzzer.playNote(NOTE_E(4),500, 10);
  delay(2000);
  buzzer.playNote(NOTE_E(6),500, 15);
  delay(1000);
}


//Elke loop zet hij de variabele currentMillis gelijk met de tegenwoordige tijd
//Voor nu kan je Pathfinding of VolgModus selecteren door de ander weg te commenten.
void loop() 
{
  currentMillis = millis();
  if(noodstop == false){
    //int motorActie = Pathfinding();
    int volgActie = VolgModus();
  }
  else if(noodstop == true){
    motors.setLeftSpeed(0);
    motors.setRightSpeed(0);
  }
}
