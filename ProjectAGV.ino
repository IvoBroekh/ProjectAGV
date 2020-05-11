#include <Wire.h>
#include <Adafruit_VCNL4010.h>
Adafruit_VCNL4010 vcnl;
extern "C" { 
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}

#define TCAADDR 0x70   //Het adres van de multiplexer
#define sensorLinks 1      //Verander hier op welk adres je de sensoren hebt gezet
#define sensorRechts 7

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
 
void SensorSelect(int nummer) {
  if (nummer > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << nummer);
  Wire.endTransmission();  
}

void setup() {
  Serial.begin(9600);
  startSensoren();
  Serial.println("VCNL4010 test");

  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(6, INPUT);
  
  SensorSelect(sensorLinks);
  if (! vcnl.begin()){
    Serial.println("Sensor not found :(");
    while (1);
  }
  Serial.println("Found VCNL4010 nummer 1");

  SensorSelect(sensorRechts);
  if (! vcnl.begin()){
    Serial.println("Sensor not found :(");
    while (1);
  }
  Serial.println("Found VCNL4010 nummer 2");
}

//Deze functie returnt het gemiddelde van 5 scans voor stabiliteit. Heeft het sensornummer nodig
int SensorAfstand(int sensorNr){
  unsigned long int average = 0;
  SensorSelect(sensorNr);
  for(int i = 0; i<5; i++){
    average += vcnl.readProximity();
  }
  return(average/5);
}

void loop() {
  //ijk de sensoren omdat ze allemaal een klein beetje afwijken
  static int sensorRechtsijk = SensorAfstand(sensorLinks);
  static int sensorLinksijk = SensorAfstand(sensorRechts);
  
  int afstand1 = (SensorAfstand(sensorLinks)-sensorRechtsijk);
  int afstand2 = (SensorAfstand(sensorRechts)-sensorLinksijk);
  Serial.print("Proximity 1: "); Serial.println(afstand1);
  Serial.print("Proximity 2: "); Serial.println(afstand2);
  if(afstand1 > afstand2+10){
    digitalWrite(8, HIGH);
    digitalWrite(7, LOW);
  }
  else if(afstand2 > afstand1+10){
    digitalWrite(8, LOW);
    digitalWrite(7, HIGH);
  }
  else{
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
  }
  //Serial.println(digitalRead(6));
}
