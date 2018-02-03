/* 29/10/2017 Created by Andrea Petrella */
/* Programma per gestire un robot capace di uscire automaticamente da un labirinto. Non basato su Millis(), ho solo modificato l'algoritmo di scansione per far centrare il robot rispetto allo spazio delle pareti*/
/* SCHEMA DI FUNZIONAMENTO:

  ARDUINO
  -> DC MOTORS controllati con modulo "L298N"
    --> 4 MOTORI PER IL MOVIMENTO DELLE RUOTE CONTROLLATI A 2 A 2 PER LATO
  -> MICRO SERVO MOTOR MODEL "SG90"
    --> 1 MOTORE PER MUOVERE DI 180° IL SENSORE AD ULTRASUONI
  -> ULTRASOUND SENSOR MODEL "HC-SR04"
    --> SENSORE PER VERIFICARE LA PRESENZA DI UN OSTACOLO
  -> IR LINE FIELDS SENSOR
    --> 3 SENSORI PER IDENTIFICARE LA FINE DEL PERCORSO
  -> BATTERY VOLTAGES SENSOR
    --> ANALOGREAD

*/
/* SCHEMA DI CONNESSIONE

  INPUT
  -> ULTRASOUND SENSOR ECHO     - pin  8
  -> ULTRASOUND SENSOR TRIGGER  - pin  9
  -> IR SENSOR 1                - pin A3
  -> IR SENSOR 2                - pin A4
  -> IR SENSOR 3                - pin A5
  -> BATTERY VOLTAGE SENSOR     - pin A6
  OUTPUT
  -> DC DX MOTORS PW            - pin 11
  -> DC DX MOTORS EN1           - pin 12
  -> DC DX MOTORS EN2           - pin 13
  -> DC SX MOTORS PW            - pin  6
  -> DC SX MOTORS EN3           - pin  4
  -> DC SX MOTORS EN4           - pin  5
  -> SERVO MOTOR                - pin 10

*/

/* ALGORITMO DI FUNZIONAMENTO

  Il robot può uscire da un labirinto applicando questo algoritmo:

  Inizio
  - Se la parete a destra è libera, vado a destra
  - Altrimenti
  - - Se la parete frontale è libera, proseguo dritto
  - - Altrimenti mi rigiro e inizio un nuovo controllo
  Fine
  

*/

//Inclusione delle librerie
#include <NewPing.h>
#include <L298NDRIVER.h> //La mia libreria! Serve a gestire i motori DC con la scheda integrata L298N
#include <Servo.h>

//Definizione dei PIN
#define ultraS_echo 8
#define ultraS_trigger 9
#define ir_1 A3
#define ir_2 A4
#define ir_3 A5
#define battery_v A2
#define motorSX_pw 11
#define motorSX_en1 12
#define motorSX_en2 13
#define motorDX_pw 6
#define motorDX_en1 4
#define motorDX_en2 5
#define servoMotor 10

#define MAX_DISTANCE 250 // Distanza massima da scansionare (in centimetri). Il sensore copre distanze massime attorno ai 400-500cm.
#define LIMIT 40 // Distanza limite per fermare il robot prima che colpisca una parete
#define LIMIT_FRONT 18 // Distanza limite per fermare il robot prima che colpisca la parete frontale

/*** COSTANTI ***/
#define rotationTime_base 610
#define k_rot_sx_base 6
#define k_rot_dx_base 9
#define goTime_base 260
#define goDX_base 476
#define goSX_base 476
#define goBack_base 872
#define k_goTime 8.102769
#define k_goDX 9.019412
#define k_goSX 8.801471
#define k_goBack 9.059862

//Drivers
NewPing sonar(ultraS_trigger, ultraS_echo, MAX_DISTANCE);
L298NDRIVER motorDriver;
Servo servoDriver;

/*
*** BATTERY LEVEL ***
* CARICA SINGOLA 1,35-1,36V
* CARICA TOTALE 8,16V
* 
* CARICA TOTALE DI SETTAGGIO VECCHIO 8,12V
* CARICA TOTALE DI SETTAGGIO NUOVO 8,20V

* CARICA TOTALE SINGOLA 1,31-1,33V
* CARICA TOTALE ATTUALE 7,98V
* 
* FUNZIONA!
* 
* 
*/

//Variabili per la regolazione delle rotazioni dei motori
int rotateTime = rotationTime_base; //a batterie cariche
//int rotateTime = 610; //a batterie scariche
int servoZenit = 82;
int scanSx = 177;
int scanDx = 0;
//int goTime = 272; //Delay = Spazio/0,46 --> goTime = delay/2 --> goTime = (250/0,46)/2
int goTime = goTime_base; //Delay = Spazio/0,46 --> goTime = delay/2 --> goTime = (250/0,46)/2
//int goTime = 320; //Delay = Spazio/0,46 --> goTime = delay/2 --> goTime = (250/0,46)/2
int cont_rot = 0;
//int cont_max = 4;
int cont_max = 3;
//int k_rot_sx = 4; //CARICA TOTALE DI SETTAGGIO 8,12V
int k_rot_sx = k_rot_sx_base; //CARICA TOTALE DI SETTAGGIO 8,06V
int k_rot_dx = k_rot_dx_base; //CARICA TOTALE DI SETTAGGIO 8,03V
//int k_rot_dx = 5; //CARICA TOTALE DI SETTAGGIO 8,14V
//int k_rot_dx = 8; //CARICA TOTALE DI SETTAGGIO 8,06V
int offsetDx = 0;
int offsetSx = 0;
double baseBatteria = 8.2; //8,2 Volt
double valueBatteria = 0;
int fattoreBatteria = 0;

//Millis settings
unsigned long old = 0;
unsigned long now = 0;

//Sonar settings

unsigned int sensor_value = 0; // Send ping, get ping time in microseconds (uS).
unsigned int distanceFront = 0;
unsigned int distanceRight = 0;
unsigned int distanceLeft = 0;

void setup() {
  
  pinMode(motorSX_en1, OUTPUT); //output perche' definisce lo stato logico del pin IN1 del modulo L298N
  pinMode(motorSX_en2, OUTPUT); //output perche' definisce lo stato logico del pin IN1 del modulo L298N
  pinMode(motorDX_en1, OUTPUT); //output perche' definisce lo stato logico del pin IN2 del modulo L298N
  pinMode(motorDX_en2, OUTPUT); //output perche' definisce lo stato logico del pin IN2 del modulo L298N
  pinMode(motorSX_pw, OUTPUT);  //output perche' definisce il valore PWM del pin ENA del modulo L298N
  pinMode(motorDX_pw, OUTPUT);  //output perche' definisce il valore PWM del pin ENB del modulo L298N

  pinMode(battery_v, INPUT);  //sensore di tensione della batteria

  //Lego il driver del servo motore e lo posiziono allo zenit
  servoDriver.attach(servoMotor);
  servoDriver.write(servoZenit);
  
  stopRobot(); //Fermo i motori
  delay(3000); //Aspetta prima di iniziare il programma

}

void loop() {

  //while(true){ //Controllo se il robot è posizionato a terra con i sensori IR
  while(!scanColor()){ //Controllo se il robot è posizionato a terra con i sensori IR

    scanWall(); //Avvia la scansione delle pareti
    delay(200); //Attesa per la stabilizzazione delle tensioni (anti rimbalzo)
    
  }
  
  //Se il robot non è posizionato a terra lo fermo
  servoDriver.write(servoZenit);
  stopRobot();
  delay(2000);

}

/*** METODI DRIVER PER I MOVIMENTI ***/

//Metodo per far muovere il robot in avanti
void goRobot(){

  //A causa delle diverse resistenze elettriche dei motori, il movimento dei motori di destra è meno agevolato quindi va data più potenza per ottenere un movimento perpendicolare alle pareti
  /*int potenzaDx = 200 + offsetDx + fattoreBatteria; //a batterie scariche
  int potenzaSx = 150 + offsetSx + fattoreBatteria; //a batterie scariche */
  int potenzaDx = 212 + offsetDx + fattoreBatteria; //A batterie cariche
  int potenzaSx = 150 + offsetSx + fattoreBatteria; //A batterie cariche

  if(potenzaDx > 255)
    potenzaDx = 255;
  if(potenzaSx > 255)
    potenzaSx = 255;

  motorDriver.setForwardMove(motorDX_en1, motorDX_en2);
  motorDriver.setForwardMove(motorSX_en1, motorSX_en2);
  
  motorDriver.setPower(motorDX_pw, potenzaDx);
  motorDriver.setPower(motorSX_pw, potenzaSx);
  
}

//Metodo per fermare il robot
void stopRobot(){

  int potenza = 0;
  
  //Blocco i motori delle ruote
  motorDriver.stopMotor(motorDX_en1, motorDX_en2);
  motorDriver.stopMotor(motorSX_en1, motorSX_en2);
  
  motorDriver.setPower(motorDX_pw, potenza);
  motorDriver.setPower(motorSX_pw, potenza);

}

//Metodo per far muovere il robot verso destra
void goDXRobot(){

  int potenza = 230;
  //int rotateDXTime = rotateTime + (rotateTime*k_rot_dx/100);
  int rotateDXTime = dipendentValue(goDX_base, k_goDX);
  
  motorDriver.setRotateDX(motorSX_en1, motorSX_en2, motorDX_en1, motorDX_en2);
  
  motorDriver.setPower(motorDX_pw, potenza);
  motorDriver.setPower(motorSX_pw, potenza);

  //delay(rotateTime); //A batterie scariche
  delay(rotateDXTime); //A batterie cariche
  
}

//Metodo per far muovere il robot verso sinistra
void goSXRobot(){

  int potenza = 230;
  //int rotateSXTime = rotateTime + (rotateTime*k_rot_sx/100);
  int rotateSXTime = dipendentValue(goSX_base, k_goSX);

  motorDriver.setRotateSX(motorSX_en1, motorSX_en2, motorDX_en1, motorDX_en2);
  
  motorDriver.setPower(motorDX_pw, potenza);
  motorDriver.setPower(motorSX_pw, potenza);

  delay(rotateSXTime);
  
}

//Metodo per far muovere il robot verso destra a batterie cariche
void goBackRobot2(){

  //int k_batterie = 80; //8.12V
  int k_batterie = 60; //8.06V
  
  int potenza = 230;
  //int rotateDXTime = (rotateTime + (rotateTime*k_rot_dx/100)) *2 -k_batterie; //8.06V
  int rotateBackTime = dipendentValue(goBack_base, k_goBack);

  motorDriver.setRotateDX(motorSX_en1, motorSX_en2, motorDX_en1, motorDX_en2);
  
  motorDriver.setPower(motorDX_pw, potenza);
  motorDriver.setPower(motorSX_pw, potenza);

  //delay(rotateTime); //A batterie scariche
  //delay(rotateDXTime); //A batterie cariche
  delay(rotateBackTime);
  
}

//Metodo per far girare su se stesso il robot
void goBackRobot(){

  //Gira due volte a destra
  for(int i = 0; i < 2; i++){
    goDXRobot();
    stopRobot();
  }
  
}

/*** LOGICA PER LA RISOLUZIONE DEI LABIRINTI ***/

double dipendentValue(int base, double k){

  double newValue = base * k /valueBatteria;
  
  return newValue;
  
}

//Metodo per controllare il corretto posizionamento a terra del robot
boolean scanColor(){

  //Acquisisco il valore dei sensori IR
  boolean finish = digitalRead(ir_1) && digitalRead(ir_2) && digitalRead(ir_3);
  
  return finish;
  
}

//Metodo per controllare il livello di tensione della batteria
void scanVoltage(){

  //Acquisisco il valore del sensore
  int value = analogRead(battery_v);
  delay(100);
  double volt = ((value + 1) * 2 * 4.9) / 1000; //Volt
  valueBatteria = volt;
  goTime = dipendentValue(goTime_base, k_goTime);
  
}

//Metodo per gestire la scansione delle pareti del labirinto
void scanWall(){
  
  
  //Fermo il robot
  stopRobot();
  delay(100);
  scanVoltage(); //Controllo la carica della batteria
  delay(100);
    
  //Acquisisco il valore del sensore ad ultra suoni
  sensor_value = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  distanceFront = sensor_value / US_ROUNDTRIP_CM;
  delay(100); //Attende per elaborare la risposta del sonar

  //Controllo se il valore è oltre il limite consentito per il movimento
  if(distanceFront <= LIMIT_FRONT){
//  if(distanceFront <= LIMIT_FRONT && distanceFront > 0){

    cont_rot = 0; //Resetto il conteggio dell'avanzamento prima di girare a destra
        
    //Controllo se la parete dx non è libera
    if(scanLateral(true, scanDx)){
  
      //Controlla se la parete sx non è libera
      if(scanLateral(false, scanSx)){
        
        //Vicolo cieco, faccio girare il robot su se stesso
        servoDriver.write(servoZenit);
        //goBackRobot(); //A batterie scariche
        goBackRobot2(); //A batterie cariche forse non va bene
        
      }
      else{
  
        //"L" verso sinistra
        /* L'algoritmo originale vorrebbe che il robot si girasse su se stesso per poi ricontrollare se andare a destra o meno,
           questo porterebbe successivamente comunque ad andare a sinistra quindi effettuo una predizione migliorando l'algoritmo andando direttamente a sinistra */
        servoDriver.write(servoZenit);
        goSXRobot();
        goRobot();
        delay(goTime/2);
        
      }
      
    }
  
    else{
    
      //Vado a destra
      servoDriver.write(servoZenit);
      goDXRobot();
      cont_rot = 0;
      //Proseguo un po'
      goRobot();
      delay(goTime/2);
      
    }

  }
  
  else{

    //Controllo se la parete dx non è libera
    if(scanLateral(true, scanDx)){
  /*
      //Controlla se la parete sx non è libera
      if(scanLateral(false, scanSx)){

        if(distanceLeft > 0){

          if(distanceRight > distanceLeft){
  
            regolaSterzo(0);
  
          }
          else if(distanceRight < distanceLeft){
  
            regolaSterzo(1);
  
          }
  
          else{
  
            regolaSterzo(2);
  
          }
          
        } 
      }
      else{
        regolaSterzo(2);
      }*/

    }
    else{
    
      //Vado a destra
      servoDriver.write(servoZenit);
      //Faccio girare immediatamente solo se sono andato un po' avanti prima
      if((cont_rot >= cont_max)){
        
        goDXRobot();
        cont_rot = 0;
        //Proseguo un po'
        goRobot();
        delay(goTime/2);
        //delay(2*goTime);
      }
      else{//Altrimenti avanzo
        //goRobot();
        //delay(goTime/2);
        cont_rot++;
      }

    }

  }

  /*** TEST ***/
    
  //Vado avanti
  servoDriver.write(servoZenit);
  goRobot();
  delay(goTime/2);
  
}

//Metodo per controllare le pareti laterali
boolean scanLateral(boolean side, int rotation){

  //Ruota la testa del robot
  servoDriver.write(rotation);
  delay(1000); //Da il tempo alla testa di girare
  //Scansiona
  sensor_value = sonar.ping(); // Send ping, get ping time in microseconds (uS)
  unsigned int distance = sensor_value / US_ROUNDTRIP_CM;
  if(side)
    distanceRight = distance;
  else
    distanceLeft = distance;
  delay(100); //Attende per elaborare la risposta del sonar

  return (distance <= LIMIT  && distance > 0);
  
}

void regolaSterzo(int side){

  int c = 5;
  int k = 12;

  switch(side){

    //Vado a DX
    case 0:
      offsetDx = -c;
      offsetSx = c;
    //Vado a SX
    case 1:
      offsetDx = c*k;
      offsetSx = -c*k;

    default:
      offsetDx = 0;
      offsetSx = 0;
    
  }
  
}












