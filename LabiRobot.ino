/* 13/10/2017 Created by Andrea Petrella */
/* Programma per gestire un robot capace di uscire da solo da un labirinto. */
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

*/
/* SCHEMA DI CONNESSIONE

  INPUT
  -> ULTRASOUND SENSOR ECHO     - pin  8
  -> ULTRASOUND SENSOR TRIGGER  - pin  9
  -> IR SENSOR 1                - pin A3
  -> IR SENSOR 2                - pin A4
  -> IR SENSOR 3                - pin A5
  OUTPUT
  -> DC DX MOTORS PW            - pin 11
  -> DC DX MOTORS EN1           - pin 12
  -> DC DX MOTORS EN2           - pin 13
  -> DC SX MOTORS PW            - pin  6
  -> DC SX MOTORS EN3           - pin  4
  -> DC SX MOTORS EN4           - pin  5
  -> SERVO MOTOR                - pin 10

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
#define motorSX_pw 11
#define motorSX_en1 12
#define motorSX_en2 13
#define motorDX_pw 6
#define motorDX_en1 4
#define motorDX_en2 5
#define servoMotor 10

#define MAX_DISTANCE 40 // Distanza massima da scansionare (in centimetri). Il sensore copre distanze massime attorno ai 400-500cm.
#define LIMIT 25 // Distanza limite per fermare il robot prima che colpisca una parete

//Drivers
NewPing sonar(ultraS_trigger, ultraS_echo, MAX_DISTANCE);
L298NDRIVER motorDriver;
Servo servoDriver;

//Variabili per la regolazione delle rotazioni dei motori
int rotateTime = 600;
int servoZenit = 78;
int scanSx = 177;
int scanDx = 1;

void setup() {
  
  pinMode(motorSX_en1, OUTPUT); //output perche' definisce lo stato logico del pin IN1 del modulo L298N
  pinMode(motorSX_en2, OUTPUT); //output perche' definisce lo stato logico del pin IN1 del modulo L298N
  pinMode(motorDX_en1, OUTPUT); //output perche' definisce lo stato logico del pin IN2 del modulo L298N
  pinMode(motorDX_en2, OUTPUT); //output perche' definisce lo stato logico del pin IN2 del modulo L298N
  pinMode(motorSX_pw, OUTPUT);  //output perche' definisce il valore PWM del pin ENA del modulo L298N
  pinMode(motorDX_pw, OUTPUT);  //output perche' definisce il valore PWM del pin ENB del modulo L298N

  //Lego il driver del servo motore e lo posiziono allo zenit
  servoDriver.attach(servoMotor);
  servoDriver.write(servoZenit);
  
  stopRobot(); //Fermo i motori
  delay(3000); //Aspetta prima di iniziare il programma

}

void loop() {

  while(!scanColor()){ //Controllo se il robot è posizionato a terra con i sensori IR
    
    scanWall(); //Avvia la scansione delle pareti
    
  }

  //Se il robot non è posizionato a terra lo fermo
  servoDriver.write(servoZenit);
  stopRobot();
  delay(2000);

}

/*** METODI DRIVER PER I MOVIMENTI ***/

//Metodo per far muovere il robot in avanti
void goRobot(){

  //A causa delle diverse resistenze elettriche dei motori, il movimento di destra è meno agevolato quindi va data più potenza per ottenere un movimento perpendicolare alle pareti
  int potenzaDx = 200;
  int potenzaSx = 160;

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

  motorDriver.setRotateDX(motorSX_en1, motorSX_en2, motorDX_en1, motorDX_en2);
  
  motorDriver.setPower(motorDX_pw, potenza);
  motorDriver.setPower(motorSX_pw, potenza);

  delay(rotateTime);
  
}

//Metodo per far muovere il robot verso sinistra
void goSXRobot(){

  int potenza = 230;

  motorDriver.setRotateSX(motorSX_en1, motorSX_en2, motorDX_en1, motorDX_en2);
  
  motorDriver.setPower(motorDX_pw, potenza);
  motorDriver.setPower(motorSX_pw, potenza);

  delay(rotateTime);
  
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

//Metodo per controllare il corretto posizionamento a terra del robot
boolean scanColor(){

  //Acquisisco il valore dei sensori IR
  boolean finish = digitalRead(ir_1) && digitalRead(ir_2) && digitalRead(ir_3);
  
  return finish;
  
}

//Metodo per gestire la scansione delle pareti del labirinto
void scanWall(){

  //Acquisisco il valore del sensore ad ultra suoni
  unsigned int sensor_value = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  unsigned int distance = sensor_value / US_ROUNDTRIP_CM;
  delay(60); //Attende per elaborare la risposta del sonar

  //Controllo se il valore è oltre il limite consentito per il movimento
  if(distance <= LIMIT && distance > 0){

    //Fermo il robot
    stopRobot();
    delay(100);

    //Controllo se la parete dx non è libera
    if(scanLateral(scanDx)){
      
      //Controlla se la parete sx non è libera
      if(scanLateral(scanSx)){

        //Vicolo cieco, faccio girare il robot su se stesso
        servoDriver.write(servoZenit);
        goBackRobot();
        
      }
      else{

        //"L" verso sinistra
        /* La regola vorrebbe che il robot si girasse su se stesso per poi ricontrollare se,
           questo porterebbe successivamente comunque ad andare a sinistra quindi effettuo una predizione migliorando l'algoritmo andando direttamente a sinistra*/
        servoDriver.write(servoZenit);
        goSXRobot();
        
      }
    }
    else{

      //"T" o "L" verso destra
      servoDriver.write(servoZenit);
      goDXRobot();
      
    }
        
  }
  else{

    //Via libera, va avanti
    servoDriver.write(servoZenit);
    goRobot();
    
  }
  
}

//Metodo per controllare le pareti laterali
boolean scanLateral(int rotation){

  //Ruota la testa del robot
  servoDriver.write(rotation);
  delay(1000); //Da il tempo alla testa di girare
  //Scansiona
  unsigned int sensor_value = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  unsigned int distance = sensor_value / US_ROUNDTRIP_CM;
  delay(60); //Attende per elaborare la risposta del sonar

  return (distance <= LIMIT  && distance > 0);
  
}
















