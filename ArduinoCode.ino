/////////// Control Remoto
#include <IRremote.h>
int RECV_PIN = 10;
IRrecv irrecv(RECV_PIN);
decode_results results;
///////////
#include "A4988.h"
#define MOTOR_STEPS 200
#define DIR1 8
#define STP1 9
#define DIR2 6
#define STP2 7
#define DIR3 4
#define STP3 5
#define DIR4 2
#define STP4 3
#define RPM 80
#define RPMC 50
//Definir limit switches
//Motor1 
#define LSD1 12
#define LSU1 11
//Motor2
#define LSD2 A0
#define LSU2 A1
//Motor3
#define LSD3 A2
#define LSU3 A3
//Motor4
#define LSL A6
#define LSR A7

//DRV8834 stepper(MOTOR_STEPS, DIR, STP, M0, M1);
A4988 stepper1(MOTOR_STEPS, DIR1, STP1, LSD1, LSU1); 
A4988 stepper2(MOTOR_STEPS, DIR2, STP2, LSD2, LSU2);
A4988 stepper3(MOTOR_STEPS, DIR3, STP3, LSD3, LSU3);
A4988 stepper4(MOTOR_STEPS, DIR4, STP4, LSR, LSL); //Motor de la base
int mult, b;
int long a;
String str1, str2;

////variables de prueba
boolean flag1 = true;
float pasos =0;
float gama=0;
float gamaAux=0;
float alfaPrima=0;
char tecla=' ';
float d1=0,d2=0;
float d1N=0,d2N=0,d3N=0,d12N=0;
int contador=0;
boolean NewFlag = true;
char datoSerial;
float anguloSerial=0;
////

////acelerometro
#include "Wire.h" // This library allows you to communicate with I2C devices.
const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data
float rollB=0;
float pitchA=0;

char tmp_str[7]; // temporary variable used in convert function
char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
  
}
////
void moveEverythingUp(short stop_pin1,short stop_pin2, short stop_pin3){
  boolean m1=true;
  boolean m2=true;
  boolean m3=true;
  while( m1 or m2 or m3){
    if(m1){
      stepper1.move(2);
      if(digitalRead(stop_pin1) == HIGH)  m1=false; 
    }
    if(m2){
      stepper2.move(2); 
      if(digitalRead(stop_pin2) == HIGH)  m2=false; 
    }
    if(m3){
      stepper3.move(2); 
      if(digitalRead(stop_pin3) == HIGH)  m3=false; 
    }
  }  
}
void centerProyector(){
  boolean tocoIzquierda = false;
  boolean tocoDerecha = false;
  int pasos=0;
  
  while(not(tocoIzquierda)){
    stepper4.move(-2);
    if(analogRead(A7) > 1000) tocoIzquierda = true;
  }
//  while(not(tocoDerecha)){
//    stepper4.move(-2);
//    pasos=pasos+2;
//    if(analogRead(A7) > 1000) tocoDerecha = true;
//    }
  //Pasos necesarios para ir de un extremo al centro.
  //Antes eran 278
  stepper4.move(278);
}
void initialPosition(){
//  stepper1.begin(150,mult);
//  stepper2.begin(150,mult);
//  stepper3.begin(150,mult);
//  stepper4.begin(150,mult);
//  stepper1.moveuntil(LSU1);
//  stepper2.moveuntil(LSU2);
//  stepper3.moveuntil(LSU3);
  moveEverythingUp(LSU1,LSU2,LSU3);
  centerProyector();
//  Serial.println("PROYECTOR CENTRADO");  
//  stepper1.begin(RPM,mult);
//  stepper2.begin(RPM,mult);
//  stepper3.begin(RPM,mult);
//  stepper4.begin(RPM,mult);
}
float distanciaMotor1(float gama, float alfaPrima){
  float d1;
  gama=gama*PI/180;
  alfaPrima=alfaPrima*PI/180;  
  d1 = (487*cos(gama)*sin(alfaPrima))/(2*sqrt(1 - pow(cos(gama),2)*pow(sin(alfaPrima),2))) + (355*sin(alfaPrima)*sin(gama))/(2*sqrt(1 - pow(sin(alfaPrima),2)*pow(sin(gama),2)));
  return d1;
}
float distanciaMotor2(float gama, float alfaPrima){
  float d2;
  gama=gama*PI/180;
  alfaPrima=alfaPrima*PI/180;  
  d2 = (487*cos(gama)*sin(alfaPrima))/(2*sqrt(1 - pow(cos(gama),2)*pow(sin(alfaPrima),2))) - (355*sin(alfaPrima)*sin(gama))/(2*sqrt(1 - pow(sin(alfaPrima),2)*pow(sin(gama),2)));
  return d2;
}
int distanciaAPasos(float d){
    int steps;
    steps = (int)(d*1600/(PI*12.3));
    return steps;
}
int pasosMotorCentral(float gama){
  return (int)(gama*278/30);
}
char teclaPresionada(decode_results *results){
  char past = ' ';
  if (results->value == 16753245){
    past = '1';
  }else if(results->value == 16736925){
    past = '2';
  }else if(results->value == 16769565){
    past = '3';
  }else if(results->value == 16720605){
    past = '4';
  }else if(results->value == 16712445){
    past = '5';
  }else if(results->value == 16761405){
    past = '6';
  }else if(results->value == 16769055){
    past = '7';
  }else if(results->value == 16754775){
    past = '8';
  }else if(results->value == 16748655){
    past = '9';
  }else if(results->value == 16738455){
    past = '*'; //asterisco
  }else if(results->value == 16750695){
    past = '0';
  }else if(results->value == 16756815){
    past = '#'; //gato
  }else if(results->value == 16718055){
    past = 'U'; //arriba
  }else if(results->value == 16716015){
    past = 'L'; //izquierda
  }else if(results->value == 16726215){
    past = 'Y'; //OK
  }else if(results->value == 16734885){
    past = 'R'; //derecha
  }else if(results->value == 16730805){
    past = 'D'; //abajo
  }
  return(past);  
}
/*void deltaAlfaPrima(){
  if (tecla == 'U'){
    alfaPrima++;
  }else if (tecla == 'D'){
    alfaPrima--;
  }else{
    alfaPrima=alfaPrima;
  }
 }*/
/*void deltaGama(){
  
  if(tecla == 'R'){
    gama++; 
  }else if(tecla == 'L'){
    gama--; 
  }else{
    gama=gama;
  }
}*/
void deltaAlfaPrima(float stp){
  if( gama == 0 ){
    if( (tecla=='U') and (digitalRead(LSD1) == LOW) and (digitalRead(LSD2) == LOW)){
      alfaPrima+=stp;
    }else if((tecla=='D') and (digitalRead(LSU1) == LOW) and (digitalRead(LSU2) == LOW)){
      alfaPrima-=stp;
    }else{
      alfaPrima = alfaPrima;
      //Se llegó a alguna posicion extrema
    }
  }else if( gama > 0){
    if( (tecla=='U') and (digitalRead(LSD1) == LOW) and (digitalRead(LSU2) == LOW)){
      alfaPrima+=stp;
    }else if((tecla=='D') and (digitalRead(LSU1) == LOW) and (digitalRead(LSD2) == LOW)){
      alfaPrima-=stp;
    }else{
      alfaPrima = alfaPrima;
      //Se llegó a alguna posicion extrema
    }
  }else if( gama < 0 ){
    if( (tecla=='U') and (digitalRead(LSU1) == LOW) and (digitalRead(LSD2) == LOW)){
      alfaPrima+=stp;
    }else if((tecla=='D') and (digitalRead(LSD1) == LOW) and (digitalRead(LSU2) == LOW)){
      alfaPrima-=stp;
    }else{
      alfaPrima = alfaPrima;
      //Se llegó a alguna posicion extrema
    }
  }else{
    alfaPrima = alfaPrima;
    //No puede entrar aqui ni a PUTAZOS
  }
}
void deltaGama(float stp){
  if( alfaPrima == 0){
    if( (tecla == 'R') and (analogRead(LSR) < 1000)){
      gama+=stp;
    }else if((tecla == 'L') and (analogRead(LSL) < 1000)){
      gama-=stp;
    }else{
      gama=gama;
    }
  }else{
    if( (tecla == 'R') and ((analogRead(LSR) < 1000)) and (digitalRead(LSU2) == LOW) and (digitalRead(LSD1) == LOW) ){
      gama+=stp;
    }else if( (tecla == 'L') and ((analogRead(LSL) < 1000)) and (digitalRead(LSD2) == LOW) and (digitalRead(LSU1) == LOW) ){
      gama-=stp;
    }
    else{
      gama=gama;
    }
  }
}
void acelerometro(){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = (Wire.read()<<8 | Wire.read())-740; // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = (Wire.read()<<8 | Wire.read())-222.14; // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = (Wire.read()<<8 | Wire.read())-194.51; // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)

  rollB =  round(- atan((float)accelerometer_y / sqrt(pow((float)accelerometer_x, 2) + pow((float)accelerometer_z, 2))) * 180 / PI);
  pitchA =  round(- atan(-1 * (float)accelerometer_x / sqrt(pow((float)accelerometer_y, 2) + pow((float)accelerometer_z, 2))) * 180 / PI);

  // print out data
//  Serial.print("aX = "); Serial.print(convert_int16_to_str(accelerometer_x));
//  Serial.print(" | aY = "); Serial.print(convert_int16_to_str(accelerometer_y));
//  Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(accelerometer_z));
//  Serial.print("  beta = "); Serial.print(rollB); 
//  Serial.print(" | alfa = "); Serial.print(pitchA); 
  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
//  Serial.println();
  
  // delay
  //delay(500);
}
void nivelacion(){
  delay(1000);
  acelerometro();
 //Nivelacion Lateral
 if (rollB > 0){
  //bajamos el motor 2 mientras el motor 1 queda fijo
  d2N=355*tan(rollB*PI/180);
  stepper2.move(-round(distanciaAPasos(d2N)));
 } 
 else if(rollB < 0){
   //bajamos el motor 1 mientras el motor 2 queda fijo
  d1N=-355*tan(rollB*PI/180);
  stepper1.move(-round(distanciaAPasos(d1N))); 
 }
 else{
  //Se encuentra nivelado de manera lateral.
 }
  delay(1000);
  acelerometro();
 //Nivelacion Frontral
 if (pitchA > 0){
  d3N = 243.5*tan(pitchA*PI/180)/*+(d1N+d2N)/2*/;
  stepper3.move(-round(distanciaAPasos(d3N)));
 } 
 else if(pitchA < 0){
  d12N = -243.5*tan(pitchA*PI/180);
  stepper1.move(-round(distanciaAPasos(d12N)));
  stepper2.move(-round(distanciaAPasos(d12N)));
 }
 else{
  //Se encuentra nivelado de manera frontal.
 }

}
void moverAnguloSerial(float angulo,float stp){
  //defines el sentido de giro
  if(angulo > 0){
    tecla = 'R';
  }else if(angulo < 0){
    tecla = 'L';
  }else{
    tecla = ' ';
  }

  for(float i=0; i< abs(angulo); i = i + stp){
    
    deltaGama(stp);
    
    stepper1.move(distanciaAPasos(d1-distanciaMotor1(gama,alfaPrima))); 
    stepper2.move(distanciaAPasos(d2-distanciaMotor2(gama,alfaPrima)));
    stepper4.move(pasosMotorCentral(gamaAux-gama)); 
    
    d1 = distanciaMotor1(gama,alfaPrima);
    d2 = distanciaMotor2(gama,alfaPrima);
    gamaAux=gama;    
  }
  tecla = ' ';
}
void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  mult=1; // medios pasos
  //Para la base pasos positivos es izquierda
  pinMode(12,INPUT);
  pinMode(11,INPUT);
  Serial.begin(9600);

  stepper1.begin(RPM,mult);
  stepper2.begin(RPM,mult);
  stepper3.begin(RPM,mult);
  stepper4.begin(RPMC,mult);
  
  stepper1.enable();
  stepper2.enable();
  stepper3.enable();
  stepper4.enable();
  
  stepper1.setRPM(RPM);
  stepper2.setRPM(RPM);
  stepper3.setRPM(RPM);
  stepper4.setRPM(RPMC);
  
// 122 115
//  Espera a que un numero sea ingresado desde el serial para realizar el seteo de los motores. 
//  int tecla = 0;
//  char dato;
//
//  while (tecla == 0){
//    dato = Serial.read();
//    tecla = String(dato).toInt();
//  }

  initialPosition();
  nivelacion();

  pinMode(13,OUTPUT);
  delay(1000);
  /////////// CONTROL REMOTO
  irrecv.enableIRIn(); // Empezamos la recepción  por IR
  ///////////
  
}
void loop() {

  if (irrecv.decode(&results)) {
    tecla=teclaPresionada(&results);
    irrecv.resume(); // empezamos una nueva recepción

    deltaGama(1.0);
    deltaAlfaPrima(1.0);
    
    stepper1.move(distanciaAPasos(d1-distanciaMotor1(gama,alfaPrima))); 
    stepper2.move(distanciaAPasos(d2-distanciaMotor2(gama,alfaPrima)));
    stepper4.move(pasosMotorCentral(gamaAux-gama)); 
    
    d1 = distanciaMotor1(gama,alfaPrima);
    d2 = distanciaMotor2(gama,alfaPrima);
    gamaAux=gama;

    tecla=' ';
    contador = 0;
    NewFlag = true;
  }
  
  //anguloSerial = Serial.parseFloat();
  
  /*if(anguloSerial != 0){
    moverAnguloSerial(anguloSerial,0.5);
  }*/
  
  
  if(NewFlag){
    contador++;
  }
  
  if(contador > 2000){
    Serial.println(alfaPrima);
    contador = 0;
    NewFlag = false;
  }
  
  delay(1);
 
}
