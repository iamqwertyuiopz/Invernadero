const int sensorPin = A0;
int led = 13;
float h;

#include <Stepper.h>
int A=10;
int B=11;
int C=9;
int D=8;
int err;
  float temp;
  float humi;
  const int PasosMotor = 300;  
Stepper Motor(PasosMotor, 8, 9, 10, 11);
#include <DHT11.h>    //incluir librería DHT11
int luz;        //Variable luz del ldr
int foco=3;     //Pin de salida al rele para el funcionamiento del foco
int pin=4;      //Pin de lectura del DHT11
DHT11 dht11(pin);  
int vent=2;      //Pin ventilador
int estado;           //BT
int estado1;          //BT                         

 #include <SoftwareSerial.h>

SoftwareSerial BT(6, 5); // RX, TX

void setup()
{
    pinMode (led,OUTPUT);
    Motor.setSpeed(100);
  pinMode(A,OUTPUT);
  pinMode(B,OUTPUT);
  pinMode(foco,OUTPUT);   //Se declara el foco como salida
  pinMode(D,OUTPUT);      //Se declara el pin 8 (Puente H) como salida
  pinMode(C,OUTPUT);      //Se declara el pin 9 (Puente H) como salida
  pinMode(vent,OUTPUT);   //Se declara el vent como salida
  Serial.begin(9600);     //Se inicia la comunicación serial con el monitor serial
  BT.begin(9600); 
    Motor.step(-2000);

}
 
void loop(){
  h = analogRead(sensorPin);
  
   if(h> 500)
   {
    BT.print("Insuficiencia de agua");   //Imprime palabras "Hace falta agua"
            BT.println();
    digitalWrite(led,HIGH);
      //hacer las acciones necesarias
   }
   else{
        BT.print("Suficiente agua");   //Imprime palabras "Suficiente agua"
            BT.println();
    digitalWrite(led,LOW);
   }
   delay(1000);
if (BT.available()) {         //Pregunta si hay BT
    estado=(BT.read());       //Lee el estado0
        estado1=(BT.read());     //Lee el estado1
  }
   BT.print("Temperatura: ");         //Imprime palabra "Temperatura" en la app
   BT.print(String(temp));  //Imprime el valor de temperatura en la app
   BT.print("°C   ");       //°C
   BT.print("Humedad:  ");          //Imprime palabra "Humedad" en la app
   BT.print(String(humi));  //Imprime el valor de humedad en la app
   BT.print("%");           //%
    BT.println();
  if((err=dht11.read(humi, temp))==0)
  {
    Serial.print("Temperatura:");  // Se imprime la palabra "Temperatura" en el monitor serial
    Serial.print(temp);             //Se muestra el valor de temperatura en C°
    Serial.print(" Humedad:");      //Se imprime la palabra "Humedad" en el monitor serial
    Serial.print(humi);             //Se muestra el valor de humedad en %
    Serial.println();               //Enter
  }
  else {
    Serial.println();
    Serial.print("Error No :");
    Serial.print(err);
    Serial.println();    
  }
  delay(DHT11_RETRY_DELAY); //delay for reread
luz=analogRead(A1);           //Se lee los datos del LDR y se los almacena en una variable entera luz
if(  luz>=700){ 
    BT.println();
       BT.print("Luz encendida ");         //Imprime palabras "Luz apagada"
digitalWrite(foco,LOW);       /*En el caso de que el sensor entregue un valor mayor o igual a 700 (Hay presencia de luz) entonces el foco estará apagado          */}
else{    
   BT.print("Luz apagada");   //Imprime palabras "Luz encendida
       BT.println();
digitalWrite(foco,HIGH);      /*Caso contrario se prenderá (Ausencia de luz)*/  
}      
  if(temp>=28){      // En el caso de que la temperatura sea mayor a 30
        BT.print("Ventilador encendido");   //Imprime palabras "Ventilador encendido"
            BT.println();
        BT.print("Techo abierto");   //Imprime palabras "Techo abierto"
            BT.println();
    digitalWrite(vent,HIGH);  // Se enciende el ventilador
  Motor.step(2000);          /*Se mueve el motor que lleva el techo (Abrir) */          
  }
  else{       //Caso contrario        
    BT.print("Techo cerrado");   //Imprime palabras "Techo abierto"
        BT.println();
    BT.print("Ventilador apagado");   //Imprime palabras "Ventilador apagado"
        BT.println();
       digitalWrite(vent,LOW);       //Se apaga el ventilador
    Motor.step(-2000);                 /*Se mueve el motor que lleva el techo (Cerrar) */    
    }    }
