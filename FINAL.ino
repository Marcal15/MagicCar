#include <Servo.h>
#include <SoftwareSerial.h>

Servo ESC;
Servo GIR;

int vel = 0; // Amplitud minima de pulso para tu ESC
int giro = 90;
const int EchoPin = 5;
const int TriggerPin = 6;

/* RemoteXY select connection mode and include library */ 
#define REMOTEXY_MODE__ESP8266_HARDSERIAL 
#include <RemoteXY.h> 

/* RemoteXY connection settings */ 
#define REMOTEXY_SERIAL Serial 
#define REMOTEXY_SERIAL_SPEED 115200 
#define REMOTEXY_WIFI_SSID "Christian" 
#define REMOTEXY_WIFI_PASSWORD "74747474"
#define REMOTEXY_SERVER_PORT 6377

// RemoteXY configurate   
#pragma pack(push, 1) 
uint8_t RemoteXY_CONF[] = 
  { 255,3,0,22,0,52,0,8,24,0,
  3,131,5,3,29,11,2,26,4,0,
  6,16,13,46,2,26,130,1,55,3,
  41,15,2,67,4,60,7,33,7,16,
  26,11,4,160,31,48,68,13,2,26,
  67,4,33,24,33,12,2,26,11 }; 
   
// this structure defines all the variables of your control interface  
struct { 

    // input variable
  uint8_t select_1; // =0 if select position A, =1 if position B, =2 if position C, ... 
  int8_t slider_1; // =0..100 slider position 
  int8_t slider_2; // =-100..100 slider position 

    // output variable
  char text_1[11];  // string UTF8 end zero 
  char text_2[11];  // string UTF8 end zero 

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY; 
#pragma pack(pop)
///////////////////////////////////////////// 
//           END RemoteXY include          // 
///////////////////////////////////////////// 

void setup()  
{ 
  RemoteXY_Init ();  
  
  Serial.begin(115200);
  delay(5000);
  Serial.setTimeout(10);
  
  pinMode(TriggerPin, OUTPUT);
  pinMode(EchoPin, INPUT);
   
  //Activar el ESC
  ESC.writeMicroseconds(2000); 
  delay(5000); //Esperar 5 segundos para hacer la activacion
    
  ESC.attach(9); // MOTOR PIN
  GIR.attach(8); // MOTOR GIRO 
} 

int ping(int TriggerPin, int EchoPin) {
   long duration, distanceCm;
   
   digitalWrite(TriggerPin, LOW);  //para generar un pulso limpio ponemos a LOW 4us
   delayMicroseconds(4);
   digitalWrite(TriggerPin, HIGH);  //generamos Trigger (disparo) de 10us
   delayMicroseconds(10);
   digitalWrite(TriggerPin, LOW);
   
   duration = pulseIn(EchoPin, HIGH);  //medimos el tiempo entre pulsos, en microsegundos
   
   distanceCm = duration * 10 / 292/ 2;   //convertimos a distancia, en cm
   return distanceCm;
}

void loop()  
{  
  RemoteXY_Handler ();

   
  switch(RemoteXY.select_1){
    case 0:
       
       int comprobar = ping(TriggerPin, EchoPin);      

       sprintf(RemoteXY.text_1, "MAGICCAR");
       
       if(comprobar<=100){  
        if (comprobar > 30){
          vel=1500;
        }
      
        vel = RemoteXY.slider_1;
        vel = (vel * 5) + 1500;       
       }
       
       
       sprintf(RemoteXY.text_2, "%d", vel);
            
       ESC.writeMicroseconds(vel); //Generar un pulso con el numero recibido

       
       if(RemoteXY.slider_2 >= 60){  // IZQUIERDA 
            
            giro = 40;
            GIR.write(giro);
        }
        else if(RemoteXY.slider_2 >= 20){  //  IZQUIERDA LEVE 
            
            giro = 70;
            GIR.write(giro);
        }
        else if(RemoteXY.slider_2 >= -20){  // RECTO
            
            giro = 90;
            GIR.write(giro);
        }
        else if(RemoteXY.slider_2 >= -60){  // DERECHA LEVE 
            
            giro = 110;
            GIR.write(giro);
        }
        else if(RemoteXY.slider_2 >= -100){  //  DERECHA
            
            giro = 140;
            GIR.write(giro);
        }

    break;

    case 1:  //STOP//
       sprintf(RemoteXY.text_1, "STOP");
       vel = 1500;
       ESC.writeMicroseconds(vel); //Generar un pulso con el numero recibido
       
       giro = 90;
       GIR.write(giro);

    break;

    case 2:  //SENSOR//
       sprintf(RemoteXY.text_1, "SENSOR");
       int cm = ping(TriggerPin, EchoPin);
       sprintf(RemoteXY.text_2, "%d cm", cm);
       delay(1000);
       
       
    break;

    default: 
       vel = 1500;
       ESC.writeMicroseconds(vel); //Generar un pulso con el numero recibido
       
       giro = 90;
       GIR.write(giro);
    break;
  }

}
