#define PWMPin 9 
#define SensorPin A0
#define Lectura_pote_low  145
#define Lectura_pote_high 868
#define Frec_muestreo_pote 100

unsigned long time1 = 0;
unsigned long time2 = 0;
int val = 0;
float deg = 0;
int OCR1A_min = 1100;
int OCR1A_max = 5000;
int OCR1A_mid = (OCR1A_max + OCR1A_min) / 2;
int angle;
int angle_map;
int lectura_pote;

void setup() {
  Serial.begin(115200); //Inicializacion de puerto serie
  pinMode(PWMPin, OUTPUT);  //Configuracion del pin 9 como salida

  TCNT1 = 0;
  
  PWM_50Hz();   //Configuracion e inicializacion del timer 1 para generar PWM
  angle_2_servo(-30);
}

void loop() {
  
  time1 = micros();
  
  lectura_pote = analogRead(SensorPin);

  angle_map = pote_2_angle();

  //angle_2_servo(angle_map);

  Serial.println(angle_map);

  //Serial.println(lectura_pote);

  int aux = 1000000/Frec_muestreo_pote;
  time2 = micros();

  delayMicroseconds(aux - (time2 - time1));   //Delay necesario para el muestreo a la frecuencia Frec_muestreo_pote
}


void PWM_50Hz(){
  //Configuracion de los registros para un Fast PWM de 50Hz

  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A = (1 << COM1A1) | (1 << WGM11) ;
  TCCR1B = (1 << CS11) | (1 << WGM13) | (1 << WGM12) ;
  ICR1 = 39999;
}


int pote_2_angle (){

  int lectura_pote = analogRead(SensorPin);
    //Se limita el rango de valores del potenciometro para un rango de entre -90° y 90°
  //if (lectura_pote <= Lectura_pote_low)
    //lectura_pote = Lectura_pote_low;
  //if (lectura_pote >= Lectura_pote_high)
    //lectura_pote = Lectura_pote_high; 

  int aux2 = map(lectura_pote, Lectura_pote_low, Lectura_pote_high, -90, 90);  

  return aux2;

}

//Retorna el valor de OICRA
void angle_2_servo(int angle){

  OCR1A = map(angle, -90, 90, OCR1A_max, OCR1A_min); 
}
