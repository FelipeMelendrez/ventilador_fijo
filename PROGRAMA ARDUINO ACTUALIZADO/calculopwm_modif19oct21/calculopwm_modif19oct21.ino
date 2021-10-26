//Definir librerías
#include<Wire.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <math.h>

// Constantes del controlador
double Kp=0.2, Ki=0.5, Kd=0.1;
// variables externas del controlador
double Input, Output, Setpoint;
 


//variable auxiliar de tiempo
unsigned long contadorintervalo = 100;
//variables para el click del encoder
const int boton = 4;               //boton conectado al pin 4
const int tiempoAntirebote = 10;
int estadoBoton;
int estadoBotonAnterior = 1;
int bandclick = 0;

int en = 0;
int a = 0;
//variables calibracion oxigeno
int airevalor = 461; //21%
int oxigenovalor = 1023; //100%
int airAddress = 0;
int oxigAddress = 10;

int silenciador = 1;
int bandtrigger = 0;
int auxperilla = 0;
int bandvol = 0;
int bandpres = 0;
int bandtest1 = 0;
int bandtest2 = 0;
int bandtest3 = 0;
int cierrevalvulaaire = 0;
float volFuga = 0;
float volumencalc = 0;
int cierrevalvulaoxigeno = 0;
float auxenc = 10.0;
static int pinA = 2; // Our first hardware interrupt pin is digital pin 2
static int pinB = 3; // Our second hardware interrupt pin is digital pin 3
volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile int encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile int oldEncPos = -1; //tenia 0, lo cambie a -1
//stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the Serial monitor)
volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent
//String encoder="";
float total = 0;
String param = "";
String datoencoder = "";
int bandbipap = 0;
byte X0, X1;
const int flowRange = 200;

//definimos variables de modos ventilatorio
int PiFuga = 0;
int te = 0;
int trigger = 0;
int PEEP = 10;
int presion = 20;
int frec;
int flujo;
int psupp = 0;
float rampa = 0;
int volumen = 0;
int oxigeno;
int aire;
int palta = 0;
int pbaja = 0;
float talto = 0;
float tbajo = 0;

float ti = 0;
float ttotal = 0;
String entrada = ""; //variable para la cadena con los parámetros del ventilador
String entrada1, entrada2; //variables para la cadena de la lectura de los flujómetros
String modo = ""; //variable para el modo ventilatorio
unsigned int adc;

int band = 0; //bandera para saber cuándo está ventilando y cuándo requiere estar en modo "recepción" y ya no lea
//band=0 está detenido el ventilador
//band=1 está ventilando
byte buffer[3];
// Definir interrupciones

// entradas generales
//int APa = A1;    //sensor de presion atmosferica Pa
int APi = A0;    //sensor presion inspiratoria bien CONFIRMADO
int APOx = A3;  //sensor de presión de oxígeno
int APAir = A1;  //sensor de presión de aire BIEN confirmado
//int AFOx = A2; //sensor de flujo de oxígeno
int AFAir = A5; //sensor de flujo de aire
int AOs = A2;    //sensor de oxígeno bien confirmado
int APexh = A4; //sensor de presión exhalatoria bien confirmado
//int AFexh = A2; //sensor de flujo exhalatorio
// salidas generales
int AVPropOx = 13; //valvula proporcional oxigeno
int AVPropAir = 9; //valvula proporcional aire
int AVexh = 29; //valvula exhalatoria (on-off) valvula exhalatoria anterior
//  int AVexh=53; //valvula exhalatoria (on-off)
int alarma = 28; //cambia de 8 a 10
// variables auxiliares
int ANPa;         // variable que almacena el valor raw (0 a 1023)
float ANPi;         // variable que almacena el valor raw (0 a 1023)
int ANPOx;         // variable que almacena el valor raw (0 a 1023)
int ANPAir;        // variable que almacena el valor raw (0 a 1023)
int ANFOx;         // variable que almacena el valor raw (0 a 1023)
int ANFAir;        // variable que almacena el valor raw (0 a 1023)
int ANOs;          //
int ANPexh;        // variable que almacena el valor raw (0 a 1023)
int ANVOx;
int ANVAir;
int ANVexh;
int ANVPropOx;
int ANVPropAir;
int ANFexh;
int bandera = 0;
unsigned long inicio = 0;
unsigned long iniciovol = 0;

float Pa;            // variable que almacena el voltaje (0.0 a 5.0)
float Pi;            // variable que almacena el voltaje (0.0 a 5.0)
float POx = 0;          // variable que almacena el voltaje (0.0 a 5.0)
float PAir = 0;          // variable que almacena el voltaje (0.0 a 5.0)
float FOx = 0;          // variable que almacena el voltaje (0.0 a 5.0)
float FAir = 0;          // variable que almacena el voltaje (0.0 a 5.0)
float Os;            // variable que almacena el voltaje (0.0 a 5.0)
float Pexh;            // variabl que almacena el voltaje (0.0 a 5.0)
float Fexh;

//variables fio2 lpm
float pwmox = 0;
float pwmair = 0;
float lpmair = 0;
float lpmoxig = 0;

void setup()
{
  //pidController.SetSampleTime(200);               // Initialise sample rate in ms (200ms default)

  //Wire.setClock(1000000UL);
  wdt_disable();
  //wdt_enable(WDTO_2S);
  // put your setup code here, to run once:
  Serial.begin(9600); //puerto serie hacia la interfaz flujometro uno
  Wire.begin();                    //Begins the I2C communication
  pinMode(pinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(0, PinA, RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(1, PinB, RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)

  ////PIN AUXILIAR PARA VER QUE SE RECIBAN LOS DATO CORRECTOS DE LA INTERFAZ AL MICROCONTROLADOR
  //pinMode(13,OUTPUT);
  //digitalWrite(13,LOW);

  //definir entradas y salidas
  //  pinMode(APa, INPUT);
  pinMode(APi, INPUT);
  pinMode(APOx, INPUT);
  pinMode(APAir, INPUT);
  //  pinMode(AFOx, INPUT);
  pinMode(AFAir, INPUT);
  pinMode(AOs, INPUT);
  pinMode(APexh, INPUT);
  //  pinMode(AFexh, INPUT);
  // pinMode(AVOx, OUTPUT);
  // pinMode(AVAir, OUTPUT);
  pinMode(AVPropAir, OUTPUT);
  pinMode(AVPropOx, OUTPUT);
  pinMode(AVexh, OUTPUT);
  pinMode(alarma, OUTPUT);

  //Definición de funciones
  float SensorPa(void);
  float SensorPi(void);
  float SensorPOx(void);
  float SensorPAir(void);
  float SensorFOx(void);
  float SensorFAir(void);
  float SensorOs(void);
  float SensorPexh(void);
  float SensorFexh(void);
  void ActuadorVPropOx(int ANVPropOx);
  void ActuadorVPropAir(int ANVPropAir);
  void ActuadorVOx(int ANVOx);
  void ActuadorVAir(int ANVAir);
  void ActuadorVexh(int ANVexh);
  void VCP (int presion, int frec, float ti);
  void VCV (int volumen, int frec, float ti);
  void lecturadatos(void);
  void parametrosvolumen (String entrada);
  void parametrospresion (String entrada);
  //Serial.println("INICIO");
  digitalWrite(AVexh, LOW);
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  digitalWrite(alarma, LOW);
  //lectura de valores de eeprom de calibracion de oxigeno y aire
  EEPROM.get( airAddress, airevalor );
  EEPROM.get( oxigAddress, oxigenovalor );
  pinMode(boton, INPUT);            //declaramos el boton como entrada
}

void loop()
{
  //flujodecreciente();
  lecturadatos();
  // put your main code here, to run repeatedly:
  /*digitalWrite(AVexh,LOW);
    Serial.print("Flujo exh");
    TCA9548A(3); // select I2C bus 1 for the air flow sensor
    Fexh=SensorData();
    Serial.println(Fexh);
    delay(500);
    analogWrite(AVPropOx,0);
    analogWrite(AVPropAir,0);
    Serial.print("Pinspiratoria");
    Serial.println (SensorPi());
    TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
    FOx=SensorData();
    //lectura flujo aire
    TCA9548A(5); // select I2C bus 1 for the air flow sensor
    FAir=SensorData();
    Serial.print("pwmox");
    Serial.println(FOx);
    Serial.print("pwmaire");
    Serial.println(FAir);
    delay(1000);*/
  //a+=50;
  //if (a==250){
  // a=0;
  // }
  //digitalWrite(AVexh,HIGH);
  //lecturadatos(); //descomentar SÓLO esta linea para funcionamiento con la interfaz. para pruebas descomentar lo demas
  //clickencoder(estadoBoton,estadoBotonAnterior);

  //flujo = 30;
  //oxigeno = 80;
  //ecuaciones fio2
  //lpmair = (-flujo * (oxigeno - 100)) / 80;
  //lpmoxig = flujo - lpmair;
  //ecuaciones conversion lpm-pwm
  /*lpmair=37;
    lpmoxig=37;
    //pwmox=0.028*pow(lpmoxig,2)-0.2352*lpmoxig+136; //ecuacion conversion lpm a pwm oxígeno aproximación cuadrática
    pwmox = (2 * pow(10, -8) * pow(lpmoxig, 6)) - (1 * pow(10, -6) * pow(lpmoxig, 5)) - (0.0004 * pow(lpmoxig, 4)) + (0.0387 * pow(lpmoxig, 3)) - (1.3439 * pow(lpmoxig, 2)) + (17.142 * lpmoxig) + 62.711; //ecuacion conversion lpm a pwm oxígeno aproximación cúbica
    pwmair = 3 * pow(10, -9) * pow(lpmair, 6) + 5 * pow(10, -7) * pow(lpmair, 5) - .0002 * pow(lpmair, 4) + .012 * pow(lpmair, 3) - .4009 * pow(lpmair, 2) + 6.9879 * lpmair + 31.917; //ecuacion conversion lpm a pwm aire
    //pwmair=6*pow(10,-8)*pow(lpmair,6)-1*pow(10,-5)*pow(lpmair,5)+0.0008*pow(lpmair,4)-.0188*pow(lpmair,3)+0.0409*pow(lpmair,2)+3.4796*lpmair+62.711; //ecuacion conversion lpm a pwm oxígeno aproximación cúbica
    //pwmox=3*pow(10,-9)*pow(lpmoxig,6)+5*pow(10,-7)*pow(lpmoxig,5)-.0002*pow(lpmoxig,4)+.012*pow(lpmoxig,3)-.4009*pow(lpmoxig,2)+6.9879*lpmoxig+31.917; //ecuacion conversion lpm a pwm aire

    Serial.print ("Flujo teorico oxigeno en pwm ");
    Serial.println(pwmox);
    Serial.print ("Flujo teorico aire en pwm ");
    Serial.println(pwmair);
    Serial.print ("Fio2 oxigeno ");
    Serial.println(lpmoxig);
    Serial.print ("Fio2 aire ");
    Serial.println(lpmair);*/
  //VCP (20,20,1, 6,21,10);
  //pruebasensores();
  /*flujo= 30;
    oxigeno=80;
    //ecuaciones fio2
    lpmair=(-flujo*(oxigeno-100))/80;
    lpmoxig=flujo-lpmair;
    //ecuaciones conversion lpm-pwm
    //pwmox=0.028*pow(lpmoxig,2)-0.2352*lpmoxig+136; //ecuacion conversion lpm a pwm oxígeno aproximación cuadrática
    pwmox=1*pow(10,-10)*pow(lpmair,6)-1*pow(10,-7)*pow(lpmair,5)+5*pow(10,-5)*pow(lpmair,4)-.0115*pow(lpmair,3)+1.306*pow(lpmair,2)-73.326*lpmair+1574.8; //ecuacion conversion lpm a pwm oxígeno aproximación cúbica
    pwmair=-5*pow(10,-11)*pow(lpmair,6)+4*pow(10,-8)*pow(lpmair,5)-1*pow(10,-5)*pow(lpmair,4)+.0017*pow(lpmair,3)-.0973*pow(lpmair,2)+2.2865*lpmair-16.17; //ecuacion conversion lpm a pwm aire

    Serial.print ("Flujo teorico oxigeno en pwm ");
    Serial.println(pwmox);
    Serial.print ("Flujo teorico aire en pwm ");
    Serial.println(pwmair);
    Serial.print ("Fio2 oxigeno ");
    Serial.println(lpmoxig);
    Serial.print ("Fio2 aire ");
    Serial.println(lpmair);

    analogWrite(AVPropAir,int (0));
    analogWrite(AVPropOx,int (0));
    digitalWrite(AVexh,LOW);
    //lectura sensores

    Serial.print("Pinspiratoria ");
    Serial.println (SensorPi());
    Serial.print("Poxigeno ");
    Serial.println (SensorPOx());
    Serial.print("Paire ");
    Serial.println (SensorPAir());
    TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
    FOx=SensorData();
    //lectura flujo aire
    TCA9548A(5); // select I2C bus 1 for the air flow sensor
    FAir=SensorData();
    Serial.print("pwmox ");
    Serial.println(FOx);
    Serial.print("pwmaire ");
    Serial.println(FAir);
    Serial.print("Os ");
    //Serial.println (analogRead(A4));
    Serial.println (SensorOs());
    Serial.print("Pexh ");
    Serial.println (SensorPexh());
    Serial.print("Flujo exh");
    TCA9548A(3); // select I2C bus 1 for the air flow sensor
    Fexh=SensorData();
    Serial.println(Fexh);
    delay(1000);
    analogWrite(AVPropAir,0);
    analogWrite(AVPropOx,0);
    digitalWrite(AVexh,HIGH);
    //lectura sensores

    Serial.print("Pinspiratoria ");
    Serial.println (SensorPi());
    Serial.print("Poxigeno ");
    Serial.println (SensorPOx());
    Serial.print("Paire ");
    Serial.println (SensorPAir());
    TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
    FOx=SensorData();
    //lectura flujo aire
    TCA9548A(5); // select I2C bus 1 for the air flow sensor
    FAir=SensorData();
    Serial.print("pwmox ");
    Serial.println(FOx);
    Serial.print("pwmaire ");
    Serial.println(FAir);
    Serial.print("Os ");
    Serial.println (SensorOs());
    //Serial.println (analogRead(A4));
    Serial.print("Pexh ");
    Serial.println (SensorPexh());
    Serial.print("Flujo exh ");
    TCA9548A(3); // select I2C bus 1 for the air flow sensor
    Fexh=SensorData();
    Serial.println(Fexh);

    delay(1000); */
  /*
    //VCP(40,60,0.5,2,2);
    analogWrite(AVPropAir,118);
    analogWrite(AVPropOx,118);
    digitalWrite(AVexh,HIGH);
    Pi=SensorPi();
    Serial.print("p");
    Serial.println(Pi);
    //Serial.print("f");
    //printSensorData();
    //Serial.println();
    Serial.print("Pexh");
    Serial.println (SensorPexh());
    delay(1000);
    analogWrite(AVPropAir,0);
    analogWrite(AVPropOx,0);
    digitalWrite(AVexh,LOW);
    Serial.print("p");
    Serial.println(Pi);
    Serial.print("Pexh");
    Serial.println (SensorPexh());
    delay(1000);*/
}

// cambio de escala entre floats
float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


float SensorPi()
{
  ANPi = analogRead(APi);          // realizar la lectura
  //Pi = fmap(ANPi, 940, 0, 0.0, 105); //cmh20
  Pi = (((ANPi) - 940) / (-8.95)) - 25;
  return Pi ;

}

float SensorPOx()
{
  ANPOx = analogRead(APOx);          // realizar la lectura
  POx = fmap(ANPOx, 0, 1023, 0, 100); //psi
  return POx ;

}

float SensorPAir()
{
  ANPAir = analogRead(APAir);          // realizar la lectura
  PAir = fmap(ANPAir, 0, 1023, 0, 100); //psi
  return PAir ;

}

float SensorOs()
{
  ANOs = analogRead(AOs);          // realizar la lectura
  Os = fmap(ANOs, airevalor, oxigenovalor, 21, 100); //% oxígeno
  return Os ;
  //1.1v-21
  //4.52 -100%

}

float SensorPexh()
{
  ANPexh = analogRead(APexh);          // realizar la lectura
  //Pexh = fmap(ANPexh, 204.8, 1023, -14, 140); //cmh20
  Pexh = (((ANPexh) - 940) / (-8.95)) - 18;
  return Pexh ;

}

void VCP (int presion, int frec, float ti, int PEEP, int oxigeno, int trigger) { //ver si hay que añadir alguna otra variable
  //Serial.println("Modo presion");
  ttotal = 60000 / frec;
  ti = ti * 1000;
  te = ttotal - ti;
  param = "";
  bandtrigger = 0;
  silenciador = 1;
  do {
    //alarmas suministros gases
    POx = SensorPOx();
    PAir = SensorPAir();
    PiFuga = 0;
    contadorintervalo = 0;
    int inicioflujo = 50;
    int contadorflujo = 0;
    int valflujo = 50;
    int presionalcanzada=0;
    //Controlador
    Setpoint = presion; 
      
    if ((PAir < 0)) {
      // if ((POx<30)||(PAir<10)){
      //if (PAir<30){
      //instruccion para encender buzzer y piloto
      digitalWrite(alarma, HIGH); //ENCIENDE ALARMA
      delay(1000);
      Serial.print("problema");
      entrada = "alto";
      Serial.print(",");
      Serial.print(POx);
      Serial.print(",");
      Serial.println(PAir);
    }
    //inhalación
    digitalWrite(AVexh, LOW);

    //da el porcentaje de acuerdo al fio2 programado
    if (oxigeno == 21)
      calculopwm (0, inicioflujo); //37 LPM aire
    else {
      if (oxigeno == 100) {
        calculopwm (inicioflujo, 0); //37 LPM oxigeno
      }
      else {
        //ecuaciones fio2 con flujo 37, podría intentar dar
        //más flujo cambiando el 37, oxigeno es la variable
        lpmair = (-inicioflujo * (oxigeno - 100)) / 80;
        lpmoxig = inicioflujo - lpmair;
        calculopwm(lpmoxig, lpmair);
      }
    }
    inicio = millis();
    bandera = 0;
    while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'p') && ((millis() - inicio) < ti)) {

      lecturapresionesin();
      if (bandtrigger == 1) {
        Serial.print("l");
      }
      else
        Serial.print("x");

      //se agregó para mejorar tiempos en visual studio
      if (contadorintervalo != 0)
        contadorintervalo = millis() - contadorintervalo;
      else
        contadorintervalo = 100;
      Serial.print (contadorintervalo);
      contadorintervalo = millis();


      if (Serial.available() > 0)
      {
        entrada = Serial.readStringUntil('\n');
        Serial.println(entrada);
      }

      //PRUEBA SILENCIO DE ALARMA
      if (entrada == "silence") {
        silenciador = 0;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(alarma,HIGH); //este solo sirve para probar ahorita con el uno, pero debe quitarse
      }
      //meter alarma
      if (entrada == "alerta") { //esto debe gnerar algun problema
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }
      if (entrada == "ruido") { //esto debe generar algun problema
        silenciador = 1;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(13,LOW); //este hay que comentarla
      }
      
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') { //tambien es pbe para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') { //tambien es tac para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') { //tambien es tbn para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada.substring(0,3) == "fio") {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if ((entrada[2] == 'l')&& (entrada != "silence")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }
      
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }
      //ventana alarmas
      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }

      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }

      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.print((int)total);
          }
          else
            Serial.print(total);
          //en = 1;
          oldEncPos = encoderPos;
          entrada = "";
        }
      }
      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.print("enc1");
          bandclick = 1;
        }
      }
      if (bandclick == 0)
        Serial.print("enc0");
      estadoBotonAnterior = estadoBoton;

      if (en == 0)
        Serial.println();
      en = 0;
      bandclick = 0;

     //filtros modos ventilatorios
     if (entrada[0] == '2') { 
        parametrosvolumen(entrada);
      }
      if (entrada[0] == '3') { 
        parametrosSIMVpresion(entrada);
      }
      if (entrada[0] == '4') { 
        parametrosSIMVvolumen(entrada);
      }
      if (entrada[0] == '5') { 
        parametrosAPRV(entrada);
      }
      if (entrada[0] == '6') { 
        parametrosBIPAP(entrada);
      }
      if (entrada[0] == '7') { 
        parametrosSPONT(entrada);
      }
     //Input = Pi ;
     //pidController.Compute();         // actualizar el PID
     //calculopwm(0,Output);

     
     if (presionalcanzada==0){
     //generación de rampa desacelerante
        contadorflujo += 3; //meter el tiempo y dividirlo entre un numero de veces(la pendiente)
        valflujo = (presion-10) * pow(1 / 1.1, contadorflujo);
        //valflujo=valflujo-;
        if (valflujo<=0)
        valflujo=0;
        if (oxigeno == 21)
        calculopwm (0, valflujo);
        else{
          lpmair = (-valflujo * (oxigeno - 100)) / 80;
          lpmoxig = valflujo - lpmair;
          calculopwm(lpmoxig, lpmair);
        }

        //verificación de la presión alcanzada
      if (Pi >= presion) {
        analogWrite(AVPropAir, 0);
        analogWrite(AVPropOx, 0);
        presionalcanzada=1;
      }
     }
      delay(100);
    }
    if ((bandera == 0) && (silenciador == 1) && (entrada != "alto")) { //agregue lo de alto ara que no mande fuga
      digitalWrite(alarma, HIGH); //ENCIENDE ALARMA
      Serial.print("fuga\n");
    }
    else {
      digitalWrite(alarma, LOW);
      if (entrada != "alto")
        Serial.print("nofuga\n");
    }
   
    //EXHALACIÓN
    digitalWrite(AVexh, HIGH);
    analogWrite(AVPropAir, 0);
    analogWrite(AVPropOx, 0);
    bandtrigger = 0;
    inicio = millis();
    contadorintervalo = 0;

    while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'p') && ((millis() - inicio) < te) && (bandtrigger == 0)) { //agregar condición de trigger para modo ac
      Pi = SensorPi();
      Os = SensorOs();

      //respiración realizada por el px.
      if (Pi < (PEEP - trigger))
        bandtrigger = 1;

      if (Pi < PEEP) { //aquí ver si es con Pexh una mejor respuesta
        digitalWrite(AVexh, LOW);
      }

      Serial.print("p");
      Serial.print(Pi);
      Serial.print("f");
      TCA9548A(3); // select I2C bus 2 for the exh flow sensor
      Fexh = SensorData();
      Serial.print(Fexh);
      Serial.print("o");
      Serial.print(Os);
      Serial.print("y");

      //envío de tiempo transcurrido en ms
      if (contadorintervalo != 0)
        contadorintervalo = millis() - contadorintervalo;
      else
        contadorintervalo = 100;
      Serial.print (contadorintervalo);
      contadorintervalo = millis();

      if (Serial.available() > 0)
      {
        entrada = Serial.readStringUntil('\n');
        //Serial.println(entrada);
      }
      if (entrada == "silence") {
        //bandera=0;
        silenciador = 0;
        entrada = ""; //para que no este leyendo nada
      }

      //meter alarma
      if (entrada == "alerta") { //esto debe gnerar algun problema
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }
      if (entrada == "ruido") { //esto debe generar algun problema
        silenciador = 1;
        entrada = ""; //para que no este leyendo nada
      }

      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada.substring(0,3) == "fio") {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if ((entrada[2] == 'l')&& (entrada != "silence")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }

      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }
      //ventana alarmas
      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.print((int)total);
          }
          else
            Serial.print(total);
          //en = 1;
          oldEncPos = encoderPos;
          entrada = "";
        }
      }
      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.print("enc1");
          bandclick = 1;
        }
      }
      if (bandclick == 0)
        Serial.print("enc0");
      estadoBotonAnterior = estadoBoton;

      if (en == 0)
        Serial.println();
      en = 0;
      bandclick = 0;
      //filtros modos ventilatorios
     if (entrada[0] == '2') { 
        parametrosvolumen(entrada);
      }
      if (entrada[0] == '3') { 
        parametrosSIMVpresion(entrada);
      }
      if (entrada[0] == '4') { 
        parametrosSIMVvolumen(entrada);
      }
      if (entrada[0] == '5') { 
        parametrosAPRV(entrada);
      }
      if (entrada[0] == '6') { 
        parametrosBIPAP(entrada);
      }
      if (entrada[0] == '7') { 
        parametrosSPONT(entrada);
      }
      delay(100);
    }
  } while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'p'));

//en caso de actualizacion de parámetros
  if (entrada[3]=='p')
  parametrospresion(entrada);
  
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  digitalWrite(alarma, LOW);
  digitalWrite(AVexh, HIGH); //abre la valvula exhalatoria
  
}
void VCV (int volumen, int frec, float ti, int PEEP, int oxigeno, int trigger, int flujo) { //ver si hay que añadir algun otra variable
  //Serial.println("Modo volumen");
  ttotal = 60000 / frec;
  ti = ti * 1000;
  te = ttotal - ti;
  param = "";
  bandtrigger = 0;
  silenciador = 1;
  bandvol = 0;
  volumencalc = 0;
  do {
    //ESTA DECLARACIÓN ES TEMPORAL o asegurando que lo que sale siempre es cero.
    volumencalc = 0;
    //alarmas suministros gases
    POx = SensorPOx();
    PAir = SensorPAir();
    volFuga = 0;
    PiFuga = 0; //se agrega para en caso de presion baja también marque que hay fuga.
    contadorintervalo = 0;
    //volumencalc=0; no se hace cero ya que debe ser en base al volumen real anterior
    //si esto hace eso es porque hay fuga
    //if ((POx<30)){
    // if ((POx<30)||(PAir<10)){
    if (PAir < 0) {
      //instruccion para encender buzzer y piloto
      digitalWrite(alarma, HIGH); //ENCIENDE ALARMA
      delay(1000);
      Serial.print("problema");
      entrada = "alto";
      Serial.print(",");
      Serial.print(POx);
      Serial.print
      (",");
      Serial.println(PAir);
    }
    //inhalación
    digitalWrite(AVexh, LOW);
    //da el porcentaje de acuerdo al fio2 programado
    if (oxigeno == 21) {
      //ecuaciones fio2
      lpmair = flujo;
      lpmoxig = 0;
      calculopwm(lpmoxig, lpmair);
    }
    else {
      if (oxigeno == 100) {
        //ecuaciones fio2
        lpmair = 0;
        lpmoxig = flujo - lpmair;
        //ecuaciones conversion lpm-pwm
        calculopwm(lpmoxig, lpmair);
      }
      else {
        //ecuaciones fio2
        lpmair = (-flujo * (oxigeno - 100)) / 80;
        lpmoxig = flujo - lpmair;
        //ecuaciones conversion lpm-pwm
        calculopwm(lpmoxig, lpmair);
      }
    }

    inicio = millis();
    iniciovol = millis();
    bandera = 0;

    while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'v') && ((millis() - inicio) < ti)) {

      //lectura sensores: presión inspiratoria, flujo aire, flujo oxígeno y sensor de oxígeno
      Pi = SensorPi();
      Os = SensorOs();
      //impresión de gráficas
      Serial.print("p");
      Serial.print(Pi);
      Serial.print("f");
      //lectura flujo oxigeno
      TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
      FOx = SensorData();
      //Serial.println("Flujo Ox");
      //Serial.println(FOx);
      //lectura flujo aire
      TCA9548A(5); // select I2C bus 1 for the air flow sensor
      FAir = SensorData();
      //Serial.println("Flujo Aire");
      //Serial.println(FAir);
      Serial.print(FAir + FOx);
      Serial.print("o");
      Serial.print(Os);

      //Alarma Fuga
      //Serial.println("Multiplicacion");
      //Serial.println((FAir + FOx) *(millis() - iniciovol) * 0.016);
      volumencalc = volumencalc + ((FAir + FOx) * (millis() - iniciovol) * 0.016); //en ml Svolumencalc+
      iniciovol = millis();

      if (volumencalc > volFuga) {
        volFuga = volumencalc;
      }

      if ((volFuga > (volumen * 0.60)) && (bandera != 1)) { //probar esta que puede funcionar, fnciona con ese
        bandera = 1;
        digitalWrite(alarma, LOW);
      }


      if (bandtrigger == 1) {
        Serial.print("l");
      }
      else
        Serial.print("x");
      //envio de tiempo a vstudio
      if (contadorintervalo != 0)
        contadorintervalo = millis() - contadorintervalo;
      else
        contadorintervalo = 100;
      Serial.print (contadorintervalo);
      contadorintervalo = millis();

      if (Serial.available() > 0)
      {
        entrada = Serial.readStringUntil('\n');
      }
      //Serial.println ("Volumen calculado");
      //Serial.println (volumencalc);
      //Serial.println ("Volumen seteado");
      //Serial.println (volumen);
      if (volumencalc > volumen) {
        analogWrite(AVPropAir, 0); //estaba en 3.8
        analogWrite(AVPropOx, 0);
      }

      //PRUEBA SILENCIO DE ALARMA
      if (entrada == "silence") {
        silenciador = 0;
        entrada = ""; //para que no este leyendo nada
      }
      //meter alarma
      if (entrada == "alerta") {
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }
      if (entrada == "ruido") {
        silenciador = 1;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(13,LOW); //este hay que comentarla
      }
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') { //tambien es pbe para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') { //tambien es tac para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') { //tambien es tbn para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada.substring(0,3) == "fio") {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if ((entrada[2] == 'l')&& (entrada != "silence")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }
      
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }

      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }

      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }

      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }

      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.print((int)total);
          }
          else
            Serial.print(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }

      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.print("enc1");
          bandclick = 1;
        }
      }
      if (bandclick == 0)
        Serial.print("enc0");
      estadoBotonAnterior = estadoBoton;

      if (en == 0)
        Serial.println();
      en = 0;
      bandclick = 0;
      //cambio modos
      if (entrada[0] == '1') { 
        parametrospresion(entrada);
      }
      if (entrada[0] == '3') { 
        parametrosSIMVpresion(entrada);
      }
      if (entrada[0] == '4') { 
        parametrosSIMVvolumen(entrada);
      }
      if (entrada[0] == '5') { 
        parametrosAPRV(entrada);
      }
      if (entrada[0] == '6') { 
        parametrosBIPAP(entrada);
      }
      if (entrada[0] == '7') { 
        parametrosSPONT(entrada);
      }
      delay(100);
    }
    if ((bandera == 0) && (silenciador == 1) && (entrada != "alto")) {
      digitalWrite(alarma, HIGH); //ENCIENDE ALARMA
      Serial.print("fuga\n");
    }
    else {
      digitalWrite(alarma, LOW);
      if (entrada != "alto")
        Serial.print("nofuga\n");
    }
    //EXHALACIÓN
    digitalWrite(AVexh, HIGH);
    analogWrite(AVPropAir, 0);
    analogWrite(AVPropOx, 0);
    bandtrigger = 0;
    bandvol = 0;
    inicio = millis();
    iniciovol = millis();
    contadorintervalo = 0;

    while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'v') && ((millis() - inicio) < te) && (bandtrigger == 0)) { //agregar condición de trigger para modo ac
      Pi = SensorPi();
      Os = SensorOs();
      //lectura flujo oxigeno
      TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
      FOx = SensorData();
      //lectura flujo aire
      TCA9548A(5); // select I2C bus 1 for the air flow sensor
      FAir = SensorData();

      //respiración realizada por el px.
      if (((FAir + FOx) > trigger) && (bandvol == 1)) {
        bandtrigger = 1;
        bandvol = 0;
      }
      if (Pi < PEEP) { //aquí ver si es con Pexh una mejor respuesta
        digitalWrite(AVexh, LOW);
        bandvol = 1;
      }

      Serial.print("p");
      Serial.print(Pi);
      Serial.print("f");
      TCA9548A(3); // select I2C bus 2 for the exh flow sensor
      Fexh = SensorData();
      Serial.print(Fexh);
      Serial.print("o");
      Serial.print(Os);
      Serial.print("y");
      //envia tiempo en ms a vstudio
      if (contadorintervalo != 0)
        contadorintervalo = millis() - contadorintervalo;
      else
        contadorintervalo = 100;
      Serial.print (contadorintervalo);
      contadorintervalo = millis();

      //calculo de volumen exh
      volumencalc = volumencalc - ((Fexh) * (millis() - iniciovol) * 0.016);
      iniciovol = millis();

      if (Serial.available() > 0)
      {
        entrada = Serial.readStringUntil('\n');
      }
      if (entrada == "silence") {
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }

      //meter alarma
      if (entrada == "alerta") { //esto debe gnerar algun problema
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }
      if (entrada == "ruido") { //esto debe generar algun problema
        silenciador = 1;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(13,LOW); //este hay que comentarla
      }

      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') { //tambien es pbe para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') { //tambien es tac para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') { //tambien es tbn para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada.substring(0,3) == "fio") {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if ((entrada[2] == 'l')&& (entrada != "silence")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }
      
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }

      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }

      //de ventana de alarmas
      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.print((int)total);
          }
          else
            Serial.print(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }

      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.print("enc1");
          bandclick = 1;
        }
      }
      if (bandclick == 0)
        Serial.print("enc0");
      estadoBotonAnterior = estadoBoton;

      if (en == 0)
        Serial.println();
      en = 0;
      bandclick = 0;

      if (entrada[0] == '1') { 
        parametrospresion(entrada);
      }
      if (entrada[0] == '3') { 
        parametrosSIMVpresion(entrada);
      }
      if (entrada[0] == '4') { 
        parametrosSIMVvolumen(entrada);
      }
      if (entrada[0] == '5') { 
        parametrosAPRV(entrada);
      }
      if (entrada[0] == '6') { 
        parametrosBIPAP(entrada);
      }
      if (entrada[0] == '7') { 
        parametrosSPONT(entrada);
      }
      
      delay(100);
    }
  } while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'v'));
  
  //en caso de actualizacion de parámetros
  if (entrada[3]=='v')
  parametrosvolumen(entrada);
  
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  digitalWrite(alarma, LOW);
  digitalWrite(AVexh, HIGH);
  param = ""; //la limpieza de entrada se hace hasta el final, igual puede quitarse este
}

void lecturadatos(void) {
  while (true){
  Serial.println("Ventanadatos");
  if (Serial.available() > 0)
  {
    entrada = Serial.readStringUntil('\n');
    //Serial.println(entrada);
  }
  //una vez finalizada la lectura de los parámetros de entrada del ventilador
  if ((entrada != "") || (param != "")) {
    modo = entrada[3]; //para ver si es modo volumen o presión
    if (modo == "w") {
      // parametrosvolumen(entrada);
      testinicial();
    }
    //modo vcvac
    if (modo == "v") {
      // parametrosvolumen(entrada);
      ventanavolumen();
    }
    //modo vcpac
    if (modo == "p") {
      //parametrospresion(entrada);
      ventanapresion();
    }
    if (modo == "x") {
      ventanasimvp();
    }
    if (modo == "j") {
      ventanasimvv();
    }
    if (modo == "r") {
      ventanaaprv();
    }
    if (modo == "b") {
      ventanabipap();
    }
    if (modo == "l") {
      ventanaspont();
    }

    //autotest
    if (modo == "t") {
      autotest();
    }
    //calib oxigeno
    if (modo == "c") {
      calibox();
    }
    //calib flujo
    if (modo == "f") {
      calibfluj();
    }
    if (modo == "a") {
      ventanalarmas();
    }
    //perilla peso
    if ((modo == "s") || (param != "")) {
      //encoderPos=0;
      if (entrada != "") {
        auxperilla = encoderPos;
        param = entrada;
        auxenc = 1;
      }
      if (oldEncPos != encoderPos) {
        total = (param.substring(4, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
        Serial.print("pees");
        //delay(10);
        Serial.println((int)total);
        oldEncPos = encoderPos;
        entrada = "";
      }
    }
  }
  delay(10);
  }
}

void parametrospresion(String cadena) {
  //para el modo presión
  Serial.println("Ventana parametros presion");
  String stroxigeno, strPEEP, strTi, strfrec, strpresion, strtrigger;
  int aoxigeno, aPEEP, aTi, afrec, apresion, atrigger;
  for (int i = 0; i < cadena.length(); i++) {
    if (cadena[i] == 's') //parámetro de presion
      apresion = i;
    if (cadena[i] == 'x') //parámetro de porcentaje oxigeno
      aoxigeno = i;
    if (cadena[i] == 'E') //parámetro de PEEP
      aPEEP = i;
    if (cadena[i] == 'i') //parámetro de Ti
      aTi = i;
    if (cadena[i] == 'r') //parámetro de frecuencia respiratoria
      afrec = i;
    if (cadena[i] == 'n') //parámetro de trigger
      atrigger = i;
  }
  strpresion = cadena.substring(apresion + 1, aoxigeno - 1); //ya que el parametro viene como Ox
  stroxigeno = cadena.substring(aoxigeno + 1, aPEEP - 1); //ya que el parametro viene como PE
  strPEEP = cadena.substring(aPEEP + 1, aTi - 1);
  strTi = cadena.substring(aTi + 1, afrec - 1);
  strfrec = cadena.substring(afrec + 1, atrigger - 1);
  strtrigger = cadena.substring(atrigger + 1, cadena.length());
  //convertir a entero todos los valores
  presion = strpresion.toInt();
  oxigeno = stroxigeno.toInt();
  PEEP = strPEEP.toInt();
  ti = strTi.toFloat();
  frec = strfrec.toFloat();
  trigger = strtrigger.toInt();
  //lineas auxiliares para corroborar correcta lectura y guardado de datos
  //if (frec==12){
  //digitalWrite (13,HIGH);
  //}
  cadena = "";
  entrada = "";
  VCP (presion, frec, ti, PEEP, oxigeno, trigger);
  if (entrada=="fuda"){
  entrada="";
  lecturadatos();
  }
  else
  ventanapresion();
  // Serial.println ("modo:"+modo+"Presion:" +strpresion+"oxigeno:"+stroxigeno+"PEEP:"+strPEEP+"Tinspi:"+strTi+"frecuencia:"+strfrec);
}

void parametrosvolumen (String cadena) {
  //para el modo volumen
  String stroxigeno, strPEEP, strTi, strfrec, strflujo, strvolumen, strtrigger;
  int aoxigeno, aPEEP, aTi, afrec, aflujo, avolumen, atrigger;

  for (int i = 0; i < cadena.length(); i++) {
    if (cadena[i] == 'l') //parámetro de flujo
      aflujo = i;
    if (cadena[i] == 'x') //parámetro de porcentaje oxigeno
      aoxigeno = i;
    if (cadena[i] == 'E') //parámetro de PEEP
      aPEEP = i;
    if (cadena[i] == 'i') //parámetro de Ti
      aTi = i;
    if (cadena[i] == 'r') //parámetro de frecuencia respiratoria
      afrec = i;
    if (cadena[i] == 'n') //parámetro de trigger
      atrigger = i;
  }
  strflujo = cadena.substring(aflujo + 1, aoxigeno - 1); //ya que el parametro viene como Ox
  stroxigeno = cadena.substring(aoxigeno + 1, aPEEP - 1); //ya que el parametro viene como PE
  strPEEP = cadena.substring(aPEEP + 1, aTi - 1);
  strTi = cadena.substring(aTi + 1, afrec - 1);
  strfrec = cadena.substring(afrec + 1, atrigger - 1);
  strtrigger = cadena.substring(atrigger + 1, cadena.length());

  trigger = strtrigger.toInt();
  oxigeno = stroxigeno.toInt();
  PEEP = strPEEP.toInt();
  ti = strTi.toFloat();
  frec = strfrec.toFloat();
  flujo = strflujo.toInt();
  volumen = (flujo * ti * 16.6);
  //if (frec==12){
  //digitalWrite (13,HIGH);
  //}
  cadena = "";
  entrada = "";
  //llamar funcion VCV
  VCV (volumen, frec, ti, PEEP, oxigeno, trigger, flujo);
  //Serial.println ("modo:"+modo+"flujo:" +flujo+"oxigeno:"+oxigeno+"PEEP:"+PEEP+"Tinspi:"+ti+"frecuencia:"+frec+"trigger"+trigger+"volumen"+volumen);
  //delay(5000);
  if (entrada=="fuda"){
  entrada="";
  lecturadatos();
  }
  else
  ventanavolumen();
}
float getChecksum() { //Verify the EEPROM checksum against factory configuration.
  Wire.beginTransmission(0x49);
  Wire.write(0x03);
  delay(1);
  Wire.requestFrom(0x49, 0x02); // Request the transmitted two bytes from the two registers
  delay(1);
  if (Wire.available() <= 2) { //
    X0 = Wire.read(); // Reads the data from the register
    X1 = Wire.read();
  }
  Wire.endTransmission();
  float checksum = word(X0, X1); //Combine two bytes recieved.
  if (checksum == 52389) { //##From datasheet
    Serial.println("EEPROM Checksum match!");
  }
  else if (checksum == 52368) { //##From datasheet
    Serial.println("EEPROM Checksum error");
  }
}

float getFlowReading() {
  Wire.beginTransmission(0x49);
  delay(1);
  Wire.requestFrom(0x49, 0x02); // Request the transmitted two bytes from the two registers

  if (Wire.available() <= 2) { //
    X0 = Wire.read(); // Reads the data from the register
    X1 = Wire.read();
  }
  Wire.endTransmission();
  //X0=float(X0);
  //X1=float(X1);
  float output = word(X0, X1); //Combine two bytes recieved.
  //float output = X0+X1;
  float flow = flowRange * ((output / 16384) - 0.1) / 0.8; // From datasheet
  float conversionFactor = 1; //en SLPM
  //float conversionFactor = ( (273.15 / 293.15) * (14.696 / 14.504) ); //Converting SLPM to NLPM;
  flow = flow * conversionFactor;
  //if ((flow>200) || (flow<0.7))
  //flow = 0;
  //flow = 3.465 * pow (flow, 0.7679); //conversion del valor dado por el sensor al valor de flujo leido por el analizador
  return flow;
}


float getError(float flow) { //#Calculate error for readings lower and higher than 12.5% of flowrange. (Datasheet spec)
  float TEB;
  if (flow < flowRange * 0.125) {
    TEB = flowRange * 0.005;
  }
  else  {
    TEB = 0.04 * flow;
  }
  return TEB;
}

float SensorData()
{
  float flow = getFlowReading();
  return flow;
}

void TCA9548A(uint8_t bus)
{
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

void pruebasensores() {

  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 100);
  digitalWrite (AVexh, HIGH);

  Pi = SensorPi();
  Serial.print("Presión inspiratoria ");
  Serial.println(Pi);

  PAir = SensorPAir();
  Serial.print("Presión de aire ");
  Serial.println(PAir);

  POx = SensorPOx();
  Serial.print("Presión oxigeno ");
  Serial.println(POx);

  Os = SensorOs();
  Serial.print("% Oxigeno ");
  Serial.println(Os);

  Pexh = SensorPexh();
  Serial.print("Presión exhalatoria ");
  Serial.println(Pexh);

  Serial.print("Sensor flujo exh ");
  TCA9548A(4);
  Serial.println(SensorData());

  Serial.print("Sensor flujo oxig ");
  TCA9548A(5);
  Serial.println(SensorData());
  Serial.print("Sensor flujo aire ");
  TCA9548A(3);

  Serial.println(SensorData());
  delay(1000);
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  digitalWrite (AVexh, LOW);
  delay(1000);
}


void PinA() {
  cli(); //stop interrupts happening before we read pin values
  reading = PINE & 0x30; // read all eight pin values then strip away all but pinA and pinB's values
  if (reading == 0x30 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos --; //decrement the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00010000) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

void PinB() {
  cli(); //stop interrupts happening before we read pin values
  reading = PINE & 0x30; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == 0x30 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos ++; //increment the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00100000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

void lecturapresionesin() {
  //lectura sensores: presión inspiratoria, flujo aire, flujo oxígeno y sensor de oxígeno
  Pi = SensorPi();
  Os = SensorOs();
  //impresión de gráficas
  Serial.print("p");
  Serial.print(Pi);
  Serial.print("f");
  //lectura flujo oxigeno
  TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
  FOx = SensorData();
  //lectura flujo aire
  TCA9548A(5); // select I2C bus 1 for the air flow sensor
  FAir = SensorData();
  Serial.print(FAir + FOx);
  Serial.print("o");
  Serial.print(Os);

  //Alarma Fuga
  if (Pi > PiFuga) {
    PiFuga = Pi;
  }
  //if ((Pi-Pexh)>5)&&(bandera!=1)){
  if ((PiFuga > (presion * 0.60)) && (bandera != 1)) { //probar esta que puede funcionar, fnciona con ese
    //valor de amplificacion de las valvulas
    bandera = 1;
    digitalWrite(alarma, LOW);
  }
}

void autotest() {
  bandtest1 = 0;
  bandtest3 = 0;
  Serial.println ("Preparando equipo para autotest...");
  delay(3000);
  Serial.println ("Test 1");
  calculopwm(5, 5);
  digitalWrite(AVexh, LOW);
  inicio = millis();
  while ((millis() - inicio) < 2000) {
    Pi = SensorPi();
    Os = SensorOs();
    //lecturapwmoxigeno
    POx = SensorPOx();
    PAir = SensorPAir();
    Serial.print("Presion de entrada oxigeno: ");
    Serial.print(POx);
    Serial.println(" cmH20");
    Serial.print("Presion de entrada aire: ");
    Serial.print(FAir);
    Serial.println(" cmH20");
    TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
    FOx = SensorData();
    Serial.print("Sensor de flujo oxigeno: ");
    Serial.print(FOx);
    Serial.println(" L/min");
    //lectura flujo aire
    TCA9548A(5); // select I2C bus 1 for the air flow sensor
    FAir = SensorData();
    Serial.print("Sensor de flujo aire: ");
    Serial.print(FAir);
    Serial.println(" L/min");
    Serial.print("Presion insiratoria: ");
    Serial.print(Pi);
    Serial.println(" cmH20");
    Serial.print("Sensor de oxigeno: ");
    Serial.print(Os);
    Serial.println(" %");
    Serial.print("Presion exhalatoria: ");
    Serial.print(SensorPexh());
    Serial.println(" cmH20");
    TCA9548A(3); // select I2C bus 1 for the oxygen flow sensor
    Fexh = SensorData();
    Serial.print("Sensor de flujo exhalatorio: ");
    Serial.print(Fexh);
    Serial.println(" L/min");
    if (FOx > 200) {
      Serial.println("Falla en sensor de flujo oxigeno");
    }
    if (FAir > 200) {
      Serial.println("Falla en sensor de flujo aire");
    }
    if (Fexh > 200) {
      Serial.println("Falla en sensor de flujo exhalatorio");
    }
    if ((FOx > 200) || (FAir > 200) || (Fexh > 200))
      bandtest1 = 1;
    else
      bandtest1 = 0;

    delay(500);
  }
  if (bandtest1 == 0)
    Serial.println ("Test 1.PASO");
  else
    Serial.println ("Test 1.FALLO");
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  delay(3000);
  Serial.println("Test 2");
  delay(500);
  Serial.println("Test de alarmas");
  digitalWrite(AVexh, HIGH);
  digitalWrite(alarma, HIGH);
  Serial.println("Alarma activada");
  delay(2000);
  Serial.println("Alarma silenciada");
  delay(1000);
  Serial.println("Test 2.PASO");
  delay(1000);
  Serial.println ("Test 3");
  digitalWrite(alarma, LOW);
  calculopwm(15, 15);
  digitalWrite(AVexh, LOW);
  inicio = millis();
  while ((millis() - inicio) < 2000) {
    Pi = SensorPi();
    Os = SensorOs();
    POx = SensorPOx();
    PAir = SensorPAir();
    Serial.print("Presion de entrada oxigeno: ");
    Serial.print(POx);
    Serial.println(" cmH20");
    Serial.print("Presion de entrada aire: ");
    Serial.print(FAir);
    Serial.println(" cmH20");
    TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
    FOx = SensorData();
    Serial.print("Sensor de flujo oxigeno: ");
    Serial.print(FOx);
    Serial.println(" L/min");
    //lectura flujo aire
    TCA9548A(5); // select I2C bus 1 for the air flow sensor
    FAir = SensorData();
    Serial.print("Sensor de flujo aire: ");
    Serial.print(FAir);
    Serial.println(" L/min");
    Serial.print("Presion insiratoria: ");
    Serial.print(Pi);
    Serial.println(" cmH20");
    Serial.print("Sensor de oxigeno: ");
    Serial.print(Os);
    Serial.println(" %");
    Serial.print("Presion exhalatoria: ");
    Serial.print(SensorPexh());
    Serial.println(" cmH20");
    TCA9548A(3); // select I2C bus 1 for the oxygen flow sensor
    Fexh = SensorData();
    Serial.print("Sensor de flujo exhalatorio: ");
    Serial.print(Fexh);
    Serial.println(" L/min");
    if (FOx > 200) {
      Serial.println("Falla en sensor de flujo oxigeno");
    }
    if (FAir > 200) {
      Serial.println("Falla en sensor de flujo aire");
    }
    if (Fexh > 200) {
      Serial.println("Falla en sensor de flujo exhalatorio");
    }
    if ((FOx > 200) || (FAir > 200) || (Fexh > 200))
      bandtest3 = 1;
    else
      bandtest3 = 0;
    delay(500);
  }
  if (bandtest3 == 0)
    Serial.println ("Test 3.PASO");
  else
    Serial.println ("Test 3.FALLO");
  calculopwm(0, 0);
  delay(3000);
  digitalWrite(alarma, LOW);
  digitalWrite(AVexh, HIGH);
  delay(2000);
  if ((bandtest1 == 0) && (bandtest3 == 0)) {
    Serial.println ("Prueba finalizada");
  }
  else
    Serial.println ("Prueba fallida");
  entrada = "";
  bandtest1 = 0;
  bandtest3 = 0;
}
void calibox() {
  entrada = "";
  Serial.println ("Preparando equipo para calibracion...");
  Serial.println ("Desconecte circuito ventilatorio");
  delay(3000);
  calculopwm(0, 15);
  digitalWrite(AVexh, LOW);
  inicio = millis();
  Serial.println("Calibracion a 21 %");
  while ((millis() - inicio) < 4000) {
    airevalor = analogRead(AOs);
    Serial.print(airevalor);
    Serial.println(" %");
    delay(500);
  }
  //guardar en eeprom el valor de aire 21%
  EEPROM.update( airAddress, airevalor );//sino funciona probar con put

  Serial.println("Calibracion a 21 % finalizada");
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  delay (3000);
  digitalWrite(AVexh, HIGH);
  delay (2000);

  Serial.println("Conecte circuito ventilatorio.De click en continuar");
  while ((entrada == "") && (entrada != "siguiente")) {
    if (Serial.available() > 0)
    {
      entrada = Serial.readStringUntil('\n');
    }
    delay(200);
  }

  Serial.println("Calibracion al 100%");
  calculopwm(15, 0);
  digitalWrite(AVexh, LOW);
  inicio = millis();
  while ((millis() - inicio) < 4000) {
    oxigenovalor = analogRead(AOs);
    Serial.print(oxigenovalor);
    Serial.println(" %");
    delay(500);
  }
  //guardar en eeprom el valor de aire 100%
  EEPROM.update( oxigAddress, oxigenovalor ); //sino funciona probar con put
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  delay (3000);
  digitalWrite(AVexh, HIGH);
  delay (2000);
  Serial.println("Calibracion finalizada");
  entrada = "";
}
void calibfluj() {
  bandtest1 = 0;
  bandtest2 = 0;
  bandtest3 = 0;
  Serial.println ("Preparando equipo para calibracion...");
  Serial.println ("Conecte circuito ventilatorio");
  delay(3000);
  Serial.println("Test 1");
  Serial.println("Test sensor de flujo oxigeno");
  calculopwm(15, 0);
  digitalWrite(AVexh, LOW);
  inicio = millis();
  while ((millis() - inicio) < 2000) {
    TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
    FOx = SensorData();
    Serial.print("Sensor de flujo oxigeno: ");
    Serial.print(FOx);
    Serial.println(" L/min");
    //lectura flujo aire
    TCA9548A(5); // select I2C bus 1 for the air flow sensor
    FAir = SensorData();
    Serial.print("Sensor de flujo aire: ");
    Serial.print(FAir);
    Serial.println(" L/min");
    if (FOx > 200) {
      Serial.println("Falla en sensor de flujo oxigeno");
    }
    if (FAir > 200) {
      Serial.println("Falla en sensor de flujo aire");
    }
    if ((FOx > 200) || (FAir > 200))
      bandtest1 = 1;
    else
      bandtest1 = 0;
    delay(500);
  }
  Serial.println("Test sensor de flujo exhalatorio");
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  delay (3000);
  digitalWrite(AVexh, HIGH);
  inicio = millis();
  while ((millis() - inicio) < 2000) {
    //lecturapwmoxigeno
    TCA9548A(3); // select I2C bus 1 for the oxygen flow sensor
    Fexh = SensorData();
    Serial.print("Sensor de flujo exhalatorio: ");
    Serial.print(Fexh);
    Serial.println(" L/min");
    if (Fexh > 200) {
      Serial.println("Falla en sensor de flujo exhalatorio");
      bandtest1 = 1 + bandtest1;
    }
    else
      bandtest1 = 0 + bandtest1;
    delay(500);
  }
  if (bandtest1 == 0)
    Serial.println("Test 1.PASO");
  else
    Serial.println("Test1.FALLO");
  delay (3000);
  Serial.println("Test 2");
  Serial.println("Test sensor de flujo aire");
  calculopwm(0, 15);
  digitalWrite(AVexh, LOW);
  inicio = millis();
  while ((millis() - inicio) < 2000) {
    TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
    FOx = SensorData();
    Serial.print("Sensor de flujo oxigeno: ");
    Serial.print(FOx);
    Serial.println(" L/min");
    //lectura flujo aire
    TCA9548A(5); // select I2C bus 1 for the air flow sensor
    FAir = SensorData();
    Serial.print("Sensor de flujo aire: ");
    Serial.print(FAir);
    Serial.println(" L/min");
    if (FOx > 200) {
      Serial.println("Falla en sensor de flujo oxigeno");
    }
    if (FAir > 200) {
      Serial.println("Falla en sensor de flujo aire");
    }

    if ((FOx > 200) || (FAir > 200))
      bandtest2 = 1;
    else
      bandtest2 = 0;
    delay(500);
  }

  Serial.println("Test sensor de flujo exhalatorio");
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  delay (3000);
  digitalWrite(AVexh, HIGH);
  inicio = millis();
  while ((millis() - inicio) < 2000) {
    //lecturapwmoxigeno
    TCA9548A(3); // select I2C bus 1 for the oxygen flow sensor
    Fexh = SensorData();
    Serial.print("Sensor de flujo exhalatorio: ");
    Serial.print(Fexh);
    Serial.println(" L/min");
    if (Fexh > 200) {
      Serial.println("Falla en sensor de flujo exhalatorio");
      bandtest2 = 1 + bandtest2;
    }
    else
      bandtest2 = 0 + bandtest2;
    delay(500);
  }
  if (bandtest2 == 0)
    Serial.println("Test 2.PASO");
  else
    Serial.println("Test 2.FALLO");
  delay (3000);

  Serial.println("Test 3");
  Serial.println("Test sensor de flujo aire y oxígeno");
  calculopwm(15, 15);
  digitalWrite(AVexh, LOW);
  inicio = millis();
  while ((millis() - inicio) < 2000) {
    TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
    FOx = SensorData();
    Serial.print("Sensor de flujo oxigeno: ");
    Serial.print(FOx);
    Serial.println(" L/min");
    //lectura flujo aire
    TCA9548A(5); // select I2C bus 1 for the air flow sensor
    FAir = SensorData();
    Serial.print("Sensor de flujo aire: ");
    Serial.print(FAir);
    Serial.println(" L/min");
    if (FOx > 200) {
      Serial.println("Falla en sensor de flujo oxigeno");
    }
    if (FAir > 200) {
      Serial.println("Falla en sensor de flujo aire");
    }

    if ((FOx > 200) || (FAir > 200))
      bandtest3 = 1;
    else
      bandtest3 = 0;
    delay(500);
  }

  Serial.println("Test sensor de flujo exhalatorio");
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  delay (3000);
  digitalWrite(AVexh, HIGH);
  inicio = millis();
  while ((millis() - inicio) < 2000) {
    //lecturapwmoxigeno
    TCA9548A(3); // select I2C bus 1 for the oxygen flow sensor
    Fexh = SensorData();
    Serial.print("Sensor de flujo exhalatorio: ");
    Serial.print(Fexh);
    Serial.println(" L/min");
    if (Fexh > 200) {
      Serial.println("Falla en sensor de flujo exhalatorio");
      bandtest3 = 1 + bandtest3;
    }
    else
      bandtest3 = 0 + bandtest3;
    delay(500);
  }
  if (bandtest3 == 0)
    Serial.println("Test 3.PASO");
  else
    Serial.println("Test 3.FALLO");
  delay (3000);

  if ((bandtest1 == 0) && (bandtest2 == 0) && (bandtest3 == 0))
    Serial.println("Calibracion finalizada");
  else
    Serial.println("Calibracion fallida");
  entrada = "";
  bandtest1 = 0;
  bandtest2 = 0;
  bandtest3 = 0;
}

void ventanapresion() { //modp
  Serial.println("Ventana presion");
  param = "";
  entrada = "";
  while (entrada != "fuda") {
    if (Serial.available() > 0)
    {
      entrada = Serial.readStringUntil('\n');
    }
    //delay(10);
    //una vez finalizada la lectura de los parámetros de entrada del ventilador
    if ((entrada != "") || (param != "")) {
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada[2] == 'o') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if (entrada[2] == 'l') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }

      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }

      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          //Serial.println();
          Serial.print(datoencoder);
          if (auxenc != 10.0) {//Definir librerías
#include <PID_v1.h>
#include<Wire.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <math.h>

// Constantes del controlador
double Kp=0.2, Ki=0.5, Kd=0.1;
// variables externas del controlador
double Input, Output, Setpoint;
 
PID pidController(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//variable auxiliar de tiempo
unsigned long contadorintervalo = 100;
//variables para el click del encoder
const int boton = 4;               //boton conectado al pin 4
const int tiempoAntirebote = 10;
int estadoBoton;
int estadoBotonAnterior = 1;
int bandclick = 0;

int en = 0;
int a = 0;
//variables calibracion oxigeno
int airevalor = 461; //21%
int oxigenovalor = 1023; //100%
int airAddress = 0;
int oxigAddress = 10;

int silenciador = 1;
int bandtrigger = 0;
int auxperilla = 0;
int bandvol = 0;
int bandpres = 0;
int bandtest1 = 0;
int bandtest2 = 0;
int bandtest3 = 0;
int cierrevalvulaaire = 0;
float volFuga = 0;
float volumencalc = 0;
int cierrevalvulaoxigeno = 0;
float auxenc = 10.0;
static int pinA = 2; // Our first hardware interrupt pin is digital pin 2
static int pinB = 3; // Our second hardware interrupt pin is digital pin 3
volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile int encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile int oldEncPos = -1; //tenia 0, lo cambie a -1
//stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the Serial monitor)
volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent
//String encoder="";
float total = 0;
String param = "";
String datoencoder = "";
int bandbipap = 0;
byte X0, X1;
const int flowRange = 200;

//definimos variables de modos ventilatorio
int PiFuga = 0;
int te = 0;
int trigger = 0;
int PEEP = 10;
int presion = 20;
int frec;
int flujo;
int psupp = 0;
float rampa = 0;
int volumen = 0;
int oxigeno;
int aire;
int palta = 0;
int pbaja = 0;
float talto = 0;
float tbajo = 0;

float ti = 0;
float ttotal = 0;
String entrada = ""; //variable para la cadena con los parámetros del ventilador
String entrada1, entrada2; //variables para la cadena de la lectura de los flujómetros
String modo = ""; //variable para el modo ventilatorio
unsigned int adc;

int band = 0; //bandera para saber cuándo está ventilando y cuándo requiere estar en modo "recepción" y ya no lea
//band=0 está detenido el ventilador
//band=1 está ventilando
byte buffer[3];
// Definir interrupciones

// entradas generales
//int APa = A1;    //sensor de presion atmosferica Pa
int APi = A0;    //sensor presion inspiratoria bien CONFIRMADO
int APOx = A3;  //sensor de presión de oxígeno
int APAir = A1;  //sensor de presión de aire BIEN confirmado
//int AFOx = A2; //sensor de flujo de oxígeno
int AFAir = A5; //sensor de flujo de aire
int AOs = A2;    //sensor de oxígeno bien confirmado
int APexh = A4; //sensor de presión exhalatoria bien confirmado
//int AFexh = A2; //sensor de flujo exhalatorio
// salidas generales
int AVPropOx = 9; //valvula proporcional oxigeno
int AVPropAir = 13; //valvula proporcional aire
int AVexh = 29; //valvula exhalatoria (on-off) valvula exhalatoria anterior
//  int AVexh=53; //valvula exhalatoria (on-off)
int alarma = 28; //cambia de 8 a 10
// variables auxiliares
int ANPa;         // variable que almacena el valor raw (0 a 1023)
float ANPi;         // variable que almacena el valor raw (0 a 1023)
int ANPOx;         // variable que almacena el valor raw (0 a 1023)
int ANPAir;        // variable que almacena el valor raw (0 a 1023)
int ANFOx;         // variable que almacena el valor raw (0 a 1023)
int ANFAir;        // variable que almacena el valor raw (0 a 1023)
int ANOs;          //
int ANPexh;        // variable que almacena el valor raw (0 a 1023)
int ANVOx;
int ANVAir;
int ANVexh;
int ANVPropOx;
int ANVPropAir;
int ANFexh;
int bandera = 0;
unsigned long inicio = 0;
unsigned long iniciovol = 0;

float Pa;            // variable que almacena el voltaje (0.0 a 5.0)
float Pi;            // variable que almacena el voltaje (0.0 a 5.0)
float POx = 0;          // variable que almacena el voltaje (0.0 a 5.0)
float PAir = 0;          // variable que almacena el voltaje (0.0 a 5.0)
float FOx = 0;          // variable que almacena el voltaje (0.0 a 5.0)
float FAir = 0;          // variable que almacena el voltaje (0.0 a 5.0)
float Os;            // variable que almacena el voltaje (0.0 a 5.0)
float Pexh;            // variabl que almacena el voltaje (0.0 a 5.0)
float Fexh;

//variables fio2 lpm
float pwmox = 0;
float pwmair = 0;
float lpmair = 0;
float lpmoxig = 0;

void setup()
{
  //pidController.SetSampleTime(200);               // Initialise sample rate in ms (200ms default)
  pidController.SetOutputLimits(0, 60);         // Initialise Output range - 0 to 255 suitable for AnalogWrite
  //Wire.setClock(1000000UL);
  wdt_disable();
  //wdt_enable(WDTO_2S);
  // put your setup code here, to run once:
  Serial.begin(9600); //puerto serie hacia la interfaz flujometro uno
  Wire.begin();                    //Begins the I2C communication
  pinMode(pinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(0, PinA, RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(1, PinB, RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)

  ////PIN AUXILIAR PARA VER QUE SE RECIBAN LOS DATO CORRECTOS DE LA INTERFAZ AL MICROCONTROLADOR
  //pinMode(13,OUTPUT);
  //digitalWrite(13,LOW);

  //definir entradas y salidas
  //  pinMode(APa, INPUT);
  pinMode(APi, INPUT);
  pinMode(APOx, INPUT);
  pinMode(APAir, INPUT);
  //  pinMode(AFOx, INPUT);
  pinMode(AFAir, INPUT);
  pinMode(AOs, INPUT);
  pinMode(APexh, INPUT);
  //  pinMode(AFexh, INPUT);
  // pinMode(AVOx, OUTPUT);
  // pinMode(AVAir, OUTPUT);
  pinMode(AVPropAir, OUTPUT);
  pinMode(AVPropOx, OUTPUT);
  pinMode(AVexh, OUTPUT);
  pinMode(alarma, OUTPUT);

  //Definición de funciones
  float SensorPa(void);
  float SensorPi(void);
  float SensorPOx(void);
  float SensorPAir(void);
  float SensorFOx(void);
  float SensorFAir(void);
  float SensorOs(void);
  float SensorPexh(void);
  float SensorFexh(void);
  void ActuadorVPropOx(int ANVPropOx);
  void ActuadorVPropAir(int ANVPropAir);
  void ActuadorVOx(int ANVOx);
  void ActuadorVAir(int ANVAir);
  void ActuadorVexh(int ANVexh);
  void VCP (int presion, int frec, float ti);
  void VCV (int volumen, int frec, float ti);
  void lecturadatos(void);
  void parametrosvolumen (String entrada);
  void parametrospresion (String entrada);
  //Serial.println("INICIO");
  digitalWrite(AVexh, LOW);
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  digitalWrite(alarma, LOW);
  //lectura de valores de eeprom de calibracion de oxigeno y aire
  EEPROM.get( airAddress, airevalor );
  EEPROM.get( oxigAddress, oxigenovalor );
  pinMode(boton, INPUT);            //declaramos el boton como entrada
}

void loop()
{
  
  //flujodecreciente();
  //Serial.print("Presión de oxígeno ");
  //Serial.println(SensorPOx());
  //Serial.print("Presión de Aire ");
  //Serial.println(SensorPAir());
  /*analogWrite(AVPropOx,0);
  analogWrite(AVPropAir,0);
  TCA9548A(4); 
  FOx=SensorData();
  TCA9548A(5); // select I2C bus 1 for the oxygen flow sensor
  FAir=SensorData();
  Serial.print("Flujo oxigeno: ");
  Serial.println(FOx);
  Serial.print("Flujo aire: ");
  Serial.println(FAir);
  delay(500);*/
  lecturadatos();
  //Serial.println (SensorPi());
  //put your main code here, to run repeatedly:
  /*digitalWrite(AVexh,LOW);
    Serial.print("Flujo exh");
    TCA9548A(3); // select I2C bus 1 for the air flow sensor
    Fexh=SensorData();
    Serial.println(Fexh);
    delay(500);
    analogWrite(AVPropOx,0);
    analogWrite(AVPropAir,0);
    Serial.print("Pinspiratoria");
    Serial.println (SensorPi());
    TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
    FOx=SensorData();
    //lectura flujo aire
    TCA9548A(5); // select I2C bus 1 for the air flow sensor
    FAir=SensorData();
    Serial.print("pwmox");
    Serial.println(FOx);
    Serial.print("pwmaire");
    Serial.println(FAir);
    delay(1000);*/
  //a+=50;
  //if (a==250){
  // a=0;
  // }
  //digitalWrite(AVexh,HIGH);
  //lecturadatos(); //descomentar SÓLO esta linea para funcionamiento con la interfaz. para pruebas descomentar lo demas
  //clickencoder(estadoBoton,estadoBotonAnterior);

  //flujo = 30;
  //oxigeno = 80;
  //ecuaciones fio2
  //lpmair = (-flujo * (oxigeno - 100)) / 80;
  //lpmoxig = flujo - lpmair;
  //ecuaciones conversion lpm-pwm
  /*lpmair=37;
    lpmoxig=37;
    //pwmox=0.028*pow(lpmoxig,2)-0.2352*lpmoxig+136; //ecuacion conversion lpm a pwm oxígeno aproximación cuadrática
    pwmox = (2 * pow(10, -8) * pow(lpmoxig, 6)) - (1 * pow(10, -6) * pow(lpmoxig, 5)) - (0.0004 * pow(lpmoxig, 4)) + (0.0387 * pow(lpmoxig, 3)) - (1.3439 * pow(lpmoxig, 2)) + (17.142 * lpmoxig) + 62.711; //ecuacion conversion lpm a pwm oxígeno aproximación cúbica
    pwmair = 3 * pow(10, -9) * pow(lpmair, 6) + 5 * pow(10, -7) * pow(lpmair, 5) - .0002 * pow(lpmair, 4) + .012 * pow(lpmair, 3) - .4009 * pow(lpmair, 2) + 6.9879 * lpmair + 31.917; //ecuacion conversion lpm a pwm aire
    //pwmair=6*pow(10,-8)*pow(lpmair,6)-1*pow(10,-5)*pow(lpmair,5)+0.0008*pow(lpmair,4)-.0188*pow(lpmair,3)+0.0409*pow(lpmair,2)+3.4796*lpmair+62.711; //ecuacion conversion lpm a pwm oxígeno aproximación cúbica
    //pwmox=3*pow(10,-9)*pow(lpmoxig,6)+5*pow(10,-7)*pow(lpmoxig,5)-.0002*pow(lpmoxig,4)+.012*pow(lpmoxig,3)-.4009*pow(lpmoxig,2)+6.9879*lpmoxig+31.917; //ecuacion conversion lpm a pwm aire

    Serial.print ("Flujo teorico oxigeno en pwm ");
    Serial.println(pwmox);
    Serial.print ("Flujo teorico aire en pwm ");
    Serial.println(pwmair);
    Serial.print ("Fio2 oxigeno ");
    Serial.println(lpmoxig);
    Serial.print ("Fio2 aire ");
    Serial.println(lpmair);*/
  //VCP (20,20,1, 6,21,10);
  //pruebasensores();
  /*flujo= 30;
    oxigeno=80;
    //ecuaciones fio2
    lpmair=(-flujo*(oxigeno-100))/80;
    lpmoxig=flujo-lpmair;
    //ecuaciones conversion lpm-pwm
    //pwmox=0.028*pow(lpmoxig,2)-0.2352*lpmoxig+136; //ecuacion conversion lpm a pwm oxígeno aproximación cuadrática
    pwmox=1*pow(10,-10)*pow(lpmair,6)-1*pow(10,-7)*pow(lpmair,5)+5*pow(10,-5)*pow(lpmair,4)-.0115*pow(lpmair,3)+1.306*pow(lpmair,2)-73.326*lpmair+1574.8; //ecuacion conversion lpm a pwm oxígeno aproximación cúbica
    pwmair=-5*pow(10,-11)*pow(lpmair,6)+4*pow(10,-8)*pow(lpmair,5)-1*pow(10,-5)*pow(lpmair,4)+.0017*pow(lpmair,3)-.0973*pow(lpmair,2)+2.2865*lpmair-16.17; //ecuacion conversion lpm a pwm aire

    Serial.print ("Flujo teorico oxigeno en pwm ");
    Serial.println(pwmox);
    Serial.print ("Flujo teorico aire en pwm ");
    Serial.println(pwmair);
    Serial.print ("Fio2 oxigeno ");
    Serial.println(lpmoxig);
    Serial.print ("Fio2 aire ");
    Serial.println(lpmair);

    analogWrite(AVPropAir,int (0));
    analogWrite(AVPropOx,int (0));
    digitalWrite(AVexh,LOW);
    //lectura sensores

    Serial.print("Pinspiratoria ");
    Serial.println (SensorPi());
    Serial.print("Poxigeno ");
    Serial.println (SensorPOx());
    Serial.print("Paire ");
    Serial.println (SensorPAir());
    TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
    FOx=SensorData();
    //lectura flujo aire
    TCA9548A(5); // select I2C bus 1 for the air flow sensor
    FAir=SensorData();
    Serial.print("pwmox ");
    Serial.println(FOx);
    Serial.print("pwmaire ");
    Serial.println(FAir);
    Serial.print("Os ");
    //Serial.println (analogRead(A4));
    Serial.println (SensorOs());
    Serial.print("Pexh ");
    Serial.println (SensorPexh());
    Serial.print("Flujo exh");
    TCA9548A(3); // select I2C bus 1 for the air flow sensor
    Fexh=SensorData();
    Serial.println(Fexh);
    delay(1000);
    analogWrite(AVPropAir,0);
    analogWrite(AVPropOx,0);
    digitalWrite(AVexh,HIGH);
    //lectura sensores

    Serial.print("Pinspiratoria ");
    Serial.println (SensorPi());
    Serial.print("Poxigeno ");
    Serial.println (SensorPOx());
    Serial.print("Paire ");
    Serial.println (SensorPAir());
    TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
    FOx=SensorData();
    //lectura flujo aire
    TCA9548A(5); // select I2C bus 1 for the air flow sensor
    FAir=SensorData();
    Serial.print("pwmox ");
    Serial.println(FOx);
    Serial.print("pwmaire ");
    Serial.println(FAir);
    Serial.print("Os ");
    Serial.println (SensorOs());
    //Serial.println (analogRead(A4));
    Serial.print("Pexh ");
    Serial.println (SensorPexh());
    Serial.print("Flujo exh ");
    TCA9548A(3); // select I2C bus 1 for the air flow sensor
    Fexh=SensorData();
    Serial.println(Fexh);

    delay(1000); */
  /*
    //VCP(40,60,0.5,2,2);
    analogWrite(AVPropAir,118);
    analogWrite(AVPropOx,118);
    digitalWrite(AVexh,HIGH);
    Pi=SensorPi();
    Serial.print("p");
    Serial.println(Pi);
    //Serial.print("f");
    //printSensorData();
    //Serial.println();
    Serial.print("Pexh");
    Serial.println (SensorPexh());
    delay(1000);
    analogWrite(AVPropAir,0);
    analogWrite(AVPropOx,0);
    digitalWrite(AVexh,LOW);
    Serial.print("p");
    Serial.println(Pi);
    Serial.print("Pexh");
    Serial.println (SensorPexh());
    delay(1000);*/
}

// cambio de escala entre floats
float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


float SensorPi()
{
  ANPi = analogRead(APi);          // realizar la lectura
  //Pi = fmap(ANPi, 940, 0, 0.0, 105); //cmh20
  Pi = (((ANPi) - 940) / (-8.95)) - 25;
  return Pi ;

}

float SensorPOx()
{
  ANPOx = analogRead(APOx);          // realizar la lectura
  POx = fmap(ANPOx, 0, 1023, 0, 100); //psi
  return POx ;

}

float SensorPAir()
{
  ANPAir = analogRead(APAir);          // realizar la lectura
  PAir = fmap(ANPAir, 0, 1023, 0, 100); //psi
  return PAir ;

}

float SensorOs()
{
  ANOs = analogRead(AOs);          // realizar la lectura
  Os = fmap(ANOs, airevalor, oxigenovalor, 21, 100); //% oxígeno
  return Os ;
  //1.1v-21
  //4.52 -100%

}

float SensorPexh()
{
  ANPexh = analogRead(APexh);          // realizar la lectura
  //Pexh = fmap(ANPexh, 204.8, 1023, -14, 140); //cmh20
  Pexh = (((ANPexh) - 940) / (-8.95)) - 18;
  return Pexh ;

}

void VCP (int presion, int frec, float ti, int PEEP, int oxigeno, int trigger) { //ver si hay que añadir alguna otra variable
  //Serial.println("Modo presion");
  ttotal = 60000 / frec;
  ti = ti * 1000;
  te = ttotal - ti;
  param = "";
  bandtrigger = 0;
  silenciador = 1;
  do {
    //alarmas suministros gases
    POx = SensorPOx();
    PAir = SensorPAir();
    PiFuga = 0;
    contadorintervalo = 0;
    int inicioflujo = 50;
    int contadorflujo = 0;
    int valflujo = 50;
    int presionalcanzada=0;
    //Controlador
    Setpoint = presion; 
    pidController.SetMode(AUTOMATIC);
    
    if ((PAir < 0)) {
      // if ((POx<30)||(PAir<10)){
      //if (PAir<30){
      //instruccion para encender buzzer y piloto
      digitalWrite(alarma, HIGH); //ENCIENDE ALARMA
      delay(1000);
      Serial.print("problema");
      entrada = "alto";
      Serial.print(",");
      Serial.print(POx);
      Serial.print
      (",");
      Serial.println(PAir);
    }
    //inhalación
    digitalWrite(AVexh, LOW);

    //da el porcentaje de acuerdo al fio2 programado
    if (oxigeno == 21){
     Serial.println("Solo oxigeno");
      calculopwm (0, inicioflujo);
    }//37 LPM aire}
    else {
      if (oxigeno == 100) {
        calculopwm (inicioflujo, 0); //37 LPM oxigeno
      }
      else {
        //ecuaciones fio2 con flujo 37, podría intentar dar
        //más flujo cambiando el 37, oxigeno es la variable
        lpmair = (-inicioflujo * (oxigeno - 100)) / 80;
        lpmoxig = inicioflujo - lpmair;
        calculopwm(lpmoxig, lpmair);
      }
    }
    inicio = millis();
    bandera = 0;
    while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'p') && ((millis() - inicio) < ti)) {

      lecturapresionesin();
      if (bandtrigger == 1) {
        Serial.print("l");
      }
      else
        Serial.print("x");

      //se agregó para mejorar tiempos en visual studio
      if (contadorintervalo != 0)
        contadorintervalo = millis() - contadorintervalo;
      else
        contadorintervalo = 100;
      Serial.print (contadorintervalo);
      contadorintervalo = millis();


      if (Serial.available() > 0)
      {
        entrada = Serial.readStringUntil('\n');
        Serial.println(entrada);
      }

      //PRUEBA SILENCIO DE ALARMA
      if (entrada == "silence") {
        silenciador = 0;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(alarma,HIGH); //este solo sirve para probar ahorita con el uno, pero debe quitarse
      }
      //meter alarma
      if (entrada == "alerta") { //esto debe gnerar algun problema
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }
      if (entrada == "ruido") { //esto debe generar algun problema
        silenciador = 1;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(13,LOW); //este hay que comentarla
      }
      
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') { //tambien es pbe para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') { //tambien es tac para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') { //tambien es tbn para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada.substring(0,3) == "fio") {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if ((entrada[2] == 'l')&& (entrada != "silence")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }
      
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }
      //ventana alarmas
      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }

      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }

      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.print((int)total);
          }
          else
            Serial.print(total);
          //en = 1;
          oldEncPos = encoderPos;
          entrada = "";
        }
      }
      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.print("enc1");
          bandclick = 1;
        }
      }
      if (bandclick == 0)
        Serial.print("enc0");
      estadoBotonAnterior = estadoBoton;

      if (en == 0)
        Serial.println();
      en = 0;
      bandclick = 0;

     //filtros modos ventilatorios
     if (entrada[0] == '2') { 
        parametrosvolumen(entrada);
      }
      if (entrada[0] == '3') { 
        parametrosSIMVpresion(entrada);
      }
      if (entrada[0] == '4') { 
        parametrosSIMVvolumen(entrada);
      }
      if (entrada[0] == '5') { 
        parametrosAPRV(entrada);
      }
      if (entrada[0] == '6') { 
        parametrosBIPAP(entrada);
      }
      if (entrada[0] == '7') { 
        parametrosSPONT(entrada);
      }
     //Input = Pi ;
     //pidController.Compute();         // actualizar el PID
     //calculopwm(0,Output);

     
     if (presionalcanzada==0){
     //generación de rampa desacelerante
        contadorflujo += 3; //meter el tiempo y dividirlo entre un numero de veces(la pendiente)
        valflujo = (presion-10) * pow(1 / 1.1, contadorflujo);
        //valflujo=valflujo-;
        if (valflujo<=0)
        valflujo=0;
        if (oxigeno == 21)
        calculopwm (0, valflujo);
        else{
          lpmair = (-valflujo * (oxigeno - 100)) / 80;
          lpmoxig = valflujo - lpmair;
          calculopwm(lpmoxig, lpmair);
        }

        //verificación de la presión alcanzada
      if (Pi >= presion) {
        analogWrite(AVPropAir, 0);
        analogWrite(AVPropOx, 0);
        presionalcanzada=1;
      }
     }
      delay(100);
    }
    if ((bandera == 0) && (silenciador == 1) && (entrada != "alto")) { //agregue lo de alto ara que no mande fuga
      digitalWrite(alarma, HIGH); //ENCIENDE ALARMA
      Serial.print("fuga\n");
    }
    else {
      digitalWrite(alarma, LOW);
      if (entrada != "alto")
        Serial.print("nofuga\n");
    }
    pidController.SetMode(MANUAL);
    //EXHALACIÓN
    digitalWrite(AVexh, HIGH);
    analogWrite(AVPropAir, 0);
    analogWrite(AVPropOx, 0);
    bandtrigger = 0;
    inicio = millis();
    contadorintervalo = 0;

    while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'p') && ((millis() - inicio) < te) && (bandtrigger == 0)) { //agregar condición de trigger para modo ac
      Pi = SensorPi();
      Os = SensorOs();

      //respiración realizada por el px.
      if (Pi < (PEEP - trigger))
        bandtrigger = 1;

      if (Pi < PEEP) { //aquí ver si es con Pexh una mejor respuesta
        digitalWrite(AVexh, LOW);
      }

      Serial.print("p");
      Serial.print(Pi);
      Serial.print("f");
      TCA9548A(3); // select I2C bus 2 for the exh flow sensor
      Fexh = SensorData();
      Serial.print(Fexh);
      Serial.print("o");
      Serial.print(Os);
      Serial.print("y");

      //envío de tiempo transcurrido en ms
      if (contadorintervalo != 0)
        contadorintervalo = millis() - contadorintervalo;
      else
        contadorintervalo = 100;
      Serial.print (contadorintervalo);
      contadorintervalo = millis();

      if (Serial.available() > 0)
      {
        entrada = Serial.readStringUntil('\n');
        //Serial.println(entrada);
      }
      if (entrada == "silence") {
        //bandera=0;
        silenciador = 0;
        entrada = ""; //para que no este leyendo nada
      }

      //meter alarma
      if (entrada == "alerta") { //esto debe gnerar algun problema
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }
      if (entrada == "ruido") { //esto debe generar algun problema
        silenciador = 1;
        entrada = ""; //para que no este leyendo nada
      }

      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada.substring(0,3) == "fio") {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if ((entrada[2] == 'l')&& (entrada != "silence")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }

      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }
      //ventana alarmas
      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.print((int)total);
          }
          else
            Serial.print(total);
          //en = 1;
          oldEncPos = encoderPos;
          entrada = "";
        }
      }
      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.print("enc1");
          bandclick = 1;
        }
      }
      if (bandclick == 0)
        Serial.print("enc0");
      estadoBotonAnterior = estadoBoton;

      if (en == 0)
        Serial.println();
      en = 0;
      bandclick = 0;
      //filtros modos ventilatorios
     if (entrada[0] == '2') { 
        parametrosvolumen(entrada);
      }
      if (entrada[0] == '3') { 
        parametrosSIMVpresion(entrada);
      }
      if (entrada[0] == '4') { 
        parametrosSIMVvolumen(entrada);
      }
      if (entrada[0] == '5') { 
        parametrosAPRV(entrada);
      }
      if (entrada[0] == '6') { 
        parametrosBIPAP(entrada);
      }
      if (entrada[0] == '7') { 
        parametrosSPONT(entrada);
      }
      delay(100);
    }
  } while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'p'));

//en caso de actualizacion de parámetros
  if (entrada[3]=='p')
  parametrospresion(entrada);
  
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  digitalWrite(alarma, LOW);
  digitalWrite(AVexh, HIGH); //abre la valvula exhalatoria
  
}
void VCV (int volumen, int frec, float ti, int PEEP, int oxigeno, int trigger, int flujo) { //ver si hay que añadir algun otra variable
  //Serial.println("Modo volumen");
  ttotal = 60000 / frec;
  ti = ti * 1000;
  te = ttotal - ti;
  param = "";
  bandtrigger = 0;
  silenciador = 1;
  bandvol = 0;
  volumencalc = 0;
  do {
    //ESTA DECLARACIÓN ES TEMPORAL o asegurando que lo que sale siempre es cero.
    volumencalc = 0;
    //alarmas suministros gases
    POx = SensorPOx();
    PAir = SensorPAir();
    volFuga = 0;
    PiFuga = 0; //se agrega para en caso de presion baja también marque que hay fuga.
    contadorintervalo = 0;
    //volumencalc=0; no se hace cero ya que debe ser en base al volumen real anterior
    //si esto hace eso es porque hay fuga
    //if ((POx<30)){
    // if ((POx<30)||(PAir<10)){
    if (PAir < 0) {
      //instruccion para encender buzzer y piloto
      digitalWrite(alarma, HIGH); //ENCIENDE ALARMA
      delay(1000);
      Serial.print("problema");
      entrada = "alto";
      Serial.print(",");
      Serial.print(POx);
      Serial.print
      (",");
      Serial.println(PAir);
    }
    //inhalación
    digitalWrite(AVexh, LOW);
    //da el porcentaje de acuerdo al fio2 programado
    if (oxigeno == 21) {
      //ecuaciones fio2
      lpmair = flujo;
      lpmoxig = 0;
      calculopwm(lpmoxig, lpmair);
    }
    else {
      if (oxigeno == 100) {
        //ecuaciones fio2
        lpmair = 0;
        lpmoxig = flujo - lpmair;
        //ecuaciones conversion lpm-pwm
        calculopwm(lpmoxig, lpmair);
      }
      else {
        //ecuaciones fio2
        lpmair = (-flujo * (oxigeno - 100)) / 80;
        lpmoxig = flujo - lpmair;
        //ecuaciones conversion lpm-pwm
        calculopwm(lpmoxig, lpmair);
      }
    }

    inicio = millis();
    iniciovol = millis();
    bandera = 0;

    while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'v') && ((millis() - inicio) < ti)) {

      //lectura sensores: presión inspiratoria, flujo aire, flujo oxígeno y sensor de oxígeno
      Pi = SensorPi();
      Os = SensorOs();
      //impresión de gráficas
      Serial.print("p");
      Serial.print(Pi);
      Serial.print("f");
      //lectura flujo oxigeno
      TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
      FOx = SensorData();
      //Serial.println("Flujo Ox");
      //Serial.println(FOx);
      //lectura flujo aire
      TCA9548A(5); // select I2C bus 1 for the air flow sensor
      FAir = SensorData();
      //Serial.println("Flujo Aire");
      //Serial.println(FAir);
      Serial.print(FAir + FOx);
      Serial.print("o");
      Serial.print(Os);

      //Alarma Fuga
      //Serial.println("Multiplicacion");
      //Serial.println((FAir + FOx) *(millis() - iniciovol) * 0.016);
      volumencalc = volumencalc + ((FAir + FOx) * (millis() - iniciovol) * 0.016); //en ml Svolumencalc+
      iniciovol = millis();

      if (volumencalc > volFuga) {
        volFuga = volumencalc;
      }

      if ((volFuga > (volumen * 0.60)) && (bandera != 1)) { //probar esta que puede funcionar, fnciona con ese
        bandera = 1;
        digitalWrite(alarma, LOW);
      }


      if (bandtrigger == 1) {
        Serial.print("l");
      }
      else
        Serial.print("x");
      //envio de tiempo a vstudio
      if (contadorintervalo != 0)
        contadorintervalo = millis() - contadorintervalo;
      else
        contadorintervalo = 100;
      Serial.print (contadorintervalo);
      contadorintervalo = millis();

      if (Serial.available() > 0)
      {
        entrada = Serial.readStringUntil('\n');
      }
      //Serial.println ("Volumen calculado");
      //Serial.println (volumencalc);
      //Serial.println ("Volumen seteado");
      //Serial.println (volumen);
      if (volumencalc > volumen) {
        analogWrite(AVPropAir, 0); //estaba en 3.8
        analogWrite(AVPropOx, 0);
      }

      //PRUEBA SILENCIO DE ALARMA
      if (entrada == "silence") {
        silenciador = 0;
        entrada = ""; //para que no este leyendo nada
      }
      //meter alarma
      if (entrada == "alerta") {
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }
      if (entrada == "ruido") {
        silenciador = 1;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(13,LOW); //este hay que comentarla
      }
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') { //tambien es pbe para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') { //tambien es tac para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') { //tambien es tbn para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada.substring(0,3) == "fio") {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if ((entrada[2] == 'l')&& (entrada != "silence")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }
      
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }

      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }

      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }

      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }

      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.print((int)total);
          }
          else
            Serial.print(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }

      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.print("enc1");
          bandclick = 1;
        }
      }
      if (bandclick == 0)
        Serial.print("enc0");
      estadoBotonAnterior = estadoBoton;

      if (en == 0)
        Serial.println();
      en = 0;
      bandclick = 0;
      //cambio modos
      if (entrada[0] == '1') { 
        parametrospresion(entrada);
      }
      if (entrada[0] == '3') { 
        parametrosSIMVpresion(entrada);
      }
      if (entrada[0] == '4') { 
        parametrosSIMVvolumen(entrada);
      }
      if (entrada[0] == '5') { 
        parametrosAPRV(entrada);
      }
      if (entrada[0] == '6') { 
        parametrosBIPAP(entrada);
      }
      if (entrada[0] == '7') { 
        parametrosSPONT(entrada);
      }
      delay(100);
    }
    if ((bandera == 0) && (silenciador == 1) && (entrada != "alto")) {
      digitalWrite(alarma, HIGH); //ENCIENDE ALARMA
      Serial.print("fuga\n");
    }
    else {
      digitalWrite(alarma, LOW);
      if (entrada != "alto")
        Serial.print("nofuga\n");
    }
    //EXHALACIÓN
    digitalWrite(AVexh, HIGH);
    analogWrite(AVPropAir, 0);
    analogWrite(AVPropOx, 0);
    bandtrigger = 0;
    bandvol = 0;
    inicio = millis();
    iniciovol = millis();
    contadorintervalo = 0;

    while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'v') && ((millis() - inicio) < te) && (bandtrigger == 0)) { //agregar condición de trigger para modo ac
      Pi = SensorPi();
      Os = SensorOs();
      //lectura flujo oxigeno
      TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
      FOx = SensorData();
      //lectura flujo aire
      TCA9548A(5); // select I2C bus 1 for the air flow sensor
      FAir = SensorData();

      //respiración realizada por el px.
      if (((FAir + FOx) > trigger) && (bandvol == 1)) {
        bandtrigger = 1;
        bandvol = 0;
      }
      if (Pi < PEEP) { //aquí ver si es con Pexh una mejor respuesta
        digitalWrite(AVexh, LOW);
        bandvol = 1;
      }

      Serial.print("p");
      Serial.print(Pi);
      Serial.print("f");
      TCA9548A(3); // select I2C bus 2 for the exh flow sensor
      Fexh = SensorData();
      Serial.print(Fexh);
      Serial.print("o");
      Serial.print(Os);
      Serial.print("y");
      //envia tiempo en ms a vstudio
      if (contadorintervalo != 0)
        contadorintervalo = millis() - contadorintervalo;
      else
        contadorintervalo = 100;
      Serial.print (contadorintervalo);
      contadorintervalo = millis();

      //calculo de volumen exh
      volumencalc = volumencalc - ((Fexh) * (millis() - iniciovol) * 0.016);
      iniciovol = millis();

      if (Serial.available() > 0)
      {
        entrada = Serial.readStringUntil('\n');
      }
      if (entrada == "silence") {
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }

      //meter alarma
      if (entrada == "alerta") { //esto debe gnerar algun problema
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }
      if (entrada == "ruido") { //esto debe generar algun problema
        silenciador = 1;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(13,LOW); //este hay que comentarla
      }

      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') { //tambien es pbe para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') { //tambien es tac para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') { //tambien es tbn para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada.substring(0,3) == "fio") {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if ((entrada[2] == 'l')&& (entrada != "silence")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }
      
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }

      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }

      //de ventana de alarmas
      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.print((int)total);
          }
          else
            Serial.print(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }

      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.print("enc1");
          bandclick = 1;
        }
      }
      if (bandclick == 0)
        Serial.print("enc0");
      estadoBotonAnterior = estadoBoton;

      if (en == 0)
        Serial.println();
      en = 0;
      bandclick = 0;

      if (entrada[0] == '1') { 
        parametrospresion(entrada);
      }
      if (entrada[0] == '3') { 
        parametrosSIMVpresion(entrada);
      }
      if (entrada[0] == '4') { 
        parametrosSIMVvolumen(entrada);
      }
      if (entrada[0] == '5') { 
        parametrosAPRV(entrada);
      }
      if (entrada[0] == '6') { 
        parametrosBIPAP(entrada);
      }
      if (entrada[0] == '7') { 
        parametrosSPONT(entrada);
      }
      
      delay(100);
    }
  } while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'v'));
  
  //en caso de actualizacion de parámetros
  if (entrada[3]=='v')
  parametrosvolumen(entrada);
  
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  digitalWrite(alarma, LOW);
  digitalWrite(AVexh, HIGH);
  param = ""; //la limpieza de entrada se hace hasta el final, igual puede quitarse este
}

void lecturadatos(void) {
  while (true){
  //Serial.println("Ventanadatos");
  if (Serial.available() > 0)
  {
    entrada = Serial.readStringUntil('\n');
    //Serial.println(entrada);
  }
  //una vez finalizada la lectura de los parámetros de entrada del ventilador
  if ((entrada != "") || (param != "")) {
    modo = entrada[3]; //para ver si es modo volumen o presión
    if (modo == "w") {
      // parametrosvolumen(entrada);
      testinicial();
    }
    //modo vcvac
    if (modo == "v") {
      // parametrosvolumen(entrada);
      ventanavolumen();
    }
    //modo vcpac
    if (modo == "p") {
      //parametrospresion(entrada);
      ventanapresion();
    }
    if (modo == "x") {
      ventanasimvp();
    }
    if (modo == "j") {
      ventanasimvv();
    }
    if (modo == "r") {
      ventanaaprv();
    }
    if (modo == "b") {
      ventanabipap();
    }
    if (modo == "l") {
      ventanaspont();
    }

    //autotest
    if (modo == "t") {
      autotest();
    }
    //calib oxigeno
    if (modo == "c") {
      calibox();
    }
    //calib flujo
    if (modo == "f") {
      calibfluj();
    }
    if (modo == "a") {
      ventanalarmas();
    }
    //perilla peso
    if ((modo == "s") || (param != "")) {
      //encoderPos=0;
      if (entrada != "") {
        auxperilla = encoderPos;
        param = entrada;
        auxenc = 1;
      }
      if (oldEncPos != encoderPos) {
        total = (param.substring(4, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
        Serial.print("ppppes");
        //delay(10);
        Serial.println((int)total);
        oldEncPos = encoderPos;
        entrada = "";
      }
    }
  }
  delay(10);
  }
}

void parametrospresion(String cadena) {
  //para el modo presión
  String stroxigeno, strPEEP, strTi, strfrec, strpresion, strtrigger;
  int aoxigeno, aPEEP, aTi, afrec, apresion, atrigger;
  for (int i = 0; i < cadena.length(); i++) {
    if (cadena[i] == 's') //parámetro de presion
      apresion = i;
    if (cadena[i] == 'x') //parámetro de porcentaje oxigeno
      aoxigeno = i;
    if (cadena[i] == 'E') //parámetro de PEEP
      aPEEP = i;
    if (cadena[i] == 'i') //parámetro de Ti
      aTi = i;
    if (cadena[i] == 'r') //parámetro de frecuencia respiratoria
      afrec = i;
    if (cadena[i] == 'n') //parámetro de trigger
      atrigger = i;
  }
  strpresion = cadena.substring(apresion + 1, aoxigeno - 1); //ya que el parametro viene como Ox
  stroxigeno = cadena.substring(aoxigeno + 1, aPEEP - 1); //ya que el parametro viene como PE
  strPEEP = cadena.substring(aPEEP + 1, aTi - 1);
  strTi = cadena.substring(aTi + 1, afrec - 1);
  strfrec = cadena.substring(afrec + 1, atrigger - 1);
  strtrigger = cadena.substring(atrigger + 1, cadena.length());
  //convertir a entero todos los valores
  presion = strpresion.toInt();
  oxigeno = stroxigeno.toInt();
  PEEP = strPEEP.toInt();
  ti = strTi.toFloat();
  frec = strfrec.toFloat();
  trigger = strtrigger.toInt();
  //lineas auxiliares para corroborar correcta lectura y guardado de datos
  //if (frec==12){
  //digitalWrite (13,HIGH);
  //}
  cadena = "";
  entrada = "";
  VCP (presion, frec, ti, PEEP, oxigeno, trigger);
  if (entrada=="fuda"){
  entrada="";
  lecturadatos();
  }
  else
  ventanapresion();
  // Serial.println ("modo:"+modo+"Presion:" +strpresion+"oxigeno:"+stroxigeno+"PEEP:"+strPEEP+"Tinspi:"+strTi+"frecuencia:"+strfrec);
}

void parametrosvolumen (String cadena) {
  //para el modo volumen
  String stroxigeno, strPEEP, strTi, strfrec, strflujo, strvolumen, strtrigger;
  int aoxigeno, aPEEP, aTi, afrec, aflujo, avolumen, atrigger;

  for (int i = 0; i < cadena.length(); i++) {
    if (cadena[i] == 'l') //parámetro de flujo
      aflujo = i;
    if (cadena[i] == 'x') //parámetro de porcentaje oxigeno
      aoxigeno = i;
    if (cadena[i] == 'E') //parámetro de PEEP
      aPEEP = i;
    if (cadena[i] == 'i') //parámetro de Ti
      aTi = i;
    if (cadena[i] == 'r') //parámetro de frecuencia respiratoria
      afrec = i;
    if (cadena[i] == 'n') //parámetro de trigger
      atrigger = i;
  }
  strflujo = cadena.substring(aflujo + 1, aoxigeno - 1); //ya que el parametro viene como Ox
  stroxigeno = cadena.substring(aoxigeno + 1, aPEEP - 1); //ya que el parametro viene como PE
  strPEEP = cadena.substring(aPEEP + 1, aTi - 1);
  strTi = cadena.substring(aTi + 1, afrec - 1);
  strfrec = cadena.substring(afrec + 1, atrigger - 1);
  strtrigger = cadena.substring(atrigger + 1, cadena.length());

  trigger = strtrigger.toInt();
  oxigeno = stroxigeno.toInt();
  PEEP = strPEEP.toInt();
  ti = strTi.toFloat();
  frec = strfrec.toFloat();
  flujo = strflujo.toInt();
  volumen = (flujo * ti * 16.6);
  //if (frec==12){
  //digitalWrite (13,HIGH);
  //}
  cadena = "";
  entrada = "";
  //llamar funcion VCV
  VCV (volumen, frec, ti, PEEP, oxigeno, trigger, flujo);
  //Serial.println ("modo:"+modo+"flujo:" +flujo+"oxigeno:"+oxigeno+"PEEP:"+PEEP+"Tinspi:"+ti+"frecuencia:"+frec+"trigger"+trigger+"volumen"+volumen);
  //delay(5000);
  if (entrada=="fuda"){
  entrada="";
  lecturadatos();
  }
  else
  ventanavolumen();
}
float getChecksum() { //Verify the EEPROM checksum against factory configuration.
  Wire.beginTransmission(0x49);
  Wire.write(0x03);
  delay(1);
  Wire.requestFrom(0x49, 0x02); // Request the transmitted two bytes from the two registers
  delay(1);
  if (Wire.available() <= 2) { //
    X0 = Wire.read(); // Reads the data from the register
    X1 = Wire.read();
  }
  Wire.endTransmission();
  float checksum = word(X0, X1); //Combine two bytes recieved.
  if (checksum == 52389) { //##From datasheet
    Serial.println("EEPROM Checksum match!");
  }
  else if (checksum == 52368) { //##From datasheet
    Serial.println("EEPROM Checksum error");
  }
}

float getFlowReading() {
  Wire.beginTransmission(0x49);
  delay(1);
  Wire.requestFrom(0x49, 0x02); // Request the transmitted two bytes from the two registers

  if (Wire.available() <= 2) { //
    X0 = Wire.read(); // Reads the data from the register
    X1 = Wire.read();
  }
  Wire.endTransmission();
  //X0=float(X0);
  //X1=float(X1);
  float output = word(X0, X1); //Combine two bytes recieved.
  //float output = X0+X1;
  float flow = flowRange * ((output / 16384) - 0.1) / 0.8; // From datasheet
  float conversionFactor = 1; //en SLPM
  //float conversionFactor = ( (273.15 / 293.15) * (14.696 / 14.504) ); //Converting SLPM to NLPM;
  flow = flow * conversionFactor;
  //if ((flow>200) || (flow<0.7))
  //flow = 0;
  //flow = 3.465 * pow (flow, 0.7679); //conversion del valor dado por el sensor al valor de flujo leido por el analizador
  return flow;
}


float getError(float flow) { //#Calculate error for readings lower and higher than 12.5% of flowrange. (Datasheet spec)
  float TEB;
  if (flow < flowRange * 0.125) {
    TEB = flowRange * 0.005;
  }
  else  {
    TEB = 0.04 * flow;
  }
  return TEB;
}

float SensorData()
{
  float flow = getFlowReading();
  return flow;
}

void TCA9548A(uint8_t bus)
{
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

void pruebasensores() {

  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 100);
  digitalWrite (AVexh, HIGH);

  Pi = SensorPi();
  Serial.print("Presión inspiratoria ");
  Serial.println(Pi);

  PAir = SensorPAir();
  Serial.print("Presión de aire ");
  Serial.println(PAir);

  POx = SensorPOx();
  Serial.print("Presión oxigeno ");
  Serial.println(POx);

  Os = SensorOs();
  Serial.print("% Oxigeno ");
  Serial.println(Os);

  Pexh = SensorPexh();
  Serial.print("Presión exhalatoria ");
  Serial.println(Pexh);

  Serial.print("Sensor flujo exh ");
  TCA9548A(4);
  Serial.println(SensorData());

  Serial.print("Sensor flujo oxig ");
  TCA9548A(5);
  Serial.println(SensorData());
  Serial.print("Sensor flujo aire ");
  TCA9548A(3);

  Serial.println(SensorData());
  delay(1000);
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  digitalWrite (AVexh, LOW);
  delay(1000);
}


void PinA() {
  cli(); //stop interrupts happening before we read pin values
  reading = PINE & 0x30; // read all eight pin values then strip away all but pinA and pinB's values
  if (reading == 0x30 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos --; //decrement the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00010000) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

void PinB() {
  cli(); //stop interrupts happening before we read pin values
  reading = PINE & 0x30; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == 0x30 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos ++; //increment the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00100000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

void lecturapresionesin() {
  //lectura sensores: presión inspiratoria, flujo aire, flujo oxígeno y sensor de oxígeno
  Pi = SensorPi();
  Os = SensorOs();
  //impresión de gráficas
  Serial.print("p");
  Serial.print(Pi);
  Serial.print("f");
  //lectura flujo oxigeno
  TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
  FOx = SensorData();
  //lectura flujo aire
  TCA9548A(5); // select I2C bus 1 for the air flow sensor
  FAir = SensorData();
  Serial.print(FAir + FOx);
  Serial.print("o");
  Serial.print(Os);

  //Alarma Fuga
  if (Pi > PiFuga) {
    PiFuga = Pi;
  }
  //if ((Pi-Pexh)>5)&&(bandera!=1)){
  if ((PiFuga > (presion * 0.60)) && (bandera != 1)) { //probar esta que puede funcionar, fnciona con ese
    //valor de amplificacion de las valvulas
    bandera = 1;
    digitalWrite(alarma, LOW);
  }
}

void autotest() {
  bandtest1 = 0;
  bandtest3 = 0;
  Serial.println ("Preparando equipo para autotest...");
  delay(3000);
  Serial.println ("Test 1");
  calculopwm(5, 5);
  digitalWrite(AVexh, LOW);
  inicio = millis();
  while ((millis() - inicio) < 2000) {
    Pi = SensorPi();
    Os = SensorOs();
    //lecturapwmoxigeno
    POx = SensorPOx();
    PAir = SensorPAir();
    Serial.print("Presion de entrada oxigeno: ");
    Serial.print(POx);
    Serial.println(" cmH20");
    Serial.print("Presion de entrada aire: ");
    Serial.print(FAir);
    Serial.println(" cmH20");
    TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
    FOx = SensorData();
    Serial.print("Sensor de flujo oxigeno: ");
    Serial.print(FOx);
    Serial.println(" L/min");
    //lectura flujo aire
    TCA9548A(5); // select I2C bus 1 for the air flow sensor
    FAir = SensorData();
    Serial.print("Sensor de flujo aire: ");
    Serial.print(FAir);
    Serial.println(" L/min");
    Serial.print("Presion insiratoria: ");
    Serial.print(Pi);
    Serial.println(" cmH20");
    Serial.print("Sensor de oxigeno: ");
    Serial.print(Os);
    Serial.println(" %");
    Serial.print("Presion exhalatoria: ");
    Serial.print(SensorPexh());
    Serial.println(" cmH20");
    TCA9548A(3); // select I2C bus 1 for the oxygen flow sensor
    Fexh = SensorData();
    Serial.print("Sensor de flujo exhalatorio: ");
    Serial.print(Fexh);
    Serial.println(" L/min");
    if (FOx > 200) {
      Serial.println("Falla en sensor de flujo oxigeno");
    }
    if (FAir > 200) {
      Serial.println("Falla en sensor de flujo aire");
    }
    if (Fexh > 200) {
      Serial.println("Falla en sensor de flujo exhalatorio");
    }
    if ((FOx > 200) || (FAir > 200) || (Fexh > 200))
      bandtest1 = 1;
    else
      bandtest1 = 0;

    delay(500);
  }
  if (bandtest1 == 0)
    Serial.println ("Test 1.PASO");
  else
    Serial.println ("Test 1.FALLO");
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  delay(3000);
  Serial.println("Test 2");
  delay(500);
  Serial.println("Test de alarmas");
  digitalWrite(AVexh, HIGH);
  digitalWrite(alarma, HIGH);
  Serial.println("Alarma activada");
  delay(2000);
  Serial.println("Alarma silenciada");
  delay(1000);
  Serial.println("Test 2.PASO");
  delay(1000);
  Serial.println ("Test 3");
  digitalWrite(alarma, LOW);
  calculopwm(15, 15);
  digitalWrite(AVexh, LOW);
  inicio = millis();
  while ((millis() - inicio) < 2000) {
    Pi = SensorPi();
    Os = SensorOs();
    POx = SensorPOx();
    PAir = SensorPAir();
    Serial.print("Presion de entrada oxigeno: ");
    Serial.print(POx);
    Serial.println(" cmH20");
    Serial.print("Presion de entrada aire: ");
    Serial.print(FAir);
    Serial.println(" cmH20");
    TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
    FOx = SensorData();
    Serial.print("Sensor de flujo oxigeno: ");
    Serial.print(FOx);
    Serial.println(" L/min");
    //lectura flujo aire
    TCA9548A(5); // select I2C bus 1 for the air flow sensor
    FAir = SensorData();
    Serial.print("Sensor de flujo aire: ");
    Serial.print(FAir);
    Serial.println(" L/min");
    Serial.print("Presion insiratoria: ");
    Serial.print(Pi);
    Serial.println(" cmH20");
    Serial.print("Sensor de oxigeno: ");
    Serial.print(Os);
    Serial.println(" %");
    Serial.print("Presion exhalatoria: ");
    Serial.print(SensorPexh());
    Serial.println(" cmH20");
    TCA9548A(3); // select I2C bus 1 for the oxygen flow sensor
    Fexh = SensorData();
    Serial.print("Sensor de flujo exhalatorio: ");
    Serial.print(Fexh);
    Serial.println(" L/min");
    if (FOx > 200) {
      Serial.println("Falla en sensor de flujo oxigeno");
    }
    if (FAir > 200) {
      Serial.println("Falla en sensor de flujo aire");
    }
    if (Fexh > 200) {
      Serial.println("Falla en sensor de flujo exhalatorio");
    }
    if ((FOx > 200) || (FAir > 200) || (Fexh > 200))
      bandtest3 = 1;
    else
      bandtest3 = 0;
    delay(500);
  }
  if (bandtest3 == 0)
    Serial.println ("Test 3.PASO");
  else
    Serial.println ("Test 3.FALLO");
  calculopwm(0, 0);
  delay(3000);
  digitalWrite(alarma, LOW);
  digitalWrite(AVexh, HIGH);
  delay(2000);
  if ((bandtest1 == 0) && (bandtest3 == 0)) {
    Serial.println ("Prueba finalizada");
  }
  else
    Serial.println ("Prueba fallida");
  entrada = "";
  bandtest1 = 0;
  bandtest3 = 0;
}
void calibox() {
  entrada = "";
  Serial.println ("Preparando equipo para calibracion...");
  Serial.println ("Desconecte circuito ventilatorio");
  delay(3000);
  calculopwm(0, 5);
  digitalWrite(AVexh, LOW);
  inicio = millis();
  Serial.println("Calibracion a 21 %");
  while ((millis() - inicio) < 2000) {
    airevalor = analogRead(AOs);
    Serial.print(airevalor);
    Serial.println(" %");
    delay(500);
  }
  //guardar en eeprom el valor de aire 21%
  EEPROM.update( airAddress, airevalor );//sino funciona probar con put

  Serial.println("Calibracion a 21 % finalizada");
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  delay (3000);
  digitalWrite(AVexh, HIGH);
  delay (2000);

  Serial.println("Conecte circuito ventilatorio.De click en continuar");
  while ((entrada == "") && (entrada != "siguiente")) {
    if (Serial.available() > 0)
    {
      entrada = Serial.readStringUntil('\n');
    }
    delay(200);
  }

  Serial.println("Calibracion al 100%");
  calculopwm(15, 0);
  digitalWrite(AVexh, LOW);
  inicio = millis();
  while ((millis() - inicio) < 4000) {
    oxigenovalor = analogRead(AOs);
    Serial.print(oxigenovalor);
    Serial.println(" %");
    delay(500);
  }
  //guardar en eeprom el valor de aire 100%
  EEPROM.update( oxigAddress, oxigenovalor ); //sino funciona probar con put
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  delay (3000);
  digitalWrite(AVexh, HIGH);
  delay (2000);
  Serial.println("Calibracion finalizada");
  entrada = "";
}
void calibfluj() {
  bandtest1 = 0;
  bandtest2 = 0;
  bandtest3 = 0;
  Serial.println ("Preparando equipo para calibracion...");
  Serial.println ("Conecte circuito ventilatorio");
  delay(3000);
  Serial.println("Test 1");
  Serial.println("Test sensor de flujo oxigeno");
  calculopwm(15, 0);
  digitalWrite(AVexh, LOW);
  inicio = millis();
  while ((millis() - inicio) < 2000) {
    TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
    FOx = SensorData();
    Serial.print("Sensor de flujo oxigeno: ");
    Serial.print(FOx);
    Serial.println(" L/min");
    //lectura flujo aire
    TCA9548A(5); // select I2C bus 1 for the air flow sensor
    FAir = SensorData();
    Serial.print("Sensor de flujo aire: ");
    Serial.print(FAir);
    Serial.println(" L/min");
    if (FOx > 200) {
      Serial.println("Falla en sensor de flujo oxigeno");
    }
    if (FAir > 200) {
      Serial.println("Falla en sensor de flujo aire");
    }
    if ((FOx > 200) || (FAir > 200))
      bandtest1 = 1;
    else
      bandtest1 = 0;
    delay(500);
  }
  Serial.println("Test sensor de flujo exhalatorio");
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  delay (3000);
  digitalWrite(AVexh, HIGH);
  inicio = millis();
  while ((millis() - inicio) < 2000) {
    //lecturapwmoxigeno
    TCA9548A(3); // select I2C bus 1 for the oxygen flow sensor
    Fexh = SensorData();
    Serial.print("Sensor de flujo exhalatorio: ");
    Serial.print(Fexh);
    Serial.println(" L/min");
    if (Fexh > 200) {
      Serial.println("Falla en sensor de flujo exhalatorio");
      bandtest1 = 1 + bandtest1;
    }
    else
      bandtest1 = 0 + bandtest1;
    delay(500);
  }
  if (bandtest1 == 0)
    Serial.println("Test 1.PASO");
  else
    Serial.println("Test1.FALLO");
  delay (3000);
  Serial.println("Test 2");
  Serial.println("Test sensor de flujo aire");
  calculopwm(0, 15);
  digitalWrite(AVexh, LOW);
  inicio = millis();
  while ((millis() - inicio) < 2000) {
    TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
    FOx = SensorData();
    Serial.print("Sensor de flujo oxigeno: ");
    Serial.print(FOx);
    Serial.println(" L/min");
    //lectura flujo aire
    TCA9548A(5); // select I2C bus 1 for the air flow sensor
    FAir = SensorData();
    Serial.print("Sensor de flujo aire: ");
    Serial.print(FAir);
    Serial.println(" L/min");
    if (FOx > 200) {
      Serial.println("Falla en sensor de flujo oxigeno");
    }
    if (FAir > 200) {
      Serial.println("Falla en sensor de flujo aire");
    }

    if ((FOx > 200) || (FAir > 200))
      bandtest2 = 1;
    else
      bandtest2 = 0;
    delay(500);
  }

  Serial.println("Test sensor de flujo exhalatorio");
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  delay (3000);
  digitalWrite(AVexh, HIGH);
  inicio = millis();
  while ((millis() - inicio) < 2000) {
    //lecturapwmoxigeno
    TCA9548A(3); // select I2C bus 1 for the oxygen flow sensor
    Fexh = SensorData();
    Serial.print("Sensor de flujo exhalatorio: ");
    Serial.print(Fexh);
    Serial.println(" L/min");
    if (Fexh > 200) {
      Serial.println("Falla en sensor de flujo exhalatorio");
      bandtest2 = 1 + bandtest2;
    }
    else
      bandtest2 = 0 + bandtest2;
    delay(500);
  }
  if (bandtest2 == 0)
    Serial.println("Test 2.PASO");
  else
    Serial.println("Test 2.FALLO");
  delay (3000);

  Serial.println("Test 3");
  Serial.println("Test sensor de flujo aire y oxígeno");
  calculopwm(15, 15);
  digitalWrite(AVexh, LOW);
  inicio = millis();
  while ((millis() - inicio) < 2000) {
    TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
    FOx = SensorData();
    Serial.print("Sensor de flujo oxigeno: ");
    Serial.print(FOx);
    Serial.println(" L/min");
    //lectura flujo aire
    TCA9548A(5); // select I2C bus 1 for the air flow sensor
    FAir = SensorData();
    Serial.print("Sensor de flujo aire: ");
    Serial.print(FAir);
    Serial.println(" L/min");
    if (FOx > 200) {
      Serial.println("Falla en sensor de flujo oxigeno");
    }
    if (FAir > 200) {
      Serial.println("Falla en sensor de flujo aire");
    }

    if ((FOx > 200) || (FAir > 200))
      bandtest3 = 1;
    else
      bandtest3 = 0;
    delay(500);
  }

  Serial.println("Test sensor de flujo exhalatorio");
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  delay (3000);
  digitalWrite(AVexh, HIGH);
  inicio = millis();
  while ((millis() - inicio) < 2000) {
    //lecturapwmoxigeno
    TCA9548A(3); // select I2C bus 1 for the oxygen flow sensor
    Fexh = SensorData();
    Serial.print("Sensor de flujo exhalatorio: ");
    Serial.print(Fexh);
    Serial.println(" L/min");
    if (Fexh > 200) {
      Serial.println("Falla en sensor de flujo exhalatorio");
      bandtest3 = 1 + bandtest3;
    }
    else
      bandtest3 = 0 + bandtest3;
    delay(500);
  }
  if (bandtest3 == 0)
    Serial.println("Test 3.PASO");
  else
    Serial.println("Test 3.FALLO");
  delay (3000);

  if ((bandtest1 == 0) && (bandtest2 == 0) && (bandtest3 == 0))
    Serial.println("Calibracion finalizada");
  else
    Serial.println("Calibracion fallida");
  entrada = "";
  bandtest1 = 0;
  bandtest2 = 0;
  bandtest3 = 0;
}

void ventanapresion() { //modp
  param = "";
  entrada = "";
  while (entrada != "fuda") {
    if (Serial.available() > 0)
    {
      entrada = Serial.readStringUntil('\n');
    }
    //delay(10);
    //una vez finalizada la lectura de los parámetros de entrada del ventilador
    if ((entrada != "") || (param != "")) {
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada[2] == 'o') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if (entrada[2] == 'l') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }

      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }

      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          //Serial.println();
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.println((int)total);
          }
          else
            Serial.println(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }
      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.println("enc1");
          bandclick = 1;
        }
      }
      //if (bandclick==0)
      //Serial.println("enc0");
      estadoBotonAnterior = estadoBoton;
      bandclick = 0;
      //modo vcpac
      if (entrada[3] == 'p') {
        //importante hacer esto
        encoderPos = 0;
        oldEncPos = 2;
        parametrospresion(entrada);
      }
      //delay(10);
    }
  }
  entrada = "";
  param = "";
}

void ventanasimvp() {
  param = "";
  entrada = "";
  while (entrada != "fuda") {
    if (Serial.available() > 0)
    {
      entrada = Serial.readStringUntil('\n');
    }
    //delay(10);
    //una vez finalizada la lectura de los parámetros de entrada del ventilador
    if ((entrada != "") || (param != "")) {
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada[2] == 'o') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if (entrada[2] == 'l') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }
      //psupp
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }
      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.println((int)total);
          }
          else
            Serial.println(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }
      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.println("enc1");
          bandclick = 1;
        }
      }
      //if (bandclick==0)
      //Serial.println("enc0");
      estadoBotonAnterior = estadoBoton;
      bandclick = 0;
      //modo simvp
      if (entrada[3] == 'p') {
        //importante hacer esto
        //encoderPos=0; creo que esto es innecesario
        //oldEncPos=2;
        parametrosSIMVpresion(entrada);
      }
      //delay(10);
    }
  }
  entrada = "";
  param = "";
}

void ventanavolumen() { //modv
  entrada = "";
  param = "";
  while (entrada != "fuda") { //lo cambie de alto
    if (Serial.available() > 0)
    {
      entrada = Serial.readStringUntil('\n');
    }
    if ((entrada != "") || (param != "")) {
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada[2] == 'o') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if (entrada[2] == 'l') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }

      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }
      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.println((int)total);
          }
          else
            Serial.println(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }
      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.println("enc1");
          bandclick = 1;
        }
      }
      //if (bandclick==0)
      //Serial.println("enc0");
      estadoBotonAnterior = estadoBoton;
      bandclick = 0;
      //modo vcpac
      if (entrada[3] == 'v') {
        //encoderPos=0;
        //oldEncPos=2;
        parametrosvolumen(entrada);
      }
    }
  }
  entrada = "";
  param = "";
}

void ventanasimvv() { //modv
  entrada = "";
  param = "";
  while (entrada != "fuda") { //lo cambie de alto
    if (Serial.available() > 0)
    {
      entrada = Serial.readStringUntil('\n');
    }
    if ((entrada != "") || (param != "")) {
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }

      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }

      if (entrada[2] == 'c') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada[2] == 'o') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if (entrada[2] == 'l') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }

      //psupp
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }

      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.println((int)total);
          }
          else
            Serial.println(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }
      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.println("enc1");
          bandclick = 1;
        }
      }
      //if (bandclick==0)
      //Serial.println("enc0");
      estadoBotonAnterior = estadoBoton;
      bandclick = 0;
      //modo vcpac
      if (entrada[3] == 'v') {
        //encoderPos=0;
        //oldEncPos=2;
        parametrosSIMVvolumen(entrada);
      }
    }
  }
  entrada = "";
  param = "";
}

void ventanaaprv() {
  param = "";
  entrada = "";
  while (entrada != "fuda") {
    if (Serial.available() > 0)
    {
      entrada = Serial.readStringUntil('\n');
    }
    //delay(10);
    //una vez finalizada la lectura de los parámetros de entrada del ventilador
    if ((entrada != "") || (param != "")) {
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pae";
        auxenc = 1;
      }
      if (entrada[2] == 'c') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tac";
        auxenc = 1;
      }
      if (entrada[2] == 'n') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tbn";
        auxenc = 10.0;
      }

      if (entrada[2] == 'o') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }

      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }

      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }

      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.println((int)total);
          }
          else
            Serial.println(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }
      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.println("enc1");
          bandclick = 1;
        }
      }
      //if (bandclick==0)
      //Serial.println("enc0");
      estadoBotonAnterior = estadoBoton;
      bandclick = 0;
      //modo simvp
      if ((entrada != "fuda") && (entrada[3] == 'a')) {
        //importante hacer esto
        //encoderPos=0; creo que esto es innecesario
        //oldEncPos=2;
        parametrosAPRV(entrada);
      }
      //delay(10);
    }
  }
  //Serial.println("Sali"); para comprobar que si haya salido.
  entrada = "";
  param = "";
}

void ventanaspont() {
  param = "";
  entrada = "";
  while (entrada != "fuda") {
    if (Serial.available() > 0)
    {
      entrada = Serial.readStringUntil('\n');
    }
    //delay(10);
    //una vez finalizada la lectura de los parámetros de entrada del ventilador
    if ((entrada != "") || (param != "")) {
      //en caso de seleccionar un parametro a modificar

      if (entrada[2] == 'e') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }

      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }

      if (entrada[2] == 'o') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      //psupp
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }

      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.println((int)total);
          }
          else
            Serial.println(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }
      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.println("enc1");
          bandclick = 1;
        }
      }
      //if (bandclick==0)
      //Serial.println("enc0");
      estadoBotonAnterior = estadoBoton;
      bandclick = 0;
      //modo simvp
      if ((entrada[3] == 's') && (entrada[0] == 'm')) {
        //importante hacer esto
        //encoderPos=0; creo que esto es innecesario
        //oldEncPos=2;
        parametrosSPONT(entrada);
      }
      //delay(10);
    }
  }
  entrada = "";
  param = "";
  //digitalWrite (alarma,LOW);
}

void ventanabipap() {
  param = "";
  entrada = "";
  while (entrada != "fuda") {
    if (Serial.available() > 0)
    {
      entrada = Serial.readStringUntil('\n');
    }
    //delay(10);
    //una vez finalizada la lectura de los parámetros de entrada del ventilador
    if ((entrada != "") || (param != "")) {
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }

      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada[2] == 'o') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if (entrada[2] == 'l') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }
      //psupp
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }

      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.println((int)total);
          }
          else
            Serial.println(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }
      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.println("enc1");
          bandclick = 1;
        }
      }
      //if (bandclick==0)
      //Serial.println("enc0");
      estadoBotonAnterior = estadoBoton;
      bandclick = 0;
      //modo simvp
      if (entrada[3] == 'p') {
        //importante hacer esto
        //encoderPos=0; creo que esto es innecesario
        //oldEncPos=2;
        parametrosBIPAP(entrada);
      }
      //delay(10);
    }
  }
  entrada = "";
  param = "";
}

void ventanalarmas() { //moda
  entrada = "";
  param = "";
  while (entrada != "fuda") { //lo cambie de alto

    if (Serial.available() > 0)
    {
      entrada = Serial.readStringUntil('\n');
      //Serial.println(entrada);
    }

    if ((entrada != "") || (param != "")) {
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }

      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }

      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }

      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.println();
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.println((int)total);
          }
          else
            Serial.println(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }
      delay(10);
    }
  }
  param = "";
  entrada = "";
}
/////checar parametros, cadenas y modos de estos que acabo de agregar.
void parametrosSIMVpresion(String cadena) {
  //para el modo presión
  String stroxigeno, strPEEP, strTi, strfrec, strpresion, strtrigger, strpsupp, strrampa;
  int aoxigeno, aPEEP, aTi, afrec, apresion, atrigger, apsupp, arampa;
  for (int i = 0; i < cadena.length(); i++) {
    if (cadena[i] == 's') //parámetro de presion
      apresion = i;
    if (cadena[i] == 'x') //parámetro de porcentaje oxigeno
      aoxigeno = i;
    if (cadena[i] == 'E') //parámetro de PEEP
      aPEEP = i;
    if (cadena[i] == 'i') //parámetro de Ti
      aTi = i;
    if (cadena[i] == 'r') //parámetro de frecuencia respiratoria
      afrec = i;
    if (cadena[i] == 'n') //parámetro de trigger
      atrigger = i;
    if (cadena[i] == 'z') //parámetro de psupp
      apsupp = i;
    if (cadena[i] == 'k') //parámetro de rampa
      arampa = i;
  }
  strpresion = cadena.substring(apresion + 1, aoxigeno - 1); //ya que el parametro viene como Ox
  stroxigeno = cadena.substring(aoxigeno + 1, aPEEP - 1); //ya que el parametro viene como PE
  strPEEP = cadena.substring(aPEEP + 1, aTi - 1);
  strTi = cadena.substring(aTi + 1, afrec - 1);
  strfrec = cadena.substring(afrec + 1, atrigger - 1);
  strtrigger = cadena.substring(atrigger + 1, apsupp - 1);
  strpsupp = cadena.substring(apsupp + 1, arampa - 1);
  strrampa = cadena.substring(arampa + 1, cadena.length());
  //convertir a entero todos los valores
  presion = strpresion.toInt();
  oxigeno = stroxigeno.toInt();
  PEEP = strPEEP.toInt();
  ti = strTi.toFloat();
  frec = strfrec.toFloat();
  trigger = strtrigger.toInt();
  psupp = strpsupp.toInt();
  rampa = strrampa.toFloat();
  cadena = "";
  entrada = "";
  SIMVP (presion, frec, ti, PEEP, oxigeno, trigger, psupp, rampa);
  if (entrada=="fuda"){
  entrada="";
  lecturadatos();
  }
  else
  ventanasimvp();
}

void parametrosSIMVvolumen(String cadena) {
  //para el modo volumen
  String stroxigeno, strPEEP, strTi, strfrec, strflujo, strvolumen, strtrigger, strpsupp, strrampa;
  int aoxigeno, aPEEP, aTi, afrec, aflujo, avolumen, atrigger, apsupp, arampa;

  for (int i = 0; i < cadena.length(); i++) {
    if (cadena[i] == 'l') //parámetro de flujo
      aflujo = i;
    if (cadena[i] == 'x') //parámetro de porcentaje oxigeno
      aoxigeno = i;
    if (cadena[i] == 'E') //parámetro de PEEP
      aPEEP = i;
    if (cadena[i] == 'i') //parámetro de Ti
      aTi = i;
    if (cadena[i] == 'r') //parámetro de frecuencia respiratoria
      afrec = i;
    if (cadena[i] == 'n') //parámetro de trigger
      atrigger = i;
    if (cadena[i] == 'z') //parámetro de psupp
      apsupp = i;
    if (cadena[i] == 'k') //parámetro de rampa
      arampa = i;
  }
  strflujo = cadena.substring(aflujo + 1, aoxigeno - 1); //ya que el parametro viene como Ox
  stroxigeno = cadena.substring(aoxigeno + 1, aPEEP - 1); //ya que el parametro viene como PE
  strPEEP = cadena.substring(aPEEP + 1, aTi - 1);
  strTi = cadena.substring(aTi + 1, afrec - 1);
  strfrec = cadena.substring(afrec + 1, atrigger - 1);
  strtrigger = cadena.substring(atrigger + 1, apsupp - 1);
  strpsupp = cadena.substring(apsupp + 1, arampa - 1);
  strrampa = cadena.substring(arampa + 1, cadena.length());

  trigger = strtrigger.toInt();
  oxigeno = stroxigeno.toInt();
  PEEP = strPEEP.toInt();
  ti = strTi.toFloat();
  frec = strfrec.toFloat();
  flujo = strflujo.toInt();
  volumen = (flujo * ti * 16.6);
  psupp = strpsupp.toInt();
  rampa = strrampa.toFloat();

  cadena = "";
  entrada = "";
  //llamar funcion VCV
  SIMVV (volumen, frec, ti, PEEP, oxigeno, trigger, flujo, psupp, rampa);
  if (entrada=="fuda"){
  entrada="";
  lecturadatos();
  }
  else
   ventanasimvv();
}

void parametrosAPRV(String cadena) {
  //para el modo presión
  String stroxigeno, strpalta, strpbaja, strtalto, strtbajo;
  int aoxigeno, apalta, apbaja, atalto, atbajo;
  for (int i = 0; i < cadena.length(); i++) {
    if (cadena[i] == 'x') //parámetro de presion
      aoxigeno = i;
    if (cadena[i] == 'a') //parámetro de porcentaje oxigeno
      apalta = i;
    if (cadena[i] == 'b') //parámetro de PEEP
      apbaja = i;
    if (cadena[i] == 't') //parámetro de Ti
      atalto = i;
    if (cadena[i] == 'r') //parámetro de frecuencia respiratoria
      atbajo = i;
  }
  stroxigeno = cadena.substring(aoxigeno + 1, apalta - 1); //ya que el parametro viene como Ox
  strpalta = cadena.substring(apalta + 1, apbaja - 1); //ya que el parametro viene como PE
  strpbaja = cadena.substring(apbaja + 1, atalto - 1);
  strtalto = cadena.substring(atalto + 1, atbajo - 1);
  strtbajo = cadena.substring(atbajo + 1, cadena.length());

  //convertir a entero todos los valores;
  oxigeno = stroxigeno.toInt();
  palta = strpalta.toInt();
  pbaja = strpbaja.toInt();
  talto = strtalto.toFloat();
  tbajo = strtbajo.toFloat();

  //lineas auxiliares para corroborar correcta lectura y guardado de datos
  //if (frec==12){
  //digitalWrite (13,HIGH);
  //}
  cadena = "";
  entrada = "";

  //Serial.println("palta:"+strpalta+"pbaja:" +strpbaja+"talto:" +strtalto+"tbajo:" +strtbajo+"oxigeno:" +stroxigeno);
  APRV (palta, pbaja, talto, tbajo, oxigeno);
  if (entrada=="fuda"){
  entrada="";
  lecturadatos();
  }
  else
  ventanaaprv();
  // Serial.println ("modo:"+modo+"Presion:" +strpresion+"oxigeno:"+stroxigeno+"PEEP:"+strPEEP+"Tinspi:"+strTi+"frecuencia:"+strfrec);
}
void parametrosBIPAP(String cadena) {
  //para el modo presión
  String stroxigeno, strPEEP, strTi, strfrec, strpresion, strtrigger, strpsupp, strrampa;
  int aoxigeno, aPEEP, aTi, afrec, apresion, atrigger, apsupp, arampa;
  for (int i = 0; i < cadena.length(); i++) {
    if (cadena[i] == 's') //parámetro de presion
      apresion = i;
    if (cadena[i] == 'x') //parámetro de porcentaje oxigeno
      aoxigeno = i;
    if (cadena[i] == 'E') //parámetro de PEEP
      aPEEP = i;
    if (cadena[i] == 'i') //parámetro de Ti
      aTi = i;
    if (cadena[i] == 'r') //parámetro de frecuencia respiratoria
      afrec = i;
    if (cadena[i] == 'n') //parámetro de trigger
      atrigger = i;
    if (cadena[i] == 'z') //parámetro de psupp
      apsupp = i;
    if (cadena[i] == 'k') //parámetro de rampa
      arampa = i;
  }
  strpresion = cadena.substring(apresion + 1, aoxigeno - 1); //ya que el parametro viene como Ox
  stroxigeno = cadena.substring(aoxigeno + 1, aPEEP - 1); //ya que el parametro viene como PE
  strPEEP = cadena.substring(aPEEP + 1, aTi - 1);
  strTi = cadena.substring(aTi + 1, afrec - 1);
  strfrec = cadena.substring(afrec + 1, atrigger - 1);
  strtrigger = cadena.substring(atrigger + 1, apsupp - 1);
  strpsupp = cadena.substring(apsupp + 1, arampa - 1);
  strrampa = cadena.substring(arampa + 1, cadena.length());
  //convertir a entero todos los valores
  presion = strpresion.toInt();
  oxigeno = stroxigeno.toInt();
  PEEP = strPEEP.toInt();
  ti = strTi.toFloat();
  frec = strfrec.toFloat();
  trigger = strtrigger.toInt();
  psupp = strpsupp.toFloat();
  rampa = strrampa.toInt();
  cadena = "";
  entrada = "";
  BIPAP(presion, frec, ti, PEEP, oxigeno, trigger, psupp, rampa);
  if (entrada=="fuda"){
  entrada="";
  lecturadatos();
  }
  else
  ventanabipap();
}

void parametrosSPONT(String cadena) {
  //para el modo presión
  String stroxigeno, strpsupp, strrampa, strpeep;
  int aoxigeno, apsupp, arampa, apeep;

  for (int i = 0; i < cadena.length(); i++) {
    if (cadena[i] == 'x')
      aoxigeno = i;
    if (cadena[i] == 'p')
      apsupp = i;
    if (cadena[i] == 'r')
      arampa = i;
    if (cadena[i] == 'e')
      apeep = i;
  }
  stroxigeno = cadena.substring(aoxigeno + 1, apsupp - 1); //ya que el parametro viene como Ox
  strpsupp = cadena.substring(apsupp + 1, arampa - 1); //ya que el parametro viene como PE
  strrampa = cadena.substring(arampa + 1, apeep - 1);
  strpeep = cadena.substring(apeep + 1, cadena.length());
  //convertir a entero todos los valores;
  oxigeno = stroxigeno.toInt();
  psupp = strpsupp.toInt();
  rampa = strrampa.toFloat();
  PEEP = strpeep.toInt();

  //lineas auxiliares para corroborar correcta lectura y guardado de datos
  //if (frec==12){
  //digitalWrite (13,HIGH);
  //}
  cadena = "";
  entrada = "";
  //Serial.println ("presion:"+strpsupp+"PEEP"+strpeep+"rampa"+strrampa+"oxigeno"+stroxigeno);
  SPONT(psupp, PEEP, rampa, oxigeno);
  if (entrada=="fuda"){
  entrada="";
  lecturadatos();
  }
  else
  ventanaspont();
  // Serial.println ("modo:"+modo+"Presion:" +strpresion+"oxigeno:"+stroxigeno+"PEEP:"+strPEEP+"Tinspi:"+strTi+"frecuencia:"+strfrec);
}

void SIMVP (int presion, int frec, float ti, int PEEP, int oxigeno, int trigger, int psupp, float rampa) { //ver si hay que añadir alguna otra variable
  //Serial.println("Modo SIMVP");
  ttotal = 60000 / frec;
  ti = ti * 1000;
  te = ttotal - ti;
  param = "";
  bandtrigger = 0;
  silenciador = 1;

  do {
    //alarmas suministros gases
    POx = SensorPOx();
    PAir = SensorPAir();
    PiFuga = 0;
    contadorintervalo = 0;

    //if ((POx<30)){
    // if ((POx<30)||(PAir<10)){
    if (PAir < 0) {
      //instruccion para encender buzzer y piloto
      digitalWrite(alarma, HIGH); //ENCIENDE ALARMA
      delay(1000);
      Serial.print("problema");
      entrada = "alto";
      Serial.print(",");
      Serial.print(POx);
      Serial.print
      (",");
      Serial.println(PAir);
    }
    //inhalación
    digitalWrite(AVexh, LOW);

    //da el porcentaje de acuerdo al fio2 programado
    if (oxigeno == 21)
      calculopwm (0, 37); //37 LPM aire
    else {
      if (oxigeno == 100) {
        calculopwm (37, 0); //37 LPM oxigeno
      }
      else {
        //ecuaciones fio2 con flujo 37, podría intentar dar
        //más flujo cambiando el 37, oxigeno es la variable
        lpmair = (-37 * (oxigeno - 100)) / 80;
        lpmoxig = 37 - lpmair;
        calculopwm(lpmoxig, lpmair);
      }
    }
    inicio = millis();
    bandera = 0;

    while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'p') && ((millis() - inicio) < ti)) {

      lecturapresionesin();
      /*if (bandtrigger==1){
        Serial.print("l");
        }
        else
      */
      Serial.print("x");

      //se agregó para mejorar tiempos en visual studio
      if (contadorintervalo != 0)
        contadorintervalo = millis() - contadorintervalo;
      else
        contadorintervalo = 100;
      Serial.print (contadorintervalo);
      contadorintervalo = millis();

      if (Serial.available() > 0)
      {
        entrada = Serial.readStringUntil('\n');
      }

      //PRUEBA SILENCIO DE ALARMA
      if (entrada == "silence") {
        silenciador = 0;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(alarma,HIGH); //este solo sirve para probar ahorita con el uno, pero debe quitarse
      }
      //meter alarma
      if (entrada == "alerta") { //esto debe gnerar algun problema
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }
      if (entrada == "ruido") { //esto debe generar algun problema
        silenciador = 1;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(13,LOW); //este hay que comentarla
      }
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') { //tambien es pbe para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') { //tambien es tac para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') { //tambien es tbn para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada.substring(0,3) == "fio") {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if ((entrada[2] == 'l')&& (entrada != "silence")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }
      
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }

      //ventana alarmas
      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }

      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }

      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.print((int)total);
          }
          else
            Serial.print(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }

      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.print("enc1");
          bandclick = 1;
        }
      }
      if (bandclick == 0)
        Serial.print("enc0");
      estadoBotonAnterior = estadoBoton;

      if (en == 0)
        Serial.println();
      en = 0;
      bandclick = 0;
      //cambio de modos
      if (entrada[0] == '1') { 
        parametrospresion(entrada);
      }
      if (entrada[0] == '2') { 
        parametrosvolumen(entrada);
      }
      if (entrada[0] == '4') { 
        parametrosSIMVvolumen(entrada);
      }
      if (entrada[0] == '5') { 
        parametrosAPRV(entrada);
      }
      if (entrada[0] == '6') { 
        parametrosBIPAP(entrada);
      }
      if (entrada[0] == '7') { 
        parametrosSPONT(entrada);
      }
      
      //verificación de la presión alcanzada
      if (Pi >= presion) {
        analogWrite(AVPropAir, 0); //estaba en 3.8
        analogWrite(AVPropOx, 0);
      }
      delay(100); //ver si lo elimino
    }
    if ((bandera == 0) && (silenciador == 1) && (entrada != "alto")) {
      digitalWrite(alarma, HIGH); //ENCIENDE ALARMA
      Serial.print("fuga\n");
    }
    else {
      digitalWrite(alarma, LOW);
      if (entrada != "alto")
        Serial.print("nofuga\n");
    }
    //EXHALACIÓN
    digitalWrite(AVexh, HIGH);
    analogWrite(AVPropAir, 0);
    analogWrite(AVPropOx, 0);
    bandtrigger = 0;
    inicio = millis();
    contadorintervalo = 0;

    while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'p') && ((millis() - inicio) < te)) { //caso provisional por valvula exh cerrada
      Pi = SensorPi();
      Os = SensorOs();

      //respiración realizada por el px.
      if ((Pi < (PEEP - trigger)) && (bandtrigger != 1)) { //lo cambie por el -1*PEEP
        bandtrigger = 1;
        //habilitar cuando este peep y todos los sensores esten leyendo bien

        //aqui seguro será nuevamente darle un valor alto
        //dando un flujo de 5 en oxigeno y 5 en aire
        calculopwm(5, 5);

        digitalWrite(AVexh, LOW);
        //bandtrigger=1;
      }

      //descomentar para la parte espontanea
      //if (Pi>=(psupp+presion)){
      if (Pi >= (psupp + PEEP)) { //ver si con PEEP funciona igual
        analogWrite(AVPropAir, 0); //estaba en 3.8
        analogWrite(AVPropOx, 0);
        digitalWrite(AVexh, HIGH);
        bandtrigger = 0;
      }
      //ESTO PASA PRIMERO ANTES QUE EL Pi<PEEP-trigger
      if (Pi < PEEP) { //aquí ver si es con Pexh una mejor respuesta
        digitalWrite(AVexh, LOW);
      }

      if (bandtrigger == 0) {
        Serial.print("p");
        Serial.print(Pi);
        Serial.print("f");
        TCA9548A(3); // select I2C bus 2 for the exh flow sensor
        Fexh = SensorData();
        Serial.print(Fexh);
        Serial.print("o");
        Serial.print(Os);
        Serial.print("y");
      }
      else {
        Serial.print("p");
        Serial.print(Pi);
        Serial.print("f");
        TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
        FOx = SensorData();
        //lectura flujo aire
        TCA9548A(5); // select I2C bus 1 for the air flow sensor
        FAir = SensorData();
        Serial.print(FAir + FOx);
        Serial.print("o");
        Serial.print(Os);
        Serial.print("l");
      }

      //envío de tiempo transcurrido en ms
      if (contadorintervalo != 0)
        contadorintervalo = millis() - contadorintervalo;
      else
        contadorintervalo = 100;
      Serial.print (contadorintervalo);
      contadorintervalo = millis();

      if (Serial.available() > 0)
      {
        entrada = Serial.readStringUntil('\n');
      }
      if (entrada == "silence") {
        //bandera=0;
        silenciador = 0;
        entrada = ""; //para que no este leyendo nada
      }
      //meter alarma
      if (entrada == "alerta") { //esto debe gnerar algun problema
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }
      if (entrada == "ruido") { //esto debe generar algun problema
        silenciador = 1;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(13,LOW); //este hay que comentarla
      }
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') { //tambien es pbe para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') { //tambien es tac para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') { //tambien es tbn para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada.substring(0,3) == "fio") {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if ((entrada[2] == 'l')&& (entrada != "silence")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }
      
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }
      //ventana alarmas
      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.print((int)total);
          }
          else
            Serial.print(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }
      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.print("enc1");
          bandclick = 1;
        }
      }
      if (bandclick == 0)
        Serial.print("enc0");
      estadoBotonAnterior = estadoBoton;

      if (en == 0)
        Serial.println();
      en = 0;
      bandclick = 0;

      //cambio de modos
      if (entrada[0] == '1') { 
        parametrospresion(entrada);
      }
      if (entrada[0] == '2') { 
        parametrosvolumen(entrada);
      }
      if (entrada[0] == '4') { 
        parametrosSIMVvolumen(entrada);
      }
      if (entrada[0] == '5') { 
        parametrosAPRV(entrada);
      }
      if (entrada[0] == '6') { 
        parametrosBIPAP(entrada);
      }
      if (entrada[0] == '7') { 
        parametrosSPONT(entrada);
      }
      
      delay(100);
    }
  } while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'p'));

  //en caso de actualizacion de parámetros
  if (entrada[3]=='p')
  parametrosSIMVpresion(entrada);
  
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  digitalWrite(alarma, LOW);
  digitalWrite(AVexh, HIGH);
}

void SIMVV (int volumen, int frec, float ti, int PEEP, int oxigeno, int trigger, int flujo, int psupp, float rampa) { //ver si hay que añadir algun otra variable
  //Serial.println("Modo SIMVV");
  ttotal = 60000 / frec;
  ti = ti * 1000;
  te = ttotal - ti;
  param = ""; //el param ya se inicializó a 0 desde antes en la función "parametrosvolumen"
  bandtrigger = 0;
  bandvol = 0;
  silenciador = 1;
  volumencalc = 0; //SOLO AL PRINCIPIO, como aun hay fugas y no corresponde, la estoy eliminando siempre
  //como no da el flujo real al inicio lo inicializo en cero para que si de los volumenes.
  do {
    //alarmas suministros gases
    //linea agregada solo porque la lectura del flujo exh no es buena.
    volumencalc = 0;
    POx = SensorPOx();
    PAir = SensorPAir();
    PiFuga = 0;
    volFuga = 0;
    contadorintervalo = 0;
    //volumencalc=0;
    //if ((POx<30)){
    // if ((POx<30)||(PAir<10)){
    if (PAir < 0) {
      //instruccion para encender buzzer y piloto
      digitalWrite(alarma, HIGH); //ENCIENDE ALARMA
      delay(1000);
      Serial.print("problema");
      entrada = "alto";
      Serial.print(",");
      Serial.print(POx);
      Serial.print
      (",");
      Serial.println(PAir);
    }
    //inhalación
    digitalWrite(AVexh, LOW);
    //da el porcentaje de acuerdo al fio2 programado
    if (oxigeno == 21) {
      //ecuaciones fio2
      lpmair = flujo;
      lpmoxig = 0;
      calculopwm(lpmoxig, lpmair);
    }
    else {
      if (oxigeno == 100) {
        //ecuaciones fio2
        lpmair = 0;
        lpmoxig = flujo - lpmair;
        //ecuaciones conversion lpm-pwm
        calculopwm(lpmoxig, lpmair);
      }
      else {
        //ecuaciones fio2
        lpmair = (-flujo * (oxigeno - 100)) / 80;
        lpmoxig = flujo - lpmair;
        //ecuaciones conversion lpm-pwm
        calculopwm(lpmoxig, lpmair);
      }
    }

    inicio = millis();
    iniciovol = millis();
    bandera = 0;

    while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'v') && ((millis() - inicio) < ti)) {

      //lectura sensores: presión inspiratoria, flujo aire, flujo oxígeno y sensor de oxígeno
      Pi = SensorPi();
      Os = SensorOs();
      //impresión de gráficas
      Serial.print("p");
      Serial.print(Pi);
      Serial.print("f");
      //lectura flujo oxigeno
      TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
      FOx = SensorData();
      //lectura flujo aire
      TCA9548A(5); // select I2C bus 1 for the air flow sensor
      FAir = SensorData();
      Serial.print(FAir + FOx);
      Serial.print("o");
      Serial.print(Os);

      //Alarma Fuga
      volumencalc = volumencalc + ((FAir + FOx) * (millis() - iniciovol) * 0.016); //en ml Svolumencalc+
      iniciovol = millis();

      if (volumencalc > volFuga) {
        volFuga = volumencalc;
      }

      if ((volFuga > (volumen * 0.60)) && (bandera != 1)) { //probar esta que puede funcionar, fnciona con ese
        bandera = 1;
        digitalWrite(alarma, LOW);
      }

      Serial.print("x");

      //envio de tiempo a vstudio
      if (contadorintervalo != 0)
        contadorintervalo = millis() - contadorintervalo;
      else
        contadorintervalo = 100;
      Serial.print (contadorintervalo);
      contadorintervalo = millis();

      if (Serial.available() > 0)
      {
        entrada = Serial.readStringUntil('\n');
      }

      if (volumencalc > volumen) {
        analogWrite(AVPropAir, 0); //estaba en 3.8
        analogWrite(AVPropOx, 0);
      }
      //PRUEBA SILENCIO DE ALARMA
      if (entrada == "silence") {
        silenciador = 0;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(13,HIGH); //este solo sirve para probar ahorita con el uno, pero debe quitarse
      }
      //meter alarma
      if (entrada == "alerta") {
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }
      if (entrada == "ruido") {
        silenciador = 1;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(13,LOW); //este hay que comentarla
      }
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') { //tambien es pbe para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') { //tambien es tac para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') { //tambien es tbn para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada.substring(0,3) == "fio") {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if ((entrada[2] == 'l')&& (entrada != "silence")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }
      
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }

      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }

      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }

      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }

      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.print((int)total);
          }
          else
            Serial.print(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }

      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.print("enc1");
          bandclick = 1;
        }
      }
      if (bandclick == 0)
        Serial.print("enc0");
      estadoBotonAnterior = estadoBoton;

      if (en == 0)
        Serial.println();
      en = 0;
      bandclick = 0;

      //cambio de modos
      if (entrada[0] == '1') { 
        parametrospresion(entrada);
      }
      if (entrada[0] == '2') { 
        parametrosvolumen(entrada);
      }
      if (entrada[0] == '3') { 
        parametrosSIMVpresion(entrada);
      }
      if (entrada[0] == '5') { 
        parametrosAPRV(entrada);
      }
      if (entrada[0] == '6') { 
        parametrosBIPAP(entrada);
      }
      if (entrada[0] == '7') { 
        parametrosSPONT(entrada);
      }
      delay(100);
    }
    if ((bandera == 0) && (silenciador == 1) && (entrada != "alto")) {
      digitalWrite(alarma, HIGH); //ENCIENDE ALARMA
      Serial.print("fuga\n");
    }

    else {
      digitalWrite(alarma, LOW);
      if (entrada != "alto")
        Serial.print("nofuga\n");
    }

    //EXHALACIÓN
    digitalWrite(AVexh, HIGH);
    analogWrite(AVPropAir, 0);
    analogWrite(AVPropOx, 0);
    bandtrigger = 0;
    bandvol = 0;
    inicio = millis();
    iniciovol = millis();
    contadorintervalo = 0;

    while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'v') && ((millis() - inicio) < te)) { //caso provisional por valvula exh cerrada
      Pi = SensorPi();
      Os = SensorOs();
      TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
      FOx = SensorData();
      //lectura flujo aire
      TCA9548A(5); // select I2C bus 1 for the air flow sensor
      FAir = SensorData();

      //respiración realizada por el px.
      //descomentar para el modo espontáneo
      if (((FAir + FOx) > trigger) && (bandvol == 1)) { //lo cambie por el

        //dando un flujo de 5 en oxigeno y 5 en aire
        calculopwm(5, 5);
        digitalWrite(AVexh, LOW);
        bandtrigger = 1;
        bandvol = 0;
      }

      if (Pi >= (psupp + PEEP)) {
        analogWrite(AVPropAir, 0); //estaba en 3.8
        analogWrite(AVPropOx, 0);
        digitalWrite(AVexh, HIGH);
        bandvol = 0;
        bandtrigger = 0;
      }

      if (Pi < PEEP) { //aquí ver si es con Pexh una mejor respuesta
        digitalWrite(AVexh, LOW);
        bandvol = 1;
        bandtrigger = 0;
      }

      if (bandtrigger == 0) {
        Serial.print("p");
        Serial.print(Pi);
        Serial.print("f");
        TCA9548A(3); // select I2C bus 2 for the exh flow sensor
        Fexh = SensorData();
        Serial.print(Fexh);
        Serial.print("o");
        Serial.print(Os);
        Serial.print("y");

        volumencalc = volumencalc - ((Fexh) * (millis() - iniciovol) * 0.016);
        iniciovol = millis();
      }
      else {
        Serial.print("p");
        Serial.print(Pi);
        Serial.print("f");
        TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
        FOx = SensorData();
        //lectura flujo aire
        TCA9548A(5); // select I2C bus 1 for the air flow sensor
        FAir = SensorData();
        Serial.print(FAir + FOx);
        Serial.print("o");
        Serial.print(Os);
        Serial.print("l");
        volumencalc = volumencalc + ((FAir + FOx) * (millis() - iniciovol) * 0.016); //en ml Svolumencalc+
        iniciovol = millis();
      }
      //envia tiempo en ms a vstudio
      if (contadorintervalo != 0)
        contadorintervalo = millis() - contadorintervalo;
      else
        contadorintervalo = 100;
      Serial.print (contadorintervalo);
      contadorintervalo = millis();

      if (Serial.available() > 0)
      {
        entrada = Serial.readStringUntil('\n');
      }
      if (entrada == "silence") {
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }
      //meter alarma
      if (entrada == "alerta") { //esto debe gnerar algun problema
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }
      if (entrada == "ruido") { //esto debe generar algun problema
        silenciador = 1;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(13,LOW); //este hay que comentarla
      }
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') { //tambien es pbe para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') { //tambien es tac para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') { //tambien es tbn para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada.substring(0,3) == "fio") {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if ((entrada[2] == 'l')&& (entrada != "silence")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }
      
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }

      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }

      //de ventana de alarmas
      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.print((int)total);
          }
          else
            Serial.print(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }
      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.print("enc1");
          bandclick = 1;
        }
      }
      if (bandclick == 0)
        Serial.print("enc0");
      estadoBotonAnterior = estadoBoton;

      if (en == 0)
        Serial.println();
      en = 0;
      bandclick = 0;

      //cambio de modos
      if (entrada[0] == '1') { 
        parametrospresion(entrada);
      }
      if (entrada[0] == '2') { 
        parametrosvolumen(entrada);
      }
      if (entrada[0] == '3') { 
        parametrosSIMVpresion(entrada);
      }
      if (entrada[0] == '5') { 
        parametrosAPRV(entrada);
      }
      if (entrada[0] == '6') { 
        parametrosBIPAP(entrada);
      }
      if (entrada[0] == '7') { 
        parametrosSPONT(entrada);
      }
      
      delay(100);
    }
  } while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'v'));

  //en caso de actualizacion de parámetros
  if (entrada[3]=='v')
  parametrosSIMVvolumen(entrada);
  
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  digitalWrite(alarma, LOW);
  digitalWrite(AVexh, HIGH);
  param = ""; //
}

void APRV (int palta, int pbaja, float talto, float tbajo, int oxigeno) { //ver si hay que añadir algun otra variable
  //Serial.println("Modo APRV");
  ttotal = 60000 / frec;
  ti = ti * 1000;
  te = ttotal - ti;
  param = "";
  talto = talto * 1000; //conversion a ms
  tbajo = tbajo * 1000; //conversion a ms
  bandtrigger = 0;
  silenciador = 1;
  bandpres = 0;
  do {
    //alarmas suministros gases
    POx = SensorPOx();
    PAir = SensorPAir();
    PiFuga = 0;
    contadorintervalo = 0;
    //if ((POx<30)){
    // if ((POx<30)||(PAir<10)){
    if (PAir < 0) {
      //instruccion para encender buzzer y piloto
      digitalWrite(alarma, HIGH); //ENCIENDE ALARMA
      delay(1000);
      Serial.print("problema");
      entrada = "alto";
      Serial.print(",");
      Serial.print(POx);
      Serial.print
      (",");
      Serial.println(PAir);
    }
    //inhalación
    digitalWrite(AVexh, LOW);

    //da el porcentaje de acuerdo al fio2 programado
    if (oxigeno == 21)
      calculopwm (0, 37); //37 LPM aire
    else {
      if (oxigeno == 100) {
        calculopwm (37, 0); //37 LPM oxigeno
      }
      else {
        //ecuaciones fio2 con flujo 37, podría intentar dar
        //más flujo cambiando el 37, oxigeno es la variable
        lpmair = (-37 * (oxigeno - 100)) / 80;
        lpmoxig = 37 - lpmair;
        calculopwm(lpmoxig, lpmair);
      }
    }
    inicio = millis();
    bandera = 0;
    //reiniciar bander bandtrigger
    bandtrigger = 0;
    bandpres = 0;
    while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'a') && ((millis() - inicio) < talto)) {

      //crear variable de espontanea o ver si con bandtrigger funciona
      //if ((Pi< (presion-trigger))&&(cierrevalvulaoxigeno==0)){ //lo cambie por el -1*PEEP

      if (Pi < (palta - 3) && (bandpres == 1)) {
        //bandtrigger=1;
        //habilitar cuando este peep y todos los sensores esten leyendo bien

        //dando un flujo de 5 en oxigeno y 5 en aire
        calculopwm(5, 5);

        bandtrigger = 1;
        bandpres = 0;
      }
      //else
      //bandtrigger=0;

      if (Pi >= (5 + palta)) { //no hay psupp, entonces nosotros ponemos el apoyo
        analogWrite(AVPropAir, 0); //estaba en 3.8
        analogWrite(AVPropOx, 0);
        digitalWrite(AVexh, HIGH); //ahorita quitar este modo espontaneo
        bandtrigger = 0;
      }
      lecturapresionesin();
      if (bandtrigger == 1) {
        Serial.print("l");
      }
      else
        Serial.print("x");

      //se agregó para mejorar tiempos en visual studio
      if (contadorintervalo != 0)
        contadorintervalo = millis() - contadorintervalo;
      else
        contadorintervalo = 100;
      Serial.print (contadorintervalo);
      contadorintervalo = millis();

      if (Serial.available() > 0)
      {
        entrada = Serial.readStringUntil('\n');
      }

      //PRUEBA SILENCIO DE ALARMA
      if (entrada == "silence") {
        silenciador = 0;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(alarma,HIGH); //este solo sirve para probar ahorita con el uno, pero debe quitarse
      }
      //meter alarma
      if (entrada == "alerta") { //esto debe gnerar algun problema
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }
      if (entrada == "ruido") { //esto debe generar algun problema
        silenciador = 1;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(13,LOW); //este hay que comentarla
      }
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') { //tambien es pbe para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') { //tambien es tac para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') { //tambien es tbn para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada.substring(0,3) == "fio") {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if ((entrada[2] == 'l')&& (entrada != "silence")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }
      
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }
      //ventana alarmas
      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }

      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }

      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.print((int)total);
          }
          else
            Serial.print(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }

      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.print("enc1");
          bandclick = 1;
        }
      }
      if (bandclick == 0)
        Serial.print("enc0");
      estadoBotonAnterior = estadoBoton;

      if (en == 0)
        Serial.println();
      en = 0;
      bandclick = 0;
      //cambio de modos
      if (entrada[0] == '1') { 
        parametrospresion(entrada);
      }
      if (entrada[0] == '2') { 
        parametrosvolumen(entrada);
      }
      if (entrada[0] == '3') { 
        parametrosSIMVpresion(entrada);
      }
      if (entrada[0] == '4') { 
        parametrosSIMVvolumen(entrada);
      }
      if (entrada[0] == '6') { 
        parametrosBIPAP(entrada);
      }
      if (entrada[0] == '7') { 
        parametrosSPONT(entrada);
      }
      
      if (Pi >= palta) {
        analogWrite(AVPropAir, 0); //estaba en 3.8
        analogWrite(AVPropOx, 0);
        bandpres = 1; //se alcanzo la presion
      }
      delay(100); //ver si lo elimino
    }
    if ((bandera == 0) && (silenciador == 1) && (entrada != "alto")) {
      digitalWrite(alarma, HIGH); //ENCIENDE ALARMA
      Serial.print("fuga\n");
    }
    else {
      digitalWrite(alarma, LOW);
      if (entrada != "alto")
        Serial.print("nofuga\n");
    }

    //EXHALACIÓN
    digitalWrite(AVexh, HIGH);
    analogWrite(AVPropAir, 0);
    analogWrite(AVPropOx, 0);
    bandpres = 0;
    inicio = millis();
    contadorintervalo = 0;

    while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'a') && ((millis() - inicio) < tbajo)) { //caso provisional por valvula exh cerrada
      Pi = SensorPi();
      Os = SensorOs();
      if (Pi < pbaja) { //aquí ver si es con Pexh una mejor respuesta
        digitalWrite(AVexh, LOW);
      }

      Serial.print("p");
      Serial.print(Pi);
      Serial.print("f");
      TCA9548A(3); // select I2C bus 2 for the exh flow sensor
      Fexh = SensorData();
      Serial.print(Fexh);
      Serial.print("o");
      Serial.print(Os);
      Serial.print("y");

      //envío de tiempo transcurrido en ms
      if (contadorintervalo != 0)
        contadorintervalo = millis() - contadorintervalo;
      else
        contadorintervalo = 100;
      Serial.print (contadorintervalo);
      contadorintervalo = millis();

      if (Serial.available() > 0)
      {
        entrada = Serial.readStringUntil('\n');
      }
      if (entrada == "silence") {
        //bandera=0;
        silenciador = 0;
        entrada = ""; //para que no este leyendo nada
      }
      //meter alarma
      if (entrada == "alerta") { //esto debe gnerar algun problema
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }
      if (entrada == "ruido") { //esto debe generar algun problema
        silenciador = 1;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(13,LOW); //este hay que comentarla
      }
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') { //tambien es pbe para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') { //tambien es tac para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') { //tambien es tbn para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada.substring(0,3) == "fio") {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if ((entrada[2] == 'l')&& (entrada != "silence")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }
      
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }
      //ventana alarmas
      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.print((int)total);
          }
          else
            Serial.print(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }
      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.print("enc1");
          bandclick = 1;
        }
      }
      if (bandclick == 0)
        Serial.print("enc0");
      estadoBotonAnterior = estadoBoton;

      if (en == 0)
        Serial.println();
      en = 0;
      bandclick = 0;

      //cambio de modos
      if (entrada[0] == '1') { 
        parametrospresion(entrada);
      }
      if (entrada[0] == '2') { 
        parametrosvolumen(entrada);
      }
      if (entrada[0] == '3') { 
        parametrosSIMVpresion(entrada);
      }
      if (entrada[0] == '4') { 
        parametrosSIMVvolumen(entrada);
      }
      if (entrada[0] == '6') { 
        parametrosBIPAP(entrada);
      }
      if (entrada[0] == '7') { 
        parametrosSPONT(entrada);
      }
      
      delay(100);
    }
  } while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'a'));

  //en caso de actualizacion de parámetros
  if (entrada[3]=='a')
  parametrosAPRV(entrada);
  
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  digitalWrite(alarma, LOW);
  digitalWrite(AVexh, HIGH);
}



void SPONT (int psupp, int PEEP , float rampa, int oxigeno) { //ver si hay que añadir algun otra variable
  //Serial.println("Modo SPONT");
  param = "";
  bandtrigger = 0;
  silenciador = 1;
  do {
    //alarmas suministros gases
    POx = SensorPOx();
    PAir = SensorPAir();
    PiFuga = 0;
    //if ((POx<30)){
    // if ((POx<30)||(PAir<10)){
    if (PAir < 0) {
      //instruccion para encender buzzer y piloto
      digitalWrite(alarma, HIGH); //ENCIENDE ALARMA
      delay(1000);
      Serial.print("problema");
      entrada = "alto";
      Serial.print(",");
      Serial.print(POx);
      Serial.print
      (",");
      Serial.println(PAir);
    }
    if (entrada != "alto") {
      bandera = 0;
      Pi = SensorPi();
      if ((Pi < (PEEP - 3)) && (bandtrigger != 1)) { //le doy un valor a vencer de 3cmh20
        //dando un flujo de 5 en oxigeno y 5 en aire
        calculopwm(5, 5);

        digitalWrite(AVexh, LOW);
        bandtrigger = 1;
      }
      if (Pi > psupp) {
        analogWrite(AVPropAir, 0); //estaba en 3.8
        analogWrite(AVPropOx, 0);
        digitalWrite(AVexh, HIGH);
        bandtrigger = 0;
      }

      if (Pi < PEEP) {
        digitalWrite(AVexh, LOW);
        //bandtrigger=0;
      }

      if (bandtrigger == 1) {
        lecturapresionesin();
        Serial.print("l");
      }
      else
      {

        Serial.print("p");
        Serial.print(Pi);
        Serial.print("f");
        TCA9548A(3); // select I2C bus 2 for the exh flow sensor
        Fexh = SensorData();
        Serial.print(Fexh);
        Serial.print("o");
        Serial.print(Os);
        Serial.print("y");
      }

      //envío de tiempo transcurrido en ms
      if (contadorintervalo != 0)
        contadorintervalo = millis() - contadorintervalo;
      else
        contadorintervalo = 100;
      Serial.print (contadorintervalo);
      contadorintervalo = millis();

      if (Serial.available() > 0)
      {
        entrada = Serial.readStringUntil('\n');
      }

      //PRUEBA SILENCIO DE ALARMA
      if (entrada == "silence") {
        silenciador = 0;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(alarma,HIGH); //este solo sirve para probar ahorita con el uno, pero debe quitarse
      }
      //meter alarma
      if (entrada == "alerta") {
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }
      if (entrada == "ruido") {
        silenciador = 1;
        entrada = ""; //para que no este leyendo nada
      }
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') { //tambien es pbe para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') { //tambien es tac para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') { //tambien es tbn para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada.substring(0,3) == "fio") {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
     if ((entrada[2] == 'l')&& (entrada != "silence")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }
      
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }
      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }

      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.print((int)total);
          }
          else
            Serial.print(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }

      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.print("enc1");
          bandclick = 1;
        }
      }
      if (bandclick == 0)
        Serial.print("enc0");
      estadoBotonAnterior = estadoBoton;

      if (en == 0)
        Serial.println();
      en = 0;
      bandclick = 0;

      //cambio de modos
      if (entrada[0] == '1') { 
        parametrospresion(entrada);
      }
      if (entrada[0] == '2') { 
        parametrosvolumen(entrada);
      }
      if (entrada[0] == '3') { 
        parametrosSIMVpresion(entrada);
      }
      if (entrada[0] == '4') { 
        parametrosSIMVvolumen(entrada);
      }
      if (entrada[0] == '5') { 
        parametrosAPRV(entrada);
      }
      if (entrada[0] == '6') { 
        parametrosBIPAP(entrada);
      }
      delay(100); //ver si lo elimino
      if ((bandera == 0) && (silenciador == 1)) {
        //digitalWrite(alarma,LOW); //ENCIENDE ALARMA
        //Serial.print("fuga\r\n");
        //delay(50);
      }
      else
        digitalWrite(alarma, LOW);
    }

  } while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 's'));

  //en caso de actualizacion de parámetros
  if (entrada[3]=='s')
  parametrosSPONT(entrada);
  
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  digitalWrite(alarma, LOW);
  digitalWrite(AVexh, HIGH);

}
void BIPAP (int presion, int frec, float ti, int PEEP, int oxigeno, int trigger, int psupp, float rampa) { //ver si hay que añadir alguna otra variable
  //Serial.println("Modo BIPAP");
  ttotal = 60000 / frec;
  ti = ti * 1000;
  te = ttotal - ti;
  param = "";
  bandtrigger = 0;
  silenciador = 1;
  bandpres = 0;
  do {
    //alarmas suministros gases
    POx = SensorPOx();
    PAir = SensorPAir();
    PiFuga = 0;
    contadorintervalo = 0;
    //if ((POx<30)){
    // if ((POx<30)||(PAir<10)){
    if (PAir < 0) {
      //instruccion para encender buzzer y piloto
      digitalWrite(alarma, HIGH); //ENCIENDE ALARMA
      delay(1000);
      Serial.print("problema");
      entrada = "alto";
      Serial.print(",");
      Serial.print(POx);
      Serial.print
      (",");
      Serial.println(PAir);
    }
    //inhalación
    digitalWrite(AVexh, LOW);

    //da el porcentaje de acuerdo al fio2 programado
    if (oxigeno == 21)
      calculopwm (0, 37); //37 LPM aire
    else {
      if (oxigeno == 100) {
        calculopwm (37, 0); //37 LPM oxigeno
      }
      else {
        //ecuaciones fio2 con flujo 37, podría intentar dar
        //más flujo cambiando el 37, oxigeno es la variable
        lpmair = (-37 * (oxigeno - 100)) / 80;
        lpmoxig = 37 - lpmair;
        calculopwm(lpmoxig, lpmair);
      }
    }
    inicio = millis();
    bandera = 0;
    //reiniciar bander bandtrigger
    bandtrigger = 0;
    bandpres = 0;
    bandbipap = 0;

    while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'p') && ((millis() - inicio) < ti)) {

      //crear variable de espontanea o ver si con bandtrigger funciona
      //if ((Pi< (presion-trigger))&&(cierrevalvulaoxigeno==0)){ //lo cambie por el -1*PEEP

      if ((Pi < (presion - trigger)) && (bandpres == 1)) {
        //bandtrigger=1;
        //dando un flujo de 5 en oxigeno y 5 en aire
        calculopwm(5, 5);

        bandtrigger = 1;
        bandpres = 0;
        bandbipap = 1;
        digitalWrite(AVexh, LOW);
      }
      else
        bandtrigger = 0;

      if ((Pi >= (psupp + presion)) && (bandbipap == 1)) {
        analogWrite(AVPropAir, 0); //estaba en 3.8
        analogWrite(AVPropOx, 0);
        digitalWrite(AVexh, HIGH);
        bandtrigger = 0;
      }

      if (Pi <= presion) {
        digitalWrite(AVexh, LOW);
      }

      lecturapresionesin();
      if (bandtrigger == 1) {
        Serial.print("l");
      }
      else
        Serial.print("x");

      //se agregó para mejorar tiempos en visual studio
      if (contadorintervalo != 0)
        contadorintervalo = millis() - contadorintervalo;
      else
        contadorintervalo = 100;
      Serial.print (contadorintervalo);
      contadorintervalo = millis();

      if (Serial.available() > 0)
      {
        entrada = Serial.readStringUntil('\n');
      }

      //PRUEBA SILENCIO DE ALARMA
      if (entrada == "silence") {
        silenciador = 0;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(alarma,HIGH); //este solo sirve para probar ahorita con el uno, pero debe quitarse
      }
      //meter alarma
      if (entrada == "alerta") { //esto debe gnerar algun problema
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }
      if (entrada == "ruido") { //esto debe generar algun problema
        silenciador = 1;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(13,LOW); //este hay que comentarla
      }
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') { //tambien es pbe para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') { //tambien es tac para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') { //tambien es tbn para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada.substring(0,3) == "fio") {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if ((entrada[2] == 'l')&& (entrada != "silence")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }
      
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }
      //ventana alarmas
      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }

      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }

      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.print((int)total);
          }
          else
            Serial.print(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }

      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.print("enc1");
          bandclick = 1;
        }
      }
      if (bandclick == 0)
        Serial.print("enc0");
      estadoBotonAnterior = estadoBoton;

      if (en == 0)
        Serial.println();
      en = 0;
      bandclick = 0;

      //cambio de modos
      if (entrada[0] == '1') { 
        parametrospresion(entrada);
      }
      if (entrada[0] == '2') { 
        parametrosvolumen(entrada);
      }
      if (entrada[0] == '3') { 
        parametrosSIMVpresion(entrada);
      }
      if (entrada[0] == '4') { 
        parametrosSIMVvolumen(entrada);
      }
      if (entrada[0] == '5') { 
        parametrosAPRV(entrada);
      }
      if (entrada[0] == '7') { 
        parametrosSPONT(entrada);
      }
      
      if (Pi >= presion) {
        analogWrite(AVPropAir, 0); //estaba en 3.8
        analogWrite(AVPropOx, 0);
        bandpres = 1; //se alcanzo la presion.
      }
      delay(100); //ver si lo elimino
    }
    if ((bandera == 0) && (silenciador == 1) && (entrada != "alto")) {
      digitalWrite(alarma, HIGH); //ENCIENDE ALARMA
      Serial.print("fuga\n");
    }
    else {
      digitalWrite(alarma, LOW);
      if (entrada != "alto")
        Serial.print("nofuga\n");
    }


    //EXHALACIÓN
    digitalWrite(AVexh, HIGH);
    analogWrite(AVPropAir, 0);
    analogWrite(AVPropOx, 0);
    inicio = millis();
    contadorintervalo = 0;
    bandtrigger = 0;
    while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'p') && ((millis() - inicio) < te)) { //caso provisional por valvula exh cerrada
      Pi = SensorPi();
      Os = SensorOs();
      //respiración realizada por el px.
      if ((Pi < (PEEP - trigger)) && (bandtrigger != 1)) { //lo cambie por el -1*PEEP
        bandtrigger = 1;

        //dando un flujo de 5 en oxigeno y 5 en aire
        calculopwm(5, 5);

        digitalWrite(AVexh, LOW);
        //bandtrigger=1;
      }

      if (Pi >= (psupp + PEEP)) {
        analogWrite(AVPropAir, 0); //estaba en 3.8
        analogWrite(AVPropOx, 0);
        digitalWrite(AVexh, HIGH);
        bandtrigger = 0;
      }

      if (Pi < PEEP) { //aquí ver si es con Pexh una mejor respuesta
        digitalWrite(AVexh, LOW);
      }

      if (bandtrigger == 0) {
        Serial.print("p");
        Serial.print(Pi);
        Serial.print("f");
        TCA9548A(3); // select I2C bus 2 for the exh flow sensor
        Fexh = SensorData();
        Serial.print(Fexh);
        Serial.print("o");
        Serial.print(Os);
        Serial.print("y");
      }
      else {
        Serial.print("p");
        Serial.print(Pi);
        Serial.print("f");
        TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
        FOx = SensorData();
        //lectura flujo aire
        TCA9548A(5); // select I2C bus 1 for the air flow sensor
        FAir = SensorData();
        Serial.print(FAir + FOx);
        Serial.print("o");
        Serial.print(Os);
        Serial.print("l");
      }

      //envío de tiempo transcurrido en ms
      if (contadorintervalo != 0)
        contadorintervalo = millis() - contadorintervalo;
      else
        contadorintervalo = 100;
      Serial.print (contadorintervalo);
      contadorintervalo = millis();

      if (Serial.available() > 0)
      {
        entrada = Serial.readStringUntil('\n');
      }
      if (entrada == "silence") {
        //bandera=0;
        silenciador = 0;
        entrada = ""; //para que no este leyendo nada
      }
      //meter alarma
      if (entrada == "alerta") { //esto debe gnerar algun problema
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }
      if (entrada == "ruido") { //esto debe generar algun problema
        silenciador = 1;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(13,LOW); //este hay que comentarla
      }
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') { //tambien es pbe para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') { //tambien es tac para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') { //tambien es tbn para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada.substring(0,3) == "fio") {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if ((entrada[2] == 'l')&& (entrada != "silence")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }
      
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }
      //ventana alarmas
      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.print((int)total);
          }
          else
            Serial.print(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }
      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.print("enc1");
          bandclick = 1;
        }
      }
      if (bandclick == 0)
        Serial.print("enc0");
      estadoBotonAnterior = estadoBoton;

      if (en == 0)
        Serial.println();
      en = 0;
      bandclick = 0;

      //cambio de modos
      if (entrada[0] == '1') { 
        parametrospresion(entrada);
      }
      if (entrada[0] == '2') { 
        parametrosvolumen(entrada);
      }
      if (entrada[0] == '3') { 
        parametrosSIMVpresion(entrada);
      }
      if (entrada[0] == '4') { 
        parametrosSIMVvolumen(entrada);
      }
      if (entrada[0] == '5') { 
        parametrosAPRV(entrada);
      }
      if (entrada[0] == '7') { 
        parametrosSPONT(entrada);
      }
      
      delay(100);
    }
  } while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'p'));

  //en caso de actualizacion de parámetros
  if (entrada[3]=='p')
 parametrosBIPAP(entrada);
  
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  digitalWrite(alarma, LOW);
  digitalWrite(AVexh, HIGH);
}

boolean antirebote(int pin) {
  int contador = 0;
  boolean estado;               //guarda el estado del boton
  boolean estadoAnterior;       //guarda el ultimo estado del boton

  do {
    estado = digitalRead(pin);
    if (estado != estadoAnterior) { //comparamos el estado actual con el anterior
      contador = 0;                 //reiniciamos el contador
      estadoAnterior = estado;
    }
    else {
      contador = contador + 1;      //aumentamos el contador en 1
    }
    delay(1);
  } while (contador < tiempoAntirebote);

  return estado;
}

void clickencoder(boolean estadoBoton, boolean estadoBotonAnterior) {
  estadoBoton = digitalRead(boton);   //leemos el estado del boton

  if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
    if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
      Serial.print("enc1");
    }
  }

  estadoBotonAnterior = estadoBoton;
  //Serial.print("enc0");
}

void calculopwm(int lpmoxig, int lpmair) {
  //pendiente actualizar oxigeno
  pwmox = 6.366 * pow(10, -12) * pow(lpmoxig, 8) + 1.371 * pow(10, -9) * pow(lpmoxig, 7) -5.564 * pow(10, -7) * pow(lpmoxig, 6) + 6.09 * pow(10, -5) * pow(lpmoxig, 5) - 0.003166 * pow(lpmoxig, 4) + 0.08617 * pow(lpmoxig, 3) - 1.204 * pow(lpmoxig, 2) + 8.168 * lpmoxig + 70.25; //ecuacion conversion lpm a pwm oxígeno aproximación cúbica
  //aire actualizado
  pwmair = 1.349 * pow(10, -8) * pow(lpmair, 6) - 9.237 * pow(10, -7) * pow(lpmair, 5) - 8.229 * pow(10, -5) * pow(lpmair, 4) + 0.009664 * pow(lpmair, 3) - 0.3156 * pow(lpmair, 2) + 4.779 * lpmair + 115; //ecuacion conversion lpm a pwm aire
  analogWrite(AVPropAir, int (pwmair));
  analogWrite(AVPropOx, int (pwmox));
  //Serial.println(pwmox);
  //Serial.println(pwmair);
}

void testinicial() { //w
  //Serial.println ("px conectado...");
  bandtest1 = 0;
  bandtest3 = 0;
  Serial.println ("Preparando equipo para autotest...");
  delay(3000);
  Serial.println ("Test 1");
  calculopwm(5, 5);
  digitalWrite(AVexh, LOW);
  inicio = millis();
  while ((millis() - inicio) < 2000) {
    Pi = SensorPi();
    Os = SensorOs();
    //lecturapwmoxigeno
    POx = SensorPOx();
    PAir = SensorPAir();
    Serial.print("Presion de entrada oxigeno: ");
    Serial.print(POx);
    Serial.println(" cmH20");
    Serial.print("Presion de entrada aire: ");
    Serial.print(FAir);
    Serial.println(" cmH20");
    TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
    FOx = SensorData();
    Serial.print("Sensor de flujo oxigeno: ");
    Serial.print(FOx);
    Serial.println(" L/min");
    //lectura flujo aire
    TCA9548A(5); // select I2C bus 1 for the air flow sensor
    FAir = SensorData();
    Serial.print("Sensor de flujo aire: ");
    Serial.print(FAir);
    Serial.println(" L/min");
    Serial.print("Presion insiratoria: ");
    Serial.print(Pi);
    Serial.println(" cmH20");
    Serial.print("Sensor de oxigeno: ");
    Serial.print(Os);
    Serial.println(" %");
    Serial.print("Presion exhalatoria: ");
    Serial.print(SensorPexh());
    Serial.println(" cmH20");
    TCA9548A(3); // select I2C bus 1 for the oxygen flow sensor
    Fexh = SensorData();
    Serial.print("Sensor de flujo exhalatorio: ");
    Serial.print(Fexh);
    Serial.println(" L/min");
    if (FOx > 200) {
      Serial.println("Falla en sensor de flujo oxigeno");
    }
    if (FAir > 200) {
      Serial.println("Falla en sensor de flujo aire");
    }
    if (Fexh > 200) {
      Serial.println("Falla en sensor de flujo exhalatorio");
    }
    if ((FOx > 200) || (FAir > 200) || (Fexh > 200))
      bandtest1 = 1;
    else
      bandtest1 = 0;

    delay(500);
  }
  if (bandtest1 == 0)
    Serial.println ("Test 1.PASO");
  else
    Serial.println ("Test 1.FALLO");
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  delay(3000);
  Serial.println("Test 2");
  delay(500);
  Serial.println("Test de alarmas");
  digitalWrite(AVexh, HIGH);
  digitalWrite(alarma, HIGH);
  Serial.println("Alarma activada");
  delay(2000);
  Serial.println("Alarma silenciada");
  delay(1000);
  Serial.println("Test 2.PASO");
  delay(1000);
  Serial.println ("Test 3");
  digitalWrite(alarma, LOW);
  calculopwm(15, 15);
  digitalWrite(AVexh, LOW);
  inicio = millis();
  while ((millis() - inicio) < 2000) {
    Pi = SensorPi();
    Os = SensorOs();
    POx = SensorPOx();
    PAir = SensorPAir();
    Serial.print("Presion de entrada oxigeno: ");
    Serial.print(POx);
    Serial.println(" cmH20");
    Serial.print("Presion de entrada aire: ");
    Serial.print(FAir);
    Serial.println(" cmH20");
    TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
    FOx = SensorData();
    Serial.print("Sensor de flujo oxigeno: ");
    Serial.print(FOx);
    Serial.println(" L/min");
    //lectura flujo aire
    TCA9548A(5); // select I2C bus 1 for the air flow sensor
    FAir = SensorData();
    Serial.print("Sensor de flujo aire: ");
    Serial.print(FAir);
    Serial.println(" L/min");
    Serial.print("Presion insiratoria: ");
    Serial.print(Pi);
    Serial.println(" cmH20");
    Serial.print("Sensor de oxigeno: ");
    Serial.print(Os);
    Serial.println(" %");
    Serial.print("Presion exhalatoria: ");
    Serial.print(SensorPexh());
    Serial.println(" cmH20");
    TCA9548A(3); // select I2C bus 1 for the oxygen flow sensor
    Fexh = SensorData();
    Serial.print("Sensor de flujo exhalatorio: ");
    Serial.print(Fexh);
    Serial.println(" L/min");
    if (FOx > 200) {
      Serial.println("Falla en sensor de flujo oxigeno");
    }
    if (FAir > 200) {
      Serial.println("Falla en sensor de flujo aire");
    }
    if (Fexh > 200) {
      Serial.println("Falla en sensor de flujo exhalatorio");
    }
    if ((FOx > 200) || (FAir > 200) || (Fexh > 200))
      bandtest3 = 1;
    else
      bandtest3 = 0;
    delay(500);
  }
  if (bandtest3 == 0)
    Serial.println ("Test 3.PASO");
  else
    Serial.println ("Test 3.FALLO");
  calculopwm(0, 0);
  delay(3000);
  digitalWrite(alarma, LOW);
  digitalWrite(AVexh, HIGH);
  delay(2000);
  if ((bandtest1 == 0) && (bandtest3 == 0)) {
    Serial.println ("Prueba finalizada");
  }
  else
    Serial.println ("Prueba fallida");
  entrada = "";
  bandtest1 = 0;
  bandtest3 = 0;
}

void flujodecreciente(){
  int valflujo=0;
  int inicioflujo=50;
  int FAir=0;
  int contadorflujo=0;
       for (int i=0; i<500;i++){
        
        contadorflujo=i;
        //if (contadorflujo>=10)
        //contadorflujo=21+i;
        
        valflujo = inicioflujo * pow(1 / 1.1, contadorflujo);
        //valflujo=valflujo-;
        if (valflujo<=0)
        valflujo=0;
        //if (oxigeno == 21)
        //calculopwm (valflujo, 0);//valvula oxigeno
        calculopwm (0, valflujo);//valvula aire
        //else{
        //  lpmair = (-valflujo * (oxigeno - 100)) / 80;
        //  lpmoxig = valflujo - lpmair;
        //  calculopwm(lpmoxig, lpmair);
        //}
        //4 oxigeno, 5 aire
         TCA9548A(5); // select I2C bus 1 for the air flow sensor
    FAir = SensorData();
    //Serial.print("Sensor de flujo aire: ");
    Serial.println(FAir);
    delay(250);
       }
}
            Serial.println((int)total);
          }
          else
            Serial.println(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }
      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.println("enc1");
          bandclick = 1;
        }
      }
      //if (bandclick==0)
      //Serial.println("enc0");
      estadoBotonAnterior = estadoBoton;
      bandclick = 0;
      //modo vcpac
      if (entrada[3] == 'p') {
        //importante hacer esto
        encoderPos = 0;
        oldEncPos = 2;
        parametrospresion(entrada);
      }
      //delay(10);
    }
  }
  entrada = "";
  param = "";
}

void ventanasimvp() {
  param = "";
  entrada = "";
  while (entrada != "fuda") {
    if (Serial.available() > 0)
    {
      entrada = Serial.readStringUntil('\n');
    }
    //delay(10);
    //una vez finalizada la lectura de los parámetros de entrada del ventilador
    if ((entrada != "") || (param != "")) {
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada[2] == 'o') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if (entrada[2] == 'l') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }
      //psupp
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }
      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.println((int)total);
          }
          else
            Serial.println(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }
      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.println("enc1");
          bandclick = 1;
        }
      }
      //if (bandclick==0)
      //Serial.println("enc0");
      estadoBotonAnterior = estadoBoton;
      bandclick = 0;
      //modo simvp
      if (entrada[3] == 'p') {
        //importante hacer esto
        //encoderPos=0; creo que esto es innecesario
        //oldEncPos=2;
        parametrosSIMVpresion(entrada);
      }
      //delay(10);
    }
  }
  entrada = "";
  param = "";
}

void ventanavolumen() { //modv
  entrada = "";
  param = "";
  while (entrada != "fuda") { //lo cambie de alto
    if (Serial.available() > 0)
    {
      entrada = Serial.readStringUntil('\n');
    }
    if ((entrada != "") || (param != "")) {
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada[2] == 'o') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if (entrada[2] == 'l') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }

      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }
      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.println((int)total);
          }
          else
            Serial.println(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }
      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.println("enc1");
          bandclick = 1;
        }
      }
      //if (bandclick==0)
      //Serial.println("enc0");
      estadoBotonAnterior = estadoBoton;
      bandclick = 0;
      //modo vcpac
      if (entrada[3] == 'v') {
        //encoderPos=0;
        //oldEncPos=2;
        parametrosvolumen(entrada);
      }
    }
  }
  entrada = "";
  param = "";
}

void ventanasimvv() { //modv
  entrada = "";
  param = "";
  while (entrada != "fuda") { //lo cambie de alto
    if (Serial.available() > 0)
    {
      entrada = Serial.readStringUntil('\n');
    }
    if ((entrada != "") || (param != "")) {
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }

      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }

      if (entrada[2] == 'c') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada[2] == 'o') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if (entrada[2] == 'l') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }

      //psupp
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }

      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.println((int)total);
          }
          else
            Serial.println(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }
      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.println("enc1");
          bandclick = 1;
        }
      }
      //if (bandclick==0)
      //Serial.println("enc0");
      estadoBotonAnterior = estadoBoton;
      bandclick = 0;
      //modo vcpac
      if (entrada[3] == 'v') {
        //encoderPos=0;
        //oldEncPos=2;
        parametrosSIMVvolumen(entrada);
      }
    }
  }
  entrada = "";
  param = "";
}

void ventanaaprv() {
  param = "";
  entrada = "";
  while (entrada != "fuda") {
    if (Serial.available() > 0)
    {
      entrada = Serial.readStringUntil('\n');
    }
    //delay(10);
    //una vez finalizada la lectura de los parámetros de entrada del ventilador
    if ((entrada != "") || (param != "")) {
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pae";
        auxenc = 1;
      }
      if (entrada[2] == 'c') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tac";
        auxenc = 1;
      }
      if (entrada[2] == 'n') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tbn";
        auxenc = 10.0;
      }

      if (entrada[2] == 'o') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }

      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }

      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }

      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.println((int)total);
          }
          else
            Serial.println(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }
      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.println("enc1");
          bandclick = 1;
        }
      }
      //if (bandclick==0)
      //Serial.println("enc0");
      estadoBotonAnterior = estadoBoton;
      bandclick = 0;
      //modo simvp
      if ((entrada != "fuda") && (entrada[3] == 'a')) {
        //importante hacer esto
        //encoderPos=0; creo que esto es innecesario
        //oldEncPos=2;
        parametrosAPRV(entrada);
      }
      //delay(10);
    }
  }
  //Serial.println("Sali"); para comprobar que si haya salido.
  entrada = "";
  param = "";
}

void ventanaspont() {
  param = "";
  entrada = "";
  while (entrada != "fuda") {
    if (Serial.available() > 0)
    {
      entrada = Serial.readStringUntil('\n');
    }
    //delay(10);
    //una vez finalizada la lectura de los parámetros de entrada del ventilador
    if ((entrada != "") || (param != "")) {
      //en caso de seleccionar un parametro a modificar

      if (entrada[2] == 'e') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }

      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }

      if (entrada[2] == 'o') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      //psupp
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }

      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.println((int)total);
          }
          else
            Serial.println(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }
      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.println("enc1");
          bandclick = 1;
        }
      }
      //if (bandclick==0)
      //Serial.println("enc0");
      estadoBotonAnterior = estadoBoton;
      bandclick = 0;
      //modo simvp
      if ((entrada[3] == 's') && (entrada[0] == 'm')) {
        //importante hacer esto
        //encoderPos=0; creo que esto es innecesario
        //oldEncPos=2;
        parametrosSPONT(entrada);
      }
      //delay(10);
    }
  }
  entrada = "";
  param = "";
  //digitalWrite (alarma,LOW);
}

void ventanabipap() {
  param = "";
  entrada = "";
  while (entrada != "fuda") {
    if (Serial.available() > 0)
    {
      entrada = Serial.readStringUntil('\n');
    }
    //delay(10);
    //una vez finalizada la lectura de los parámetros de entrada del ventilador
    if ((entrada != "") || (param != "")) {
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }

      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada[2] == 'o') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if (entrada[2] == 'l') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }
      //psupp
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }

      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.println((int)total);
          }
          else
            Serial.println(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }
      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.println("enc1");
          bandclick = 1;
        }
      }
      //if (bandclick==0)
      //Serial.println("enc0");
      estadoBotonAnterior = estadoBoton;
      bandclick = 0;
      //modo simvp
      if (entrada[3] == 'p') {
        //importante hacer esto
        //encoderPos=0; creo que esto es innecesario
        //oldEncPos=2;
        parametrosBIPAP(entrada);
      }
      //delay(10);
    }
  }
  entrada = "";
  param = "";
}

void ventanalarmas() { //moda
  entrada = "";
  param = "";
  while (entrada != "fuda") { //lo cambie de alto

    if (Serial.available() > 0)
    {
      entrada = Serial.readStringUntil('\n');
      //Serial.println(entrada);
    }

    if ((entrada != "") || (param != "")) {
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }

      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }

      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }

      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.println();
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.println((int)total);
          }
          else
            Serial.println(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }
      delay(10);
    }
  }
  param = "";
  entrada = "";
}
/////checar parametros, cadenas y modos de estos que acabo de agregar.
void parametrosSIMVpresion(String cadena) {
  //para el modo presión
  String stroxigeno, strPEEP, strTi, strfrec, strpresion, strtrigger, strpsupp, strrampa;
  int aoxigeno, aPEEP, aTi, afrec, apresion, atrigger, apsupp, arampa;
  for (int i = 0; i < cadena.length(); i++) {
    if (cadena[i] == 's') //parámetro de presion
      apresion = i;
    if (cadena[i] == 'x') //parámetro de porcentaje oxigeno
      aoxigeno = i;
    if (cadena[i] == 'E') //parámetro de PEEP
      aPEEP = i;
    if (cadena[i] == 'i') //parámetro de Ti
      aTi = i;
    if (cadena[i] == 'r') //parámetro de frecuencia respiratoria
      afrec = i;
    if (cadena[i] == 'n') //parámetro de trigger
      atrigger = i;
    if (cadena[i] == 'z') //parámetro de psupp
      apsupp = i;
    if (cadena[i] == 'k') //parámetro de rampa
      arampa = i;
  }
  strpresion = cadena.substring(apresion + 1, aoxigeno - 1); //ya que el parametro viene como Ox
  stroxigeno = cadena.substring(aoxigeno + 1, aPEEP - 1); //ya que el parametro viene como PE
  strPEEP = cadena.substring(aPEEP + 1, aTi - 1);
  strTi = cadena.substring(aTi + 1, afrec - 1);
  strfrec = cadena.substring(afrec + 1, atrigger - 1);
  strtrigger = cadena.substring(atrigger + 1, apsupp - 1);
  strpsupp = cadena.substring(apsupp + 1, arampa - 1);
  strrampa = cadena.substring(arampa + 1, cadena.length());
  //convertir a entero todos los valores
  presion = strpresion.toInt();
  oxigeno = stroxigeno.toInt();
  PEEP = strPEEP.toInt();
  ti = strTi.toFloat();
  frec = strfrec.toFloat();
  trigger = strtrigger.toInt();
  psupp = strpsupp.toInt();
  rampa = strrampa.toFloat();
  cadena = "";
  entrada = "";
  SIMVP (presion, frec, ti, PEEP, oxigeno, trigger, psupp, rampa);
  if (entrada=="fuda"){
  entrada="";
  lecturadatos();
  }
  else
  ventanasimvp();
}

void parametrosSIMVvolumen(String cadena) {
  //para el modo volumen
  String stroxigeno, strPEEP, strTi, strfrec, strflujo, strvolumen, strtrigger, strpsupp, strrampa;
  int aoxigeno, aPEEP, aTi, afrec, aflujo, avolumen, atrigger, apsupp, arampa;

  for (int i = 0; i < cadena.length(); i++) {
    if (cadena[i] == 'l') //parámetro de flujo
      aflujo = i;
    if (cadena[i] == 'x') //parámetro de porcentaje oxigeno
      aoxigeno = i;
    if (cadena[i] == 'E') //parámetro de PEEP
      aPEEP = i;
    if (cadena[i] == 'i') //parámetro de Ti
      aTi = i;
    if (cadena[i] == 'r') //parámetro de frecuencia respiratoria
      afrec = i;
    if (cadena[i] == 'n') //parámetro de trigger
      atrigger = i;
    if (cadena[i] == 'z') //parámetro de psupp
      apsupp = i;
    if (cadena[i] == 'k') //parámetro de rampa
      arampa = i;
  }
  strflujo = cadena.substring(aflujo + 1, aoxigeno - 1); //ya que el parametro viene como Ox
  stroxigeno = cadena.substring(aoxigeno + 1, aPEEP - 1); //ya que el parametro viene como PE
  strPEEP = cadena.substring(aPEEP + 1, aTi - 1);
  strTi = cadena.substring(aTi + 1, afrec - 1);
  strfrec = cadena.substring(afrec + 1, atrigger - 1);
  strtrigger = cadena.substring(atrigger + 1, apsupp - 1);
  strpsupp = cadena.substring(apsupp + 1, arampa - 1);
  strrampa = cadena.substring(arampa + 1, cadena.length());

  trigger = strtrigger.toInt();
  oxigeno = stroxigeno.toInt();
  PEEP = strPEEP.toInt();
  ti = strTi.toFloat();
  frec = strfrec.toFloat();
  flujo = strflujo.toInt();
  volumen = (flujo * ti * 16.6);
  psupp = strpsupp.toInt();
  rampa = strrampa.toFloat();

  cadena = "";
  entrada = "";
  //llamar funcion VCV
  SIMVV (volumen, frec, ti, PEEP, oxigeno, trigger, flujo, psupp, rampa);
  if (entrada=="fuda"){
  entrada="";
  lecturadatos();
  }
  else
   ventanasimvv();
}

void parametrosAPRV(String cadena) {
  //para el modo presión
  String stroxigeno, strpalta, strpbaja, strtalto, strtbajo;
  int aoxigeno, apalta, apbaja, atalto, atbajo;
  for (int i = 0; i < cadena.length(); i++) {
    if (cadena[i] == 'x') //parámetro de presion
      aoxigeno = i;
    if (cadena[i] == 'a') //parámetro de porcentaje oxigeno
      apalta = i;
    if (cadena[i] == 'b') //parámetro de PEEP
      apbaja = i;
    if (cadena[i] == 't') //parámetro de Ti
      atalto = i;
    if (cadena[i] == 'r') //parámetro de frecuencia respiratoria
      atbajo = i;
  }
  stroxigeno = cadena.substring(aoxigeno + 1, apalta - 1); //ya que el parametro viene como Ox
  strpalta = cadena.substring(apalta + 1, apbaja - 1); //ya que el parametro viene como PE
  strpbaja = cadena.substring(apbaja + 1, atalto - 1);
  strtalto = cadena.substring(atalto + 1, atbajo - 1);
  strtbajo = cadena.substring(atbajo + 1, cadena.length());

  //convertir a entero todos los valores;
  oxigeno = stroxigeno.toInt();
  palta = strpalta.toInt();
  pbaja = strpbaja.toInt();
  talto = strtalto.toFloat();
  tbajo = strtbajo.toFloat();

  //lineas auxiliares para corroborar correcta lectura y guardado de datos
  //if (frec==12){
  //digitalWrite (13,HIGH);
  //}
  cadena = "";
  entrada = "";

  //Serial.println("palta:"+strpalta+"pbaja:" +strpbaja+"talto:" +strtalto+"tbajo:" +strtbajo+"oxigeno:" +stroxigeno);
  APRV (palta, pbaja, talto, tbajo, oxigeno);
  if (entrada=="fuda"){
  entrada="";
  lecturadatos();
  }
  else
  ventanaaprv();
  // Serial.println ("modo:"+modo+"Presion:" +strpresion+"oxigeno:"+stroxigeno+"PEEP:"+strPEEP+"Tinspi:"+strTi+"frecuencia:"+strfrec);
}
void parametrosBIPAP(String cadena) {
  //para el modo presión
  String stroxigeno, strPEEP, strTi, strfrec, strpresion, strtrigger, strpsupp, strrampa;
  int aoxigeno, aPEEP, aTi, afrec, apresion, atrigger, apsupp, arampa;
  for (int i = 0; i < cadena.length(); i++) {
    if (cadena[i] == 's') //parámetro de presion
      apresion = i;
    if (cadena[i] == 'x') //parámetro de porcentaje oxigeno
      aoxigeno = i;
    if (cadena[i] == 'E') //parámetro de PEEP
      aPEEP = i;
    if (cadena[i] == 'i') //parámetro de Ti
      aTi = i;
    if (cadena[i] == 'r') //parámetro de frecuencia respiratoria
      afrec = i;
    if (cadena[i] == 'n') //parámetro de trigger
      atrigger = i;
    if (cadena[i] == 'z') //parámetro de psupp
      apsupp = i;
    if (cadena[i] == 'k') //parámetro de rampa
      arampa = i;
  }
  strpresion = cadena.substring(apresion + 1, aoxigeno - 1); //ya que el parametro viene como Ox
  stroxigeno = cadena.substring(aoxigeno + 1, aPEEP - 1); //ya que el parametro viene como PE
  strPEEP = cadena.substring(aPEEP + 1, aTi - 1);
  strTi = cadena.substring(aTi + 1, afrec - 1);
  strfrec = cadena.substring(afrec + 1, atrigger - 1);
  strtrigger = cadena.substring(atrigger + 1, apsupp - 1);
  strpsupp = cadena.substring(apsupp + 1, arampa - 1);
  strrampa = cadena.substring(arampa + 1, cadena.length());
  //convertir a entero todos los valores
  presion = strpresion.toInt();
  oxigeno = stroxigeno.toInt();
  PEEP = strPEEP.toInt();
  ti = strTi.toFloat();
  frec = strfrec.toFloat();
  trigger = strtrigger.toInt();
  psupp = strpsupp.toFloat();
  rampa = strrampa.toInt();
  cadena = "";
  entrada = "";
  BIPAP(presion, frec, ti, PEEP, oxigeno, trigger, psupp, rampa);
  if (entrada=="fuda"){
  entrada="";
  lecturadatos();
  }
  else
  ventanabipap();
}

void parametrosSPONT(String cadena) {
  //para el modo presión
  String stroxigeno, strpsupp, strrampa, strpeep;
  int aoxigeno, apsupp, arampa, apeep;

  for (int i = 0; i < cadena.length(); i++) {
    if (cadena[i] == 'x')
      aoxigeno = i;
    if (cadena[i] == 'p')
      apsupp = i;
    if (cadena[i] == 'r')
      arampa = i;
    if (cadena[i] == 'e')
      apeep = i;
  }
  stroxigeno = cadena.substring(aoxigeno + 1, apsupp - 1); //ya que el parametro viene como Ox
  strpsupp = cadena.substring(apsupp + 1, arampa - 1); //ya que el parametro viene como PE
  strrampa = cadena.substring(arampa + 1, apeep - 1);
  strpeep = cadena.substring(apeep + 1, cadena.length());
  //convertir a entero todos los valores;
  oxigeno = stroxigeno.toInt();
  psupp = strpsupp.toInt();
  rampa = strrampa.toFloat();
  PEEP = strpeep.toInt();

  //lineas auxiliares para corroborar correcta lectura y guardado de datos
  //if (frec==12){
  //digitalWrite (13,HIGH);
  //}
  cadena = "";
  entrada = "";
  //Serial.println ("presion:"+strpsupp+"PEEP"+strpeep+"rampa"+strrampa+"oxigeno"+stroxigeno);
  SPONT(psupp, PEEP, rampa, oxigeno);
  if (entrada=="fuda"){
  entrada="";
  lecturadatos();
  }
  else
  ventanaspont();
  // Serial.println ("modo:"+modo+"Presion:" +strpresion+"oxigeno:"+stroxigeno+"PEEP:"+strPEEP+"Tinspi:"+strTi+"frecuencia:"+strfrec);
}

void SIMVP (int presion, int frec, float ti, int PEEP, int oxigeno, int trigger, int psupp, float rampa) { //ver si hay que añadir alguna otra variable
  //Serial.println("Modo SIMVP");
  ttotal = 60000 / frec;
  ti = ti * 1000;
  te = ttotal - ti;
  param = "";
  bandtrigger = 0;
  silenciador = 1;

  do {
    //alarmas suministros gases
    POx = SensorPOx();
    PAir = SensorPAir();
    PiFuga = 0;
    contadorintervalo = 0;

    //if ((POx<30)){
    // if ((POx<30)||(PAir<10)){
    if (PAir < 0) {
      //instruccion para encender buzzer y piloto
      digitalWrite(alarma, HIGH); //ENCIENDE ALARMA
      delay(1000);
      Serial.print("problema");
      entrada = "alto";
      Serial.print(",");
      Serial.print(POx);
      Serial.print
      (",");
      Serial.println(PAir);
    }
    //inhalación
    digitalWrite(AVexh, LOW);

    //da el porcentaje de acuerdo al fio2 programado
    if (oxigeno == 21)
      calculopwm (0, 37); //37 LPM aire
    else {
      if (oxigeno == 100) {
        calculopwm (37, 0); //37 LPM oxigeno
      }
      else {
        //ecuaciones fio2 con flujo 37, podría intentar dar
        //más flujo cambiando el 37, oxigeno es la variable
        lpmair = (-37 * (oxigeno - 100)) / 80;
        lpmoxig = 37 - lpmair;
        calculopwm(lpmoxig, lpmair);
      }
    }
    inicio = millis();
    bandera = 0;

    while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'p') && ((millis() - inicio) < ti)) {

      lecturapresionesin();
      /*if (bandtrigger==1){
        Serial.print("l");
        }
        else
      */
      Serial.print("x");

      //se agregó para mejorar tiempos en visual studio
      if (contadorintervalo != 0)
        contadorintervalo = millis() - contadorintervalo;
      else
        contadorintervalo = 100;
      Serial.print (contadorintervalo);
      contadorintervalo = millis();

      if (Serial.available() > 0)
      {
        entrada = Serial.readStringUntil('\n');
      }

      //PRUEBA SILENCIO DE ALARMA
      if (entrada == "silence") {
        silenciador = 0;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(alarma,HIGH); //este solo sirve para probar ahorita con el uno, pero debe quitarse
      }
      //meter alarma
      if (entrada == "alerta") { //esto debe gnerar algun problema
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }
      if (entrada == "ruido") { //esto debe generar algun problema
        silenciador = 1;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(13,LOW); //este hay que comentarla
      }
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') { //tambien es pbe para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') { //tambien es tac para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') { //tambien es tbn para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada.substring(0,3) == "fio") {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if ((entrada[2] == 'l')&& (entrada != "silence")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }
      
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }

      //ventana alarmas
      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }

      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }

      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.print((int)total);
          }
          else
            Serial.print(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }

      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.print("enc1");
          bandclick = 1;
        }
      }
      if (bandclick == 0)
        Serial.print("enc0");
      estadoBotonAnterior = estadoBoton;

      if (en == 0)
        Serial.println();
      en = 0;
      bandclick = 0;
      //cambio de modos
      if (entrada[0] == '1') { 
        parametrospresion(entrada);
      }
      if (entrada[0] == '2') { 
        parametrosvolumen(entrada);
      }
      if (entrada[0] == '4') { 
        parametrosSIMVvolumen(entrada);
      }
      if (entrada[0] == '5') { 
        parametrosAPRV(entrada);
      }
      if (entrada[0] == '6') { 
        parametrosBIPAP(entrada);
      }
      if (entrada[0] == '7') { 
        parametrosSPONT(entrada);
      }
      
      //verificación de la presión alcanzada
      if (Pi >= presion) {
        analogWrite(AVPropAir, 0); //estaba en 3.8
        analogWrite(AVPropOx, 0);
      }
      delay(100); //ver si lo elimino
    }
    if ((bandera == 0) && (silenciador == 1) && (entrada != "alto")) {
      digitalWrite(alarma, HIGH); //ENCIENDE ALARMA
      Serial.print("fuga\n");
    }
    else {
      digitalWrite(alarma, LOW);
      if (entrada != "alto")
        Serial.print("nofuga\n");
    }
    //EXHALACIÓN
    digitalWrite(AVexh, HIGH);
    analogWrite(AVPropAir, 0);
    analogWrite(AVPropOx, 0);
    bandtrigger = 0;
    inicio = millis();
    contadorintervalo = 0;

    while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'p') && ((millis() - inicio) < te)) { //caso provisional por valvula exh cerrada
      Pi = SensorPi();
      Os = SensorOs();

      //respiración realizada por el px.
      if ((Pi < (PEEP - trigger)) && (bandtrigger != 1)) { //lo cambie por el -1*PEEP
        bandtrigger = 1;
        //habilitar cuando este peep y todos los sensores esten leyendo bien

        //aqui seguro será nuevamente darle un valor alto
        //dando un flujo de 5 en oxigeno y 5 en aire
        calculopwm(5, 5);

        digitalWrite(AVexh, LOW);
        //bandtrigger=1;
      }

      //descomentar para la parte espontanea
      //if (Pi>=(psupp+presion)){
      if (Pi >= (psupp + PEEP)) { //ver si con PEEP funciona igual
        analogWrite(AVPropAir, 0); //estaba en 3.8
        analogWrite(AVPropOx, 0);
        digitalWrite(AVexh, HIGH);
        bandtrigger = 0;
      }
      //ESTO PASA PRIMERO ANTES QUE EL Pi<PEEP-trigger
      if (Pi < PEEP) { //aquí ver si es con Pexh una mejor respuesta
        digitalWrite(AVexh, LOW);
      }

      if (bandtrigger == 0) {
        Serial.print("p");
        Serial.print(Pi);
        Serial.print("f");
        TCA9548A(3); // select I2C bus 2 for the exh flow sensor
        Fexh = SensorData();
        Serial.print(Fexh);
        Serial.print("o");
        Serial.print(Os);
        Serial.print("y");
      }
      else {
        Serial.print("p");
        Serial.print(Pi);
        Serial.print("f");
        TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
        FOx = SensorData();
        //lectura flujo aire
        TCA9548A(5); // select I2C bus 1 for the air flow sensor
        FAir = SensorData();
        Serial.print(FAir + FOx);
        Serial.print("o");
        Serial.print(Os);
        Serial.print("l");
      }

      //envío de tiempo transcurrido en ms
      if (contadorintervalo != 0)
        contadorintervalo = millis() - contadorintervalo;
      else
        contadorintervalo = 100;
      Serial.print (contadorintervalo);
      contadorintervalo = millis();

      if (Serial.available() > 0)
      {
        entrada = Serial.readStringUntil('\n');
      }
      if (entrada == "silence") {
        //bandera=0;
        silenciador = 0;
        entrada = ""; //para que no este leyendo nada
      }
      //meter alarma
      if (entrada == "alerta") { //esto debe gnerar algun problema
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }
      if (entrada == "ruido") { //esto debe generar algun problema
        silenciador = 1;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(13,LOW); //este hay que comentarla
      }
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') { //tambien es pbe para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') { //tambien es tac para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') { //tambien es tbn para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada.substring(0,3) == "fio") {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if ((entrada[2] == 'l')&& (entrada != "silence")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }
      
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }
      //ventana alarmas
      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.print((int)total);
          }
          else
            Serial.print(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }
      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.print("enc1");
          bandclick = 1;
        }
      }
      if (bandclick == 0)
        Serial.print("enc0");
      estadoBotonAnterior = estadoBoton;

      if (en == 0)
        Serial.println();
      en = 0;
      bandclick = 0;

      //cambio de modos
      if (entrada[0] == '1') { 
        parametrospresion(entrada);
      }
      if (entrada[0] == '2') { 
        parametrosvolumen(entrada);
      }
      if (entrada[0] == '4') { 
        parametrosSIMVvolumen(entrada);
      }
      if (entrada[0] == '5') { 
        parametrosAPRV(entrada);
      }
      if (entrada[0] == '6') { 
        parametrosBIPAP(entrada);
      }
      if (entrada[0] == '7') { 
        parametrosSPONT(entrada);
      }
      
      delay(100);
    }
  } while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'p'));

  //en caso de actualizacion de parámetros
  if (entrada[3]=='p')
  parametrosSIMVpresion(entrada);
  
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  digitalWrite(alarma, LOW);
  digitalWrite(AVexh, HIGH);
}

void SIMVV (int volumen, int frec, float ti, int PEEP, int oxigeno, int trigger, int flujo, int psupp, float rampa) { //ver si hay que añadir algun otra variable
  //Serial.println("Modo SIMVV");
  ttotal = 60000 / frec;
  ti = ti * 1000;
  te = ttotal - ti;
  param = ""; //el param ya se inicializó a 0 desde antes en la función "parametrosvolumen"
  bandtrigger = 0;
  bandvol = 0;
  silenciador = 1;
  volumencalc = 0; //SOLO AL PRINCIPIO, como aun hay fugas y no corresponde, la estoy eliminando siempre
  //como no da el flujo real al inicio lo inicializo en cero para que si de los volumenes.
  do {
    //alarmas suministros gases
    //linea agregada solo porque la lectura del flujo exh no es buena.
    volumencalc = 0;
    POx = SensorPOx();
    PAir = SensorPAir();
    PiFuga = 0;
    volFuga = 0;
    contadorintervalo = 0;
    //volumencalc=0;
    //if ((POx<30)){
    // if ((POx<30)||(PAir<10)){
    if (PAir < 0) {
      //instruccion para encender buzzer y piloto
      digitalWrite(alarma, HIGH); //ENCIENDE ALARMA
      delay(1000);
      Serial.print("problema");
      entrada = "alto";
      Serial.print(",");
      Serial.print(POx);
      Serial.print
      (",");
      Serial.println(PAir);
    }
    //inhalación
    digitalWrite(AVexh, LOW);
    //da el porcentaje de acuerdo al fio2 programado
    if (oxigeno == 21) {
      //ecuaciones fio2
      lpmair = flujo;
      lpmoxig = 0;
      calculopwm(lpmoxig, lpmair);
    }
    else {
      if (oxigeno == 100) {
        //ecuaciones fio2
        lpmair = 0;
        lpmoxig = flujo - lpmair;
        //ecuaciones conversion lpm-pwm
        calculopwm(lpmoxig, lpmair);
      }
      else {
        //ecuaciones fio2
        lpmair = (-flujo * (oxigeno - 100)) / 80;
        lpmoxig = flujo - lpmair;
        //ecuaciones conversion lpm-pwm
        calculopwm(lpmoxig, lpmair);
      }
    }

    inicio = millis();
    iniciovol = millis();
    bandera = 0;

    while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'v') && ((millis() - inicio) < ti)) {

      //lectura sensores: presión inspiratoria, flujo aire, flujo oxígeno y sensor de oxígeno
      Pi = SensorPi();
      Os = SensorOs();
      //impresión de gráficas
      Serial.print("p");
      Serial.print(Pi);
      Serial.print("f");
      //lectura flujo oxigeno
      TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
      FOx = SensorData();
      //lectura flujo aire
      TCA9548A(5); // select I2C bus 1 for the air flow sensor
      FAir = SensorData();
      Serial.print(FAir + FOx);
      Serial.print("o");
      Serial.print(Os);

      //Alarma Fuga
      volumencalc = volumencalc + ((FAir + FOx) * (millis() - iniciovol) * 0.016); //en ml Svolumencalc+
      iniciovol = millis();

      if (volumencalc > volFuga) {
        volFuga = volumencalc;
      }

      if ((volFuga > (volumen * 0.60)) && (bandera != 1)) { //probar esta que puede funcionar, fnciona con ese
        bandera = 1;
        digitalWrite(alarma, LOW);
      }

      Serial.print("x");

      //envio de tiempo a vstudio
      if (contadorintervalo != 0)
        contadorintervalo = millis() - contadorintervalo;
      else
        contadorintervalo = 100;
      Serial.print (contadorintervalo);
      contadorintervalo = millis();

      if (Serial.available() > 0)
      {
        entrada = Serial.readStringUntil('\n');
      }

      if (volumencalc > volumen) {
        analogWrite(AVPropAir, 0); //estaba en 3.8
        analogWrite(AVPropOx, 0);
      }
      //PRUEBA SILENCIO DE ALARMA
      if (entrada == "silence") {
        silenciador = 0;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(13,HIGH); //este solo sirve para probar ahorita con el uno, pero debe quitarse
      }
      //meter alarma
      if (entrada == "alerta") {
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }
      if (entrada == "ruido") {
        silenciador = 1;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(13,LOW); //este hay que comentarla
      }
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') { //tambien es pbe para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') { //tambien es tac para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') { //tambien es tbn para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada.substring(0,3) == "fio") {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if ((entrada[2] == 'l')&& (entrada != "silence")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }
      
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }

      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }

      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }

      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }

      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.print((int)total);
          }
          else
            Serial.print(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }

      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.print("enc1");
          bandclick = 1;
        }
      }
      if (bandclick == 0)
        Serial.print("enc0");
      estadoBotonAnterior = estadoBoton;

      if (en == 0)
        Serial.println();
      en = 0;
      bandclick = 0;

      //cambio de modos
      if (entrada[0] == '1') { 
        parametrospresion(entrada);
      }
      if (entrada[0] == '2') { 
        parametrosvolumen(entrada);
      }
      if (entrada[0] == '3') { 
        parametrosSIMVpresion(entrada);
      }
      if (entrada[0] == '5') { 
        parametrosAPRV(entrada);
      }
      if (entrada[0] == '6') { 
        parametrosBIPAP(entrada);
      }
      if (entrada[0] == '7') { 
        parametrosSPONT(entrada);
      }
      delay(100);
    }
    if ((bandera == 0) && (silenciador == 1) && (entrada != "alto")) {
      digitalWrite(alarma, HIGH); //ENCIENDE ALARMA
      Serial.print("fuga\n");
    }

    else {
      digitalWrite(alarma, LOW);
      if (entrada != "alto")
        Serial.print("nofuga\n");
    }

    //EXHALACIÓN
    digitalWrite(AVexh, HIGH);
    analogWrite(AVPropAir, 0);
    analogWrite(AVPropOx, 0);
    bandtrigger = 0;
    bandvol = 0;
    inicio = millis();
    iniciovol = millis();
    contadorintervalo = 0;

    while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'v') && ((millis() - inicio) < te)) { //caso provisional por valvula exh cerrada
      Pi = SensorPi();
      Os = SensorOs();
      TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
      FOx = SensorData();
      //lectura flujo aire
      TCA9548A(5); // select I2C bus 1 for the air flow sensor
      FAir = SensorData();

      //respiración realizada por el px.
      //descomentar para el modo espontáneo
      if (((FAir + FOx) > trigger) && (bandvol == 1)) { //lo cambie por el

        //dando un flujo de 5 en oxigeno y 5 en aire
        calculopwm(5, 5);
        digitalWrite(AVexh, LOW);
        bandtrigger = 1;
        bandvol = 0;
      }

      if (Pi >= (psupp + PEEP)) {
        analogWrite(AVPropAir, 0); //estaba en 3.8
        analogWrite(AVPropOx, 0);
        digitalWrite(AVexh, HIGH);
        bandvol = 0;
        bandtrigger = 0;
      }

      if (Pi < PEEP) { //aquí ver si es con Pexh una mejor respuesta
        digitalWrite(AVexh, LOW);
        bandvol = 1;
        bandtrigger = 0;
      }

      if (bandtrigger == 0) {
        Serial.print("p");
        Serial.print(Pi);
        Serial.print("f");
        TCA9548A(3); // select I2C bus 2 for the exh flow sensor
        Fexh = SensorData();
        Serial.print(Fexh);
        Serial.print("o");
        Serial.print(Os);
        Serial.print("y");

        volumencalc = volumencalc - ((Fexh) * (millis() - iniciovol) * 0.016);
        iniciovol = millis();
      }
      else {
        Serial.print("p");
        Serial.print(Pi);
        Serial.print("f");
        TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
        FOx = SensorData();
        //lectura flujo aire
        TCA9548A(5); // select I2C bus 1 for the air flow sensor
        FAir = SensorData();
        Serial.print(FAir + FOx);
        Serial.print("o");
        Serial.print(Os);
        Serial.print("l");
        volumencalc = volumencalc + ((FAir + FOx) * (millis() - iniciovol) * 0.016); //en ml Svolumencalc+
        iniciovol = millis();
      }
      //envia tiempo en ms a vstudio
      if (contadorintervalo != 0)
        contadorintervalo = millis() - contadorintervalo;
      else
        contadorintervalo = 100;
      Serial.print (contadorintervalo);
      contadorintervalo = millis();

      if (Serial.available() > 0)
      {
        entrada = Serial.readStringUntil('\n');
      }
      if (entrada == "silence") {
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }
      //meter alarma
      if (entrada == "alerta") { //esto debe gnerar algun problema
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }
      if (entrada == "ruido") { //esto debe generar algun problema
        silenciador = 1;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(13,LOW); //este hay que comentarla
      }
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') { //tambien es pbe para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') { //tambien es tac para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') { //tambien es tbn para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada.substring(0,3) == "fio") {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if ((entrada[2] == 'l')&& (entrada != "silence")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }
      
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }

      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }

      //de ventana de alarmas
      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.print((int)total);
          }
          else
            Serial.print(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }
      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.print("enc1");
          bandclick = 1;
        }
      }
      if (bandclick == 0)
        Serial.print("enc0");
      estadoBotonAnterior = estadoBoton;

      if (en == 0)
        Serial.println();
      en = 0;
      bandclick = 0;

      //cambio de modos
      if (entrada[0] == '1') { 
        parametrospresion(entrada);
      }
      if (entrada[0] == '2') { 
        parametrosvolumen(entrada);
      }
      if (entrada[0] == '3') { 
        parametrosSIMVpresion(entrada);
      }
      if (entrada[0] == '5') { 
        parametrosAPRV(entrada);
      }
      if (entrada[0] == '6') { 
        parametrosBIPAP(entrada);
      }
      if (entrada[0] == '7') { 
        parametrosSPONT(entrada);
      }
      
      delay(100);
    }
  } while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'v'));

  //en caso de actualizacion de parámetros
  if (entrada[3]=='v')
  parametrosSIMVvolumen(entrada);
  
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  digitalWrite(alarma, LOW);
  digitalWrite(AVexh, HIGH);
  param = ""; //
}

void APRV (int palta, int pbaja, float talto, float tbajo, int oxigeno) { //ver si hay que añadir algun otra variable
  //Serial.println("Modo APRV");
  ttotal = 60000 / frec;
  ti = ti * 1000;
  te = ttotal - ti;
  param = "";
  talto = talto * 1000; //conversion a ms
  tbajo = tbajo * 1000; //conversion a ms
  bandtrigger = 0;
  silenciador = 1;
  bandpres = 0;
  do {
    //alarmas suministros gases
    POx = SensorPOx();
    PAir = SensorPAir();
    PiFuga = 0;
    contadorintervalo = 0;
    //if ((POx<30)){
    // if ((POx<30)||(PAir<10)){
    if (PAir < 0) {
      //instruccion para encender buzzer y piloto
      digitalWrite(alarma, HIGH); //ENCIENDE ALARMA
      delay(1000);
      Serial.print("problema");
      entrada = "alto";
      Serial.print(",");
      Serial.print(POx);
      Serial.print
      (",");
      Serial.println(PAir);
    }
    //inhalación
    digitalWrite(AVexh, LOW);

    //da el porcentaje de acuerdo al fio2 programado
    if (oxigeno == 21)
      calculopwm (0, 37); //37 LPM aire
    else {
      if (oxigeno == 100) {
        calculopwm (37, 0); //37 LPM oxigeno
      }
      else {
        //ecuaciones fio2 con flujo 37, podría intentar dar
        //más flujo cambiando el 37, oxigeno es la variable
        lpmair = (-37 * (oxigeno - 100)) / 80;
        lpmoxig = 37 - lpmair;
        calculopwm(lpmoxig, lpmair);
      }
    }
    inicio = millis();
    bandera = 0;
    //reiniciar bander bandtrigger
    bandtrigger = 0;
    bandpres = 0;
    while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'a') && ((millis() - inicio) < talto)) {

      //crear variable de espontanea o ver si con bandtrigger funciona
      //if ((Pi< (presion-trigger))&&(cierrevalvulaoxigeno==0)){ //lo cambie por el -1*PEEP

      if (Pi < (palta - 3) && (bandpres == 1)) {
        //bandtrigger=1;
        //habilitar cuando este peep y todos los sensores esten leyendo bien

        //dando un flujo de 5 en oxigeno y 5 en aire
        calculopwm(5, 5);

        bandtrigger = 1;
        bandpres = 0;
      }
      //else
      //bandtrigger=0;

      if (Pi >= (5 + palta)) { //no hay psupp, entonces nosotros ponemos el apoyo
        analogWrite(AVPropAir, 0); //estaba en 3.8
        analogWrite(AVPropOx, 0);
        digitalWrite(AVexh, HIGH); //ahorita quitar este modo espontaneo
        bandtrigger = 0;
      }
      lecturapresionesin();
      if (bandtrigger == 1) {
        Serial.print("l");
      }
      else
        Serial.print("x");

      //se agregó para mejorar tiempos en visual studio
      if (contadorintervalo != 0)
        contadorintervalo = millis() - contadorintervalo;
      else
        contadorintervalo = 100;
      Serial.print (contadorintervalo);
      contadorintervalo = millis();

      if (Serial.available() > 0)
      {
        entrada = Serial.readStringUntil('\n');
      }

      //PRUEBA SILENCIO DE ALARMA
      if (entrada == "silence") {
        silenciador = 0;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(alarma,HIGH); //este solo sirve para probar ahorita con el uno, pero debe quitarse
      }
      //meter alarma
      if (entrada == "alerta") { //esto debe gnerar algun problema
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }
      if (entrada == "ruido") { //esto debe generar algun problema
        silenciador = 1;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(13,LOW); //este hay que comentarla
      }
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') { //tambien es pbe para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') { //tambien es tac para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') { //tambien es tbn para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada.substring(0,3) == "fio") {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if ((entrada[2] == 'l')&& (entrada != "silence")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }
      
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }
      //ventana alarmas
      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }

      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }

      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.print((int)total);
          }
          else
            Serial.print(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }

      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.print("enc1");
          bandclick = 1;
        }
      }
      if (bandclick == 0)
        Serial.print("enc0");
      estadoBotonAnterior = estadoBoton;

      if (en == 0)
        Serial.println();
      en = 0;
      bandclick = 0;
      //cambio de modos
      if (entrada[0] == '1') { 
        parametrospresion(entrada);
      }
      if (entrada[0] == '2') { 
        parametrosvolumen(entrada);
      }
      if (entrada[0] == '3') { 
        parametrosSIMVpresion(entrada);
      }
      if (entrada[0] == '4') { 
        parametrosSIMVvolumen(entrada);
      }
      if (entrada[0] == '6') { 
        parametrosBIPAP(entrada);
      }
      if (entrada[0] == '7') { 
        parametrosSPONT(entrada);
      }
      
      if (Pi >= palta) {
        analogWrite(AVPropAir, 0); //estaba en 3.8
        analogWrite(AVPropOx, 0);
        bandpres = 1; //se alcanzo la presion
      }
      delay(100); //ver si lo elimino
    }
    if ((bandera == 0) && (silenciador == 1) && (entrada != "alto")) {
      digitalWrite(alarma, HIGH); //ENCIENDE ALARMA
      Serial.print("fuga\n");
    }
    else {
      digitalWrite(alarma, LOW);
      if (entrada != "alto")
        Serial.print("nofuga\n");
    }

    //EXHALACIÓN
    digitalWrite(AVexh, HIGH);
    analogWrite(AVPropAir, 0);
    analogWrite(AVPropOx, 0);
    bandpres = 0;
    inicio = millis();
    contadorintervalo = 0;

    while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'a') && ((millis() - inicio) < tbajo)) { //caso provisional por valvula exh cerrada
      Pi = SensorPi();
      Os = SensorOs();
      if (Pi < pbaja) { //aquí ver si es con Pexh una mejor respuesta
        digitalWrite(AVexh, LOW);
      }

      Serial.print("p");
      Serial.print(Pi);
      Serial.print("f");
      TCA9548A(3); // select I2C bus 2 for the exh flow sensor
      Fexh = SensorData();
      Serial.print(Fexh);
      Serial.print("o");
      Serial.print(Os);
      Serial.print("y");

      //envío de tiempo transcurrido en ms
      if (contadorintervalo != 0)
        contadorintervalo = millis() - contadorintervalo;
      else
        contadorintervalo = 100;
      Serial.print (contadorintervalo);
      contadorintervalo = millis();

      if (Serial.available() > 0)
      {
        entrada = Serial.readStringUntil('\n');
      }
      if (entrada == "silence") {
        //bandera=0;
        silenciador = 0;
        entrada = ""; //para que no este leyendo nada
      }
      //meter alarma
      if (entrada == "alerta") { //esto debe gnerar algun problema
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }
      if (entrada == "ruido") { //esto debe generar algun problema
        silenciador = 1;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(13,LOW); //este hay que comentarla
      }
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') { //tambien es pbe para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') { //tambien es tac para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') { //tambien es tbn para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada.substring(0,3) == "fio") {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if ((entrada[2] == 'l')&& (entrada != "silence")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }
      
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }
      //ventana alarmas
      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.print((int)total);
          }
          else
            Serial.print(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }
      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.print("enc1");
          bandclick = 1;
        }
      }
      if (bandclick == 0)
        Serial.print("enc0");
      estadoBotonAnterior = estadoBoton;

      if (en == 0)
        Serial.println();
      en = 0;
      bandclick = 0;

      //cambio de modos
      if (entrada[0] == '1') { 
        parametrospresion(entrada);
      }
      if (entrada[0] == '2') { 
        parametrosvolumen(entrada);
      }
      if (entrada[0] == '3') { 
        parametrosSIMVpresion(entrada);
      }
      if (entrada[0] == '4') { 
        parametrosSIMVvolumen(entrada);
      }
      if (entrada[0] == '6') { 
        parametrosBIPAP(entrada);
      }
      if (entrada[0] == '7') { 
        parametrosSPONT(entrada);
      }
      
      delay(100);
    }
  } while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'a'));

  //en caso de actualizacion de parámetros
  if (entrada[3]=='a')
  parametrosAPRV(entrada);
  
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  digitalWrite(alarma, LOW);
  digitalWrite(AVexh, HIGH);
}



void SPONT (int psupp, int PEEP , float rampa, int oxigeno) { //ver si hay que añadir algun otra variable
  //Serial.println("Modo SPONT");
  param = "";
  bandtrigger = 0;
  silenciador = 1;
  do {
    //alarmas suministros gases
    POx = SensorPOx();
    PAir = SensorPAir();
    PiFuga = 0;
    //if ((POx<30)){
    // if ((POx<30)||(PAir<10)){
    if (PAir < 0) {
      //instruccion para encender buzzer y piloto
      digitalWrite(alarma, HIGH); //ENCIENDE ALARMA
      delay(1000);
      Serial.print("problema");
      entrada = "alto";
      Serial.print(",");
      Serial.print(POx);
      Serial.print
      (",");
      Serial.println(PAir);
    }
    if (entrada != "alto") {
      bandera = 0;
      Pi = SensorPi();
      if ((Pi < (PEEP - 3)) && (bandtrigger != 1)) { //le doy un valor a vencer de 3cmh20
        //dando un flujo de 5 en oxigeno y 5 en aire
        calculopwm(5, 5);

        digitalWrite(AVexh, LOW);
        bandtrigger = 1;
      }
      if (Pi > psupp) {
        analogWrite(AVPropAir, 0); //estaba en 3.8
        analogWrite(AVPropOx, 0);
        digitalWrite(AVexh, HIGH);
        bandtrigger = 0;
      }

      if (Pi < PEEP) {
        digitalWrite(AVexh, LOW);
        //bandtrigger=0;
      }

      if (bandtrigger == 1) {
        lecturapresionesin();
        Serial.print("l");
      }
      else
      {

        Serial.print("p");
        Serial.print(Pi);
        Serial.print("f");
        TCA9548A(3); // select I2C bus 2 for the exh flow sensor
        Fexh = SensorData();
        Serial.print(Fexh);
        Serial.print("o");
        Serial.print(Os);
        Serial.print("y");
      }

      //envío de tiempo transcurrido en ms
      if (contadorintervalo != 0)
        contadorintervalo = millis() - contadorintervalo;
      else
        contadorintervalo = 100;
      Serial.print (contadorintervalo);
      contadorintervalo = millis();

      if (Serial.available() > 0)
      {
        entrada = Serial.readStringUntil('\n');
      }

      //PRUEBA SILENCIO DE ALARMA
      if (entrada == "silence") {
        silenciador = 0;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(alarma,HIGH); //este solo sirve para probar ahorita con el uno, pero debe quitarse
      }
      //meter alarma
      if (entrada == "alerta") {
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }
      if (entrada == "ruido") {
        silenciador = 1;
        entrada = ""; //para que no este leyendo nada
      }
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') { //tambien es pbe para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') { //tambien es tac para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') { //tambien es tbn para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada.substring(0,3) == "fio") {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
     if ((entrada[2] == 'l')&& (entrada != "silence")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }
      
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }
      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }

      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.print((int)total);
          }
          else
            Serial.print(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }

      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.print("enc1");
          bandclick = 1;
        }
      }
      if (bandclick == 0)
        Serial.print("enc0");
      estadoBotonAnterior = estadoBoton;

      if (en == 0)
        Serial.println();
      en = 0;
      bandclick = 0;

      //cambio de modos
      if (entrada[0] == '1') { 
        parametrospresion(entrada);
      }
      if (entrada[0] == '2') { 
        parametrosvolumen(entrada);
      }
      if (entrada[0] == '3') { 
        parametrosSIMVpresion(entrada);
      }
      if (entrada[0] == '4') { 
        parametrosSIMVvolumen(entrada);
      }
      if (entrada[0] == '5') { 
        parametrosAPRV(entrada);
      }
      if (entrada[0] == '6') { 
        parametrosBIPAP(entrada);
      }
      delay(100); //ver si lo elimino
      if ((bandera == 0) && (silenciador == 1)) {
        //digitalWrite(alarma,LOW); //ENCIENDE ALARMA
        //Serial.print("fuga\r\n");
        //delay(50);
      }
      else
        digitalWrite(alarma, LOW);
    }

  } while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 's'));

  //en caso de actualizacion de parámetros
  if (entrada[3]=='s')
  parametrosSPONT(entrada);
  
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  digitalWrite(alarma, LOW);
  digitalWrite(AVexh, HIGH);

}
void BIPAP (int presion, int frec, float ti, int PEEP, int oxigeno, int trigger, int psupp, float rampa) { //ver si hay que añadir alguna otra variable
  //Serial.println("Modo BIPAP");
  ttotal = 60000 / frec;
  ti = ti * 1000;
  te = ttotal - ti;
  param = "";
  bandtrigger = 0;
  silenciador = 1;
  bandpres = 0;
  do {
    //alarmas suministros gases
    POx = SensorPOx();
    PAir = SensorPAir();
    PiFuga = 0;
    contadorintervalo = 0;
    //if ((POx<30)){
    // if ((POx<30)||(PAir<10)){
    if (PAir < 0) {
      //instruccion para encender buzzer y piloto
      digitalWrite(alarma, HIGH); //ENCIENDE ALARMA
      delay(1000);
      Serial.print("problema");
      entrada = "alto";
      Serial.print(",");
      Serial.print(POx);
      Serial.print
      (",");
      Serial.println(PAir);
    }
    //inhalación
    digitalWrite(AVexh, LOW);

    //da el porcentaje de acuerdo al fio2 programado
    if (oxigeno == 21)
      calculopwm (0, 37); //37 LPM aire
    else {
      if (oxigeno == 100) {
        calculopwm (37, 0); //37 LPM oxigeno
      }
      else {
        //ecuaciones fio2 con flujo 37, podría intentar dar
        //más flujo cambiando el 37, oxigeno es la variable
        lpmair = (-37 * (oxigeno - 100)) / 80;
        lpmoxig = 37 - lpmair;
        calculopwm(lpmoxig, lpmair);
      }
    }
    inicio = millis();
    bandera = 0;
    //reiniciar bander bandtrigger
    bandtrigger = 0;
    bandpres = 0;
    bandbipap = 0;

    while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'p') && ((millis() - inicio) < ti)) {

      //crear variable de espontanea o ver si con bandtrigger funciona
      //if ((Pi< (presion-trigger))&&(cierrevalvulaoxigeno==0)){ //lo cambie por el -1*PEEP

      if ((Pi < (presion - trigger)) && (bandpres == 1)) {
        //bandtrigger=1;
        //dando un flujo de 5 en oxigeno y 5 en aire
        calculopwm(5, 5);

        bandtrigger = 1;
        bandpres = 0;
        bandbipap = 1;
        digitalWrite(AVexh, LOW);
      }
      else
        bandtrigger = 0;

      if ((Pi >= (psupp + presion)) && (bandbipap == 1)) {
        analogWrite(AVPropAir, 0); //estaba en 3.8
        analogWrite(AVPropOx, 0);
        digitalWrite(AVexh, HIGH);
        bandtrigger = 0;
      }

      if (Pi <= presion) {
        digitalWrite(AVexh, LOW);
      }

      lecturapresionesin();
      if (bandtrigger == 1) {
        Serial.print("l");
      }
      else
        Serial.print("x");

      //se agregó para mejorar tiempos en visual studio
      if (contadorintervalo != 0)
        contadorintervalo = millis() - contadorintervalo;
      else
        contadorintervalo = 100;
      Serial.print (contadorintervalo);
      contadorintervalo = millis();

      if (Serial.available() > 0)
      {
        entrada = Serial.readStringUntil('\n');
      }

      //PRUEBA SILENCIO DE ALARMA
      if (entrada == "silence") {
        silenciador = 0;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(alarma,HIGH); //este solo sirve para probar ahorita con el uno, pero debe quitarse
      }
      //meter alarma
      if (entrada == "alerta") { //esto debe gnerar algun problema
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }
      if (entrada == "ruido") { //esto debe generar algun problema
        silenciador = 1;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(13,LOW); //este hay que comentarla
      }
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') { //tambien es pbe para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') { //tambien es tac para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') { //tambien es tbn para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada.substring(0,3) == "fio") {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if ((entrada[2] == 'l')&& (entrada != "silence")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }
      
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }
      //ventana alarmas
      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }

      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }

      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.print((int)total);
          }
          else
            Serial.print(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }

      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.print("enc1");
          bandclick = 1;
        }
      }
      if (bandclick == 0)
        Serial.print("enc0");
      estadoBotonAnterior = estadoBoton;

      if (en == 0)
        Serial.println();
      en = 0;
      bandclick = 0;

      //cambio de modos
      if (entrada[0] == '1') { 
        parametrospresion(entrada);
      }
      if (entrada[0] == '2') { 
        parametrosvolumen(entrada);
      }
      if (entrada[0] == '3') { 
        parametrosSIMVpresion(entrada);
      }
      if (entrada[0] == '4') { 
        parametrosSIMVvolumen(entrada);
      }
      if (entrada[0] == '5') { 
        parametrosAPRV(entrada);
      }
      if (entrada[0] == '7') { 
        parametrosSPONT(entrada);
      }
      
      if (Pi >= presion) {
        analogWrite(AVPropAir, 0); //estaba en 3.8
        analogWrite(AVPropOx, 0);
        bandpres = 1; //se alcanzo la presion.
      }
      delay(100); //ver si lo elimino
    }
    if ((bandera == 0) && (silenciador == 1) && (entrada != "alto")) {
      digitalWrite(alarma, HIGH); //ENCIENDE ALARMA
      Serial.print("fuga\n");
    }
    else {
      digitalWrite(alarma, LOW);
      if (entrada != "alto")
        Serial.print("nofuga\n");
    }


    //EXHALACIÓN
    digitalWrite(AVexh, HIGH);
    analogWrite(AVPropAir, 0);
    analogWrite(AVPropOx, 0);
    inicio = millis();
    contadorintervalo = 0;
    bandtrigger = 0;
    while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'p') && ((millis() - inicio) < te)) { //caso provisional por valvula exh cerrada
      Pi = SensorPi();
      Os = SensorOs();
      //respiración realizada por el px.
      if ((Pi < (PEEP - trigger)) && (bandtrigger != 1)) { //lo cambie por el -1*PEEP
        bandtrigger = 1;

        //dando un flujo de 5 en oxigeno y 5 en aire
        calculopwm(5, 5);

        digitalWrite(AVexh, LOW);
        //bandtrigger=1;
      }

      if (Pi >= (psupp + PEEP)) {
        analogWrite(AVPropAir, 0); //estaba en 3.8
        analogWrite(AVPropOx, 0);
        digitalWrite(AVexh, HIGH);
        bandtrigger = 0;
      }

      if (Pi < PEEP) { //aquí ver si es con Pexh una mejor respuesta
        digitalWrite(AVexh, LOW);
      }

      if (bandtrigger == 0) {
        Serial.print("p");
        Serial.print(Pi);
        Serial.print("f");
        TCA9548A(3); // select I2C bus 2 for the exh flow sensor
        Fexh = SensorData();
        Serial.print(Fexh);
        Serial.print("o");
        Serial.print(Os);
        Serial.print("y");
      }
      else {
        Serial.print("p");
        Serial.print(Pi);
        Serial.print("f");
        TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
        FOx = SensorData();
        //lectura flujo aire
        TCA9548A(5); // select I2C bus 1 for the air flow sensor
        FAir = SensorData();
        Serial.print(FAir + FOx);
        Serial.print("o");
        Serial.print(Os);
        Serial.print("l");
      }

      //envío de tiempo transcurrido en ms
      if (contadorintervalo != 0)
        contadorintervalo = millis() - contadorintervalo;
      else
        contadorintervalo = 100;
      Serial.print (contadorintervalo);
      contadorintervalo = millis();

      if (Serial.available() > 0)
      {
        entrada = Serial.readStringUntil('\n');
      }
      if (entrada == "silence") {
        //bandera=0;
        silenciador = 0;
        entrada = ""; //para que no este leyendo nada
      }
      //meter alarma
      if (entrada == "alerta") { //esto debe gnerar algun problema
        bandera = 0;
        entrada = ""; //para que no este leyendo nada
      }
      if (entrada == "ruido") { //esto debe generar algun problema
        silenciador = 1;
        entrada = ""; //para que no este leyendo nada
        //digitalWrite(13,LOW); //este hay que comentarla
      }
      //en caso de seleccionar un parametro a modificar
      if (entrada[2] == 'w') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paw";
        auxenc = 1;
      }
      if (entrada[2] == 'e') { //tambien es pbe para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "pee";
        auxenc = 1;
      }
      if (entrada[2] == 'c') { //tambien es tac para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frc";
        auxenc = 1;
      }
      if (entrada[2] == 'n') { //tambien es tbn para aprv
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tin";
        auxenc = 10.0;
      }
      if (entrada[2] == 's') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tes";
        auxenc = 10.0;
      }
      if (entrada.substring(0,3) == "fio") {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fio";
        auxenc = 1;
      }
      if (entrada[2] == 'i') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "tri";
        auxenc = 1;
      }
      if ((entrada[2] == 'l')&& (entrada != "silence")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "voz";
        auxenc = 1;
      }
      
      if (entrada[2] == 'q') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "suq";
        auxenc = 1;
      }
      //rampa
      if (entrada[2] == 'u') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "rau";
        auxenc = 10.0;
      }
      //ventana alarmas
      if (entrada[2] == 'k') {
        auxperilla = encoderPos; //para siempre restar el valor que traiga
        param = entrada;
        datoencoder = "pak";
        auxenc = 1;
      }
      if (entrada[2] == 'j') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "paj";
        auxenc = 1;
      }
      if (entrada[2] == 'g') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "frg";
        auxenc = 1;
      }
      if (entrada[2] == 'h') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "aph";
        auxenc = 1;
      }
      if (entrada[2] == 'f') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fif";
        auxenc = 1;
      }
      if (entrada[2] == 'z') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fiz";
        auxenc = 1;
      }
      if ((entrada[2] == 't') && (entrada != "alto")) {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vot";
        auxenc = 1;
      }
      if (entrada[2] == 'b') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "vob";
        auxenc = 1;
      }
      if (entrada[2] == 'y') {
        auxperilla = encoderPos;
        param = entrada;
        datoencoder = "fry";
        auxenc = 1;
      }
      if (param != "") {
        if (oldEncPos != encoderPos) {
          total = (param.substring(3, param.length())).toFloat() + (encoderPos / auxenc) - (auxperilla / auxenc);
          Serial.print(datoencoder);
          if (auxenc != 10.0) {
            Serial.print((int)total);
          }
          else
            Serial.print(total);
          oldEncPos = encoderPos;
          entrada = "";
        }
      }
      //FUNCION CLICK ENCODER
      //clickencoder(estadoBoton,estadoBotonAnterior);
      estadoBoton = digitalRead(boton);   //leemos el estado del boton

      if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
        if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
          Serial.print("enc1");
          bandclick = 1;
        }
      }
      if (bandclick == 0)
        Serial.print("enc0");
      estadoBotonAnterior = estadoBoton;

      if (en == 0)
        Serial.println();
      en = 0;
      bandclick = 0;

      //cambio de modos
      if (entrada[0] == '1') { 
        parametrospresion(entrada);
      }
      if (entrada[0] == '2') { 
        parametrosvolumen(entrada);
      }
      if (entrada[0] == '3') { 
        parametrosSIMVpresion(entrada);
      }
      if (entrada[0] == '4') { 
        parametrosSIMVvolumen(entrada);
      }
      if (entrada[0] == '5') { 
        parametrosAPRV(entrada);
      }
      if (entrada[0] == '7') { 
        parametrosSPONT(entrada);
      }
      
      delay(100);
    }
  } while ((entrada != "alto") && (entrada != "fuda") && (entrada[3] != 'p'));

  //en caso de actualizacion de parámetros
  if (entrada[3]=='p')
 parametrosBIPAP(entrada);
  
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  digitalWrite(alarma, LOW);
  digitalWrite(AVexh, HIGH);
}

boolean antirebote(int pin) {
  int contador = 0;
  boolean estado;               //guarda el estado del boton
  boolean estadoAnterior;       //guarda el ultimo estado del boton

  do {
    estado = digitalRead(pin);
    if (estado != estadoAnterior) { //comparamos el estado actual con el anterior
      contador = 0;                 //reiniciamos el contador
      estadoAnterior = estado;
    }
    else {
      contador = contador + 1;      //aumentamos el contador en 1
    }
    delay(1);
  } while (contador < tiempoAntirebote);

  return estado;
}

void clickencoder(boolean estadoBoton, boolean estadoBotonAnterior) {
  estadoBoton = digitalRead(boton);   //leemos el estado del boton

  if (estadoBoton != estadoBotonAnterior) { //si hay cambio con respecto al estado anterior
    if (antirebote(boton)) {                //checamos si esta presionado y si lo esta
      Serial.print("enc1");
    }
  }

  estadoBotonAnterior = estadoBoton;
  //Serial.print("enc0");
}

void calculopwm(int lpmoxig, int lpmair) {
  //pendiente actualizar oxigeno
  pwmox = (-0.001529 * pow(lpmoxig,3) + 2.042 * pow(lpmoxig,2) + 108.9  * lpmoxig + (-109))/(lpmoxig + (-0.9659)); //ecuacion conversion lpm a pwm oxígeno aproximación cúbica
  //aire actualizado
  pwmair = (-0.001835 * pow(lpmair,3) + 1.6  * pow(lpmair,2) + 111.1  * lpmair + 9.165)/(lpmair+0.2037); //ecuacion conversion lpm a pwm aire
  analogWrite(AVPropAir, int (pwmair));
  analogWrite(AVPropOx, int (pwmox));
}

void testinicial() { //w
  //Serial.println ("px conectado...");
  bandtest1 = 0;
  bandtest3 = 0;
  Serial.println ("Preparando equipo para autotest...");
  delay(3000);
  Serial.println ("Test 1");
  calculopwm(5, 5);
  digitalWrite(AVexh, LOW);
  inicio = millis();
  while ((millis() - inicio) < 2000) {
    Pi = SensorPi();
    Os = SensorOs();
    //lecturapwmoxigeno
    POx = SensorPOx();
    PAir = SensorPAir();
    Serial.print("Presion de entrada oxigeno: ");
    Serial.print(POx);
    Serial.println(" cmH20");
    Serial.print("Presion de entrada aire: ");
    Serial.print(FAir);
    Serial.println(" cmH20");
    TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
    FOx = SensorData();
    Serial.print("Sensor de flujo oxigeno: ");
    Serial.print(FOx);
    Serial.println(" L/min");
    //lectura flujo aire
    TCA9548A(5); // select I2C bus 1 for the air flow sensor
    FAir = SensorData();
    Serial.print("Sensor de flujo aire: ");
    Serial.print(FAir);
    Serial.println(" L/min");
    Serial.print("Presion insiratoria: ");
    Serial.print(Pi);
    Serial.println(" cmH20");
    Serial.print("Sensor de oxigeno: ");
    Serial.print(Os);
    Serial.println(" %");
    Serial.print("Presion exhalatoria: ");
    Serial.print(SensorPexh());
    Serial.println(" cmH20");
    TCA9548A(3); // select I2C bus 1 for the oxygen flow sensor
    Fexh = SensorData();
    Serial.print("Sensor de flujo exhalatorio: ");
    Serial.print(Fexh);
    Serial.println(" L/min");
    if (FOx > 200) {
      Serial.println("Falla en sensor de flujo oxigeno");
    }
    if (FAir > 200) {
      Serial.println("Falla en sensor de flujo aire");
    }
    if (Fexh > 200) {
      Serial.println("Falla en sensor de flujo exhalatorio");
    }
    if ((FOx > 200) || (FAir > 200) || (Fexh > 200))
      bandtest1 = 1;
    else
      bandtest1 = 0;

    delay(500);
  }
  if (bandtest1 == 0)
    Serial.println ("Test 1.PASO");
  else
    Serial.println ("Test 1.FALLO");
  analogWrite(AVPropAir, 0);
  analogWrite(AVPropOx, 0);
  delay(3000);
  Serial.println("Test 2");
  delay(500);
  Serial.println("Test de alarmas");
  digitalWrite(AVexh, HIGH);
  digitalWrite(alarma, HIGH);
  Serial.println("Alarma activada");
  delay(2000);
  Serial.println("Alarma silenciada");
  delay(1000);
  Serial.println("Test 2.PASO");
  delay(1000);
  Serial.println ("Test 3");
  digitalWrite(alarma, LOW);
  calculopwm(15, 15);
  digitalWrite(AVexh, LOW);
  inicio = millis();
  while ((millis() - inicio) < 2000) {
    Pi = SensorPi();
    Os = SensorOs();
    POx = SensorPOx();
    PAir = SensorPAir();
    Serial.print("Presion de entrada oxigeno: ");
    Serial.print(POx);
    Serial.println(" cmH20");
    Serial.print("Presion de entrada aire: ");
    Serial.print(FAir);
    Serial.println(" cmH20");
    TCA9548A(4); // select I2C bus 1 for the oxygen flow sensor
    FOx = SensorData();
    Serial.print("Sensor de flujo oxigeno: ");
    Serial.print(FOx);
    Serial.println(" L/min");
    //lectura flujo aire
    TCA9548A(5); // select I2C bus 1 for the air flow sensor
    FAir = SensorData();
    Serial.print("Sensor de flujo aire: ");
    Serial.print(FAir);
    Serial.println(" L/min");
    Serial.print("Presion insiratoria: ");
    Serial.print(Pi);
    Serial.println(" cmH20");
    Serial.print("Sensor de oxigeno: ");
    Serial.print(Os);
    Serial.println(" %");
    Serial.print("Presion exhalatoria: ");
    Serial.print(SensorPexh());
    Serial.println(" cmH20");
    TCA9548A(3); // select I2C bus 1 for the oxygen flow sensor
    Fexh = SensorData();
    Serial.print("Sensor de flujo exhalatorio: ");
    Serial.print(Fexh);
    Serial.println(" L/min");
    if (FOx > 200) {
      Serial.println("Falla en sensor de flujo oxigeno");
    }
    if (FAir > 200) {
      Serial.println("Falla en sensor de flujo aire");
    }
    if (Fexh > 200) {
      Serial.println("Falla en sensor de flujo exhalatorio");
    }
    if ((FOx > 200) || (FAir > 200) || (Fexh > 200))
      bandtest3 = 1;
    else
      bandtest3 = 0;
    delay(500);
  }
  if (bandtest3 == 0)
    Serial.println ("Test 3.PASO");
  else
    Serial.println ("Test 3.FALLO");
  calculopwm(0, 0);
  delay(3000);
  digitalWrite(alarma, LOW);
  digitalWrite(AVexh, HIGH);
  delay(2000);
  if ((bandtest1 == 0) && (bandtest3 == 0)) {
    Serial.println ("Prueba finalizada");
  }
  else
    Serial.println ("Prueba fallida");
  entrada = "";
  bandtest1 = 0;
  bandtest3 = 0;
}

void flujodecreciente(){
  int valflujo=0;
  int inicioflujo=50;
  int FAir=0;
  int contadorflujo=0;
       for (int i=0; i<500;i++){
        
        contadorflujo=i;
        //if (contadorflujo>=10)
        //contadorflujo=21+i;
        
        valflujo = inicioflujo * pow(1 / 1.1, contadorflujo);
        //valflujo=valflujo-;
        if (valflujo<=0)
        valflujo=0;
        //if (oxigeno == 21)
        //calculopwm (valflujo, 0);//valvula oxigeno
        calculopwm (0, valflujo);//valvula aire
        //else{
        //  lpmair = (-valflujo * (oxigeno - 100)) / 80;
        //  lpmoxig = valflujo - lpmair;
        //  calculopwm(lpmoxig, lpmair);
        //}
        //4 oxigeno, 5 aire
         TCA9548A(5); // select I2C bus 1 for the air flow sensor
    FAir = SensorData();
    //Serial.print("Sensor de flujo aire: ");
    Serial.println(FAir);
    delay(250);
       }
}
