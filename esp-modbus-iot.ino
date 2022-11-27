// ESP32 - IoT - Modbus
// ESP32 as a link to modbus networks
// Tank level control

// Communicates with MqTT broker EMQX via DASH MqTT App or any other mqtt web dashboard 
// 

// Variables:
// 2 digital INPUT Modbus - tank high level indicator - tank low level indicator
// 1 Digital OUTPUT Modbus - Pump
// 1 Analog ESP32 - Ultrasonic sensor

#ifdef ESP8266
 #include <ESP8266WiFi.h>
#else //ESP32
 #include <WiFi.h>
#endif
#include <ModbusIP_ESP8266.h>



#include <PubSubClient.h> //library for MQTT author: nick oleary
#include <ArduinoJson.h> //library for Parsing JSON author: benoit blanchon

//MQTT Credentials
//const char* mqttServer = "broker.mqttdashboard.com"; //MQTT URL
const char* mqttServer = "broker.emqx.io"; //MQTT URL
const int   port = 1883; //MQTT port
const char* mqttUserName = "gui05";  // MQTT username
const char* mqttPwd = "12345";  // MQTT password
const char* clientID = "user0001"; // client id user+0001
const char* topic1 = "dash/canal1"; //publish topic
const char* topic7 = "dash/canal7"; //receive topic
const char* topic8 = "dash/canal8"; //receive topic
const char* topic9 = "dash/canal9"; //receive topic

//setting up wifi and mqtt client
WiFiClient espClient;
PubSubClient client(espClient);

void reconnect() {
  while (!client.connected()) {
    if (client.connect(clientID, mqttUserName, mqttPwd)) {
      Serial.println("MQTT connected");
      client.subscribe(topic7);
      client.subscribe(topic8);
      client.subscribe(topic9);
      Serial.print("Topics Subscribed");
    }
    else {
      Serial.print("failed, attempting to reconnect");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);  // wait 5sec and retry
    }

  }

}

//Modbus Contacts, Coils and Registers Offsets
// Coil
const int BOMBA_COIL = 100;
const int HIGH_COIL = 101;
const int LOW_COIL = 102;
// Contact 
const int BUT_IR = 500;
// Input register
const int SONIC_IREG = 200;
// Holding register
const float TEST_HREG = 700;

//Used Pins
const int ledPin = 32; 
const int butPin2 = 33; 
//const int tempPin  = A0; 

//Pinos do Sensor Ultrasonic HC-SR04
#define echoPin 12 
#define trigPin 13
//attach pin D3 Arduino to pin Trig of HC-SR04

// Variaveis Sensor Ultrassonico
long duration; // variable for the duration of sound wave travel
float levelperc; // percentual level variable
float distance; // variable for the distance measurement
String mostrar = "";
String msgStr2 = "";

// Analog Out Config - LED PWM
//#define LED_GPIO   23
//#define PWM1_Ch    0
//#define PWM1_Res   8
//#define PWM1_Freq  1000
 
//int PWM1_DutyCycle = 0;


//ModbusIP object
ModbusIP mb;
  
void setup() {
  Serial.begin(115200);
 
  WiFi.begin("ssid", "pwd");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
 
  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  mb.server();

  // Pins for Ultrasonic sensor
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT

  // Pins on ESP32 
  pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(18, OUTPUT);
  //pinMode(ledPin, OUTPUT);
  //pinMode(butPin2, INPUT);
  //pinMode(tempPinH, OUTPUT);

  client.setServer(mqttServer, port); //setting MQTT server
  client.setCallback(callback); //definindo funcao a ser executada quando chega mensagem em tópico de interesse para o controlador

  mb.addCoil(HIGH_COIL);
  mb.addCoil(LOW_COIL);
  mb.addCoil(BOMBA_COIL);
  mb.addIsts(BUT_IR);
  mb.addIreg(SONIC_IREG);
  //mb.addHreg(TEST_HREG, 0x0);

  //ledcAttachPin(LED_GPIO, PWM1_Ch);
  //ledcSetup(PWM1_Ch, PWM1_Freq, PWM1_Res);
}



//subscribe call back - funcao a ser executada quando chega mensagem em tópico de interesse

void callback(char*topic, byte* payload, unsigned int length) {
  String response;

  for (int i = 0; i < length; i++) {
    response += (char)payload[i];
  }
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println(response);

  String aa = String(topic);
  //Serial.print(String(topic));

  if (aa=="dash/canal7"){
      if (response == "1") {
        digitalWrite(16, HIGH);
        mb.Coil(BOMBA_COIL, true);
      }
      else if (response == "0") { 
        digitalWrite(16, LOW);
        mb.Coil(BOMBA_COIL, false);
      }
  }
  else if (aa=="dash/canal8"){
      if (response == "1") {
        digitalWrite(17, HIGH);
        mb.Coil(HIGH_COIL, true);
      }
      else if (response == "0") {
        digitalWrite(17, LOW);
        mb.Coil(HIGH_COIL, false);
      }     
    }
  else if (aa=="dash/canal9"){
      if (response == "1") {
        digitalWrite(18, HIGH);
        mb.Coil(LOW_COIL, true);
      }
      else if (response == "0") {
        digitalWrite(18, LOW);
        mb.Coil(LOW_COIL, false);
      }    
    }
}


 
void loop() {

  // procedimento reconexao
  if (!client.connected()) { //if mqtt client is not connected
    reconnect(); //try to reconnect
  }
  client.loop();

  
  //Call once inside loop() - all magic here
  mb.task();
/*
  // Logica de controle de nivel de tanque - se nivel < 0.30 aciona bomba
  if (levelperc < 0.65) {
    //Alerta nivel ativo
    mb.Coil(HIGH_COIL, false);
    delay(8);
    mb.Coil(LOW_COIL, true);
    delay(8);
    mb.Coil(BOMBA_COIL, true);
    delay(8);
  }
  else if (levelperc > 0.65 && levelperc < 0.80) {
    mb.Coil(LOW_COIL, false);
    delay(8);
    mb.Coil(HIGH_COIL, false);
    delay(8);
    mb.Coil(BOMBA_COIL, false);
    delay(8);
  }
  else {
    mb.Coil(LOW_COIL, false);
    delay(8);
    mb.Coil(HIGH_COIL, true);
    delay(8);
    mb.Coil(BOMBA_COIL, false);
    delay(8);
  }
*/

  //Saida digital para Modbus - mb.Coil
  //digitalWrite(ledPin, mb.Coil(LED_COIL));
  digitalWrite(16, mb.Coil(BOMBA_COIL));
  delay(8);

  digitalWrite(17, mb.Coil(HIGH_COIL));
  delay(8);

  digitalWrite(18, mb.Coil(LOW_COIL));
  delay(8);
   
  //Entrada digital para Modbus - Ists: input states
  //mb.Ists(BUT_IR,digitalRead(butPin2));
  //delay(8);

  //Saida analogica para Modbus - Ireg: input registers - sensor ultrassonico
  mb.Ireg(SONIC_IREG,levelperc);
  delay(8);

  // Imprimindo na serial
  Serial.println("Indicador nivel Alto: "+ String(mb.Coil(HIGH_COIL)));
  delay(100);
  Serial.println("Indicador nivel Baixo: "+ String(mb.Coil(LOW_COIL)));
  delay(500);
  Serial.println("Estado bomba: "+ String(mb.Coil(BOMBA_COIL)));
  //Entrada analogica para Modbus - : 

  //ledcWrite(PWM1_Ch, mb.Hreg(TEST_HREG));
  //delay(3);
  /*
   while(PWM1_DutyCycle < 255)
  {
    ledcWrite(PWM1_Ch, PWM1_DutyCycle++);
    delay(10);
  }
  while(PWM1_DutyCycle > 0)
  {
    ledcWrite(PWM1_Ch, PWM1_DutyCycle--);
    delay(10);
  }
  */

   /*
    //Canal YY: leitura de pino - esp32 --> dashboard
    int readpin = 0;
    readpin = digitalRead(D8);
    Serial.println("Status pino zero: "+String(readpin));
    msgStr2 = "Canal1: " + String(readpin); 
    byte arrSize2 = msgStr2.length() + 1;
    char msg2[arrSize2];
    msgStr2.toCharArray(msg2, arrSize2);
    client.publish(topic1, msg2); 
    msgStr2 = "";
    delay(500);
    */

  // Treating Ultrasonic Sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Level to percentage 0-1
  //levelperc = (1-distance*3.1415*pow(1,2)/4);
  // Tank - diameter 100cm - height 200cm
  levelperc = (200-distance)/200;
  
  mostrar = String(levelperc);
  msgStr2 = mostrar; 
    byte arrSize2 = msgStr2.length() + 1;
    char msg2[arrSize2];
    msgStr2.toCharArray(msg2, arrSize2);
    client.publish(topic1, msg2); 
    msgStr2 = "";
  client.publish(topic1, msg2); 
    
  //levelperc = distance;
  //strlevel = String(levelperc);
  // Displays the distance on the Serial Monitor
  Serial.println("Dist: ");
  Serial.println(distance);
  Serial.println("Nivel: ");
  Serial.println(levelperc);
  Serial.println(" %"); 
  //client.publish(topic1, mostrar);    

  
  delay(3000);
   
}
