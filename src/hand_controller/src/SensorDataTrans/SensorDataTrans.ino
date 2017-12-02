#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

int status = WL_IDLE_STATUS;
#include "arduino_secrets.h" 
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)
unsigned int localPort = 8080;      // local port to listen on
char  SendBuffer[] = "acknowledged";       // a string to send back
char remoteip[]="192.168.137.1";
uint16_t remoteport=5060;

double previoustime;

WiFiUDP Udp;

#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();

#define BUTTONLED1 2
#define BUTTON1 A1
int ledStatues1=HIGH;
int ButtonStatues1;
#define BUTTONLED2 4
#define BUTTON2 A2
int ledStatues2=HIGH;
int ButtonStatues2;

void setup() {
  WiFi.setPins(8,7,4);
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

    if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to wifi");
  printWiFiStatus();
  Serial.println("\nStarting connection to server...");
  Udp.begin(localPort);
  
//time setup
  previoustime=millis();
  
//button setup
  pinMode(BUTTONLED1,OUTPUT);
  pinMode(BUTTON1,INPUT);
  digitalWrite(BUTTONLED1, ledStatues1);
  pinMode(BUTTONLED2,OUTPUT);
  pinMode(BUTTON2,INPUT);
  digitalWrite(BUTTONLED2, ledStatues2);
  
}

void loop() {

  if(millis()-previoustime>10){

    //button part

      int reading1=analogRead(BUTTON1);
      int reading2=analogRead(BUTTON2);

 // Serial.print(reading1);
  
  if (reading1>200){
    ButtonStatues1=1;
    digitalWrite(BUTTONLED1,HIGH);
  }else{
    ButtonStatues1=0;
    digitalWrite(BUTTONLED1,LOW);
  }
  Serial.print(ButtonStatues1);
  Serial.print("\n");

    if (reading2>200){
    ButtonStatues2=1;
    digitalWrite(BUTTONLED2,HIGH);
  }else{
    ButtonStatues2=0;
    digitalWrite(BUTTONLED2,LOW);
  }
  Serial.print(ButtonStatues2);
  Serial.print("\n");
  


  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
  
  Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.print("\n");
 
    Udp.beginPacket(remoteip, remoteport);
    Udp.print(euler.x());
    Udp.write("\t");
    Udp.print(euler.y());
    Udp.write("\t");
    Udp.print(euler.z());
    Udp.write("\t");
    Udp.print(ButtonStatues1);
    Udp.write("\t");
    Udp.print(ButtonStatues2);
    Udp.write("\n");
    //Udp.write(euler.x);
    Udp.endPacket();
    previoustime=millis();
  }
}


void printWiFiStatus() {
 
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());


  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);


  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}




