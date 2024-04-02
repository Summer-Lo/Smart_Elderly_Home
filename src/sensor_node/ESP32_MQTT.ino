
/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include "ArduinoJson.h"
#include <stdio.h>

#define TX1 1
#define RX1 3

// Replace the next variables with your SSID/Password combination
const char* ssid = "IE_Maker_User";
const char* password = "summer2023";

// Add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";
const char* mqtt_server = "broker.hivemq.com";
const char* client_id = "SummerClient121212";
const char* subTopic = "ICW503/IoT";

// UART message define
uint8_t MD1st = 0xFF;
uint8_t MD2nd = 0xFF;
uint8_t dataR;
uint8_t dataC;
uint8_t CS1st;
uint8_t CS2nd;
char hex_str[5];
char dataR_temp[10], dataC_temp[10];

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

HardwareSerial SerialPort(1);

void setup() {
  SerialPort.begin(115200,SERIAL_8N1,RX1,TX1);
  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  //status = bme.begin();  
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);  
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  SerialPort.println();
  SerialPort.print("Connecting to ");
  SerialPort.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    SerialPort.print(".");
  }

  SerialPort.println("");
  SerialPort.println("WiFi connected");
  SerialPort.println("IP address: ");
  SerialPort.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  SerialPort.print("Message arrived on topic: ");
  SerialPort.print(topic);
  SerialPort.print(". Message: ");
  String messageTemp = "";
  
  for (int i = 0; i < length; i++) {
    SerialPort.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  SerialPort.println();

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == String(subTopic)) {
    SerialPort.println("Same Topic");
    StaticJsonDocument<200> jsonBuffer;
    // parse the JSON message
    DeserializationError error = deserializeJson(jsonBuffer, messageTemp);
    // Test if parsing succeeds.
    if (error) {
      SerialPort.print(F("deserializeJson() failed: "));
      SerialPort.println(error.f_str());
      return;
    }
    String type = jsonBuffer["Type"];         //Get msg type value
    int row = jsonBuffer["Row"];                  //Get msg value of row
    int col = jsonBuffer["Col"];                  //Get msg value of row
    String status = jsonBuffer["Status"];     //Get msg status
    //SerialPort.print("Type: ");
    //SerialPort.println(String(type));
    //SerialPort.print("Row value: ");
    //SerialPort.println(row);
    //SerialPort.print("Column value: ");
    //SerialPort.println(col);
    //SerialPort.print("Status: ");
    //SerialPort.println(status);
    //SerialPort.println();
    if(type == "IoT_station" and status == "Open")
    {
      uint8_t num_hex[10] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};
      uint8_t bytes1st = 0xFF;                  // Starting Byte
      uint8_t bytes2nd = 0xFF;                  // Starting Byte
      uint8_t bytes3rd = num_hex[row-1];    // Data Byte 1st
      uint8_t bytes4th = num_hex[col-1];    // Data Byte 2nd
      uint8_t bytes5th = 0xAA;                  // Ending Byte
    
      SerialPort.write(bytes1st);
      SerialPort.write(bytes2nd);
      SerialPort.write(bytes3rd);
      SerialPort.write(bytes4th);
      SerialPort.write(bytes5th);
      SerialPort.println();
      //uart_Tx(row,col);
      //delay(200);
      // Computes quotient
      //int dividend = lock;
      //int divisor = 10;
      //int Row = dividend / divisor;
      // Computes remainder
      //int Column = dividend % divisor;
      //dataR = num_hex[Row-1];
      //dataC = num_hex[Column-1];
      //SerialPort.println(dataR);
      //SerialPort.println(dataC);
      //SerialPort.write(0x33);
      //SerialPort.write(0x31);
      //int Row_temp, Column_temp;
      //Row_temp = intToAscii(Row);
      //Column_temp = intToAscii(Column);
      //SerialPort.println(Row_temp);
      //SerialPort.println(Column_temp);
      //sprintf(dataR, "0x%02X", Row_temp); // Convert the integer to a hexadecimal string with "0x" prefix and zero padding
      //SerialPort.println(dataR);
      //sprintf((char*)dataC, "0x%02X", Column_temp); // Convert the integer to a hexadecimal string with "0x" prefix and zero padding
      //SerialPort.println(dataC);
    }
    //Clear the buffer
    jsonBuffer.clear();  
    delay(100);
  }
}

int uart_Tx(int byteRow, int byteCol){
  int num_key[10] = {0,1,2,3,4,5,6,7,8,9}; // Num_key = num of row/col - 1
  uint8_t num_hex[10] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};
  unsigned char bytesSent = 0xFFFF3138;  //send the string "hello" and return the length of the string.
  uint8_t bytes1st = 0xFF;                  // Starting Byte
  uint8_t bytes2nd = 0xFF;                  // Starting Byte
  uint8_t bytes3rd = num_hex[byteRow-1];    // Data Byte 1st
  uint8_t bytes4th = num_hex[byteCol-1];    // Data Byte 2nd
  uint8_t bytes5th = 0xAA;                  // Ending Byte

  SerialPort.write(bytes1st);
  SerialPort.write(bytes2nd);
  SerialPort.write(bytes3rd);
  SerialPort.write(bytes4th);
  SerialPort.write(bytes5th);
  SerialPort.println();
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    SerialPort.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(client_id)) {
      SerialPort.println("connected");
      // Subscribe
      client.subscribe(subTopic);
    } else {
      SerialPort.print("failed, rc=");
      SerialPort.print(client.state());
      SerialPort.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  //if(SerialPort.available()){
    //String Buffer = SerialPort.readString();
    //SerialPort.print(Buffer);
  //}

}
