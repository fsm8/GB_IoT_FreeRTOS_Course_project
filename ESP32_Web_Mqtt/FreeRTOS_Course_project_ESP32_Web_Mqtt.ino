#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include <PubSubClient.h>

/* topics */
#define LED_TOPIC "smarthome/room1/Thresholdfsm" /* Вводим через программу. Получаем с MQTT-сервера уровень порога включения светодиода */
#define MASK_TOPIC "smarthome/room1/Mask" /* Вводим через программу. Получаем с MQTT-сервера битовые маски для Нуклео */
#define LED_TOPIC_PUBLISH "smarthome/room1/Luminosityfsm" /* Освещенность с Нуклео */
#define MASK_TOPIC_PUBLISH "smarthome/room1/Masksfsm" /* Битовые маски с Нуклео */

/* create an instance of PubSubClient client */
WiFiClient espClient;
PubSubClient client(espClient);


const char *ssid = "dlink";
const char *password = "538stm32";
//const char* ssid = "iPhone"; // WiFi ssid
//const char* password = "tn8M18)ncK"; // WiFi password

const char* mqtt_server = "51.250.11.159"; // IP adress MQTT-server

const char led = 23; // Пин светодиода
unsigned long myTimer1; // для millis
byte value = 0;  // Величина освещенности (получаем с платы Нуклео)
String message = "0"; // Промежуточная переменная для Luminosity
String sliderValue1 = "0"; // Уровень включения светодиода
String Luminosity = "0"; // Уровень освещенности
String BitMaskWs = ""; // Битовые маски для WebSocket
String maskFromMqtt = "0"; // Битовая маска для Нуклео. Принимаем с mqtt
char buff_msg[32]; // Величина освещенности для MQTT-сервера
char buff_msg2[32]; // Введенные маски для MQTT-сервера
char buff_mask[4] = { 0 }; // Принимаемые с mqtt битовые маски. 4 шт.
uint8_t tmp = 0; // для записи битовых масок в массив c mqtt или Web'а

void receivedCallback(char* topic, byte* payload, unsigned int length)
{
  if (strcmp(topic, "smarthome/room1/Thresholdfsm") == 0)
  {
    payload[length] = 0;
    message = (char *)payload;
    sliderValue1 = message.substring(0); // Передаем уровень защелки в вэб-сокет (а оттуда уже в Нуклео)!
    notifyClients(getSliderValues()); // Передаем величину освещенности через вэб-сокет
  }

  if (strcmp(topic, "smarthome/room1/Mask") == 0)
  {
    maskFromMqtt = "0";
    for (int i = 0; i < length; i++)
    {
      maskFromMqtt += (char)payload[i]; // Запишем маску с mqtt в строку для передачи в Нуклео
    }
    buff_mask[tmp] = (uint8_t)maskFromMqtt.toInt(); // Запишем маски в массив для передачи mqtt
    tmp ++;
    if (tmp == 4)
    {
      tmp = 0;
    }
  }

}

void mqttconnect()
{
  /* Loop until reconnected */
  while (!client.connected())
  {
    Serial.print("MQTT connecting ...");

    String clientId = "ESP32Clientfsm"; /* client ID */

    if (client.connect(clientId.c_str(), "user", "qwope354F") ) /* connect now */
    {
      Serial.println("MQTT connected");
      /* subscribe topic with default QoS 0*/
      client.subscribe(LED_TOPIC);
      client.subscribe(MASK_TOPIC);
    }
    else
    {
      Serial.print("failed, status code =");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");

      delay(5000); /* Wait 5 seconds before retrying */
    }
  }
}

AsyncWebServer server(80); // Create AsyncWebServer object on port 80

AsyncWebSocket ws("/ws"); // Create a WebSocket object

JSONVar sliderValues; //Json Variable to Hold Slider Values

String getSliderValues() //Get Slider Values
{
  sliderValues["Lux"] = String(Luminosity);
  sliderValues["Bitmask"] = String(BitMaskWs);
  sliderValues["sliderValue1"] = String(sliderValue1);

  String jsonString = JSON.stringify(sliderValues);
  return jsonString;
}

void initFS()
{
  if (!SPIFFS.begin()) // Initialize SPIFFS
  {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  else
  {
    Serial.println("SPIFFS mounted successfully");
  }
}

void initWiFi() // Initialize WiFi
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

void notifyClients(String sliderValues)
{
  ws.textAll(sliderValues);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
  {
    data[len] = 0;
    message = (char *)data;

    if (message.indexOf("1s") >= 0)
    {
      sliderValue1 = message.substring(2);
      Serial.print(getSliderValues());
      notifyClients(getSliderValues());
    }

    if (message.indexOf("2s") >= 0)
    {
      maskFromMqtt = "";
      maskFromMqtt = message.substring(2);
      buff_mask[tmp] = (uint8_t)maskFromMqtt.toInt(); // Запишем маски в массив для передачи mqtt
      tmp ++;
      if (tmp == 4)
      {
        tmp = 0;
      }
      notifyClients(getSliderValues());
    }

    if (strcmp((char *)data, "getValues") == 0)
    {
      notifyClients(getSliderValues());
    }
  }
}

void onEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  switch (type)
  {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket()
{
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200);

  /* set led as output to control led on-off */
  pinMode(led, OUTPUT);

  initFS();
  initWiFi();
  initWebSocket();

  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index.html", "text/html");
  });
  server.serveStatic("/", SPIFFS, "/");
  server.begin(); // Start server

  client.setServer(mqtt_server, 1883); /* configure the MQTT server with IPaddress and port */
  client.setCallback(receivedCallback); /* this receivedCallback function will be invoked when client received subscribed topic */
}

void loop()
{

  if (millis() - myTimer1 >= 1000)
  {
    //digitalWrite(led, 1);

    myTimer1 = millis();
    uint8_t tmp[2] = {0}; // перевод порога и маски из строк в int

    tmp[0] = sliderValue1.toInt(); // Записываем порог выключения светодиода в Нуклео. Получаем с вэба.
    tmp[1] = maskFromMqtt.toInt(); // Записываем маски в Нуклео. Получаем с mqtt.
    value = 0;
    
    Serial2.write(tmp, 2); // // Записываем порог выключения светодиода и битовую маску в Нуклео.
    if (Serial2.available() > 0) // Данные с Нуклео пришли
    {
      Serial2.readBytes(&value, 1); // Величина освещенности (читаем с платы Нуклео)
    }
    else
    {
      Serial.print("No value from Nucleo"); Serial.println(""); // Данных с Нуклео нет
    }
    while (Serial2.read() >= 0); // Очищаем буфер приема

    BitMaskWs = ""; // "Обнуляем" строку
    for (int i = 0; i < 4; i++)
    {
      BitMaskWs += String(buff_mask[i], DEC);
      if (i != 3)
      {
        BitMaskWs += ", ";
      }

    }

    message = "";
    message = String(value, DEC);
    Luminosity = "";
    Luminosity = message.substring(0);

    notifyClients(getSliderValues()); // Передаем величину освещенности через вэб-сокет

    sprintf(buff_msg2, "%d, %d, %d, %d", buff_mask[0], buff_mask[1], buff_mask[2], buff_mask[3]);
    client.publish(MASK_TOPIC_PUBLISH, buff_msg2); // Публикуем введенные маски на MQTT-сервере в Masksfsm

    for (int i = 0; i < 32; i++)
    {
      buff_msg[i] = 0; // Обнуляем массив
    }

    sprintf(buff_msg, "%d", value);
    client.publish(LED_TOPIC_PUBLISH, buff_msg); // Публикуем величину освещенности на MQTT-сервере в Luminosityfsm

    Serial.print("Порог включения светодиода = "); Serial.print(sliderValue1); Serial.print(" ");
    Serial.print("Величина освещенности = "); Serial.print(value); Serial.print(" ");
    Serial.print("Битовые маски = "); Serial.print(buff_msg2); Serial.println(" ");
  }

  if (!client.connected()) /* if client mqtt was disconnected then try to reconnect again */
  {
    mqttconnect();
  }

  client.loop(); /* this function will listen for incomming subscribed topic-process-invoke receivedCallback */

  if (WiFi.status() != WL_CONNECTED) /* if WiFi was disconnected then try to reconnect again */
  {
    Serial.print("Connecting to WiFi ..");

    initWiFi();
    initWebSocket();

    // Web Server Root URL
    server.on("/", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      request->send(SPIFFS, "/index.html", "text/html");
    });
    server.serveStatic("/", SPIFFS, "/");
    server.begin(); // Start server
  }

  ws.cleanupClients();

}
