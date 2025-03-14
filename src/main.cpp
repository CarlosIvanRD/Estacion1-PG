#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NewPing.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Client Id
const char *clientid = "ESP32Client_1";

// WiFi
const char *ssid = "Redmi10";
const char *password = "180921red";

// Credenciales MongoDB
const char *mqtt_user = "emqx_u"; // Usuario configurado en MongoDB
const char *mqtt_pass = "secret"; // Contraseña antes del hash bcrypt

// MQTT Broker
const char *mqtt_server = "junction.proxy.rlwy.net";
const int mqtt_port = 19243;

// Definiciones del sensor HC-SR04
#define TRIGGER_PIN 5
#define ECHO_PIN 18
#define MAX_DISTANCE 200 // Máxima distancia a medir (en cm)

// Inicializa el sensor HC-SR04
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Definiciones para la pantalla OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Definiciones para el sensor de flujo de agua
#define FLOW_SENSOR_PIN 4
volatile uint16_t pulseCount = 0;
float flowRate = 0.0;
unsigned long oldFlowTime = 0;

// Definiciones para el sensor de lluvia MH-RD
#define SENSOR_ANALOG_PIN 36 // Pin analógico

WiFiClient espClient;
PubSubClient client(espClient);

// Intervalos de tiempo para cada sensor
unsigned long distInterval = 30000;  // 5 minutos
unsigned long flowInterval = 12000;  // 2 minutos
unsigned long waterInterval = 12000; // 2 minutos

// Temporizadores para cada sensor
unsigned long lastDistTime = 0;
unsigned long lastFlowTime = 0;
unsigned long lastWaterTime = 0;
unsigned long lastDisplayTime = 0; // Para controlar la frecuencia de actualización de la pantalla

void setup_wifi()
{
  delay(10);
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void reconnect()
{
  while (!client.connected())
  {
    if (client.connect(clientid, mqtt_user, mqtt_pass))
    {
      Serial.println("Connected to MQTT Broker!");
      client.subscribe("esp32/estacion1/config"); // Suscribirse al tópico de configuración
    }
    else
    {
      Serial.println("Failed to connect, trying again in 5 seconds...");
      delay(5000); // Esperar 5 segundos antes de intentar reconectar
    }
  }
}

// Interrupción para contar los pulsos del sensor de flujo
void IRAM_ATTR pulseCounter()
{
  pulseCount++;
}

void callback(char *topic, byte *message, unsigned int length)
{
  String messageTemp;
  for (int i = 0; i < length; i++)
  {
    messageTemp += (char)message[i];
  }
  Serial.println("Message arrived [" + String(topic) + "]: " + messageTemp);

  StaticJsonDocument<256> doc;
  deserializeJson(doc, messageTemp);

  if (String(topic) == "esp32/estacion1/config")
  {
    const char *action = doc["action"];
    if (strcmp(action, "update_interval") == 0)
    {
      distInterval = doc["data"]["distInterval"];
      flowInterval = doc["data"]["flowInterval"];
      waterInterval = doc["data"]["waterInterval"];
      Serial.println("Intervals updated:");
      Serial.println("  distInterval: " + String(distInterval));
      Serial.println("  flowInterval: " + String(flowInterval));
      Serial.println("  waterInterval: " + String(waterInterval));
    }
  }
}

void sendSensorData(const char *id, float value, const char *type)
{
  StaticJsonDocument<256> doc;
  doc["station"] = "66455fd1903474beccdb69b5";
  JsonArray sensors = doc.createNestedArray("sensors");
  JsonObject sensor = sensors.createNestedObject();
  sensor["id"] = id;
  sensor["value"] = value;
  sensor["type"] = type;

  char buffer[256];
  size_t n = serializeJson(doc, buffer);

  // Mensaje de depuración
  Serial.print("Publicando mensaje: ");
  Serial.println(buffer);

  // Publicar los datos en los tópicos MQTT
  if (client.publish("esp32/estacion1", buffer, n))
  {
    Serial.println("Mensaje publicado exitosamente");
  }
  else
  {
    Serial.println("Error al publicar el mensaje");
  }
}

void displayData(unsigned int distance, float flowRate, float percentage)
{
  display.clearDisplay();              // Limpia la pantalla
  display.setTextSize(1);              // Tamaño del texto
  display.setTextColor(SSD1306_WHITE); // Color del texto

  display.setCursor(0, 0);  // Posición del cursor
  display.print("level: "); // Texto fijo
  display.print(distance);  // Distancia medida
  display.print("cm");      // Unidad de medida

  display.setCursor(0, 10); // Posición del cursor para el flujo
  display.print("flow: ");  // Texto fijo
  display.print(flowRate);  // Flujo medido
  display.print("L/m");     // Unidad de medida

  display.setCursor(0, 20);  // Posición del cursor para el sensor de lluvia
  display.print("rain: ");   // Texto fijo
  display.print(percentage); // Porcentaje de agua
  display.print("%");        // Unidad de medida

  // Determinar el estado del sensor de lluvia
  display.setCursor(78, 20); // Posición del cursor para el estado del sensor de lluvia
  if (percentage < 30)
  {
    display.print("Seco");
  }
  else if (percentage < 55)
  {
    display.print("Llovizna");
  }
  else if (percentage < 75)
  {
    display.print("Llueve");
  }
  else
  {
    display.print("Inundado");
  }

  display.display(); // Muestra el contenido en la pantalla
}

void setup()
{
  Serial.begin(115200);

  // Inicializa la pantalla OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Dirección I2C 0x3C para la pantalla OLED
    Serial.println(F("Fallo al inicializar SSD1306"));
    for (;;)
      ;
  }
  display.display();
  delay(2000); // Pausa para permitir que la pantalla se inicie
  display.clearDisplay();

  // Configura el pin del sensor de flujo y la interrupción
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, FALLING);

  oldFlowTime = millis();

  // Conectar a WiFi y al broker MQTT
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop()
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  unsigned long currentMillis = millis();

  // Lee la distancia del sensor ultrasónico
  if (currentMillis - lastDistTime >= distInterval)
  {
    lastDistTime = currentMillis;
    unsigned int distance = sonar.ping_cm();
    sendSensorData("6788456c3ecaa45dca94725f", distance, "level");
  }

  // Calcula el flujo de agua
  if ((currentMillis - oldFlowTime) > 1000) // Cada segundo
  {
    detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN));
    flowRate = ((1000.0 / (currentMillis - oldFlowTime)) * pulseCount) / 7.5; // Conversión según el datasheet
    oldFlowTime = currentMillis;
    pulseCount = 0;
    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, FALLING);
  }

  if (currentMillis - lastFlowTime >= flowInterval)
  {
    lastFlowTime = currentMillis;
    sendSensorData("678845a43ecaa45dca947264", flowRate, "flow");
  }

  // Lee el valor del sensor de lluvia
  if (currentMillis - lastWaterTime >= waterInterval)
  {
    lastWaterTime = currentMillis;
    int analogValue = analogRead(SENSOR_ANALOG_PIN);
    int invertedAnalogValue = 4095 - analogValue;
    float percentage = invertedAnalogValue / 40.95;
    sendSensorData("678845b13ecaa45dca947269", percentage, "rain");
  }

  // Actualizar la pantalla cada 2 segundos
  if (currentMillis - lastDisplayTime >= 2000)
  {
    lastDisplayTime = currentMillis;
    unsigned int distance = sonar.ping_cm();
    int analogValue = analogRead(SENSOR_ANALOG_PIN);
    int invertedAnalogValue = 4095 - analogValue;
    float percentage = invertedAnalogValue / 40.95;
    displayData(distance, flowRate, percentage);
  }
}
