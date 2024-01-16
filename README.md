# PARCTICA-N-5-ESP32_DTH22_HC-SR04_LCD

Este repositorio muestra la quinta practica del diplomado de como podemos programar una ESP32 con el sensor ultrasonico HC-SR04 mostrando la distancia en cm y un sensor DTH22  que muestra temperatura y humedad en la LCD.

## Introducción

### Descripción

La ```Esp32``` la utilizamos en un entorno de adquision de datos, lo cual en esta practica ocuparemos un sensor (```Ultrasonico HC-SR04```)  medicion de distancia, añidimos un sensor (```DTH22```) para la obtención de datos de temperatura y humedad, ademas agregaremos un (```LCD 16X2 I2C```) para mostrar los resultados en la pantalla lcd; Esta practica se usara un simulador llamado [WOKWI](https://wokwi.com/).


## Material Necesario

A continuacion se utilizaron los siguientes materiales.

- [WOKWI](https://https://wokwi.com/)
- Tarjeta ESP 32
- Sensor ultrasonico HC-SR04
- LCD 16X2 I2C
- Sensor DHT22



## Instrucciones

### Requisitos previos

Para realizar la practica de este repositorio se necesita entrar a la plataforma [WOKWI](https://https://wokwi.com/).


### Instrucciones de preparación de entorno 

1. Abrir la terminal de programación y colocar la siguente codigo:

```
#include "DHTesp.h"
#include <LiquidCrystal_I2C.h>
#define I2C_ADDR    0x27
#define LCD_COLUMNS 20
#define LCD_LINES   4

const int DHT_PIN = 15;
const int Trigger = 4;   //Pin digital 2 para el Trigger del sensor
const int Echo = 2;   //Pin digital 3 para el Echo del sensor

DHTesp dhtSensor;

LiquidCrystal_I2C lcd(I2C_ADDR, LCD_COLUMNS, LCD_LINES);

void setup() {

  Serial.begin(115200);
  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);
  pinMode(Trigger, OUTPUT); //pin como salida
  pinMode(Echo, INPUT);  //pin como entrada
  digitalWrite(Trigger, LOW);//Inicializamos el pin con 0
  lcd.init();
  lcd.backlight();

}

void loop() {

  TempAndHumidity  data = dhtSensor.getTempAndHumidity();
  Serial.println("Temp: " + String(data.temperature, 1) + "°C");
  Serial.println("Humidity: " + String(data.humidity, 1) + "%");
  Serial.println("---");
  
  long t; //timepo que demora en llegar el eco
  long d; //distancia en centimetros

  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);          //Enviamos un pulso de 10us
  digitalWrite(Trigger, LOW);
  
  t = pulseIn(Echo, HIGH); //obtenemos el ancho del pulso
  d = t/59;             //escalamos el tiempo a una distancia en cm
  
  Serial.print("Distancia: ");
  Serial.print(d);      //Enviamos serialmente el valor de la distancia
  Serial.print("cm");
  Serial.println();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("  Temp: " + String(data.temperature, 1) + "\xDF"+"C  ");
  lcd.setCursor(0, 1);
  lcd.print(" Humidity: " + String(data.humidity, 1) + "% ");
  delay(2000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Distancia: " + String(d) + "cm");
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("  Yasmin Zagal    ");
  lcd.setCursor(0, 1);
  lcd.print("      IEE     ");
  delay(2000);
}

```


2. Instalar la libreria de **Cristal líquido I2C** **DTH sensor library for ESPx**. 
   - Seleccionar pestaña de Librery Manager --> Add a New library --> Colocamos el nombre de libreria 

![](https://github.com/YasminZagal/PARCTICA-N-5-ESP32_DTH11_HC-SR04_LCD/blob/main/librerias%20practica5.png)


3. Realizar la conexion de **HC-SR04**, **LCD** y **DTH22** con la **ESP32** de la siguiente manera.

![](https://github.com/YasminZagal/PARCTICA-N-5-ESP32_DTH11_HC-SR04_LCD/blob/main/conexiones%20practica%205.png)

  **Conexion lcd**
  -GND
  -VCC
  -SDA --> esp:21
  -SCL --> esp:22

  **Conexión HC-SR04**
  -VCC --> 5V
  -TRIG --> esp:4
  -ECHO --> esp:15
  -GND  --> GND
  
  **Conexión DTH22**
  -VCC --> 3V
  -SDA --> esp:15
  -NC 
  -GND  --> GND

### Instrucciónes de operación

1. Iniciar simulador.
2. Visualizar los datos en el monitor serial y en la lcd .
3. Colocar distancia *doble click* al sensor **HC-SR04**
4. Colocar temperatura y humedad *doble click* al sensor **DTH22**

![](https://github.com/YasminZagal/PARCTICA-N-5-ESP32_DTH11_HC-SR04_LCD/blob/main/sensor%20ultrasonico.png)
![](https://github.com/YasminZagal/PARCTICA-N-5-ESP32_DTH11_HC-SR04_LCD/blob/main/sensor%20DHT22.png)
  
## Resultados

Una vez terminado iniciamos simulacion y se observaran los valores de la humedad y temperatura como la distancia en cm en la lcd.
Ademas de que en la LCD aparece nuestro nombre y carrera 

![](https://github.com/YasminZagal/PARCTICA-N-5-ESP32_DTH11_HC-SR04_LCD/blob/main/resultado%20practica5.png)
![](https://github.com/YasminZagal/PARCTICA-N-5-ESP32_DTH11_HC-SR04_LCD/blob/main/LCD%20personalizado.png)
