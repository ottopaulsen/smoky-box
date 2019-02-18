# Smoky Box

Se beskrivelse i [smoky](https://github.com/ottopaulsen/smoky)

## NodeMCU

![NodeMCU Pinout](http://cdn.frightanic.com/blog/wp-content/uploads/2015/09/esp8266-nodemcu-dev-kit-v2-pins.png)


## Vifte

Cooler Master A9225-...
12V, 0.6A

Pinout:
Sort: GND
Rød: 12V
Gul: Sense. Open collector. Kan kobles til 3,3V eller 5V. Frekvens forteller hastighet.
Blå: Control. PWM kontrollerer farta. 3,3V fungerer.

Use analogWrite() to write a value between 0 and 1023 on any of pin 0-8. You may not use the serial port if you use the serial pins.

## Components

DHT22 pinout:

![DHT22 Pinout](http://electropark.pl/img/cms/Czujniki/temperatury/dht22_wyproadzenia.jpg)

The DHT22 is connected using a 3 pin 3,5mm jack as follows:

* GND is connected to the outer part
* VCC is connected to the middle part
* Data is connected to the inner part (center)

## Generell doc

[ESP8266 Reference Doc](http://arduino.esp8266.com/versions/1.6.5-1160-gef26c5f/doc/reference.html) Lots of useful information.

[How to connect and program the ESP8266](doc/ESP8266 Programming.md)

[Programming the NodeMCU](doc/NodeMCU.md)

[220V Detector](doc/220V detector.md)

[OTA Update](http://esp8266.github.io/Arduino/versions/2.0.0/doc/ota_updates/ota_updates.html) "Over the air" update av firmware i ESP8266. Jeg bruker [Web Browser varianten](http://esp8266.github.io/Arduino/versions/2.0.0/doc/ota_updates/ota_updates.html#web-browser)

[Integrating with Homekit](doc/Homekit.md)

