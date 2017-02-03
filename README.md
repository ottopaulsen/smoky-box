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

## Generell doc

[How to connect and program the ESP8266](doc/ESP8266 Programming.md)

[Programming the NodeMCU](doc/NodeMCU.md)

[220V Detector](doc/220V detector.md)

[OTA Update](http://esp8266.github.io/Arduino/versions/2.0.0/doc/ota_updates/ota_updates.html) "Over the air" update av firmware i ESP8266. Jeg bruker [Web Browser varianten](http://esp8266.github.io/Arduino/versions/2.0.0/doc/ota_updates/ota_updates.html#web-browser)

[Integrating with Homekit](doc/Homekit.md)

