# Smoky Box

Se beskrivelse i [smoky](https://github.com/ottopaulsen/smoky)

## Vifte

Cooler Master A9225-...
12V, 0.6A

Pinout:
Sort: GND
Rød: 12V
Gul: Sense. Open collector. Kan kobles til 3,3V eller 5V. Frekvens forteller hastighet.
Blå: Control. PWM kontrollerer farta. 3,3V fungerer.

Use analogWrite() to write a value between 0 and 1023 on any of pin 0-8. You may not use the serial port if you use the serial pins.
