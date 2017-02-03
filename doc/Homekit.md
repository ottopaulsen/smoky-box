# Integrating with Homekit

## Introduction

I want to control the smoky box from my iPhone, and why not use [Homekit](https://developer.apple.com/library/content/documentation/NetworkingInternet/Conceptual/HomeKitDeveloperGuide/Introduction/Introduction.html#//apple_ref/doc/uid/TP40015050)? There is a tool called [Homebridge](https://github.com/nfarina/homebridge) that can be used to integrate with Homekit.


I have a Raspberry Pi 3 with [Ubuntu Mate](https://ubuntu-mate.org/) installed. It is doing almost nothing, so I want to install homebridge there, for this purpose.

## Installation log

Ubuntu Mate is already running on my Pi, so I start following the [installation instructions](https://github.com/nfarina/homebridge#installation) for Homebridge, including some of the things mentioned in the Raspberry Pi [Wiki](https://github.com/nfarina/homebridge/wiki/Running-HomeBridge-on-a-Raspberry-Pi).

One note about [Installing Homebridge and dependencies](https://github.com/nfarina/homebridge/wiki/Running-HomeBridge-on-a-Raspberry-Pi#install-homebridge-and-dependencies): On Ubuntu Mate, the node_modules are installed under `/usr/lib`, not `/usr/local/lib`.

Installed the [homebridge-mqtt-temperature plugin](https://www.npmjs.com/package/homebridge-mqtt-temperature), the [homebridge-mqtt-humidity plugin](https://www.npmjs.com/package/homebridge-mqtt-humidity) and the [mqtt library](https://www.npmjs.com/package/mqtt).

According to the installation instructions for homebridge, it should not work until I created the config.json file. However, it did start without this file. Probably because there were some files under the accessories and the persist directories in the ~/.homebridge folder. I deleted those directories and created a new config.json file like this:
```
{
  "bridge": {
    "name": "HomeBridge-Otto",
    "username": "AC:33:3B:D3:CE:32",
    "port": 55149,
    "pin": "031-45-154"
  },

  "description": "Otto sin homebridge hjemme",

  "accessories": [
    {
      "accessory": "mqtt-temperature",
      "name": "Smoky temperatur",
      "url": "mqtt://m20.cloudmqtt.com:15555",
      "topic": "smoky/1/inside/temperature",
      "username": "henqvddw",
      "password": "hemmelig"
    },
    {
      "accessory": "mqtt-humidity",
      "name": "Smoky luftfuktighet",
      "url": "mqtt://m20.cloudmqtt.com:15555",
      "topic": "smoky/1/inside/humidity",
      "username": "henqvddw",
      "password": "hemmelig"
    }
  ],

  "platforms": []
}
```

The name can be anything.
I have tried to figure out what the username is, without luck, but you can change it to something with legal hex numbers, I think. I believe you can select the port you like (as long as it is not used by something else), and also the pin, but the format must be the same.

The accessory is the name of the plugins, and if you have an MQTT running, you will recognize the other values under accessories.

I found the homebridge in the Home app on my iPhone, but every time I restarted homebridge, I had to set ig up again. I dont know why. Should not be necessary.


## References


[Introduction to Homekit (apple)](https://developer.apple.com/library/content/documentation/NetworkingInternet/Conceptual/HomeKitDeveloperGuide/Introduction/Introduction.html#//apple_ref/doc/uid/TP40015050)

[Homebridge (Node)](https://github.com/nfarina/homebridge)

[Homebridge on RaspberryPi](https://github.com/nfarina/homebridge/wiki/Running-HomeBridge-on-a-Raspberry-Pi)

[Homecontrol (go)](http://selfcoded.com/homecontrol/)

[Plugin simple example](https://github.com/nfarina/homebridge/tree/6500912f54a70ff479e63e2b72760ab589fa558a/example-plugins/homebridge-lockitron)


[Homebridge MQTT switch](https://www.npmjs.com/package/homebridge-mqttswitch)

[Homebridge MQTT temperature](https://www.npmjs.com/package/homebridge-mqtt-temperature)


