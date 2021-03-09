# Bosch-CISS
The *Bosch* ***Connected Industrial Sensor Solution (CISS)***  is a small and robust multi-sensor device for harsh industrial environments. Designed to detect acceleration and vibration, as well as a wide range of environmental conditions. Is equipped with an accelerometer, gyroscope, magnetometer, temperature sensor, humidity sensor, pressure sensor, microphone, and light sensor. 
It is best used for machine condition monitoring and is perfectly suited for Industry 4.0 applications. The Connected Industrial Sensor Solution monitors machines, machine modules, and plants in operation. 

![CISS](https://i.postimg.cc/Hnx9rYRJ/CISS2.png)

In this repository you will find explanations and step by step for the development of a system that every certain time interval (which you will set) will collect data from the internal sensors of ***CISS*** device, this data package will be sent to a special gateway designed by *IOT2TANGLE* that you will have on the local network, or even on an *Raspberry Pi*. This gateway will be in charge of adding these packages to **Tangle Network of IOTA**, through *Streams*.

## Available connectivity
- **[HTTP-sender](https://github.com/iot2tangle/bosch-ciss/tree/main/http-sender)** (*CISS* will send the sensors data through HTTP to **[I2T HTTP Gateway](https://github.com/iot2tangle/Streams-http-gateway)** or **[Keepy](https://github.com/iot2tangle/Keepy)**
