# Bosch CISS  --  HTTP Protocol

The **CISS** is a small and robust multi-sensor device for harsh industrial environments. Is equipped with an accelerometer, gyroscope, magnetometer, temperature sensor, humidity sensor, pressure sensor, microphone, and light sensor.

This Repository contains the source code and the steps to follow to be able to make ***CISS*** read sensor data and send it, in an organized way, to the **[Tangle](https://www.youtube.com/watch?v=ESF8UZM70wU) (DLT)** of the **[IOTA Network](https://www.iota.org/)** through the **[Streams](https://www.iota.org/solutions/streams)** layer.

## Setting up your Bosch CISS

You just have to make sure to make the correct USB connection from the CISS device to your computer.

Once you have done this, the device will be "mounted" on your computer. This happens both on Linux, Windows and Mac, it is not necessary to install any additional driver for it to be recognized.

### Software Configuration:

#### 1) Copy the repository to the local file system of your computer.
```
cd ~
git clone --recursive https://github.com/iot2tangle/Bosch-CISS.git
cd Bosch-CISS/http-sender
```
#### 2) Edit the file *CISS_Configuration.ini*
You must locate the port where the device was mounted on your computer, and put the path on *line 11* of the  *CISS_Configuration.ini* file.
In *Windows* it will be something similar to ```port = COM4```, while in Linux ```port = /dev/ttyACM0 ```.

#### 2) Edit the file *config.json*

Edit the **config.json** file to define the values for your configuration: The *device_name*, the *http_address* (with the port) that will have the *I2T Streams HTTP Gateway* running, and the *interval* in seconds between the data collection. 

The *Device Id* you define here must be between the devices you set in on the *Streams Gateway configuration file*.

```
{
    "device_name": "CISS-Bosch-HTTP",
    "HTTP-address": "http://192.168.1.115:8080/sensor_data",
    "interval": 30
}
```
#### 3) Run:
```
python3 CISS.py
```


If the *I2T Streams HTTP Gateway* is configured correctly (we will explain this next), ***you will be sending data to Tangle via Streams***. 

The following capture shows a ***CISS Bosch*** sending data to Tangle:

![CISS sending data to the Tangle](https://i.postimg.cc/8c1C1sFv/CISS-sending.png)
	
# Setting up the Streams HTTP Gateway

## Preparation

Install Rust if you don't have it already. More info about Rust here https://www.rust-lang.org/tools/install

```
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Make sure you also have the build dependencies installed, if not run:  

```
sudo apt update
sudo apt install build-essential pkg-config libssl-dev
```

## Installing the Streams Gateway
Get the Streams Gateway repository
https://github.com/iot2tangle/Streams-http-gateway

```
git clone https://github.com/iot2tangle/Streams-http-gateway
```

Navigate to the root of **Streams-http-gateway** directory and edit the **config.json** file to define yours *device names*, *endpoint*, *port*, you can also change the IOTA Full Node used, among others.

## Start the Streams Server

### Sending messages to the Tangle

Run the Streams Gateway:

```
cargo run --release  
```

This will compile and start the *Streams HTTP Gateway*. Note that the compilation process may take from 3 to 25 minutes (Pi3 took us around 15/25 mins, Pi4 8 mins and VPS or desktop machines will generally compile under the 5 mins) depending on the device you are using as host.
You will only go through the compilation process once and any restart done later will take a few seconds to have the Gateway working.

Once started, the ***Channel Id*** will be displayed, and the gateway will be open waiting for data to send to the Tangle.
