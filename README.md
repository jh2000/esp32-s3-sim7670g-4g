This project is based on USB Host RNDIS 4G Module Example for the WaveShare ESP32-S3-SIM7670G-4G V2 Module from Amazon.

Starting with the different examples from [WaveShare Wiki](https://docs.waveshare.com/ESP32-S3-SIM7670G-4G), i created an C++ project utilizing almost all module functions.

This includes:
- GPS (via SIM7670 RX/TX Serial Pins)
- RNDIS or ECM Network access via USB-CDC
- Camera with HTTP(S) Upload
- RGB-LED WS2812B
- MAX17048 I2C Battery Monitor
- OTA HTTP(S) Updates with limited crash recovery
- Config/Status via HTTP(S) from Server
- PSRAM

Additionally i added/modified this Hardware:
- AIS Receiver (GNS5851)
- ADSB Receiver (GNS5894TAC)
- DS18B20 OneWire Temperature Sensors
- I2C Bus Pullups
- Removed the 0Ohm Resistor from GPIO44/UART0 RX and bridge GPIO44 to the camera power control switch for software control.
- a OV2640 camera with 160° FOV and 75mm cable

Not used is the TFCard (the pins are in use by the adsb module)

All functions are available at the same time, not as separate examples.
The coding is not separated into Modules and there can't be disabled during build. At runtime, the different functions can be kept disabled if not used.

 Some functions were pulled together from various great contributors, special thanks to all of you:
- [AIS Decoder](https://github.com/KimBP/AIS) in AIS.cpp and AIS.h from Kim Bøndergaarg
- [NMEA Parser](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/uart/nmea0183_parser) in nmea_parser.c and nmea_parser.h from Espressif
- many dependencies from the IDF component registry
- many more snippets from around the globe, mostly heavily modified for my purposes.

## Intention

With AIS, ADSB and other things as a hobbie of myself, i already created 2 prior versions of an stand alone battery/solar powered AIS receivers, always looking on how to improve things.
Looking at this Module, it seems to have everything from my wishlist onboard (Recent Modem, ESP32, solar and battery controller, free gpio pins ...)

As always, it was a very hard start and i almost send it back, as some stated it looks like the product can't do everything which was annouced, since there are no full (open source) examples. Just a few from specific functions and some binary blobs "with the good stuff".
After many attemps, i got that SIM7670 running without tinygsm, tinyusb and other things, where i had to do any networking by myself. Now it is just pure RNDIS networking (ECM also working)

At this moment i can't tell how fast it is up or downloading, but as i began to build my "transfer image in chunks" code from scratch which was slow as hell with some 10s of seconds per image, it takes a fraction of a second now, almost making a video stream over LTE/5G possible.

So concluding, this modules is the heart to serve as control and uplink for AIS and ADS-B receiver modules.

## Usage

The unmodified module should be able to run the code. I had problems accessing the MAX17048 chip while the camera was powered off, as it seems like the pull ups are internal on the camera module. After adding separate pull up resistors, the chip works independend from the camera.
So if you have problems with the battery gauge, try to enable the on board switch for the camera.
The gsm module has a parallel software controllable switch on GPIO21, so this switch can be kept off, as all others.
The Hub switch will disconnect the usb cdc connection from esp and connect it to the onboard hub to the external usb port.
This switch is needed if you want to set up rndis/ecm mode, switch pin etc, as these functions are not in the code.

**Building the Binary**
I use the recent VSCode with ESP-IDF 6.x (5.5.x also works) and all components and header files are marked in the registry or directly in this repo. I also use Gitlab on Linux with a CD/CI Pipeline to check if the requirements are still met. This downloads a plain esp-idf 5.5x or 6.x with ESP32-S3 as target chip, git clones this repo and builds it (this downloads everything from the component registry). The attached gitlab-ci.yml requires esp-idf with esp32-s3 to be installed in the gitlab-runner user and with that in the right path, should work instantly.

So for Windows just download esp-idf, enable esp32-s3 in Installation Manager, git clone this repo somewhere and it should do the trick.
The resulting files should be uploaded via usb on the esp32 for a full clean flash. After that, you can copy the built binary anywhere (e.g. scp it to the host providing the .php endpoints) and use OTA.

**Config**
I tried to put every general relevant configuration option into the menuconfig tree (Kconfig.projbuild)
This brings the chapter "WaveShare ESP32-S3 SIM7670 4G AIS/ADSB Configuration" with all options and submenus like:
- Mode select (RNDIS/ECM)
- Pin definition for everything except camera
- Bootup defaults
- Power settings (Battery levels for various features)
- NMEA Settings (GPS)
- Server URLs

**Server side endpoints**
There are 3 URLs with the following functions:
FIRMWARE_UPG_URL is where the Firmware file is downloaded from on OTA.
SYNC_SERVER_URL is where a status json is POSTed to and the config json is read in return.
UPLOAD_SERVER_URL is where the camera image is POSTed to.
All endpoints can be used with http or https, using http for firmware needs a menuconfig allow flag and https urls need https to be enabled. The module will complain for 30 seconds on startup and keep going then (without a successfull connection).

There is some UDP remote logging to a specific IP and Port with various log levels.
AIS and ADSB UDP remote endpoints can be configured independend.
For tests you can listen for the udp output for example with 'netcat -u -l -k -p 12345'

**Bootup**
The module boots with its default settings and checks for the battery level. If it is high enought or the max17048 not responding, boot is continued. Otherwise the module deep sleeps for 60 seconds and repeats the check until the battery is filled to the set up level.

The module sets up and starts the SIM7670 Module then and creates the basic tasks.
After the set up interval, the SYNC_SERVER_URL is queried with the current status, an initial flag and the reply contains a full (whatever is set on the server) config json.
On further syncs, the full status is posted but the reply contains only a delta, so all changed config settings since last sync.
This is done to save some data usage.

**OTA**
On bootup, the firmware checks which partition to boot.
On config sync, when a different firmware hash is detected and the module is not freshly booted (initial flag not set), the module is commanded to reboot.
After that reboot, the initial flag is set on sync and since the hash is still different, the OTA flag is set and only the OTA task is started.
The module downloads the firmware then, which can take multiple tries but is always safe to power faultes etc...
After a successfull download and verification, the boot partition is set to the new one.
The module reboots and the firmware tries the new partition only for one boot.
This boot has to reach a successfull connection to the sync server to mark the partition as "successfull bootable" for further boots.
If the boot is interrupted, e.g. the server can't be reached or there is a crash before config, the bootloader falls back to the previous partition. This will cause another firmware download and flash then. So keep an eye on the log output. You have to either provide another 'fixed' firmware or disable the firmware update mechanism in the sync php endpoint to keep it on the old (but working) fw.

**Camera**
The camera is used in some kind of single shot mode. On every first use after reboot, the picture might be very dark, since it's auto illumination or whatever settings are to be adjusted. The follow up image should be good.
The image quality settings can go up to 2 or 1 for best images, but sometimes the space needed for the jpeg is not enougth then.
So play with that setting and adjust the focus for better quality. Higher resolutions show less dark/bright rows which are from interference onboard or unclean power. I thought that my camera modules were bad since i tested with low res but it's okay with high resolutions.


**Power consumption**
I tried different things and most of the power is used by the SIM7670-Module with around 1.3W.
During send/receive the power spikes to almost double.
Since my intended use is to send out all incomming ADSB/AIS packets, there is no reason for me to implement a way to reduce idle power consumption.
Datasheets say:
OV2640: 140mW with JPEG in UXGA @15FPS
GNS5851: 66mW (AIS reception)
GNS5894TAC: 365mW (ADS-B reception)
SIM7670G: 3mW "idle" (unconnected), >1300mW with lte connection established
ESP32-S3: 66mW-330mW (40MHz Idle - 240MHz busy)