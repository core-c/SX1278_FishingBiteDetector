# SX1278 Fishing Bite Detector
This is an Arduino project using the **SX1278 transceiver** in **FSK-mode**.<br>
The board i used is a **LilyGO TTGO ESP32 433 (T3_v1.6.1 20210104)**.<br>
I used **RadioLib**, which i like a lot because it works well: https://github.com/jgromes/RadioLib<br>
The LilyGO board features an **SSD1306 OLED**: I used the AdaFruit library.<br>
(I am not yet using the board WiFi and/or BlueTooth).<br>
<br>
The SX1278 is used for receiving a fishing bite detector signal.<br>
The bite-detector is from the brand Ultimate. I do not know what model it is. (it's not mine, and i didn't buy it).<br>
<br>
The radio receives the **2-FSK** signal with a center frequency of **433.96 MHz**.<br>
The bitrate is very low, **0.666 bits per second**.<br>
The sent packet-format is quite simple: **No preamble, no syncword, no CRC, just a 32-bit value**.<br>
A packet ends with an extra long pulse, followed by an extra long pause (in between multiple sent packets).<br>
The 0-bit pulse has a shorter duration than the 1-bit pulse.<br>
In fact, the signal looks a lot like a morse-code transmission.<br>
<br>
I do not use any packet-mode, nor bit-sequencer. Only the signal-strength is used to interpret the transmissions.<br>
The bite-detector has some drift around the FSK center-frequency (about -/+ 3.5KHz). To compensate for that drift, i had to use the **AFC (auto frequency correction)**.<br>
<br>
<br>
It is possible to send short text messages to the board via the **serial**- or **BLE**-connection.<br>
This way you can edit the radio config without flashing a new program. This is very handy when working with the device in the field, and you want to make some changes.<br>
The following strings can be used:<br>
* d Frequency deviation<br>
* w RX bandwidth<br>
* r Bitrate<br>
* s RSSI smoothing<br>
* n Noisefloor<br>
* b Turn off Bluetooth BLE

You can send something like **u-90.0** to set the noisefloor at -90.0 dBm.<br>
Or you could send **s2** to set the RSSI-smoothing to 8 samples.<br>
Turn off BLE by sending **b0**.<br>
<br>
![Bite Detector](IMG_20230525_095853.jpg "Bite detector")
![LilyGO](IMG_20230525_095848.jpg "LilyGO")
![Spectrum](signal_spectrum.png "Signal spectrum")
![Burst](signal_1burstofpackets.png "Signal burst of packets")
![Packet](signal_1packet.png "Signal single packet")
![Bits](signal_1bit.png "Signal bit")
[![In Action](https://img.youtube.com/vi/jb8pLU9u8rA/0.jpg)](https://www.youtube.com/watch?v=jb8pLU9u8rA)
