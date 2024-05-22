---
title: State Collection and Relay Module Control
description: The computing and network device we use
---

## Overview
This system encompasses two major components: the Arduino and the ESP8266. The Arduino collects states from various modules, combines these states into a single byte, and sends this byte to the ESP8266. The ESP8266 then controls the relays based on the received byte. If the byte is correctly received by the ESP8266, it sends an acknowledgment back to the Arduino.

## Arduino: State Collector
### Input:
- States from different modules (for example, LEDModule, Module2, ..., Module8).
### Output:
- A single byte that represents the combined states of the different modules.
- An acknowledgment ('A') sent to the ESP8266 if the sent and received states match. (maybe need to delete later)
### Processing:
1. The collectStates() function collects states from various modules. If a module is in a certain state (e.g., turned on), the corresponding bit in the state byte is set.
2. The collected state byte is sent to the ESP8266.
3. The Arduino waits for an acknowledgment from the ESP8266. If the acknowledgment is received within a certain timeout and the sent and received states match, the Arduino sends an acknowledgment ('A') to the ESP8266. (maybe need to delete later)
### Code Overview:
- StateCollector class is responsible for collecting and sending states to the ESP8266.
- collectStates() function collects the states from various modules and combines them into a single byte.
- write_states() function handles the sending of the state byte (and waits for acknowledgment from) the ESP8266.

## ESP8266: Relay Controller
### Input:
- A byte received from the Arduino representing the states of various modules.

### Output:
- Control signals sent to the relays to turn them on/off based on the received byte.

### Processing:
1. The ESP8266 reads the received byte from the Arduino.
2. (If an acknowledgment ('A') is received,) the ESP8266 updates the relay states based on the (pending) state byte. 
3. If a byte other than 'A' is received, it is considered as the pending state byte, and the ESP8266 sends this byte back to the Arduino for acknowledgment. (maybe need to delete later)

### Code Overview:
- The setup() function initializes the relay pins as OUTPUT and sets them to LOW.
- The loop() function continuously checks for available data from the Arduino. (Based on the received data, it either updates the relay states or sets the pending state byte.)

### Communication Protocol:
7. The Arduino collects states from various modules and sends a combined state byte to the ESP8266.
8. The ESP8266 receives the state byte and sends it back to the Arduino for acknowledgment. (maybe need to delete later)
(3. The Arduino checks if the sent and received state bytes match. If they match, the Arduino sends an acknowledgment ('A') to the ESP8266.
9. Upon receiving the acknowledgment ('A')), the ESP8266 updates the relay states based on the received state byte.