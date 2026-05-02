# lightgate

![diagram](biggest.png)


Pull down to GND


- GPIO 4, SOLO MODE 
- GPIO 5, JOINT MODE 
- GPIO 6, Calibration MODE

## pressing the same button again (or any button from an active state) returns to CLEAR **

## Solo Mode 
- 300ms debounce time 
- lap time = (time beam is broken) - (last time it was broken)

## Joint Mode 
- no dounce
- sender and receiver gates set up

## Calibration Mode 
- Sensor voltage displayed on screen sampled at 2Hz
- Use to calibrate laser 


---

## How to Run

### 1. Clone and set up the project
```bash
git clone <repo>
cd <repo>
source ~/esp/esp-idf/export.sh
idf.py build
```


### 2. Select device role
```bash
idf.py menuconfig
```
Navigate to: 

App config
* Device role
  
** Sender   (transmit ESP-NOW on HIGH)

** Receiver (listen for ESP-NOW; no transmit on remote HIGH)



### Project Architecture
High-level Flow

- main.c is the program entry point
- A single role is chosen at build time

- The selected role provides app_role_start()

- main.c calls app_role_start() to start role-specific logic

### File Responsibilities
main.c
- Role-agnostic entry point

- Calls app_role_start()

- Role selection is determined at build time:

- CONFIG_ROLE_TX → sender.c

- CONFIG_ROLE_RX → receiver.c

sender.c (TX role)
Handles sensing, classification, and transmission.

- Initialize ESP-NOW

- Wi-Fi setup

- Peer registration

- Send callback

- Initialize and start sensor subsystem

- Define thresholds and state machine (LOW / MID / HIGH)

- Decide when to transmit ESP-NOW messages

- Log system behavior

- Sensor workflow:

- - Calls:

* * sensor_init(channels)

* * sensor_start()

* * sensor_read_window(500 ms, &window)

- Decision logic:

* * If window.max_raw >= HIGH_THRESH → send ESP-NOW HIGH message

* * If window.min_raw <= LOW_THRESH → perform LOW action

receiver.c (RX role)

Handles reception and actuation.


- Initialize ESP-NOW

- Wi-Fi setup

- Receive callback

- Configure LED GPIO

- Run LED task

- Parse and log received messages

