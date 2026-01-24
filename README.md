# lightgate
zoom


# HOW TO RUN TS  

git clone <repo>

cd <repo>


source ~/esp/esp-idf/export.sh


idf.py build

select if you're uploading to the sender or receiver with: idf.py menuconfig

Menu will look like this: 
App config
└── Device role
    ├── Sender (transmit ESP-NOW on HIGH)
    └── Receiver (listen for ESP-NOW; do not transmit on remote HIGH)

** S - to save ** 

then ur good to run, flash, build




# What's happening

+------------------------------------------------------------------+
|                           main.c                                 |
|------------------------------------------------------------------|
| Responsibilities:                                                |
|  1) Select role entrypoint (compile-time)                         |
|     - calls app_role_start()                                      |
|                                                                  |
| Build-time choice (menuconfig):                                  |
|   CONFIG_ROLE_TX  -> sender.c provides app_role_start()           |
|   CONFIG_ROLE_RX  -> receiver.c provides app_role_start()         |
+------------------------------------|-----------------------------+
                                     |
                                     | calls role entrypoint
                                     v
                +----------------------------------------------+
                |                 app_role_start()             |
                +---------------------------+------------------+
                                            |
                 +--------------------------+--------------------------+
                 |                                                     |
                 v                                                     v
+------------------------------------------------------------------+  +------------------------------------------------------------------+
|                           sender.c                               |  |                          receiver.c                             |
|------------------------------------------------------------------|  |------------------------------------------------------------------|
| Responsibilities (TX role):                                      |  | Responsibilities (RX role):                                      |
|  1) Initialize ESP-NOW (WiFi + peer + send callback)             |  |  1) Initialize ESP-NOW (WiFi + recv callback)                     |
|  2) Initialize + start sensor subsystem                          |  |  2) Setup LED GPIO + LED task                                     |
|  3) Define thresholds + state machine (LOW/MID/HIGH)             |  |  3) On receiving HIGH message -> notify LED task                  |
|  4) Decide when to send messages                                 |  |  4) Print/log received seq/type                                   |
|  5) Print/log high-level results                                 |  |                                                                  |
|                                                                  |  | Message handling:                                                 |
| Uses sensor.c like a "data provider":                            |  |  - espnow_recv_cb() parses high_msg_t                              |
|   - calls sensor_init(channels)                                  |  |  - if msg_type==1 -> LED blink                                    |
|   - calls sensor_start()                                         |  |                                                                  |
|   - repeatedly calls sensor_read_window(500ms, &w)               |  | (No sensor usage in RX role)                                      |
|     -> gets window summary: min/max/avg/count                    |  |                                                                  |
|                                                                  |  |                                                                  |
| Then:                                                            |  |                                                                  |
|   if (w.max_raw >= HIGH_THRESH) => send ESPNOW msg (HIGH)         |  |                                                                  |
|   if (w.min_raw <= LOW_THRESH)  => low action                    |  |                                                                  |
+-----------------------------|------------------------------------+  +------------------------------------------------------------------+
                              |
                              | sensor_* API calls (TX only)
                              v
+------------------------------------------------------------------+
|                            sensor.c                              |
|------------------------------------------------------------------|
| Responsibilities:                                                |
|  1) Configure ADC continuous mode                                |
|     - sample rate, bit width, attenuation, channels              |
|  2) Register ADC ISR callback (conversion done)                  |
|  3) Drain ADC driver buffer                                      |
|  4) Aggregate samples into a time window (500ms)                 |
|     - tracks: min_raw, max_raw, sum_raw, count                   |
|     - computes avg_raw                                           |
|  5) Return sensor_window_t back to sender.c                      |
|                                                                  |
| Internals:                                                       |
|  - ADC driver calls ISR -> notifies waiting task                 |
|  - sensor_read_window() blocks until window complete             |
+-----------------------------|------------------------------------+
                              |
                              | (ESP-IDF ADC driver events)
                              v
+------------------------------------------------------------------+
|               ESP-IDF ADC Continuous Driver (IDF)                |
|------------------------------------------------------------------|
| - hardware sampling + DMA                                        |
| - buffers raw ADC samples                                        |
| - triggers "conversion done" event                               |
+------------------------------------------------------------------+


 
