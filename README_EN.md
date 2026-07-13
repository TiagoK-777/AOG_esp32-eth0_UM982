[📖 Versão em português](README.md)

# AgOpenGPS ESP32 Firmware (WT32-ETH01 + UM982)

This firmware is an adaptation of the AgOpenGPS precision agriculture autosteer system, originally developed for Teensy 4.1, ported to the **ESP32 WT32-ETH01**. It manages RTK GNSS data from the **UM982** receiver, processes IMU sensor fusion (BNO085), and controls the steering motor via PWM.

## 🚀 Key Features

- **Ethernet Communication:** Native UDP via LAN8720 (WT32-ETH01).
- **UM982 Support:** NMEA message processing (GGA, VTG, HPR) and dual antenna support.
- **IMU Sensor Fusion:** BNO085 support via UART-RVC.
- **Steering Control:** Non-blocking PID algorithm for DC motors (Cytron or IBT-2).
- **AgIO Configuration:** Full support for AgOpenGPS PGN protocols.

---

## 🛠️ Hardware Requirements

- **Microcontroller:** WT32-ETH01 (ESP32 with Ethernet).
- **GNSS:** Unicore Communications UM982 (RTK).
- **IMU:** BNO085 (UART-RVC mode recommended).
- **ADC:** ADS1115 (for Wheel Angle Sensor - WAS reading).
- **Motor Driver:** Cytron MD30C or IBT-2.

### Pinout (WT32-ETH01 Mapping)

| Function | ESP32 Pin | Notes |
| :--- | :--- | :--- |
| **GPS RX (Serial2)** | GPIO 5 | Connect to UM982 TX |
| **GPS TX (Serial2)** | GPIO 17 | Connect to UM982 RX |
| **IMU RX (Serial1)** | GPIO 2 | UART-RVC mode (BNO085) |
| **I2C SDA** | GPIO 33 | ADS1115 |
| **I2C SCL** | GPIO 32 | ADS1115 |
| **PWM Motor 1** | GPIO 4 | Left Direction / Cytron PWM |
| **PWM Motor 2** | GPIO 12 | Right Direction |
| **Direction (DIR)** | GPIO 14 | Enable/Direction for Cytron |
| **WAS Sensor** | ADS1115 | Via I2C (Address 0x48) |
| **CURRENT Sensor** | GPIO 35 | ACS723 |
| **Remote** | GPIO 39 | Input (Requires 10k Pull-up resistor) |
| **Steer** | GPIO 36 | Input (Requires 10k Pull-up resistor) |
| **Work** | GPIO 15 | Input (Requires 10k Pull-up resistor) |

---

## 💻 How to Build and Flash

This project uses **PlatformIO**.

1. **Install VS Code** and the **PlatformIO IDE** extension.
2. **Clone this repository** or download the files.
3. **Open the project folder** in VS Code.
4. **Dependencies:** PlatformIO will automatically download:
    - `SimpleKalmanFilter`
5. **Build Configuration:** The `platformio.ini` file is pre-configured for the `wt32-eth01` environment.
6. **Compile:** Click the "Check" icon (✔) on the PlatformIO bottom toolbar.
7. **Upload:** Connect the ESP32 via USB (using a Serial-USB adapter, since the WT32-ETH01 does not have native USB for programming) and click the right arrow (➡).

> **Note:** To enter bootloader mode on the WT32-ETH01, connect **GPIO 0 to GND** during startup.

---

## ⚙️ Firmware Configuration

Key settings can be adjusted at the top of `src/AOG_Esp32_UM982.cpp`:

- `udpPassthrough`: Defines whether GPS sends full NMEA or only KSXT.
- `makeOGI`: `true` for PAOGI messages, `false` for PANDA.
- `baseLineCheck`: Enables dual antenna fusion if using UM982 with two antennas.
- `filterRoll` / `filterHeading`: Enables Kalman filters for smoothing.

### Network Configuration (UDP)

The firmware uses default AgOpenGPS IP addresses:
- **Module IP:** Configured via AgIO (Default is typically `192.168.137.126`).
- **Destination IP (Broadcast):** `192.168.137.255`.
- **Ports:** 5120 (PAOGI), 8888 (Autosteer), 2233 (NTRIP).

---

## 🔍 Debugging

Monitor the serial output at **115200 baud**.
- If Ethernet initializes correctly, the local IP address will appear in the monitor.
- If the GPS is connected, processed NMEA messages will appear according to the configuration.

---

## 📜 License

Distributed under the GNU GPL v3 license. See the `LICENSE` file for details.

---

*Developed for precision agriculture use with AgOpenGPS.*
