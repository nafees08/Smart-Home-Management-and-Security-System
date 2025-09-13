# SENTINEL — Smart Home Management and Security

A low-cost smart home automation and security prototype built on **Raspberry Pi Pico W** with cloud integration via **Firebase** and local convenience features like **Bluetooth voice control**. The system monitors hazards (gas, fire, intrusion), enforces door access control, and automates basic utilities (fan/light) while pushing live data and notifications to a companion mobile app.

---

## 🔹 Project Overview
The system combines:
- **Safety sensing**: Gas leakage (MQ-2), fire indicators (temperature & humidity via DHT11).
- **Intrusion detection**: PIR/IR motion + ultrasonic visitor proximity.
- **Access control**: 4×4 keypad with password check, servo-based door lock, 3-attempt lockout with buzzer.
- **Utility control**: Fan & light (LEDs) via relays, switchable by Bluetooth voice commands.
- **User feedback**: OLED display shows live status; Firebase Realtime Database logs events for mobile app.
- **Notifications**: A companion mobile app (“SENTINEL”) reads from Firebase to show alarms and updates.

---

## 🔹 Hardware Components
| Component                | Purpose                                                   |
|---------------------------|-----------------------------------------------------------|
| Raspberry Pi Pico W       | Core MCU with Wi-Fi, runs main firmware                   |
| MQ-2 Gas Sensor           | Detects LPG / propane / smoke leaks                       |
| DHT11                     | Temperature and humidity sensing                          |
| PIR/IR Sensor             | Detects human motion for intrusion                        |
| Ultrasonic HC-SR04        | Visitor proximity, doorbell trigger, intruder detection   |
| 4×4 Keypad                | Password input for door access                            |
| SG90 Servo                | Drives door lock mechanism                               |
| Buzzer                    | Alerts for alarms and lockout                             |
| OLED 0.96" (SSD1306)      | Displays system status and alarms locally                 |
| Bluetooth H5 Module       | Voice/text commands from smartphone                      |
| LEDs + Relays             | Represent and switch fan and light                        |
| Firebase Realtime DB      | Cloud backend for logs and notifications                  |

**Total Budget:** ≈ **3375 BDT**  
(see Chapter 6 of the report for full breakdown)

---

## 🔹 Software & Platform
- **Firmware:** MicroPython / C SDK on Raspberry Pi Pico W.
- **Cloud:** Firebase Realtime Database (2 s update interval).
- **Mobile App:** Flutter-based companion app (“SENTINEL”) to read Firebase data and notify users.
- **Bluetooth Control:** Arduino Bluetooth App (Android) to send voice commands.
- **Testing:** Campus Wi-Fi & hotspot for latency measurements, Firebase console for logs.

---

## 🔹 How It Works
1. **Hazard Detection**  
   - MQ-2 monitors for gas/smoke → triggers buzzer + Firebase alert within ~2 s.  
   - DHT11 checks temperature >40°C or abnormal humidity → triggers alarm.  

2. **Intrusion & Visitor Monitoring**  
   - PIR detects motion.  
   - Ultrasonic checks visitor distance (<10 cm) → acts as doorbell or intruder alert.  

3. **Secure Access**  
   - User enters password on keypad.  
   - Correct → servo unlocks door (~0.4 s).  
   - Wrong password ×3 → lockout, continuous buzzer until reset.  

4. **Utility Automation**  
   - Fan/light toggled by Bluetooth voice commands.  
   - Not security-critical; convenience only.  

5. **Feedback & Cloud Sync**  
   - OLED screen shows door status, alarms, attempts left, etc.  
   - Pico W pushes JSON state to Firebase every 2 seconds.  
   - App listens for updates → instant push notifications to user (<500 ms latency observed).  

---

## 🔹 Testing & Results
- Gas alarm triggers in **≤2 s** after leak threshold crossed.  
- Servo door lock opens/closes in **0.4–0.6 s**.  
- Keypad lockout works consistently after 3 wrong attempts.  
- Bluetooth command latency **≤500 ms**.  
- Firebase updates round-trip latency: **<500 ms** on campus Wi-Fi.  
- OLED refreshes every **200 ms**, prioritizing alarms.  
- 12-hour soak test: **no resets or lockups**.  

---

## 🔹 Limitations
- MQ-2 is sensitive to multiple gases → not selective.  
- DHT11 accuracy limited (±2°C).  
- Ultrasonic unreliable on soft/absorptive targets (clothing).  
- Bluetooth depends on smartphone speech recognition quality.  
- Password access only (no cryptographic tokens or biometric yet).  

---

## 🔹 Future Improvements
- Upgrade sensors (DHT22, NDIR gas sensor).  
- Add camera-based verification.  
- Use secure token-based authentication.  
- Implement OTA updates.  
- Develop a full-featured cross-platform mobile app (Flutter).  
- Custom PCB + enclosure for durability.  
