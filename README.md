# StercusVadis
After the sons of bit***s at QuoVadis left us in the lurch, I've devised a solution to avoid depending on third-party clouds and prevent this from happening again. Therefore, leveraging how easy it is to have an in-house SIP system or switch providers, I've focused on a SIP-based solution for old doorbell 4+1solution for alert, bidirectional communication via sip and open door
# Hardware Components
- ESP32 Development Board
- MAX9814 Microphone Amplifier Module
- LM386 Audio Amplifier Module
- PC817 Optocoupler
- BC547 NPN Transistor
- 5V Relay Module
- 1N4148 Flyback Diode (D1)
- 3 x 1µF Capacitor (C1, C3, C4)
- 2 x 100nF Capacitor (C5, C6)
- 2x 10kΩ Resistors (R1, R2)
- 1kΩ Resistor (R3)
# Schematic
## Microphone Input Circuit (Intercom to ESP32):
Wire 2 (Intercom Mic) --- C1: 1µF --- MICIN (MAX9814)<br/>
Wire 3 (Intercom GND) ------------------- GND (MAX9814) --- GND (ESP32)<br/>
3.3V (ESP32) --- C5: 100nF --- VDD (MAX9814)<br/>
GND (ESP32) ----------------------------- GND (C5)<br/>
OUT (MAX9814) --- C2: 1µF --- GPIO36 (ESP32 ADC)<br/>
GAIN (MAX9814) --- 3.3V// 40 dB gain<br/>
## Audio Output Circuit (ESP32 to Intercom):
GPIO25 (ESP32 DAC) --- C3: 1µF --- IN+ (LM386)<br/>
GND (ESP32) ---------------------------- GND (LM386)<br/>
3.3V/5V (Power) --- C6: 100nF --- VCC (LM386)<br/>
GND (ESP32) ---------------------------- GND (C6)<br/>
OUT (LM386) --- C4: 1µF --- Wire 1 (Intercom Audio)<br/>
GND (LM386) ---------------------------- Wire 3 (Intercom GND)<br/>
## Call Detection Circuit:
Wire 4 (Call Signal) --- R1: 10kΩ --- LED+ (PC817 Optocoupler)<br/>
Wire 3 (Intercom GND) ------------------- LED- (PC817)<br/>
3.3V (ESP32) --- R2: 10kΩ --- Collector (PC817) --- GPIO4 (ESP32)<br/>
GND (ESP32) ----------------------------- Emitter (PC817)<br/>
## Door Opening Circuit:
GPIO5 (ESP32) --- R3: 1kΩ --- Base (BC547 NPN)<br/>
GND (ESP32) ----------------------------- Emitter (BC547)<br/>
Collector (BC547) --- Relay Coil --- D1: 1N4148 (Flyback) --- 5V (External)<br/>
GND (ESP32) ----------------------------- GND (5V Source)<br/>
Relay NO --- Wire 5 (Open Signal)<br/>
Relay COM --- Wire 3 (Intercom GND)<br/>
# Software/INO code
```
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncUDP.h>
#include <driver/adc.h>
#include <driver/dac.h>

// Wi-Fi Configuration
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

// Asterisk SIP Configuration
const char* asterisk_ip = "192.168.1.X"; // or www.your-isp-provider.com
const int sip_port = 5060;
const char* sip_user = "doorphone";
const char* sip_pass = "password";
const char* sip_extension = "100";  // Extension to call

// Pin Definitions
#define CALL_PIN 4     // Optocoupler input (call detection)
#define OPEN_PIN 5     // Relay control (door open)
#define ADC_PIN ADC1_CHANNEL_0  // GPIO36 (mic input)
#define DAC_PIN DAC_CHANNEL_1   // GPIO25 (audio output)

// Audio Settings
#define SAMPLE_RATE 8000  // 8 kHz for VoIP
#define RTP_PORT 5004     // RTP audio port

// Web Server & UDP
AsyncWebServer server(80);
AsyncUDP udp;

// Global Variables
bool call_active = false;    // Call status flag
String sip_call_id = "";     // Unique SIP call ID

// Function Prototypes
void setupWiFi();
void setupAudio();
void setupSIP();
void sendSIPRegister();
void sendSIPInvite();
void handleRTP();
void openDoor();
String generateCallID();

void setup() {
  Serial.begin(115200);

  // Pin Configuration
  pinMode(CALL_PIN, INPUT_PULLUP);  // Call detection (active-low)
  pinMode(OPEN_PIN, OUTPUT);        // Door control relay
  digitalWrite(OPEN_PIN, LOW);      // Ensure relay is OFF

  // Initialize Wi-Fi
  setupWiFi();

  // Initialize Audio Hardware
  setupAudio();

  // Web Server Endpoints
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<html><body>";
    html += "<h1>Doorphone System</h1>";
    html += "<form action='/open' method='POST'>";
    html += "<button type='submit'>Open Door</button>";
    html += "</form></body></html>";
    request->send(200, "text/html", html);
  });

  server.on("/open", HTTP_POST, [](AsyncWebServerRequest *request){
    openDoor();             // Trigger door open
    request->redirect("/"); // Redirect to homepage
  });

  server.begin();  // Start web server

  // Initialize SIP Communication
  setupSIP();
}

void loop() {
  // Detect Call (Optocoupler active-low)
  static bool last_call_state = HIGH;
  bool current_call_state = digitalRead(CALL_PIN);

  if (last_call_state == HIGH && current_call_state == LOW) {
    Serial.println("Call detected!");
    if (!call_active) {
      sendSIPInvite();  // Initiate SIP call
      call_active = true;
    }
    delay(500);  // Debounce
  }
  last_call_state = current_call_state;

  // Handle RTP Audio if call is active
  if (call_active) {
    handleRTP();
  }

  // Send periodic SIP REGISTER (every 30s)
  static unsigned long last_register = 0;
  if (millis() - last_register > 30000) {
    sendSIPRegister();
    last_register = millis();
  }
}

// Configure Wi-Fi Connection
void setupWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
}

// Initialize Audio Hardware (ADC/DAC)
void setupAudio() {
  // Configure ADC for mic input
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC_PIN, ADC_ATTEN_DB_11);

  // Enable DAC for audio output
  dac_output_enable(DAC_PIN);
}

// Configure SIP/UDP 
void setupSIP() {
  if (udp.listen(RTP_PORT)) {
    udp.onPacket([](AsyncUDPPacket packet) {
      // Process incoming RTP audio packets
      if (call_active) {
        uint8_t* data = packet.data();
        size_t len = packet.length();
        // Send audio to DAC (skip 12-byte RTP header)
        for (size_t i = 12; i < len; i++) {
          dac_output_voltage(DAC_PIN, data[i]);
        }
      }
    });
  }
}

// Send SIP REGISTER to Asterisk
void sendSIPRegister() {
  String register_msg = "REGISTER sip:" + String(asterisk_ip) + " SIP/2.0\r\n";
  // ... (full SIP message construction)
  udp.writeTo((uint8_t*)register_msg.c_str(), register_msg.length(), IPAddress(192, 168, 1, 100), sip_port);
  Serial.println("Sent REGISTER");
}

// Initiate SIP Call
void sendSIPInvite() {
  sip_call_id = generateCallID();
  String invite_msg = "INVITE sip:" + String(sip_extension) + "@" + String(asterisk_ip) + " SIP/2.0\r\n";
  // ... (full SIP message construction)
  udp.writeTo((uint8_t*)invite_msg.c_str(), invite_msg.length(), IPAddress(192, 168, 1, 100), sip_port);
  Serial.println("Sent INVITE");
}

// Handle RTP Audio Transmission
void handleRTP() {
  static uint8_t rtp_buffer[160];   // 20ms buffer (8000Hz / 50 packets/sec)
  static int rtp_index = 0;
  static uint16_t seq = 0;          // RTP sequence counter
  static uint32_t timestamp = 0;    // RTP timestamp
  static unsigned long last_sample = 0;

  // Sample audio at 8kHz
  if (micros() - last_sample >= 125) {  // 1/8000 sec = 125µs
    int16_t sample = adc1_get_raw(ADC_PIN);     // Read ADC
    uint8_t ulaw_sample = linear2ulaw(sample);  // Convert to µ-law
    rtp_buffer[rtp_index++] = ulaw_sample;      // Buffer sample
    last_sample = micros();
  }

  // Send packet when buffer full (160 samples = 20ms)
  if (rtp_index >= 160) {
    uint8_t packet[172];  // 12-byte header + 160 samples

    // Build RTP Header
    packet[0] = 0x80;  // RTP version
    packet[1] = 0x00;  // Payload type: PCMU
    packet[2] = seq >> 8;    // Sequence number (high byte)
    packet[3] = seq & 0xFF;  // Sequence number (low byte)
    // ... (timestamp and SSRC)
    memcpy(packet + 12, rtp_buffer, 160);  // Add audio data

    // Send to Asterisk
    udp.writeTo(packet, sizeof(packet), IPAddress(192, 168, 1, 100), RTP_PORT);

    // Update counters
    seq++;
    timestamp += 160;
    rtp_index = 0;
  }
}

// Trigger Door Open Relay
void openDoor() {
  digitalWrite(OPEN_PIN, HIGH);  // Activate relay
  Serial.println("Opening door...");
  delay(1000);                   // Hold for 1 second
  digitalWrite(OPEN_PIN, LOW);   // Deactivate relay
  Serial.println("Door closed");
}

// Generate Unique SIP Call ID
String generateCallID() {
  String call_id = "";
  for (int i = 0; i < 16; i++) {
    call_id += String(random(0, 16), HEX);  // Random hex digits
  }
  return call_id + "@esp32";  // Append device ID
}

// Convert 16-bit PCM to µ-law (G.711)
uint8_t linear2ulaw(int16_t pcm_val) {
  // ... (full conversion algorithm)
}
```
# ToDo
- password for html menu
- add enable/disable menu
