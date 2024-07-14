STM32 Microcontroller Hardware Connections

1. Power Supply
   - STM32 Power: 3.3V or 5V
   - ESP32 Power: 3.3V

2. UART Communication
   - STM32 USART1 TX (PA9) -> ESP32 RX
   - STM32 USART1 RX (PA10) -> ESP32 TX

3. Motors
   - Left Motor:
     - Left Motor Terminal 1 -> GPIO PB12
     - Left Motor Terminal 2 -> GND
   - Right Motor:
     - Right Motor Terminal 1 -> GPIO PB13
     - Right Motor Terminal 2 -> GND

4. Proximity Sensors
   - Left Proximity Sensor Output -> GPIO PC0
   - Mid Proximity Sensor Output -> GPIO PC1
   - Right Proximity Sensor Output -> GPIO PC2

5. User Button (optional)
   - Button (B1) -> GPIO PC13
   - Configure to pull low when pressed (external pull-up resistor or internal configuration)

6. LED Indicator (optional)
   - LED (LD2):
     - Anode -> GPIO PA5
     - Cathode -> GND (through a 220-ohm current-limiting resistor)
