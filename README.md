STM32-DHT-LCD
This project is designed to interface an STM32 microcontroller with a DHT sensor (for temperature and humidity) and display the readings on an I2C-based LCD screen.

🛠 Project Overview
The firmware reads temperature and humidity data from the DHT sensor and displays the values on an LCD screen using I2C communication. It runs on an STM32F4 series microcontroller and is implemented using HAL drivers.

🔧 Hardware Requirements
To run this project, you need:

STM32F4 microcontroller (e.g., STM32F407 Discovery Board)
DHT11 or DHT22 temperature & humidity sensor
I2C 16x2 LCD module (with PCF8574 I2C adapter)
Pull-up resistors (4.7kΩ for DHT sensor data line)
Jumper wires for connections
Power supply (5V or 3.3V)
⚙ Setup and Wiring
1️⃣ Connect the DHT Sensor
DHT Pin	STM32 Pin
VCC	3.3V or 5V
GND	GND
Data	Any GPIO pin (e.g., PB5)
Use a 4.7kΩ pull-up resistor on the Data pin.
2️⃣ Connect the I2C LCD
LCD Pin	STM32 Pin
VCC	5V
GND	GND
SDA	PB9 (I2C1_SDA)
SCL	PB6 (I2C1_SCL)
💻 Software Requirements
STM32CubeIDE (or Keil, IAR Embedded Workbench)
STM32 HAL Library
DHT sensor library
I2C LCD driver (PCF8574-based)
🚀 How to Compile & Flash
1️⃣ Clone the Repository
sh
Copy
Edit
git clone https://github.com/ali-kazz/STM32-DHT-LCD.git
cd STM32-DHT-LCD
2️⃣ Open the Project in STM32CubeIDE
Open STM32CubeIDE
Click File > Open Projects from File System
Select the project folder
3️⃣ Compile the Code
Click Build Project (🔨) in STM32CubeIDE
4️⃣ Flash the Firmware
Connect the STM32 board via USB
Click Run > Debug
Select ST-LINK as the debugger
5️⃣ Observe the Output
Temperature & Humidity should appear on the LCD screen!
🛠 Troubleshooting
🔹 LCD Not Displaying Data?

Check I2C connections (SDA/SCL)
Ensure the correct I2C address (0x27 or 0x3F)
Try adjusting LCD contrast using the potentiometer
🔹 DHT Sensor Not Responding?

Ensure the correct GPIO pin is used
Verify pull-up resistor (4.7kΩ)
Use a delay of at least 1 second between readings
📜 License
This project is open-source and can be used under the MIT License.

📩 Contact
For any issues or improvements, open an issue on GitHub or contact: 📧 ali-kazz@gmail.com

