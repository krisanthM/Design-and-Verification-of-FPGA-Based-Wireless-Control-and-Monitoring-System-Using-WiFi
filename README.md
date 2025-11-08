# ⚡Design and Verification of FPGA-Based Wireless Control and Monitoring System Using WiFi

## 🌐 Project Overview

This project designs and verifies an **FPGA-based wireless control and monitoring system** using **WiFi connectivity via UART protocol**. Implemented in **Verilog HDL**, the system enables **remote control of 8 LEDs** and **monitoring of two simulated sensor values** on the FPGA board.

Commands are received wirelessly through an **ESP8266 WiFi module** acting as a UART bridge, allowing **real-time device management**. 🛰️💡

### ✨ The design features:

* 🧠 **UART command parser** that interprets instructions like turning LEDs **ON/OFF** and requesting **sensor status**.
* 🔄 **State machine** for UART transmission managing start, data, and end bytes efficiently.
* 🧪 **Comprehensive Verilog testbench** simulating UART communication for system validation.

---

## 🔧 Hardware Components

* 💻 **FPGA development board** with at least **8 onboard LEDs**
* 📡 **ESP8266** or similar **WiFi module** (for UART-based wireless communication)
* ⚙️ **Simulated sensors** implemented in FPGA registers *(can be replaced with real sensors)*

---

## 💾 Software Components

* 🧩 **Verilog HDL source code** for FPGA logic (`design.v`)
* 🧠 **Simulation testbench** to verify UART communication and command execution (`testbench.v`)
* ⏱️ **UART communication settings:** 50 MHz clock, 115200 baud rate

---

## 🌟 Key Features

* 🔘 Control **8 individual LEDs** through wireless commands
* 📤 Real-time **sensor status transmission** upon request
* 🧩 **Robust UART RX/TX logic** integrated on FPGA
* 🗂️ **Command buffer and parser** for multi-character command handling
* ✅ **Simulation-based verification** ensures functionality before hardware deployment

---

## 🚀 Usage Instructions

1. 🔌 Connect the **FPGA** to a **WiFi-enabled UART bridge** (e.g., ESP8266).
2. ⚙️ Load the **Verilog design (`design.v`)** onto the FPGA board.
3. 🧪 Run the **simulation testbench (`testbench.v`)** to verify system operation.
4. 📲 Send UART commands wirelessly such as:

   * `ON1`, `OF1`, `ON2`, `OF2` → Control individual LEDs
   * `STATUS` → Retrieve sensor data
5. 💡 Observe **LED responses** on the FPGA and **sensor data** received via UART terminal.

---

## 🪪 License

This project is licensed under the **MIT License**. 📝
See the `LICENSE` file for more details.

---

## 📬 Contact

For questions, feedback, or collaboration opportunities, feel free to reach out:
📧 **[your-email@example.com](mailto:your-email@example.com)** 🌟

---

### 🏁 Summary

💡 A modern and verified **FPGA-based IoT system** that bridges **digital logic** and **wireless communication**, paving the way for **next-generation smart embedded systems**. ⚙️📡
