<h1 align="center">
  <a href="https://www.youtube.com/@eccentric_engineer">
	<img
		width="200"
		alt="Avinashee Tech"
		src="img/Avinashee Tech Logo New.png">
  </a>  
</h1>

<h3 align="center">
	DIY Custom BLE KeyBoard using nRF52840 controller 
</h3>




  
## 📝 Overview

This project is a Bluetooth Low Energy (BLE) HID Keyboard built using the Nordic nRF52 series (tested on nRF52840 Dongle). 
It enables wireless communication with computers, smartphones, or any BLE-compatible host by emulating a standard HID keyboard.

Whether you're building a custom input device, a wireless macro pad, or experimenting with BLE HID profiles, this firmware provides
a lightweight, modular foundation to get started quickly.

Platform used for firmware development is nRF Connect SDK on VSCode.
Learn more 👇👇  
  
[![BLE KeyBoard Youtube Video](nrf52 ble kb thumbnail.png)](https://youtu.be/0vzwbgLEY-s?si=kmnJE1kYjhTBlR5s)

## ✔️ Requirements

### 📦 Hardware
- nRF52840 Dongle (main controller  board)
- USB Female to Male Cable (optional, but recommended)
- Tactile button with cap  (to execute arrow key functions - Top, Bottom, Left and Right) 

### 📂 Software
- VSCode (https://code.visualstudio.com/)  
- nRF Connect SDK (https://www.nordicsemi.com/Products/Development-software/nRF-Connect-SDK)
- nRF Connect for Desktop (https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-Desktop)

## 🛠️ Installation and usage

```sh
git clone https://github.com/AvinasheeTech/nRF52-BLE-HID-Keyboard.git
Open project in VSCode
Add Build Configuration -> Select nrf52840dongle_nrf52840 as board target -> Generate and Build
Open nRF Connect For Desktop App
Next Select Programmer -> Put Board in Bootloader mode -> Select Device -> Upload zephyr.hex output file from Build Directory of Project
Turn on Bluetooth on PC/Mobile Device -> Connect with Avinashee Tech Device -> Check for Connected Status with 100% Battery 
Enjoy...🍹
```
To learn more about how to upload code to nRF52840 Dongle using VSCode and nRF Connect SDK, click link below 👇👇  

[![nRF5240 Dongle Youtube Video](nrf52840 thumbnail.png)](https://youtu.be/TeBvb645NZA?si=z5goAc1ic0ipf2cX)


## ⭐️ Show Your Support

If you find this helpful or interesting, please consider giving us a star on GitHub. Your support helps promote the project and lets others know that it's worth checking out. 

Thank you for your support! 🌟

[![Star this project](https://img.shields.io/github/stars/AvinasheeTech/nRF52-BLE-HID-Keyboard?style=social)](https://github.com/AvinasheeTech/nRF52-BLE-HID-Keyboard/stargazers)
