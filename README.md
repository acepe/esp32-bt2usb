# ESP32-S3 Bluetooth/BLE keyboard to USB adapter

Project to connect a BLE keyboard wirelessly via USB to a computer. Usefull for editing BIOS-Settings or unlocking full-disk-encryption when bluetooth is not (yet) available.

 **Developed and tested on the ESP32-S3-WROOM-1 DevKit board, other variants may not work!**

WARNING: This project is for use in ESP-32-S3 module with BLE and USB-OTG support. If you have another variant, you'll have to adapt the code. Plain ESP32 will not work, due to missing USB-OTG support.

# Missing Features

Keyboard LEDs (Caps-Lock, Num-Lock etc) are not working (the keys are working fine, just not the LEDs).
Media-Keys might not work.

I currently have no intention to fix that, because it is not needed for my usecase.

# Building and flashing

Project works as-is under Visual Studio Code (2023). You need to have the Espressif IDF extension installed and the v5.1 of the ESP-IDF SDK. Workflow is as follows:

1- Install Visual Studio Code

2- On the left, open the extensions panel

3- Search and install the Espressif IDF extension

4- Open the command prompt pressing Ctrl+Shift+P

5- Execute command "ESP-IDF: Configure ESP-IDF extension"

6- Select Express, and under "Select ESP-IDF version" choose v5.1-rc2

7- After installation, select File > Open Folder and open the ESP32-BT2USB project folder

8- Start building by pressing Ctrl+E and then B, or using the Build button in the bottom bar

9- If succesfully built, connect and flash your ESP32 board (Ctrl+E then F, or the flash button)

Refer to the following link for more instructions:

* https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/tutorial/install.md

Once succesfully built and flashed, you're ready to rock!.


# Usage and debugging

Once powered up the module scans for nearby BLE devices. If the last bonded keyboard is in range, it will try to connect to it using the keys stored on the NVS flash, so no pairing is needed for every connection. If it doesn't detect a previously bonded device, it will try to connect to the nearest keyboard in pairing mode. If both processes fail, it will wait one second and scan again until it finds anything.

BLE KEYBOARD PAIRING: Set keyboard in pairing mode and power on the board. No code entry required.

BLUETOOTH CLASSIC KEYBOARD PAIRING (code pairing):

1- Set keyboard in pairing mode and power on the board

2- Wait until you see a short blast of quick flashes, then pay attention:

3- For each code digit, the board will flash the LED the number of times equal to the digit

4- Press each digit on the keyboard as you read them (do not wait until it finishes! Some keyboards have a timeout)

5- If the digit is 0, a steady light will show for 1.5 seconds instead of the digit

6- When you see the blast of quick flashes again, press ENTER

OR you can always look at the code using the Serial monitor console, whatever you find easier, you fancy boy.

Once connected you can start using your keyboard, blue LED should be on. Connect the board to the computer or PS/2 compatible system and enjoy.

You can hot-disconnect the keyboard. The module will detect the disconnection and repeatedly try to reconnect, so it will be back online as soon as the keyboard gets up again. This is critical for keyboards that go to sleep and disconnect, or if you swap between computers using a multi-connection keyboard. 

Please note that pairing is only done after power up. If you wish to pair a new device, please reset the module with the reset button or power off and on the computer (not just reset it, because we need a power cycle). Note that if a previously paired device is on and in range, it will always connect to it first.

In case something doesn't work, you'll need to debug. 

# History
 
 * based heavily on esp32-BT2PS2 by Hambert, 2023. https://github.com/Hamberthm/esp32-bt2ps2
 * adapted to use a USB-Connection instead of PS2 by acepe, 2024
 