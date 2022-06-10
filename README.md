# DIY-Spider-Robot
Spider robot made with esp-32, servo motors and 3D printing

![image](https://user-images.githubusercontent.com/47931157/173132883-9d6713b0-3048-493d-87ef-1cbfaefe7c9e.png)

## Setup
1. Equipment you will be needed:

    * ESP-32 micro-controller
    * 8 mini servo motors with GPIO cables
    * Small lithuim battery
    * DIY spider legs (NOTE- I'll attach the objects files soon)
    * Small philips screws and a screwdriver
2. Download my code:

  ```
  git clone https://github.com/DanielKirshner/DIY-Spider-Robot
  ```
3. Install Arduino IDE software from here:  

   https://www.arduino.cc/en/software
   
5. Open my `firmware_source.ini` file in your IDE and connect the ESP-32 to your computer

7. Burn my code to your micro-controller by pressing the upload button -> ![image](https://user-images.githubusercontent.com/47931157/173133145-825896b1-a957-4fe7-a7b6-b5084fb52f8d.png)


## Usage
1. Turn on the spider robot
2. Open your wifi settings and search for the next wifi network:
  ```
  SSID: Robot-XXX
  Password: 12345678
  ```
5. Open your browser and enter the url:
  ```
  http://192.168.4.1/
  ```
6. You'll be enter the control interface, and you can control the spider robot

![image](https://user-images.githubusercontent.com/47931157/173129521-a7eea656-01c1-4cba-a4f1-923844121e91.png)
