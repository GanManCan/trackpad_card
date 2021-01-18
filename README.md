# MNIST Business Card

This software is based on the [projet](https://github.com/mr-goldhands/ESP32_Fashion_MNIST), which you can read about i the following medium [article](https://medium.com/@dmytro.korablyov/first-steps-with-esp32-and-tensorflow-lite-for-microcontrollers-c2d8e238accf)

This project makes a PCB "Business Card" with a touchpad input that outputs the numerical ddigit input on the toucpad. using ESP32 microcontroller and tensorflow lite. 


### Hardware

**Microcontroller:**  ESP32.  Specifically, this project uses the ESP32XXXX, which is the single core version with XX MB of flash. 

I choose the ESP32 for two reasons **cost** and **tensorflow lite support**.  The ESP32 is very cheap ,$1.60 in low quanities,  and has numberous tensorflow lite examples that made it easy to learn. 
 
The most signficant drawback of the ESP32 is the power draw, esecially when trying to power the card using a coin cell.  Understandably, when operating at 160 MHz the draws around XX ma.  The ESP32 also has low power and ultra-low power (ULP) mode, which draw XX ma and XX ma, respectivly.  While the ESP32 could techinally commuicate with the I2C touchpad controller in  ULP mode, this would require writing a pretty extivsive program in assembly that is more work than I wanted to take on for this project.  As diiscussed int eh software section, this results in operating almost contiously in the full power mode.

**Touchpad Controller:**

** Schematic ** 


### ESP32 Software.

As mentioned above this project is based on [ESP32_Fashion_MNIST'](https://github.com/mr-goldhands/ESP32_Fashion_MNIST) and [tensorflow lite examples](https://www.tensorflow.org/lite/microcontrollers). 


## Python
