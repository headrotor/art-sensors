
Code to read and display data from an Sensiron SCD30 CO2 sensor

This is an optical CO2 sensor sensitive to ±30 ppm CO2. Breath on it to see it respond to the CO2 you exhale!


Notes:

The sensor’s range goes up to 10,000 but I only display it up to 2000 ppm. The NDIR (“non-dispersive infrared”) sensor measures optical absorption in the 4.26 micron band of CO2. Oddly, the sensor emits visible light during a sensing period, though the IR band is not visible.

Imagine a HVAC system that not only responds to temperature, but will increase ventilation when the conference room gets measurably stuffy!

Hardware: Sensiron SCD30 ($50-60, available from Sparkfun and major distributors)
MCU/Display:  M5Stack https://m5stack.com/ with Espressif ESP32 and TFT LCD, cost ~$50
