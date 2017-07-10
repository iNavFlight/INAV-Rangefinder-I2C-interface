# I2C interface for HC-SR04 sonar rangefinder

Most modern flight controllers does not allows to connect HC-SR04 rangefinders. Reason in quite simple: not enough pins for quite low priority device. 

This interface can be used to connect HC-SR04 sonar rangefinder to boards equipped with I2C bus and running latest versions of INAV. 

## Diagram

![Diagram](diagram.png)

## Notes

* HC-SR04 is 5V, while STM32 CPU are 3.3V devices and might not tolerate 5V on I2C lines. This is why, you either have to run ATtiny on 3.3V or run it on 5V too (remove inline resistors) but connect using logic level shifters
* You can remove 3.3V stabilizer and take 3.3V from flight controller too
* Latest Arduino IDE (suggested 1.8.x, should work with older)
* [ATtiny Universal](https://github.com/SpenceKonde/ATTinyCore) ATtiny core files
* IPS programmer. I use USBasp, but even Arduino as ISP can be used
* [ATtiny flashing guide](https://quadmeup.com/programming-attiny85-and-attiny45-with-arduino-ide/)
