# betaLogger - [Formula UFSC](https://www.instagram.com/formulaufsc/?hl=pt-br)

Automotive data logger for a Formula SAE car.

![Preview-Screens](https://github.com/patrickmetzner/betaLogger/blob/master/TR18.PNG) ![Preview-Screens](https://github.com/patrickmetzner/betaLogger/blob/master/betaLogger.jpeg)


# About the project

This project was developed using an **STM32 microcontroller**. It is capable of logging the following data:
* 7 analog channels [ADC], 3 axis Accelerometer [I2C], 3 axis Gyroscope [I2C] (200 Hz);
* GPS [USART] – Latitude, longitude and speed (10 Hz);
* Data logged into an SD Card [SPI];
* CAN bus will be implemented in the future.

These logging rates were chosen according to **Jorge Segers** book, **Analysis Techniques for Racecar Data Acquisition**. According to Segers, in order to **avoid aliasing** (figure below), an effect that causes different signals to become indistinguishable (or aliases of one another) when sampled, the data acquisition must be made at rates higher than the double of the highest frequencies present in each signal. Segers suggests that the minimum data acquisition rates must follow the table below, adapted from his book.

![Preview-Screens](https://github.com/patrickmetzner/betaLogger/blob/master/loggingRates.png) ![Preview-Screens](https://github.com/patrickmetzner/betaLogger/blob/master/aliasing.png)

The logged data, saved as **DLF files** can be analyzed like in the figure below.

![Preview-Screens](https://github.com/patrickmetzner/betaLogger/blob/master/dataGraphs.PNG)

When logging long track sessions, the files with the logged data can get inconveniently big for some analysis (200 lines/second). In these cases, when high precision is not needed, the **MATLAB** project **[file20hz.m](https://github.com/patrickmetzner/betaLogger/blob/master/dlfFileCreator/file20hz/file20hz.m)** can be used to **reduce by 10x** the number of lines in the DLF file. 


The **[betaLogger.pdsprj](https://github.com/patrickmetzner/betaLogger/blob/master/betaLogger.pdsprj)** contains the **CAD project** to fabricate the PCB seen in the picture below.

![Preview-Screens](https://github.com/patrickmetzner/betaLogger/blob/master/PCB.jpeg)
