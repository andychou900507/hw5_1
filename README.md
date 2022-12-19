# hw5_1

How to set up this program?
1. Import bbcar.h and PwmIn.h to control the car, and <iostream> and uLCD_4DGL to utilize the uLCD.
2. Connect the PwmOut and PwmIn of the two servos, four QTI sensor pin, and Tx and Rx of uLCD.
3. Connect the board and then compile and run the program.
4. Press the reset button.
5. Connect the mbed board and the two servos to the power supply.
6. Put the car infront of and between the two objects, whose distance are going to be measured.
7. Before using the car to measure the distance between the two objects, we have to make the ping direct at the object at the right hand side.

What are the results?
1. After setting up the car and pressing the reset button, the car will start to turn left continuously until it scan the left edge of the objects at the right hand side.
2. The car will stop for a moment and then display the distance between the first scanned object and the car.
3. After that, the car will start to turn left again until it scan the right edge of the objects at the left hand side.
4. The car will stop again and then display the distance between the second scanned object and the car.
5. The car will then calculate and display the angle between the car to object1 and the car to object2, and finally show the distance between the two objects.
6. After many times of trial, the measured distance and the actual distance have an error about 20 %.
