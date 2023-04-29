# Edge_detection_robot
embedded project with RT cadmium

the project is developed on mbed os so mnake sure you have mbed os exmple and toolset running in your pc.

git pull https://github.com/JoshiRitvik/Edge_detection_robot.git

then go to the top model inside the repo, 
you can use the command cd top_model

then connect the hardware to the PC. 
open makefile and change the name of the controller with the na,e of hardware that you are using. (confirm that the hadrware is supported by mbed)

for testing PC simulation enter command
make all

You can see the the results in the form of csv inside the repo.

now deletate the .o file using command
make clean

to compile the hardware apis and flash the simulation enter the command
make embedded

the program will be flashed on the board. Now you can test the simulation. 
