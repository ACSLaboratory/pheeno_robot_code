# pheeno_robot_code

This is the ACSLab GitHub repository for the files relating to the coding of the Pheeno robot platform. Pheeno is designed to be an inexpensive (cost of parts = $320), research-level platform for multi-robot experiments, robotics education, and outreach activities. This repository will be updated with additional modules, code, and improvements to existing modules as they are developed.

Thanks for checking out the robot!

## Installation

Currently, Pheeno files can run on all major platforms (Windows, Linux, and OS X) all that is required is cloning the repository to your system.

```
git clone https://github.com/ACSLab/pheeno_robot_code.git
```

The following dependencies/programs are required to use these files:
- **[OpenCV](http://opencv.org/)**
- **[Arduino IDE](https://www.arduino.cc/en/Main/Software)**
- **[pySerial](https://github.com/pyserial/pyserial)**
- **[picam](https://www.raspberrypi.org/learning/getting-started-with-picamera/)**
    - **NOTE:** This should be a default package on the Raspberry Pi.
- **[NumPy](http://www.numpy.org/)**
    - **NOTE:** The SciPy stack is rather difficult to install on a Windows environment. [This](http://www.lfd.uci.edu/~gohlke/pythonlibs/) guide may help. Make sure to install all the dependencies for each version part of the SciPy stack!
- **[Tkinter](http://tkinter.unpythonic.net/wiki/How_to_install_Tkinter)**
    - **NOTE:** Tkinter is typically installed by default. If it is not, please refer to the link about for installation instructions.

Currently, Python 2.7+ is the only supported platform. Getting these files to work Python 3 is possible if one was able to install OpenCV on Python 3. Full Python 3 compatibility will be implemented as soon as an OpenCV Python 3 implementation is made stable.

## Getting Started

This section will give a brief introduction to installing the Arduino program on your computer, uploading a script to Pheeno, and installing the pre-made libraries onto your computer.

### Installing Arduino

If you do not have the Arduino software on your computer, first go to the [Arduino software download page](https://www.arduino.cc/en/Main/Software). Download the appropriate software for your operating system and install it!

### Installing the Pheeno Libraries

To use the pre-made libraries for Pheeno you must first download the files and place them in the right place on your computer so the Arduino software can access them. From this GitHub repository, download the `arduino` Code folder. Inside this folder should be an Arduino subfolder with folders labeled Encoder, LSM303, and Pheeno. Copy those folders to your "Documents/Arduino/libraries" folder. Now open the Arduino software. If all was done correctly, when you go to 'File --> Examples', you should see 'Encoder', 'LSM303', and 'Pheeno' at the bottom (you may have to scroll down if you have a lot of Arduino libraries already)! If this does not work or the above instructions are unclear, it is recommended to look through the [Arduino Library Guide](https://www.arduino.cc/en/Guide/Libraries/).

### Uploading Your First Script

There should be a USB cord coming out from the Pheeno robot to the Raspberry Pi. This cord allows for serial communication between the Raspberry Pi and the Arduino (and the programming of the Arduino from the Raspberry Pi but this will be explored later). This USB cord can also be used to program the Arduino Pro Mini directly from your computer. Plug the USB into your computer and allow the drivers to install. Once the drivers have installed, open the Arduino software. Go to the 'Tools' tab of the Arduino software and change the board to the 'Arduino Pro and Pro Mini' option.

Next, go the the 'Tools' tab of the Arduino software and change the port to the one that the robot is plugged into. Typically there should only be one unless you have another USB device that uses serial communication connected. NOTE: The number associated with the port may not be the same!

Now the Arduino software knows what type of board to talk to the through what port. So now you can upload any code you want! To test this all out let's upload an example script that will be explained later. Go to 'File --> Examples --> Pheeno --> RandomWalkObstacleAvoidExample'. This should load a pre-made script if you have done everything correctly. Now click the Arrow in the upper left hand corner of the Arduino software to upload it to the robot. **NOTE: THIS SCRIPT MAKES PHEENO RUN AROUND RANDOMLY! MAKE SURE YOU'RE HOLDING THE PLATFORM SO IT DOESN'T RUN OFF THE TABLE OR MAKE SURE THE ROBOT IS OFF WHILE UPLOADING!** When the Arduino software says "Done uploading" you can disconnect the robot and let it run around.
