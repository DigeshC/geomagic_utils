# Interact with AMBF Simulator using ROS Based Geomagic Devices
# Author:
### Adnan Munawar
### amunawar@wpi.edu

# Description:
This python code mimics the drivers for dVRK MTMs for use in AMBF Simulator for puzzle solving using the Geomagic, Phantom Omnis.
This is helpful if for some reason the device drivers are not working in AMBF and you cannot use two devices together. This allows
you to use one to two Geomagics. This code exploits the DVRK_ARM driver to make AMBF aware of DVRK MTMs.

The DVRK_ARM driver is located here:
https://github.com/WPI-AIM/dvrk_arm

# IMPORTANT:
Make sure that sawIntuitiveResearhKit code is not running on your computer and dvrk topics are not being published.

# Instructions:
To use this code, make sure to specify the correct commands line arguments.

There are three command line arguments

# Example Usage;

```
python mimic_mtms.py /Geomagic1/ /Geomagic2/ 500
```
The first argument is the base name of the ROS /Geomagic Device
The second argument is the base name of the second ROS /Geomagic Device
The third argument is the publishing frequency

This example assumes that your Geomagic topics names are as follows:

```
/Geomagic1/pose
/Geomagic1/buttons/
/Geomagic2/pose/
/Geomagic2/buttons
```

If for instance your Geomagic/Phantom device topic names are different, like so:

```
/OminR_pose
/OmniR_buttons/
/OmniL_pose/
/OmniL_buttons
```

then you can use the following example

```
python mimic_mtms.py /OmniR_ /OmniL_
```

Finally, you don't have to use to command line arguments. You can specify just one argument as well
and in that case only one end-effector will be shown in the AMBF Simulator
