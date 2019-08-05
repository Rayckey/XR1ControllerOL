#!/bin/bash  
echo "Removing"
rm -r *.h
rm -r *.hpp
echo "Copying"  
cd ../XR1ControllerPM
cp -r *.h ../XR1Controller-head/
cp -r *.hpp ../XR1Controller-head/
cd ../XR1Controller
cp -r *.h ../XR1Controller-head/
cp -r *.hpp ../XR1Controller-head/
cd ../XR1ControllerBLC
cp -r *.h ../XR1Controller-head/
cp -r *.hpp ../XR1Controller-head/
cd ../XR1ControllerALP
cp -r *.h ../XR1Controller-head/
cp -r *.hpp ../XR1Controller-head/
cd ../XR1IMU
cp -r *.h ../XR1Controller-head/
cp -r *.hpp ../XR1Controller-head/
