#!/bin/bash  
echo "Removing"
rm -r *.h
rm -r *.hpp
echo "Copying"  
cp -r  ~/qtXR1/XR1Controller/*.h ../
cp -r  ~/qtXR1/XR1Controller/*.hpp ../
cp -r  ~/qtXR1/XR1ControllerPM/*.h  ../
cp -r  ~/qtXR1/XR1ControllerPM/*.hpp  ../
cp -r  ~/qtXR1/XR1ControllerALP/*.h ../
cp -r  ~/qtXR1/XR1ControllerALP/*.hpp ../
cp -r  ~/qtXR1/XR1ControllerBLC/*.h ../
cp -r  ~/qtXR1/XR1ControllerBLC/*.hpp ../
cp -r  ~/qtXR1/XR1IMU/*.h ../
cp -r  ~/qtXR1/XR1IMU/*.hpp ../
