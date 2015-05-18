#pidTuner

This is a repo for a yarp module for tuning PID control gains for each individual joint of the iCub.

##Compiling
```bash
mkdir build
cd build
```
Before running `cmake` make sure to compile the `.ui` form under `/include/guiModule/`.
```bash
uic ../include/guiModule/mainwindow.ui -o ../include/guiModule/ui_mainwindow.h
```
Now `cmake` and compile...
```bash
cmake ..
make
```
