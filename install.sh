apt update
apt install qtbase5-dev libqt5svg-dev libqt5core5a libqt5widgets5 python3 python3-numpy python3-nmea2
cd external/QFlightinstruments/qfi
qmake qfi.pro
make
cd ..
mkdir build
cd build
cmake ..
make
make install
cd ../../../
mkdir build
cd build
cmake ..
make
make install

