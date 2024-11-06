#!/bin/bash

thispath=$(realpath .)

mkdir esp
cd esp
git clone --depth 1 --branch v5.1.4 --recursive https://github.com/espressif/esp-idf.git 

mkdir watchy
cd watchy
git clone --depth 1 --branch 1.16.1 https://github.com/adafruit/Adafruit_BusIO.git
git clone --depth 1 --branch 1.11.9 https://github.com/adafruit/Adafruit-GFX-Library.git
git clone --depth 1 --branch 3.0.0-rc3 https://github.com/espressif/arduino-esp32.git arduino && \
    cd arduino && \
    git submodule update --init --recursive && cd ..
git clone --depth 1 --branch 0.2.0 https://github.com/arduino-libraries/Arduino_JSON.git
git clone --depth 1 --branch 2.0.1 https://github.com/JChristensen/DS3232RTC.git
git clone --depth 1 --branch 1.5.6 https://github.com/ZinggJM/GxEPD2.git
git clone --depth 1 --branch 3.2.1 https://github.com/arduino-libraries/NTPClient.git
git clone --depth 1 --branch 1.0.3 https://github.com/orbitalair/Rtc_Pcf8563.git
git clone --depth 1 --branch v1.6.1 https://github.com/PaulStoffregen/Time.git
git clone --depth 1 --branch v1.4.7 https://github.com/sqfmi/Watchy.git
git clone --depth 1 --branch v2.0.17 https://github.com/tzapu/WiFiManager.git

cd $thispath
cp patches/Arduino_JSON.cmake esp/watchy/Arduino_JSON/CMakeLists.txt
cp patches/DS3232RTC.cmake esp/watchy/DS3232RTC/CMakeLists.txt
cp patches/GxEPD2.cmake esp/watchy/GxEPD2/CMakeLists.txt
cp patches/NTPClient.cmake esp/watchy/NTPClient/CMakeLists.txt
cp patches/Rtc_Pcf8563.cmake esp/watchy/Rtc_Pcf8563/CMakeLists.txt
cp patches/Time.cmake esp/watchy/Time/CMakeLists.txt
cp patches/Watchy.cmake esp/watchy/Watchy/CMakeLists.txt

cp patches/espidf-patch.diff esp/esp-idf/
cp patches/watchy-patch.diff esp/watchy/Watchy/
cp patches/wifimanager-patch.diff esp/watchy/WiFiManager/

cd $thispath/esp/esp-idf/
git apply espidf-patch.diff

cd $thispath/esp/watchy/Watchy/
git apply watchy-patch.diff

cd $thispath/esp/watchy/WiFiManager/
git apply wifimanager-patch.diff

cd $thispath/esp/esp-idf/
chmod +x install.sh
./install.sh 
. ./export.sh