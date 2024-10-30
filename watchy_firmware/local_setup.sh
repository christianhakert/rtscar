#!/bin/bash

thispath=$(realpath .)

git clone -b v5.2.2 --recursive https://github.com/espressif/esp-idf.git

cd $thispath/esp-idf
chmod +x install.sh
./install.sh all
# cp $thispath/esp-idf.patch $thispath/esp-idf
# git apply esp-idf.patch
. ./export.sh