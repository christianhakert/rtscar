#!/bin/bash

thispath=$(realpath .)

mkdir esp
cd esp
git clone --depth 1 --branch v5.2.2 --recursive https://github.com/espressif/esp-idf.git 

cd $thispath/esp/esp-idf/
chmod +x install.sh
./install.sh 
. ./export.sh