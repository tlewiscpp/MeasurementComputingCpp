#!/bin/bash

SOURCE_DIR='libusb'

git clone https://github.com/libusb/$SOURCE_DIR || { cd $SOURCE_DIR; git pull origin master; }

