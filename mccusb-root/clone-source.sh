#!/bin/bash

SOURCE_DIR='Linux_Drivers'

git clone https://github.com/wjasper/$SOURCE_DIR || { cd $SOURCE_DIR; git pull origin master; }
