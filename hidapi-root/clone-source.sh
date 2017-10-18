#!/bin/bash

SOURCE_DIR='hidapi'

git clone https://github.com/signal11/$SOURCE_DIR || { cd $SOURCE_DIR; git pull origin master; }

