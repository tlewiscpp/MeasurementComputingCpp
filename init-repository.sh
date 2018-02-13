#!/bin/bash -e

if [[ ! -f ".repo-init" ]]; then
    git submodule update --init --recursive
    touch .repo-init
else
    git submodule update --recursive --remote
fi

