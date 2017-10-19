
#!/bin/bash

##########################################
# uninstall-measurementcomputingcpp.sh
#
# Intended to be a placeholder until 
# I learn how to add a sudo make 
# install/uninstall to CMake
#
##########################################

baseName="MeasurementComputingCpp"
programLongName="MeasurementComputingCpp"
programName="MeasurementComputingCpp"
iconName="$baseName.png"
skeletonDesktopFileName=".$baseName.desktop.skel"
desktopFileName="$baseName.desktop"

absolutePath="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
fileName="${absolutePath##*/}"
filePath="$(dirname "$absolutePath")/"
iconPath="$resourceDir/$iconName"
globalBinDir="/usr/bin/"
globalLibDir="/usr/lib/"
globalIncludeDir="/usr/include/"
buildType="Release"

function bailout() {
    rm -rf "$buildDir"
}


function cleanUp() {
    echo "All cleaned up"
}

function showSuccess() {
    echo "success"
}

function showFailure() {
    echo "failure"
    cleanUp
}

function removeFile() {
    echo -n "Removing \"$1\"..."
    rm -f "$1"
    if [[ "$?" -ne "0" ]]; then
        showFailure
        return 1
    else 
        showSuccess
        return 0
    fi
}

function removeDirectory() {
    echo -n "Removing directory \"$1\"..."
    rm -rf "$1"
    if [[ "$?" -ne "0" ]]; then
        showFailure
        return 1
    else 
        showSuccess
        return 0
    fi
}

function suRemoveDirectory() {
    echo -n "Removing directory \"$1\"..."
    $SUDO rm -rf "$1"
    if [[ "$?" -ne "0" ]]; then
        showFailure
        return 1
    else 
        showSuccess
        return 0
    fi
}

function suRemoveFile() {
    echo -n "Removing \"$1\"..."
    $SUDO rm -f "$1"
    if [[ "$?" -ne "0" ]]; then
        showFailure
        return 1
    else 
        showSuccess
        return 0
    fi
}

function linkFile() {
    echo -n "Linking \"$1\" to \"$2\"..."
    ln -s -f "$1" "$2"
    if [[ "$?" -ne "0" ]]; then
        showFailure
        return 1
    else 
        showSuccess
        return 0
    fi
}

function suLinkFile() {
    echo -n "Linking \"$1\" to \"$2\"..."
    $SUDO ln -s -f "$1" "$2"
    if [[ "$?" -ne "0" ]]; then
        showFailure
        return 1
    else 
        showSuccess
        return 0
    fi
}

function copyFile() {
    echo -n "Copying \"$1\" to \"$2\"..."
    cp -R "$1" "$2"
    if [[ "$?" -ne "0" ]]; then
        showFailure
        return 1
    else 
        showSuccess
        return 0
    fi
}

function suCopyFile() {
    echo -n "Copying \"$1\" to \"$2\"..."
    $SUDO cp -R "$1" "$2"
    if [[ "$?" -ne "0" ]]; then
        showFailure
        return 1
    else
        showSuccess
        return 0
    fi
}

function appendStringToFile() {
    echo -n "Putting string \"$1\" into file $2..."
    echo "$1" >> "$2"
    if [[ "$?" -ne "0" ]]; then
        showFailure
        return 1
    else 
        showSuccess
        return 0
    fi
}

function changeDirectory() {
    echo -n "Entering directory \"$1\"..."
    cd "$1"
    if [[ "$?" -ne "0" ]]; then
        showFailure
        return 1
    else 
        showSuccess
        return 0
    fi
}

function createDirectory() {
    echo -n "Creating directory \"$1\"..."
    mkdir -p "$1"
    if [[ "$?" -ne "0" ]]; then
        showFailure
        return 1
    else 
        showSuccess
        return 0
    fi
}
function suCreateDirectory() {
    echo -n "Creating directory \"$1\"..."
    $SUDO mkdir -p "$1"
    if [[ "$?" -ne "0" ]]; then
        showFailure
        return 1
    else 
        showSuccess
        return 0
    fi
}

function runGitClone() {
    echo -n "Cloning \"$1\" using git..."
    git clone "$1"
    if [[ "$?" -ne "0" ]]; then
        showFailure
        return 1
    else 
        showSuccess
        return 0
    fi
}

function runGitPullOrigin() {
    echo -n "Fetching source for \"$1\" using git..."
    git pull origin master
    if [[ "$?" -ne "0" ]]; then
        showFailure
        return 1
    else 
        showSuccess
        return 0
    fi
}

function runCmake() {
    echo "Running cmake (BuildType = $buildType) from source directory \"$1\""
    if [[ "$verboseOutput" == "1" ]]; then
        verboseOutputArgs="-DCMAKE_VERBOSE_MAKEFILE:BOOL=ON"
    else
        verboseOutputArgs=""
    fi
    echo -n "Running command: \"cmake $verboseOutputArgs-DCMAKE_BUILD_TYPE=$buildType \"$1\"..." 
    cmake $verboseOutputArgs -DCMAKE_BUILD_TYPE=$buildType "$1" 
    if [[ "$?" -ne "0" ]]; then
        showFailure
        return 1
    else 
        showSuccess
        return 0
    fi
}

function runQmake() {
    echo -n "Running qmake from source directory \"$1\"..."
    qmake "$1"
    if [[ "$?" -ne "0" ]]; then
        showFailure
        return 1
    else 
        showSuccess
        return 0
    fi
}

function runMake() {
    echo -n "Running make..."
    make -j2
    if [[ "$?" -ne "0" ]]; then
        showFailure
        return 1
    else 
        showSuccess
        return 0
    fi
}

function bailout() {
    rm -rf "$buildDir"
}

function generateDesktopFile() {
    copyFile "$utilityDir/$skeletonDesktopFileName" "$buildDir/$desktopFileName" || { echo "Failed to generate desktop file, bailing out"; exit 1; }
    copyFile "$iconPath" "$buildDir/" || { echo "Failed to generate desktop file, bailing out"; exit 1; }  
    appendStringToFile "Exec=$buildDir/inspection-app/$programName" "$buildDir/$desktopFileName" || { echo "Failed to generate desktop file, bailing out"; exit 1; }
    appendStringToFile "Icon=$buildDir/$iconName" "$buildDir/$desktopFileName" || { echo "Failed to generate desktop file, bailing out"; exit 1; }
}

function checkLibUdev() {
    echo -n "Checking for libudev..."
    msysResult=$(uname -a | grep -i 'msys')
    cygwinResult=$(uname -a | grep -i 'cygwin')
    if [[ ! -z "$cygwinResult" || ! -z "$msysResult" ]]; then
        showSuccess
        return 0
    fi
    arch_result=$(uname -a | grep 'x86_64')
    if [[ -z "$arch_result" ]]; then
        ARCH='i386'
    else
        ARCH='i686'
    fi
    if [[ ! -e "/usr/lib/libudev.so" ]]; then
        $SUDO ln -s -f /lib/$ARCH-linux-gnu/libudev.so.1 /usr/lib/libudev.so
    fi
    if [[ "$?" -ne "0" ]]; then
        showFailure
        return 1
    else 
        showSuccess
        return 0
    fi
}   

if [[ "$EUID" -eq "0" ]]; then
    SUDO=
else
    SUDO=sudo
fi

trap bailout INT QUIT TERM

somethingFailed=""
suRemoveDirectory "$globalIncludeDir/$programName/" || { echo "Could not remove include path headers. Remove them manually"; somethingFailed="1"; }
suRemoveFile "$globalLibDir/lib$programName.so" || { echo "Could not remove library file. Remove it manually"; somethingFailed="1"; }

if [[ -z "$somethingFailed" ]]; then
    installMessage="$programLongName Uninstalled Successfully!"
else
    installMessage="$programLongName did not uninstall successfully, manual removal is necessary. See above for problems"
fi

totalLength=${#installMessage} 

echo
for (( i=0; i<totalLength+4; i++ )); do  
   echo -n "*"
done
echo
echo "**$installMessage**"
for (( i=0; i<totalLength+4; i++ )); do  
   echo -n "*"
done
echo
echo
exit 0
