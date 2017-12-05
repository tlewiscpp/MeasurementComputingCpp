
#!/bin/bash

##########################################
# install.sh
#
# Intended to be a placeholder until 
# I learn how to add a sudo make 
# install/uninstall to CMake
#
##########################################

baseName="MeasurementComputingCpp"
programLongName="MeasurementComputingCpp"
programName="MeasurementComputingCpp"

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

function displayHelp() {
    echo "Usage: install-$programName.sh [--install/--uninstall] [build-dir]"
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

buildDir="build"
appDir="$HOME/.ArduinoLinController/share/applications/"

if [[ "$EUID" -eq "0" ]]; then
    SUDO=
else
    SUDO=sudo
fi

trap bailout INT QUIT TERM

declare -i loopCounter
loopCounter=0
for var in "$@"; do
    if [[ "$var" == "-r" || "$var" == "--r" || "$var" == "-release" || "$var" == "--release" ]]; then
        buildType="Release"
    elif [[ "$var" == "-d" || "$var" == "--d" || "$var" == "-debug" || "$var" == "--debug" ]]; then
        buildType="Debug"
    elif [[ "$var" == "-v" || "$var" == "--v" || "$var" == "-verbose" || "$var" == "--verbose" ]]; then
        verboseOutput=1
    fi
    loopCounter=$((loopCounter+1))
done

if [[ $# -gt 0 ]]; then 
    var=""
    buildDir="$filePath/$buildDir"
    for var in "$@"; do
        if [[ $var == -* ]]; then
            continue
        fi
        buildDir="$var"
    done
else
    buildDir="$filePath/$buildDir"
fi
if ! [[ -d "$buildDir" ]]; then
    createDirectory "$buildDir" || { echo "Unable to make build directory \"$buildDir\", exiting"; exit 1; }
fi

if [[ ! -f ".init-repo" ]]; then
    source init-repository.sh
fi

changeDirectory "$buildDir" || { echo "Unable to enter build directory \"$buildDir\""; exit 1; }
runCmake "$filePath" || { echo "cmake failed"; exit 1; }
runMake || { echo "make failed"; exit 1; }
suCopyFile "$buildDir/$programName/lib$programName.so" "$globalLibDir"  || { echo "Could not copy file"; exit 1; }
suCreateDirectory "$globalIncludeDir/$programName"
for headerFile in $(ls $filePath/$programName/*.h*); do
    suCopyFile "$headerFile" "$globalIncludeDir/$programName/" || { echo "Could not copy header file to $programName directory"; exit 1; }
done

suCopyFile "$filePath/mccusb-root/Linux_Drivers/61-mcc.rules" /etc/udev/rules.d/ || { echo "Could not copy 61-mcc.rules to /etc/udev/rules.d"; exit 1; }
$SUDO /sbin/udevcontrol --reload_rules || $SUDO /sbin/udevadm control --reload-rules || $SUDO /sbin/udevadm control --reload
suCreateDirectory "$globalIncludeDir/mcc-libusb" || { echo "Unable to create mcc-libusb directory"; exit 1; }
for headerFile in $(ls $filePath/mccusb-root/Linux_Drivers/USB/mcc-libusb/*.h*); do
    suCopyFile "$headerFile" "$globalIncludeDir/mcc-libusb/" || { echo "Unable to copy header file to mccusb directory"; exit 1; }
done


installMessage="$programLongName Installed Successfully!"
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
