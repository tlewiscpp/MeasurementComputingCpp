
#!/bin/bash -e

##########################################
# install-measurementcomputingcpp.sh
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

function displayHelp() {
    echo "Usage: install-$programName.sh [--install/--uninstall] [build-dir]"
}

function doInternetCheck() {

    echo -n "Checking for Cygwin..."
    cygwinCheck=$(uname -a | grep -i 'cygwin')
    showSuccess

    echo -n "Checking for internet connection..."
    if [[ -z "$cygwinCheck" ]]; then
        internetCheck=$(ping 8.8.8.8 -n1 | grep -i 'received = 1')
    else
        internetCheck=$(ping 8.8.8.8 -c1 -W1 | grep -i '1 received')
    fi

    if [[ -z "$internetCheck" ]]; then
        showFailure
        return 1
    else 
        showSuccess
        return 0
    fi
}

function doCygwinCheck() {
    echo -n "Checking for Cygwin..."
    cygwinCheck=$(uname -a | grep -i 'cygwin')
    msysCheck=$(uname -a | grep -i 'msys')
    showSuccess

    echo -n "Setting sudo to correct value..."
    if [[ -z "$cygwinCheck" && -z "$msysCheck" ]]; then
        SUDO="sudo"
    else
        SUDO=""
    fi
    showSuccess
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

buildDir="build"
appDir="$HOME/.local/share/applications/"

if [[ "$EUID" -eq "0" ]]; then
    SUDO=
else
    SUDO=sudo
fi

trap bailout INT QUIT TERM

declare -i incrementPatchVersion
declare -i incrementMinorVersion
declare -i incrementMajorVersion

incrementPatchVersion=0
incrementMinorVersion=0
incrementMajorVersion=0
buildAndroid=0
verboseOutput=0

declare -i loopCounter
loopCounter=0
for var in "$@"; do
    if [[ "$var" == "-r" || "$var" == "--r" || "$var" == "-release" || "$var" == "--release" ]]; then
        buildType="Release"
    elif [[ "$var" == "-d" || "$var" == "--d" || "$var" == "-debug" || "$var" == "--debug" ]]; then
        buildType="Debug"
    elif [[ "$var" == "-p" || "$var" == "--p" || "$var" == "-patch" || "$var" == "--patch" || "$var" == "-increment-patch" || "$var" == "--increment-patch" ]]; then
        incrementPatchVersion=1
    elif [[ "$var" == "-i" || "$var" == "--i" || "$var" == "-minor" || "$var" == "--minor" || "$var" == "-increment-minor" || "$var" == "--increment-minor" ]]; then
        incrementMinorVersion=1
    elif [[ "$var" == "-m" || "$var" == "--m" || "$var" == "-major" || "$var" == "--major" || "$var" == "-increment-major" || "$var" == "--increment-major" ]]; then
        incrementMajorVersion=1
    elif [[ "$var" == "-a" || "$var" == "--a" || "$var" == "-android" || "$var" == "--android" ]]; then
        buildAndroid=1
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

checkLibUdev || { echo "Unable to link libudev, bailing out"; exit 1; }
changeDirectory "$buildDir" || { echo "Unable to enter build directory \"$buildDir\", bailing out"; exit 1; }
runCmake "$filePath" || { echo "cmake failed, bailing out"; exit 1; }
runMake || { echo "make failed, bailing out"; exit 1; }
suCopyFile "$buildDir/$programName/lib$programName.so" "$globalLibDir"  || { echo "Could not copy file, bailing out"; exit 1; }

suCopyFile "$buildDir/$programName/lib$programName.so" "$globalLibDir"  || { echo "Could not copy file, bailing out"; exit 1; }

suCreateDirectory "$globalIncludeDir/$programName"
$SUDO cp -R "$filePath/$programName/*.hpp" "$globalIncludeDir/$programName/"

createDirectory "$HOME/Desktop" || { echo "Could not create directory \"$HOME/Desktop\", bailing out"; exit 1; }

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
