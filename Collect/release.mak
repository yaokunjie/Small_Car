#Generated by VisualGDB (http://visualgdb.com)
#DO NOT EDIT THIS FILE MANUALLY UNLESS YOU ABSOLUTELY NEED TO
#USE VISUALGDB PROJECT PROPERTIES DIALOG INSTEAD

BINARYDIR := Release\obj

#Toolchain
CC := E:/Compiler/Linaro/bin/arm-linux-gnueabihf-gcc.exe
CXX := E:/Compiler/Linaro/bin/arm-linux-gnueabihf-g++.exe
LD := $(CXX)
AR := E:/Compiler/Linaro/bin/arm-linux-gnueabihf-ar.exe
OBJCOPY := E:/Compiler/Linaro/bin/arm-linux-gnueabihf-objcopy.exe

#Additional flags
PREPROCESSOR_MACROS := NDEBUG=1 RELEASE=1
INCLUDE_DIRS := E:\Code_Library\include\boost_1_61_0 E:\Code_Library\include\pcduino include
LIBRARY_DIRS := E:\Code_Library\lib
LIBRARY_NAMES := arduino pthread
ADDITIONAL_LINKER_INPUTS := 
MACOS_FRAMEWORKS := 
LINUX_PACKAGES := 

CFLAGS := -ggdb -ffunction-sections -O3
CXXFLAGS := -ggdb -ffunction-sections -O3 -std=c++11 -pthread
ASFLAGS := 
LDFLAGS := -Wl,-gc-sections
COMMONFLAGS := 
LINKER_SCRIPT := 

START_GROUP := -Wl,--start-group
END_GROUP := -Wl,--end-group

#Additional options detected from testing the toolchain
USE_DEL_TO_CLEAN := 1

CP_NOT_AVAILABLE := 1
