#!/bin/bash

CUR_DIR=$(pwd)

echo "Building for platform: Editor"
msbuild vFrame.Lockstep.Core.sln /t:Clean,Rebuild /p:Configuration=Release /p:Platform="Any CPU" /p:DefineConstants="TRACE"
mkdir -p ${CUR_DIR}/Output/vFrame.Lockstep.Core/Editor
mkdir -p ${CUR_DIR}/Output/vFrame.Lockstep.Core/Runtime
cp -f ${CUR_DIR}/Build/vFrame.Lockstep.Core/Release/vFrame.Lockstep.Core.* ${CUR_DIR}/Output/vFrame.Lockstep.Core/Runtime/
cp -f ${CUR_DIR}/Build/vFrame.Lockstep.Core/Release/vFrame.Lockstep.Editor.* ${CUR_DIR}/Output/vFrame.Lockstep.Core/Editor/

echo "Building for platform: Standalone"
msbuild vFrame.Lockstep.Core.sln /t:Clean,Rebuild /p:Configuration=Release /p:Platform="Any CPU" /p:DefineConstants="TRACE UNITY_STANDALONE"
mkdir -p ${CUR_DIR}/Output/vFrame.Lockstep.Core/Runtime/Standalone
cp -f ${CUR_DIR}/Build/vFrame.Lockstep.Core/Release/vFrame.Lockstep.Core.* ${CUR_DIR}/Output/vFrame.Lockstep.Core/Runtime/Standalone/

echo "Building for platform: Android"
msbuild vFrame.Lockstep.Core.sln /t:Clean,Rebuild /p:Configuration=Release /p:Platform="Any CPU" /p:DefineConstants="TRACE UNITY_ANDROID"
mkdir -p ${CUR_DIR}/Output/vFrame.Lockstep.Core/Runtime/Android
cp -f ${CUR_DIR}/Build/vFrame.Lockstep.Core/Release/vFrame.Lockstep.Core.* ${CUR_DIR}/Output/vFrame.Lockstep.Core/Runtime/Android/

echo "Building for platform: iOS"
msbuild vFrame.Lockstep.Core.sln /t:Clean,Rebuild /p:Configuration=Release /p:Platform="Any CPU" /p:DefineConstants="TRACE UNITY_IOS"
mkdir -p ${CUR_DIR}/Output/vFrame.Lockstep.Core/Runtime/iOS
cp -f ${CUR_DIR}/Build/vFrame.Lockstep.Core/Release/vFrame.Lockstep.Core.* ${CUR_DIR}/Output/vFrame.Lockstep.Core/Runtime/iOS/

echo "Build finished!"
echo

read -p "Press any key to continue.."
