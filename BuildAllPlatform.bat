@echo off

SET CURDIR=%~dp0

echo Building for platform: Editor
msbuild vFrame.Lockstep.Core.sln /t:Clean,Rebuild /p:Configuration=Debug /p:Platform="Any CPU"
if not exist "%CURDIR%\Output\vFrame.Lockstep.Core\Runtime" mkdir %CURDIR%\Output\vFrame.Lockstep.Core\Runtime
copy /Y %CURDIR%\Build\vFrame.Lockstep.Core\Debug\vFrame.Lockstep.Core.* %CURDIR%\Output\vFrame.Lockstep.Core\Runtime\

echo Building for platform: Standalone
msbuild vFrame.Lockstep.Core.sln /t:Clean,Rebuild /p:Configuration=Release /p:Platform="Any CPU" /p:DefineConstants="TRACE UNITY_STANDALONE"
if not exist "%CURDIR%\Output\vFrame.Lockstep.Core\Runtime\Standalone" mkdir %CURDIR%\Output\vFrame.Lockstep.Core\Runtime\Standalone
copy /Y %CURDIR%\Build\vFrame.Lockstep.Core\Release\vFrame.Lockstep.Core.* %CURDIR%\Output\vFrame.Lockstep.Core\Runtime\Standalone\

echo Building for platform: Android
msbuild vFrame.Lockstep.Core.sln /t:Clean,Rebuild /p:Configuration=Release /p:Platform="Any CPU" /p:DefineConstants="TRACE UNITY_ANDROID"
if not exist "%CURDIR%\Output\vFrame.Lockstep.Core\Runtime\Android" mkdir %CURDIR%\Output\vFrame.Lockstep.Core\Runtime\Android
copy /Y %CURDIR%\Build\vFrame.Lockstep.Core\Release\vFrame.Lockstep.Core.* %CURDIR%\Output\vFrame.Lockstep.Core\Runtime\Android\

echo Building for platform: iOS
msbuild vFrame.Lockstep.Core.sln /t:Clean,Rebuild /p:Configuration=Release /p:Platform="Any CPU" /p:DefineConstants="TRACE UNITY_IOS"
if not exist "%CURDIR%\Output\vFrame.Lockstep.Core\Runtime\iOS" mkdir %CURDIR%\Output\vFrame.Lockstep.Core\Runtime\iOS
copy /Y %CURDIR%\Build\vFrame.Lockstep.Core\Release\vFrame.Lockstep.Core.* %CURDIR%\Output\vFrame.Lockstep.Core\Runtime\iOS\

Pause
