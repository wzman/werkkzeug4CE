:: This script copy al different release of werkkzeug4 and player binaries into data/bin

@echo off

::-------------- editor --------------

echo Copy Wz4 release dx9 x32 :
COPY /Y wz4\werkkzeug4\release_dx9_Win32\werkkzeug4_dx9_x32.exe data\bin\werkkzeug4_dx9_x32.exe

echo Copy Wz4 release dx9 x64 :
COPY /Y wz4\werkkzeug4\release_dx9_x64\werkkzeug4.exe data\bin\werkkzeug4_dx9_x64.exe

echo Copy Wz4 release dx11 x64 :
COPY /Y wz4\werkkzeug4\release_dx11_x64\werkkzeug4_dx11_x64.exe data\bin\werkkzeug4_dx11_x64.exe

::-------------- player --------------

echo Copy Player release dx9 x32 :
COPY /Y wz4\wz4player\release_dx9_Win32\wz4player_dx9_x32.exe data\bin\wz4player_dx9_x32.exe

echo Copy Player release dx9 x64 :
COPY /Y wz4\wz4player\release_dx9_x64\wz4player_dx9_x64.exe data\bin\wz4player_dx9_x64.exe

echo Copy Player release dx11 x64 :
COPY /Y wz4\wz4player\release_dx11_x64\wz4player_dx11_x64.exe data\bin\wz4player_dx11_x64.exe

pause