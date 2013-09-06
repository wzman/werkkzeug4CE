:: This script create zip packages
:: note : need 7zip installed and path added to system environment variable

@echo off

set PATH_TO_PHYSX=C:\library\PhysX-3.2.3_PC_SDK_Core\Bin

:: dx9_x86
7z a -tzip -mx=9 Wz4CE_dx9_x86.zip werkkzeug4_dx9_x32.exe
7z a -tzip -mx=9 Wz4CE_dx9_x86.zip wz4player_dx9_x32.exe
7z a -tzip -mx=9 Wz4CE_dx9_x86.zip %PATH_TO_PHYSX%\win32\PhysX3CHECKED_x86.dll
7z a -tzip -mx=9 Wz4CE_dx9_x86.zip %PATH_TO_PHYSX%\win32\PhysX3CommonCHECKED_x86.dll
7z a -tzip -mx=9 Wz4CE_dx9_x86.zip %PATH_TO_PHYSX%\win32\PhysX3CookingCHECKED_x86.dll
7z a -tzip -mx=9 Wz4CE_dx9_x86.zip %PATH_TO_PHYSX%\win32\PhysX3CookingCHECKED_x86.dll
7z a -tzip -mx=9 Wz4CE_dx9_x86.zip changes.txt

:: dx9_x64
7z a -tzip -mx=9 Wz4CE_dx9_x64.zip werkkzeug4_dx9_x64.exe 
7z a -tzip -mx=9 Wz4CE_dx9_x64.zip wz4player_dx9_x64.exe
7z a -tzip -mx=9 Wz4CE_dx9_x64.zip %PATH_TO_PHYSX%\win64\PhysX3CHECKED_x64.dll
7z a -tzip -mx=9 Wz4CE_dx9_x64.zip %PATH_TO_PHYSX%\win64\PhysX3CommonCHECKED_x64.dll
7z a -tzip -mx=9 Wz4CE_dx9_x64.zip %PATH_TO_PHYSX%\win64\PhysX3CookingCHECKED_x64.dll
7z a -tzip -mx=9 Wz4CE_dx9_x64.zip changes.txt

:: dx11_x64
7z a -tzip -mx=9 Wz4CE_dx11_x64.zip werkkzeug4_dx11_x64.exe
7z a -tzip -mx=9 Wz4CE_dx11_x64.zip wz4player_dx11_x64.exe
7z a -tzip -mx=9 Wz4CE_dx11_x64.zip %PATH_TO_PHYSX%\win64\PhysX3CHECKED_x64.dll
7z a -tzip -mx=9 Wz4CE_dx11_x64.zip %PATH_TO_PHYSX%\win64\PhysX3CommonCHECKED_x64.dll
7z a -tzip -mx=9 Wz4CE_dx11_x64.zip %PATH_TO_PHYSX%\win64\PhysX3CookingCHECKED_x64.dll
7z a -tzip -mx=9 Wz4CE_dx11_x64.zip changes.txt


pause