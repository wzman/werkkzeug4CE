# Werkkzeug4 Community Edition

Here is a dedicated repository of the Werkkzeug4 project.

## What changes ?

- Independant fork from Farbrausch
- Only focus on the werkkzeug4 tool
- Cleaned version of unused stuff
- Many new features

## Structure

So basically there's two directories.

"Altona" contains the base framework for graphics, sound, IO and the rest of the essential stuff that's needed for building werkkzeug4.

"wz4" contains the library with the "Farbrausch engine" and all the effects, and also a GUI and player version of the werkkzeug4 that uses this library. That's where the demos come from.

## Prerequisite

- Visual Studio 2010, at least Visual Studio 2008 (Express versions will do, also VS2005 should work but hasn't been tested for a long time)
- DirectX SDK
- YASM
- PhysX SDK v3.2.3 (https://developer.nvidia.com/rdp/physx-downloads)

## PhysX configuration

Werkkzeug4 default project is preconfigured to find the PhysX SDK on **C:\library\PhysX-3.2.3_PC_SDK_Core\** folder.
If you want to change that, edit **wz4\wz4frlib\wz4_physx.hpp** and change PhysX path preprocessor variables.

Add PhysX library path to your system environment variable to be able to run Werkkzeug4 from Visual Studio :
**```C:\library\PhysX-3.2.3_PC_SDK_Core\Bin\win32```** and/or **```C:\library\PhysX-3.2.3_PC_SDK_Core\Bin\win64```**

## Compiling (the very easy way)

Use this one if your configuration is Visual Studio 10, Directx 9 and if you want to build a 32 bits version.

1. use **C:\github\werkkzeug4** as root directory

2. run **build_project.bat**. If everything goes according to plan, you should now find solution and project files in every directory.

3. Open the Visual Studio **wz4/werkkzeug4** or **wz4/wz4player** projects, compile, enjoy.


## Compiling (the easy way)

Use this one for a custom configuration.

1. Look for **altona_config.hpp** in the **altona/** dir, open it and change the constants found therein (most prominently the VS version, SDK version and WIN32 or WIN64). Make sure altona/bin is in the PATH from now on, then open a command line and type :

2. **``` makeproject -r path_to_source ```** - 
If everything goes according to plan, you should now find solution and project files in every directory.

3. Open the Visual Studio **wz4/werkkzeug4** or **wz4/wz4player** projects, compile, enjoy.


## Compiling (the hard way)

To create all the tools found in altona/bin/ from scratch you need to locate the "bootstrap" project in altona/tools/makeproject/bootstrap - this should build without any further dependencies. Create the makeproject.exe, then call it (if you got sCONFIG_CODEROOT_WINDOWS in altona_config.hpp right, you can from now on omit the -r parameter). The VS projects should now be created and you can proceed to compile at least ASC and Wz4Ops (in this order). Put all executables in the PATH and Werkkzeug should compile.