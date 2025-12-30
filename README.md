# Skidpad Planner
The current skidpad planner is using python with the aid of the following libraries:
- numpy
- matplotlib

Rotated Skidpad            |  Random Starting Position
:-------------------------:|:-------------------------:
![](saved_animations\skidpad_animation.gif)  |  ![](saved_animations\skidpad_animation_random_point.gif)

### Next steps:
- Convert python code to C++
- Create a node to test new algorithm with real data

New node to test the algorithm with real time data will follow the architecture:
![](saved_animations\diagram.png)

# Install Compiler Steps:
1. Install C/C++ extension for VS code: `ms-vscode.cpptools` ![](saved_animations\cpp-for-visualstudiocode.png)
2. Install the latest installer from the MSYS2 page [link here](https://github.com/msys2/msys2-installer/releases/download/2025-08-30/msys2-x86_64-20250830.exe).
3. After install, run this command to install MinGW-w64 toolchain: `pacman -S --needed base-devel mingw-w64-ucrt-x86_64-toolchain`
   1. Press `Enter` to proceed
   2. Press `y` to proceed
4. `Edit environment variable` to point to compiller![](saved_animations\envvar.png)
5. `C/C++ Sleect IntelliSense Configuration` and then select `Use gcc.exe`

And these are the steps to setup your C++ compiller in VScode.