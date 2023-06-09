# cmake_ros_study_cdsl_uos

CMake and ROS Study on CDSL of UOS

## Requirements

- Visual Studio Code
- Git
- CMake
- C/C++ Compiler

## Setup

- Clone the GitHub repository
  - Make a root directory
  - Open the terminal and move to the root directory
    ```
    $ cd /move/to/the/root/directory
    ```
  - Clone the repository by running the following on the terminal:
    ```
    $ git clone https://github.com/flyingwith/cmake_ros_study_cdsl_uos source
    ```
  - The directory structure should be like the below:
    ```
    root/
    |- source/
        |- .git/
        |- README.md
        |- ...
    ```
  - Move to the `source` directory and make a `dev` branch and checkout.
    ```
    $ cd source
    $ git branch dev
    $ git checkout dev
    ```
    or
    ```
    $ cd source
    $ git checkout -b dev
    ```
  - Check the current git branches by running the following:
    ```
    $ git log
    commit d2785aa895dbd7fafb354dca9e9f23391a8d8e1f (HEAD -> dev, origin/main, origin/HEAD, main)
    Author: flyingwith <87165733+flyingwith@users.noreply.github.com>
    Date:   Fri Jun 2 13:07:40 2023 +0900

        Initial commit
    ```
    - `origin` is the remote repository `cmake_ros_study_cdsl_uos`.
    - `origin/main` is the remote branch that points to the main branch in `origin`.
    - `origin/HEAD` is the remote branch that points to the HEAD in `origin`.
    - `origin/main` and `origin/HEAD` moves automatically whenever you do any network communication with `origin`. Do NOT try to change or remove both.
    - `dev` is the local branch that can be used for various developments.
    - `main` is the local branch that should be merged to `dev` when the development is completed.
    - `HEAD -> dev` shows that your current working branch is `dev`. Make all local changes on the `dev` branch, NOT the `main` branch.
  - Working with the remote git branch `origin`
    - `git fetch origin` pulls down all the data from `origin` that you don't have yet.
    - `git pull origin` automatically fetch and then merge the remote branch into your current branch.
    - `git push origin main --tag` uploads the local repository content in the `main` branch to `origin`.
      - You need the personal access token (PAT) of the remote repository to push.
- Install Mambaforge
  - [Mamba Documentation](https://mamba.readthedocs.io/en/latest/index.html)
  - Download the installer from [Mambaforge](https://github.com/conda-forge/miniforge#mambaforge).
    - Ubuntu/Mac
      - Open the terminal and run the following: 
        ```
        $ cd /move/to/the/download/directory
        $ bash <filename>.sh
        ```
      - **Caution**: Type `yes` when asking the initialization of conda.
      - Open a new terminal and check if the prompt shows the current conda environment (`base`) like below:
        ```
        (base)$
        ```
    - Some basic conda commands are:
      - Create environment
        ```
        $ conda create --name <env_name>
        ```
      - Activate environment
        ```
        $ conda activate <env_name>
        ```
      - Deactivate environment
        ```
        $ conda deactivate
        ```
      - Install a package or a program on the conda environment `<env_name>`:
        ```
        $ conda activate <env_name>
        (<env_name>)$ conda install <package or program name>
        ```
- Install ROS by using robostack
  - [robostack](https://robostack.github.io/index.html)
  - As of June 2, 2023, Robostack supports only ROS Noetic and ROS2 Humble.
  - Install ROS/ROS2 by following the instruction on [Installation ros](https://robostack.github.io/GettingStarted.html).
  - Installation Guide for Ubuntu/Mac
    ```
    $ mamba create -n ros_noetic
    $ mamba activate ros_noetic
    $ conda config --env --add channels conda-forge
    $ conda config --env --add channels robostack-staging
    $ conda config --env --remove channels defaults
    $ mamba install ros-noetic-desktop-full
    $ mamba deactivate
    $ mamba activate ros_noetic
    $ mamba install compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools
    ```
    - `ros_noetic` is the name of the conda environment.
    - `ros-noetic-desktop-full` is the ROS version to be installed.
    - For ROS Humble, use `ros_humble` and `ros-humble-desktip-full`.
- VS Code and CMake Setup
  - In vscode, install extensions `CMake` and `CMake Tools`.
  - CMake build directory location on vscode
    - By default, the build directory is made under the vscode workspace. If we need to make the build directory on the parent directory of the vscode workspace, go to Settings > Cmake: Build Directory and change the value to the next one:
      ```
      ${workspaceFolder}/../build-${workspaceFolderBasename}-${buildType}
      ```
  - In order to compile and build the C++ examples by using the vscode, do
    - VS Code > File > Open Folder... > root/source
      - `root/source` is the directory we cloned from the GitHub before.
    - VS Code > File > Save Workspace As... > Save

## CMake Study

### Example 1: print "Hello World!"

- Add two files `main.cpp` and `CMakeLists.txt` to the source directory.
- `source/main.cpp`
  ```cpp
  #include <iostream>

  int main(int argv, char* argc[])
  {
      std::cout << "Hello World!\n";
      
      return 0;
  }
  ```
- `source/CMakeLists.txt`
  ```cmake
  cmake_minimum_required(VERSION 3.19.0)

  project(cmake_ros_study)

  set(APP_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)
  set(APP_INSTALL_DIR ${APP_ROOT_DIR}/install)

  add_executable(${PROJECT_NAME} main.cpp)

  install(
      TARGETS ${PROJECT_NAME}
      DESTINATION "${APP_INSTALL_DIR}"
  )
  ```
  - The minimum cmake version shouldn't be greater than the installed cmake version.
- We will practice two ways to compile/build/install the C++ program by using CMake.
- Method 1: Command-Line Interface (CLI)
  - Open the terminal and run the following commands:
    ```
    $ cd go/to/the/root/directory
    $ mkdir build
    $ cd build
    $ cmake ../source     # It generates build/Makefile.
    $ make                # It generates the executable build/cmake_ros_study.
    $ make install        # It copies the executable to the install directory.
    ```
  - After running these commands, the directory structure should be like the below:
    ```
    root/
    |- build/
        |- Makefile
        |- cmake_ros_study
        |- ...
    |- install/
        |- cmake_ros_study
    |- source/
        |- README.md
        |- CMakeLists.txt
        |- ...
    ```
  - Check if the program works fine by running the executable `install/cmake_ros_study`.
- Method 2: Visual Studio Code
  - In vscode, open the `Command Palette` by typing Ctrl+Shift+P on Ubuntu/Windows and Cmd+Shift+P on macOS.
  - type cmake and run `CMake: Configure`.
  - On the same way, run `CMake: Build` and `CMake: Install` in the order named.
  - Check the directory structure and the executable as before.

### Example 2: build a library

- Add two files `simulation.h` and `simulation.cpp` to the source directory and modify `source/main.cpp` and `source/CMakeLists.txt`.
- `source/simulation.h`
  ```cpp
  #ifndef SIMULATION_H
  #define SIMULATION_H

  class Simulation
  {
  public:
      Simulation();
      ~Simulation();
  };

  #endif // SIMULATION_H
  ```
- `source/simulation.cpp`
  ```cpp
  #include <iostream>
  #include "simulation.h"

  Simulation::Simulation()
  {
      std::cout << "Simulation Constructor\n";
  }

  Simulation::~Simulation()
  {
      std::cout << "Simulation Destructor\n";
  }
  ```
- `source/main.cpp`
  ```cpp
  #include <iostream>
  #include "simulation.h"

  int main(int argv, char* argc[])
  {
      Simulation sim;

      return 0;
  }
  ```
- `source/CMakeLists.txt`
  ```cmake
  cmake_minimum_required(VERSION 3.19.0)

  project(cmake_ros_study)

  # ==============================================================================
  # setup cmake

  set(APP_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)
  set(APP_LIBRARY_DIR ${APP_ROOT_DIR}/library)
  set(APP_INSTALL_DIR ${APP_ROOT_DIR}/install)

  # ==============================================================================
  # library

  add_library(simulation simulation.cpp)
  install(
      FILES simulation.h
      DESTINATION "${APP_LIBRARY_DIR}/include/simulation"
  )
  install(
      TARGETS simulation
      DESTINATION "${APP_LIBRARY_DIR}/lib"
  )

  # ==============================================================================
  # executable

  add_executable(${PROJECT_NAME} main.cpp)
  target_include_directories(${PROJECT_NAME}
      PUBLIC "${APP_LIBRARY_DIR}/include/simulation"
  )
  target_link_directories(${PROJECT_NAME}
      PUBLIC "${APP_LIBRARY_DIR}/lib"
  )
  target_link_libraries(${PROJECT_NAME}
      PUBLIC simulation
  )
  install(
      TARGETS ${PROJECT_NAME}
      DESTINATION "${APP_INSTALL_DIR}"
  )
  ```
- Build and install the program by following one of the two methods introduced in Example 1.
- After the installation, the directory structure should be like below:
  ```
  root/
  |- build/
      |- Makefile
      |- cmake_ros_study
      |- ...
  |- install/
      |- cmake_ros_study
  |- library/
      |- include/
          |- simulation/
              |- simulation.h
      |- lib/
          |- libsimulation.a      # The library file name can be different for Linux and Windows.
  |- source/
      |- README.md
      |- CMakeLists.txt
      |- ...
  ```
- Test the program by running `install/cmake_ros_study`:
  ```
  $ cd go/to/root/install
  $ ./cmake_ros_study
  Simulation Constructor
  Simulation Destructor
  ```

### Example 3: add subdirectories

- Make a subdirectory `source/sim` and add two files `simulation.h` and `simulation.cpp` that are identical to Example 2.
- Add `source/sim/CMakeLists.txt` with the following contents:
  ```cmake
  add_library(simulation simulation.cpp)
  ```
- Modify the contents of `source/CMakeLists.txt` as follows:
  ```cmake
  cmake_minimum_required(VERSION 3.19.0)

  project(cmake_ros_study)

  # ==============================================================================
  # setup cmake

  set(APP_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)
  set(APP_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
  set(APP_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
  set(APP_LIBRARY_DIR ${APP_ROOT_DIR}/library)
  set(APP_INSTALL_DIR ${APP_ROOT_DIR}/install)

  # ==============================================================================
  # library

  add_subdirectory(sim)

  # include_directories(${APP_SOURCE_DIR}/sim)

  # ==============================================================================
  # executable

  add_executable(${PROJECT_NAME} main.cpp)
  target_include_directories(${PROJECT_NAME}
      PUBLIC "${APP_SOURCE_DIR}/sim"
  )
  target_link_directories(${PROJECT_NAME}
      PUBLIC "${APP_BINARY_DIR}/sim"
  )
  target_link_libraries(${PROJECT_NAME}
      PUBLIC simulation
  )
  install(
      TARGETS ${PROJECT_NAME}
      DESTINATION "${APP_INSTALL_DIR}"
  )
  ```
- Build and install the program and check the directory structure:
  ```
  root/
  |- build/
      |- sim/
          |- Makefile
          |- libsimulation.a    # The library file name can be different for Linux and Windows.
          |- ...
      |- Makefile
      |- cmake_ros_study
      |- ...
  |- install/
      |- cmake_ros_study
  |- source/
      |- sim/
          |- simulation.h
          |- simulation.cpp
      |- README.md
      |- CMakeLists.txt
      |- ...
  ```

### Example 4: add Eigen math library

- Download [Eigen library](https://eigen.tuxfamily.org/index.php?title=Main_Page) and extract to the directory `root/library/eigen-3.4.0`.