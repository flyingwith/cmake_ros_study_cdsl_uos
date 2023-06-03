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