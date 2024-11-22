
# mprocs Installation and Configuration Guide on WSL

## Introduction

This guide will help you install, configure, and run `mprocs` on Windows Subsystem for Linux (WSL). `mprocs` is a powerful process manager that allows you to run and manage multiple commands in parallel.

## Prerequisites

Make sure you have the following installed on your WSL environment:
- **Git** for cloning the repository
- **Rust** and **Cargo** for building `mprocs`
- To be inside **wsl**:
```bash
   wsl -d (your VM)
   su fourdds
```

### Install Git

To check if Git is installed, run:
```bash
git --version
```
If Git is not installed, you can install it with:
```bash
sudo apt update
sudo apt install git
```

### Install Rust and Cargo

To check if Rust is installed, run:
```bash
rustc --version
```
If Rust is not installed, install it by running:
```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env
```

## Step 1: Clone the `mprocs` repository

Run the following commands to clone the `mprocs` repository and navigate to the project folder:
```bash
git clone https://github.com/pvolok/mprocs.git
cd mprocs
```

## Step 2: Build `mprocs`

Since `mprocs` is a workspace, navigate to the source directory and build the project:
```bash
cd src
cargo build --release
```

This will create the `mprocs` binary in the `target/release` directory.

## Step 3: Run `mprocs`

To run `mprocs`, use the following command:
```bash
../target/release/mprocs
```

## Step 4: Create the `mprocs.yaml` configuration file

To customize `mprocs`, create a configuration file in the `.config` directory:

1. Create the configuration directory:
   ```bash
   mkdir -p ~/.config/mprocs
   ```

2. Create the `mprocs.yaml` file:
   ```bash
   gedit ~/.config/mprocs/mprocs.yaml
   ```

3. Add this configuration:
   ```yaml
```yaml
procs:
  unified_window:
    shell: |
      bash -c "
      echo 'Apertura mprocs.yaml';
      vi ~/.config/mprocs/mprocs.yaml;

      echo 'Esecuzione PX4 Update';
      cd PX4-Autopilot && make px4_sitl;

      echo 'Esecuzione ROS2 Update';
      cd ~/ros2_ws && source setup.bash && rm -rf log_* && colcon build;

      echo 'Esecuzione ROS WS Update';
      cd ~/ws && source setup.bash && colcon build --symlink-install;

      echo 'Esecuzione MicroXRCEAgent';
      cd && MicroXRCEAgent udp4 -p 8888;

      echo 'Esecuzione PX4-SITL';
      cd ~/ws && source setup.bash && cd && ./PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n 3 -m iris -t 2 -w find_targets;

      echo 'Esecuzione ROS2';
      cd ~/ros2_ws && source setup.bash && bash run.sh -n 3 -m iris;

      echo 'Avvio log recorder';
      cd ~/ros2_ws && source setup.bash && bash register.bash;

      echo 'Conversione dei log in CSV';
      cd ~/ros2_ws && source setup.bash && ros2 run analysis ConvertToCSV $(ls log_*/log_*_0.db3) && rm -rf log_*;
      "
    autostart: false

   ```

4. Save and close the file (`Ctrl + O`, then `Enter`, and `Ctrl + X` to exit).

## Step 5: Create an alias to run `mprocs`

To simplify the execution of `mprocs`, add an alias to your `.bashrc` file:

1. Open the `.bashrc` file for editing:
   ```bash
   nano ~/.bashrc
   ```

2. Add this line to create an alias:
   ```bash
   alias mprocs='/home/fourdds/mprocs/target/release/mprocs --config ~/.config/mprocs/mprocs.yaml'
   ```

3. Save and exit the file (`Ctrl + O`, then `Enter`, and `Ctrl + X` to exit).

4. Reload the `.bashrc` file to apply the changes:
   ```bash
   source ~/.bashrc
   ```

5. Try:
   ```bash
      mprocs
   ```

## Referancials
Here you can find mprocs documentation
 ```bash
   https://github.com/pvolok/mprocs
 ```


