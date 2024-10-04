
# mprocs Installation and Configuration Guide on WSL

## Introduction

This guide will help you install, configure, and run `mprocs` on Windows Subsystem for Linux (WSL). `mprocs` is a powerful process manager that allows you to run and manage multiple commands in parallel.

## Prerequisites

Make sure you have the following installed on your WSL environment:
- **Git** for cloning the repository
- **Rust** and **Cargo** for building `mprocs`

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
   nano ~/.config/mprocs/mprocs.yaml
   ```

3. Add this example configuration:
   ```yaml
   panes:
     - command: ["htop"]
     - command: ["tail", "-f", "/var/log/syslog"]
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
   alias mprocs='mprocs --config ~/.config/mprocs/mprocs.yaml'
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


