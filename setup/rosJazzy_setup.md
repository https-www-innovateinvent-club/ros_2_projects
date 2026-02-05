# ROS Jazzy Installation Guide

This guide provides step-by-step instructions for installing ROS Jazzy on WSL2 Ubuntu 24.04.

## Installation Steps

1. **Check available WSL distributions:**

   ```bash
   wsl --list --online
   ```

2. **Install Ubuntu 24.04:**

   ```bash
   wsl --install -d Ubuntu-24.04
   ```

3. **Open Ubuntu 24.04 and set up username/password:**

   ```bash
   wsl.exe -d Ubuntu-24.04
   ```

4. **Update package lists:**

   ```bash
   sudo apt update
   ```

5. **Upgrade packages:**

   ```bash
   sudo apt upgrade -y
   ```

6. **Install essential build tools and utilities:**

   ```bash
   sudo apt install -y build-essential curl wget git software-properties-common lsb-release gnupg
   ```

7. **Update package lists again:**

   ```bash
   sudo apt update
   ```

8. **Add ROS 2 GPG key:**

   ```bash
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   ```

9. **Add ROS 2 repository to sources list:**

   ```bash
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

10. **Update package lists after adding ROS repository:**

    ```bash
    sudo apt update
    ```

11. **Install ROS Jazzy Desktop:**

    ```bash
    sudo apt install ros-jazzy-desktop -y
    ```

12. **Configure environment (Add to .bashrc):**

    ```bash
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    ```

13. **Source .bashrc to apply changes:**

    ```bash
    source ~/.bashrc
    ```

## Testing the Installation

1. **Run a Talker node:**

   ```bash
   ros2 run demo_nodes_py talker
   ```

2. **Run a Listener node (in a new terminal, ensure setup.bash is sourced):**

   ```bash
   ros2 run demo_nodes_py listener
   ```

## Additional Configuration

### Set ROS Domain ID

To isolate your ROS traffic, you can set a `ROS_DOMAIN_ID`:

```bash
echo "export ROS_DOMAIN_ID=10" >> ~/.bashrc
source ~/.bashrc
```

