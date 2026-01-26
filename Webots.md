# Webots Installation & Troubleshooting Guide (WSL2)

This guide provides steps to cleanly uninstall and reinstall Webots on Ubuntu (WSL2).

## Phase 1: Completely Uninstall Webots (DEB version)

If you installed Webots 2023b using `.deb`, follow these steps to remove all leftovers.

1. **Remove Webots package:**

   ```bash
   sudo apt remove webots -y
   ```

2. **Purge configuration files:**

   ```bash
   sudo apt purge webots -y
   ```

3. **Remove installation directory:**

   ```bash
   sudo rm -rf /usr/local/webots
   ```

4. **Remove command symlink:**

   ```bash
   sudo rm -f /usr/bin/webots
   ```

5. **Remove downloaded .deb file:**

   ```bash
   rm -f ~/webots_2023b_amd64.deb
   ```

6. **Clean dependencies:**

   ```bash
   sudo apt autoremove -y
   sudo apt autoclean
   ```

7. **Verify removal:**

   ```bash
   which webots      # Should show no output
   ls /usr/local/webots # Should say "No such file or directory"
   ```

## Phase 2: Clean Reinstallation

1. **Update system:**

   ```bash
   sudo apt update
   ```

2. **Download Webots 2023b:**

   ```bash
   wget https://github.com/cyberbotics/webots/releases/download/R2023b/webots_2023b_amd64.deb
   ```

3. **Install:**

   ```bash
   sudo apt install ./webots_2023b_amd64.deb
   ```

   > [!NOTE]
   > Ignore warnings about `libcuda.so.1` or `unsandboxed as root`; these are normal in WSL.

4. **Verify files:**

   ```bash
   ls /usr/local/webots
   # Should see: bin, lib, projects, resources
   ```

5. **Create command symlink:**

   ```bash
   sudo ln -s /usr/local/webots/webots /usr/bin/webots
   ```

6. **Run Webots:**

   ```bash
   webots
   ```

## Troubleshooting: Shell Caching Issue

If Webots doesn't open or tries to use an old Snap path:

1. **Clear Bash command cache:**

   ```bash
   hash -r
   ```

2. **Verify detection:**

   ```bash
   which webots
   # Expected: /usr/bin/webots
   ```

3. **Run again:**

   ```bash
   webots
   ```
