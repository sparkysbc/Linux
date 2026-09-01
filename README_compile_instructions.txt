# 2026-08-23
# Cross compile instructions of Sparky Linux kernel on a Linux computer. (guided by Google AI :)
# I left out sudo in front of the commands

# prepare 1: install compile tools
apt install build-essential git libncurses5-dev libssl-dev bc bison xz-utils u-boot-tools

# prepare 2: download legacy gcc compiler: gcc-linaro-4.9.4-2017.01-x86_64_arm-linux-gnueabihf.tar.xz from https://developer.arm.com/Downloads/-/Legacy%20Linaro%20GNU%20Toolchains. It is under tab "Downloads : 4.9-2017.01	February 20, 2026"
cp gcc-linaro-4.9.4-2017.01-x86_64_arm-linux-gnueabihf.tar.xz /opt/toolchains/
tar -xf gcc-linaro-4.9.4-2017.01-x86_64_arm-linux-gnueabihf.tar.xz -C /opt/toolchains/

# Download the sparky Linux kernel from git
git clone https://github.com/sparkysbc/Linux.git
cd Linux
# Compile the kernel
make mrproper
export ARCH=arm
export CROSS_COMPILE=/opt/toolchains/gcc-linaro-4.9.4-2017.01-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-
make actduino_sparky_linux_defconfig
#First make compiles everything, second make compiles to uImage only
make -j$(nproc) HOSTCFLAGS="-fcommon"
make -j$(nproc) HOSTCFLAGS="-fcommon" uImage

# Prepare for transfer to sparky sbc
mkdir -p ~/Desktop/sparky_update/boot
# Copy the compiled kernel image
cp arch/arm/boot/uImage ~/Desktop/sparky_update/boot/

# Copy the device tree files
cp arch/arm/boot/dts/actduino_sparky_linux.dtb ~/Desktop/sparky_update/boot/kernel.dtb

# Create a temporary staging directory for the drivers
mkdir -p ~/Desktop/sparky_update/modules

# Extract the modules into the staging directory
make modules_install INSTALL_MOD_PATH=~/Desktop/sparky_update/modules

cd ~/Desktop
tar -czf sparky_kernel_update.tar.gz sparky_update/

# Transfer to sparky
scp ~/Desktop/sparky_kernel_update.tar.gz <user or root>@<sparky sbc ip addr>:/<user home or root>/

# Backup current kernel and modules
cp /boot/uImage /boot/uImage.bak
cp /boot/kernel.dtb /boot/kernel.dtb.bak
cp -r /lib/modules/3.10.38 /lib/modules/3.10.38.bak

# Install newly compiled kernel:
tar -xzf sparky_kernel_update.tar.gz
# Copy the new modules over (this merges with/overwrites files in /lib/modules/3.10.38/)
cp -r ~/sparky_update/modules/lib/modules/3.10.38/* /lib/modules/3.10.38/
# Copy the boot files
cp ~/sparky_update/boot/uImage /boot/uImage
cp ~/sparky_update/boot/kernel.dtb /boot/kernel.dtb

# Sync storage cache and restart
sync
reboot



