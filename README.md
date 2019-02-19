# GRAB Cable Robot <img align="right" src="https://www.gnu.org/graphics/gplv3-127x51.png">

## Overview

This repository contains the code to control and operate a custom cable-driven robot built at GRAB laboratory. 

It relies on [_EtherCAT_](https://www.ethercat.org/default.htm) network system and requires a _Real Time Linux Operating System_. Both robot controller and human interface are handled by the same working station, i.e. a standard PC with RT characteristic, and covered here.

## Installation

Follow these instructions thoroughly to get you a copy of the project up and running on your local machine for development and testing purposes. Please contact a maintaner if you incur any trouble.

### Prerequisites

- A PC case with at least one USB 2.0 port
- A power supply unit; better use low power ones with high efficiency (more stable voltage and faster response)
- A 6th or 7th Intel CPU
- Any recent standard or micro ATX motherboard compatible with the processor. Recent test demonstrated that the most simple choice would be one with and integrated network card using e1000e driver, but it is not mandatory
- At least 16 GB of the fastest RAM compatible with the CPU and motherboard
- a PCIe network card with at least one ethernet port. It is important that at least one between the integrated network card and the PCIe one is based on one these drivers: 8139too, e1000, e100, r8169, e1000e. Usually the modern Intel integrated network card are based on e1000e but it is necessary to check. If an integrated network card with e1000e driver is embedded onto the motherboard, a r8169 lan card must be chosen
- It is **strongly** recommended to use the r8169 LAN card as internet port and the e1000e one as the ethercat port due to some minor compatibility issue with newer Linux kernel and EtherLab software, that need to be installed next

It is best to keep the configuration of the PC to its minimum, not adding video card or any other PCI card, if it is not crucial to the application.

### Install Linux OS

First of all a basic Debian based operating system must be installed on the machine. For this project distribution only Ubuntu 16.04.3 LTS is supported and tested for. If you are already working on such platform you can skip this section. Please note that since RT features are required for this application, it is not recommended to run Linux on a virtual machine.

1. Go to https://www.ubuntu-it.org/download and download _Ubuntu 16.04.3 LTS_ image
2. Create a bootable USB stick (for instance using [_Rufus_](https://rufus-usb.it.uptodown.com/windows) for Windows)
3. Install the operating system alongside Microsoft Windows (it is necessary because the majority of drive systems has to be set up in a Windows based environment)
4. Make sure to leave twice the RAM space as _swap area_ and to have only one additional partition as root `(/)`
5. Download all the necessary updates during installation (better having wired ethernet connection)

Once Ubuntu is up and running, a real-time kernel must be installed.

### Install real-time kernel

Some kernel version cannot be installed, depending on the Linux distribution used. For example, with Ubuntu 16.04.3 LTS only _4.x_ kernel can be used. One of the lastest stable release of the RT kernel is the _4.13.13_, which is employed here.
To obtain it, follow the instructions below:

1. Go to https://www.kernel.org/pub/linux/kernel/v4.x/ and download _linux-4.13.13.tar.xz_ (use `CTRL+F` to quickly spot it);
2. Go to https://www.kernel.org/pub/linux/kernel/projects/rt/4.13/ and download _patch-4.13.13-rt5.patch.xz_ (it may be in `older/`);
3. Open a terminal and execute the following commands:

```bash
cd Download
tar xvfj linux-4.13.13.tar.xz
cd linux-4.13.13
xzcat ../patch-4.13.13-rt5.patch.xz | patch -p1
cp /boot/config-$(uname -r) .config
yes "" | make oldconfig
sudo apt-get install libncurses-dev libssl-dev
make menuconfig
```
In the DOS-stlye window you are prompted to, use the keyboard to navigate to `Processor type and features` -> `Preemption Model` and select `Fully Preemptible Kernel (RT)`. Then save and exit.

Next, conclude the installation by compiling the kernel as follows (it will take some minutes):

```bash
make -j$(nproc)
make modules -j$(nproc)
sudo make modules_install
sudo make install
sudo update-grub
```

Now you should be able to boot with the newly installed real-time kernel by using advanced option in the grub at start-up. As far as the user is concerned, some minor lagging in the monitor might be experience but it is normal. To test the kernel
and getting an idea of how a real-time application is set up, https://wiki.linuxfoundation.org/realtime/start can be consulted.

### Install EtherCAT Master

To setup an EtherCAT Master application, please open a terminal and follow instructions below. Further information can be found at original source at https://sourceforge.net/u/uecasm/etherlab-patches/ci/tip/tree/.

```bash
sudo -s
apt install mercurial
gedit /etc/mercurial/hgrc/hgrc
```
And add the following lines:
```txt
[extensions]
mq =
```
Save, exit and return to the open terminal:
```bash
cd /usr/src
hg clone -u 33b922ec1871 http://hg.code.sf.net/p/etherlabmaster/code ether-
lab
hg clone http://hg.code.sf.net/u/uecasm/etherlab-patches etherlab/.hg/patches
cd etherlab
hg qpush -a
```
At this point we retrieved the source code of EtherCAT master and patched it so that it is compatible with newer kernel version, up to 4.13.13, and the relative network card drivers. We have now to configure the source code so that it builds according to our needs (among several available configuration, we will use the _fully-preemptible_ one with dedicated network devices and drivers, see https://www.etherlab.org/en/ethercat/index.php for the details).

In the same terminal, type:
```bash
apt-get install dh-autoreconf
./bootstrap
./configure --disable-8139too --enable-$(YOUR_PCI_DRIVER) --enable-generic
```
where `$(YOUR_PCI_DRIVER)` needs to be replaced by your PCI driver. If you do not know which driver is installed on your machine, on a different terminal type `lshw -class network` and note down the _driver_ name as well as the _serial_ under the group descripted as `Ethernet interface`. Go back to the original terminal and type:
```bash
make -j$(nproc)
make modules -j$(nproc)
make modules_install
make install
```
Now the ethercat master is installed in ”/opt/etherlab” and has to be configured at application level. Therefore:
```bash
cd /opt/etherlab/
mkdir /etc/sysconfig
cp etc/sysconfig/ethercat /etc/sysconfig/
ln -s /opt/etherlab/etc/init.d/ethercat /etc/init.d/ethercat
/usr/lib/insserv/insserv /etc/init.d/ethercat
```
Open "/etc/sysconfig/ethercat" (for instance with _gedit_) and edit it with the previously noted information:
```txt
MASTER0_DEVICE="$(YOUR_PCI_SERIAL)"
DEVICE_MODULES="$(YOUR_PCI_DRIVER)"
```

Now we are good to go and we can test if the master can work properly with:
```bash
./etc/init.d/ethercat start
```
You should see the following line printed on the console:
```bash
Starting EtherCAT master 1.5.2 done
```
Additional commmand line tools can be installed by creating the following link:
```bash
ln -s /opt/etherlab/bin/ethercat /usr/local/bin/ethercat
```

At this point, if you reboot the PC, at startup of the real-time kernel the EtherCAT master will be automatically loaded, so that it can be used right away by any application. If for any reason you are not using e1000e as the ethercat communication port, but the r8169 one, you might experience the failure of the driver to load at boot. This is still an unresolved issue by the developers but a pretty easy workaround is to shut down the PC and unplug it from its power cord: in this way
the LAN card is initialized again and will work properly next time you boot with the real-time kernel.

## Authors

- **Edoardo Idà** - _Initial work_ - edoardo.ida2@unibo.it
- **Simone Comari** - simone.comari2@unibo.it

## Maintainers

- **Simone Comari** - simone.comari2@unibo.it
