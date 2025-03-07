# Robot setup

There are a few initial steps to set up the robot environment for hardware setup.

## Udev rules

To allow access to the robot hardware, you need to set up udev rules. These rules define how devices are accessed and managed by the system.

```shell
sudo cp .devcontainer/robot/data/etc/udev/rules.d/* /etc/udev/rules.d/
```

To apply the new rules without rebooting:

```shell
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Kernel module parameters

To set kernel module parameters, you need to create a configuration file in `/etc/modprobe.d/` to apply the changes during the boot process.

```shell
sudo cp .devcontainer/robot/data/etc/modprobe.d/* /etc/modprobe.d/
```

To emulate the new parameters without rebooting:

```shell
sudo sh -c 'echo 1024 > /sys/module/usbcore/parameters/usbfs_memory_mb'
```
