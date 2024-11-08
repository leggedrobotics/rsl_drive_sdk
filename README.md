# rsl_drive_sdk SDK

Software Development Kit for all RSL drives.\
The source code is released under  [BSD 3](LICENSE).

## Supported drives

| Name | Note |
| ---|  --- |
| Dynadrive | Fully supported, including calibration |
| Halodi drives | Fully supported, including calibration |
| Other | There are additional, but not officially supported drives | 


## Dependencies

#### Manual dependencies:

| Repository | URL | License | Content |
| --- | - | --- | --- |
| ethercat_sdk_master | https://github.com/leggedrobotics/ethercat_sdk_master/tree/master/ethercat_sdk_master | BSD 3-Clause | Ethercat master implementation | 
| soem_interface | https://github.com/leggedrobotics/soem_interface | GPLv3 | Wrapper around [SOEM](https://github.com/OpenEtherCATsociety/soem) | 
| message_logger | https://github.com/leggedrobotics/message_logger.git | BSD 3-Clause | Simple logger |

#### Installable via rosdep:

| Repository | URL | License | Content |
| --- | - | --- | --- |
| yaml_cpp_vendor |https://github.com/ros2/yaml_cpp_vendor | Apache 2.0 / MIT | yaml library | 

## Usage

Pull in the latest code into your local ROS2 workspace including the above mentioned _manual dependencies_.
For an example on how to use the SDK standalone take a look at the [example](https://github.com/leggedrobotics/ethercat_device_configurator/blob/master/ethercat_device_configurator/src/standalone.cpp) at the [ethercat_device_configurator](https://github.com/leggedrobotics/ethercat_device_configurator).

## Setup

### Allow your user to set priorities and niceness

The provided executables perform best when they are given higher priority than other processes.
By default, a linux user or group cannot set priorities higher than 0.
To allow your linux user account to do so, you need to append the following entries to the ``/etc/security/limits.conf`` file (replacing ``<username>``):

```
<username>       -       rtprio          99
<username>       -       nice            -20
```

To allow your entire group to set higher priorities, append (replacing ``<groupname>``):

```
@<groupname>     -       rtprio          99
@<groupname>     -       nice            -20
```



## Configuration files

See the setup in the `ethercat_device_configurator`: [link](https://github.com/leggedrobotics/ethercat_device_configurator/tree/master/ethercat_device_configurator/example_config)
