# Obsolete project

Nowadays the Crazyflie contains all LPS-related positioning algorithm onboard and the Crazyflie client is used to setup the LPS syste. Hence, this package is not required anymore.

  - To setup an LPS system follow the getting [started guide](https://www.bitcraze.io/getting-started-with-the-loco-positioning-system/) using the Crazyflie client.
  - For simple flight of up to a couple of Crazyflie you can directly use the [Crazyflie_ros](https://github.com/whoenig/crazyflie_ros) package.
  - To fly complex trajectories of more Crazyflies, you can use the ros-based [Crazyswarm project](https://crazyswarm.readthedocs.io/en/latest/) by selecting [motion_capture_type to none](https://crazyswarm.readthedocs.io/en/latest/usage.html#select-motion-capture-system)


# Bitcraze UWB LPS position estimator ROS package [![CI](https://github.com/bitcraze/lps-ros/workflows/CI/badge.svg)](https://github.com/bitcraze/lps-ros/actions?query=workflow%3ACI)
This ROS package contains the Particle filter, launch files and bridges used
to estimate the Bitcraze Crazyflie 2.0 position with using the LPS UWB ranging
deck and LPS nodes.

More information can be found on the
[Bitcraze wiki](https://wiki.bitcraze.io/projects:lps:index).

## Setting anchor position

To use the particle filter and the visualisation node, the anchor position needs
to be set up in the ROS parameter server. The launcher files will load the
anchor position from data/anchor_pos.yaml. See data/anchor_pos.yaml.sample.

## Contribute
Go to the [contribute page](https://www.bitcraze.io/contribute/) on our website to learn more.
