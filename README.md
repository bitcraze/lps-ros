# Bitcraze UWB LPS position estimator ROS package

This ROS package contains the Particle filter, launch files and bridges used
to estimate the Bitcraze Crazyflie 2.0 position with using the LPS UWB ranging
deck and LPS nodes.

More information can be found on the
[Bitcraze wiki](https://wiki.bitcraze.io/projects:lps:index).

## Setting anchor position

To use the particle filter and the visualisation node, the anchor position needs
to be set up in the ROS parameter server. The launcher files will load the
anchor position from data/anchor_pos.yaml. See data/anchor_pos.yaml.sample.
