# Z3N's Wonder Robot Toolkits

[Chinese](README_CN.md)

A collection of handy and mystical utilities developed by Z3N during the debugging of Unitree robots.

This repo is updated sporadically. Feel free to report bugs via Issues—I'll check them over the weekend.

## Contents

### H2 Teaching Mode

Teaching mode for the H2 upper body.

Enables switching the H2 robot into teaching mode to record or play back demonstration motions.

The robot can walk while playing back motions, and the recorded data includes head movements.

A lightweight tool built with the ArmSDK, ideal for simple demos such as Chinese brush calligraphy, gesture dancing, drumming, etc.

⚠️ **Caution:** The H2 upper body is extremely heavy—please prioritize safety during operation!

### Unitree SDK2 Isaac Bridge

A bridging tool between Unitree SDK2 and Isaac Lab, allowing direct control of simulated robots in Isaac Lab via the `/lowcmd` topic.

Currently supports all robots in the Unitree RL Lab. Includes support for suspending robots using a gantry system.