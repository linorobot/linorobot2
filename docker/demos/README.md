### 1. Install Tmuxinator

Follow the installation instructions for Tmuxinator [here](https://github.com/tmuxinator/tmuxinator?tab=readme-ov-file#installation)

### 2. Running the demos using Tmuxinator and Docker

#### 2.1. First, export the tmuxinator project path:

```
cd linorobot2/docker/demos
export TMUXINATOR_CONFIG=$PWD
```

#### 2.2. Running the Nav2 demo in Gazebo::

```
tmuxinator start sim
```

To stop the simulation, stop any process by pressing Ctrl + C and run:
```
tmuxinator stop sim
```
