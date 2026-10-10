# Development setup

## Workspace checks

```bash
source /opt/ros/humble/setup.bash
colcon list
colcon build --packages-select care_pick_place_bridge
source install/setup.bash
```

See [the workspace layout](PROJECT_STRUCTURE.md) for package responsibilities.

## Containers and user configuration

Shared Docker configuration lives in `docker/standard/`. User overrides, WSL AI
configuration mounts and host reference mounts live in `docker/user/`.
Follow the [Docker setup instructions](../docker/README.md) to select either setup.

Source under `/workspace` survives container rebuilds. Manually installed tools
outside mounts do not necessarily survive; put installations in the Dockerfile or
a Dev Container setup command. VS Code extensions belong in
`.devcontainer/devcontainer.json`.

## Windows reference access

The user override exposes the previous project at `/care_reference`, including
`README.md` and `assets/logo.png`, and its configs at `/care_configs`. This takes
effect only after recreating the container on a host with the configured paths.
Editing Compose does not add mounts to an already-running agent environment.

## Remaining environment improvements

Make navigation, camera and application dependencies reproducible; document ROS
networking and hardware device access. Physical odometry and LiDAR input must be
provided independently of Gazebo plugins.
