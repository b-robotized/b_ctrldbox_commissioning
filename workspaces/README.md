This directory contains robot-specific commissioning workspaces.

Each robot folder with ROS 2 driver dependencies has a `<robot>-branches.repos` file.
Import it into the `src` folder of a workspace:

```bash
vcs import --input workspaces/<robot>/<robot>-branches.repos <workspace>/src
```
