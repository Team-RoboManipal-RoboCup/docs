# Perception

The perception pipeline provides 3D spatial awareness, object recognition, and 6-DOF pose estimation for RoboCup Smart Manufacturing League (SML) manipulation tasks. 

By fusing deep learning (YOLO oriented bounding boxes) with 3D point clouds from RGB-D cameras using the Point Cloud Library (PCL), the perception pipeline identifies raw materials, assembly parts, and workstation planes. It delivers filtered, temporally averaged 6D grasp and place targets directly to the MoveIt 2 motion planning system.

```{toctree}
:maxdepth: 2
:caption: Perception Modules

object_detection
```
