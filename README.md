# ROS2 Effort Controller
<table>
    <tr>
        <td>
        </td>
        <td>
            Humble
        </td>
        <td>
            Jazzy
        </td>
        <td>
            Rolling
        </td>
    </tr>
        <td>
            Branch main
        </td>
        <td>
            <a href='humble main'><img src='https://github.com/idra-lab/ros2_effort_controller/actions/workflows/humble.yml/badge.svg'></a><br/>
        </td>
        <td>
            <a href='jazzy main'><img src='https://github.com/idra-lab/ros2_effort_controller/actions/workflows/jazzy.yml/badge.svg'></a><br/>
        </td>
        <td>
            <a href='rolling main'><img src='https://github.com/idra-lab/ros2_effort_controller/actions/workflows/rolling.yml/badge.svg'></a><br/>
        </td>
    </tr>
    <tr>
        <td>
            Branch kuka-prop-ctrl
        </td>
        <td>
            <a href='humble kuka-prop-ctrl'><img src='https://github.com/idra-lab/ros2_effort_controller/actions/workflows/humble-prop-ctrl.yml/badge.svg'></a><br/>
        </td>
        <td>
            <a href='jazzy kuka-prop-ctrl'><img src='https://github.com/idra-lab/ros2_effort_controller/actions/workflows/jazzy-prop-ctrl.yml/badge.svg'></a><br/>
        </td>
       <td>
            <a href='rolling kuka-prop-ctrl'><img src='https://github.com/idra-lab/ros2_effort_controller/actions/workflows/rolling-prop-ctrl.yml/badge.svg'></a><br/>
        </td>
    </tr>
</table>


This repository aim to create a robot independent torque controller based on ROS2 controllers. 
There is a base controller that communicate with the hardware interface of the robot, on top of that controller different types of controllers can be implemented. 
For now a cartesian impedance controller, a joint impedance controller and a gravity compensation are implemented.

**If you want to use just the KUKA's proprietary controllers sending just the target joint position checkout the `kuka-prop-ctrl` branch.**

Check out their use in the KUKA LBR example [here](https://github.com/idra-lab/kuka_impedance)!  

The structure of the code and some libraries have been taken from the repo [Cartesian Controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers).

## Citation
If you use these controllers, please consider citing our work and leaving us a star to support the project. :mechanical_arm: 🫶
```
@article{nardi2026anatomy,
  author={Nardi, Davide and Lamon, Edoardo and Fontanelli, Daniele and Saveriano, Matteo and Palopoli, Luigi},
  journal={IEEE Robotics and Automation Letters}, 
  title={An Anatomy-Aware Shared Control Approach for Assisted Teleoperation of Lung Ultrasound Examinations}, 
  year={2026},
  volume={11},
  number={3},
  pages={2570-2577},
  keywords={Robots;Ribs;Probes;Ultrasonic imaging;Skin;Solid modeling;Computational modeling;Three-dimensional displays;Biological system modeling;Cameras;Medical robots and systems;physical human-robot interaction;telerobotics and teleoperation},
  doi={10.1109/LRA.2026.3653292}}
```
