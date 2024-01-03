<div align="center">
<p align="center">
  <img src="https://raw.githubusercontent.com/PKief/vscode-material-icon-theme/ec559a9f6bfd399b82bb44393651661b08aaf7ba/icons/folder-markdown-open.svg" width="100" />
</p>
<p align="center">
    <h1 align="center">FRANKAEMIKA_PICK_PLACE</h1>
</p>
<p align="center">
    <em>Error generating summary: HTTPStatusError occurred. See logs for details.</em>
</p>
<p align="center">
	<img src="https://img.shields.io/github/license/Tejendra00/FrankaEmika_Pick_Place?style=standard" alt="license">
	<img src="https://img.shields.io/github/last-commit/Tejendra00/FrankaEmika_Pick_Place?style=standard" alt="last-commit">
	<img src="https://img.shields.io/github/languages/top/Tejendra00/FrankaEmika_Pick_Place?style=standard" alt="repo-top-language">
	<img src="https://img.shields.io/github/languages/count/Tejendra00/FrankaEmika_Pick_Place?style=standard" alt="repo-language-count">
<p>
<p align="center">
	<!-- standard option, no dependency badges. -->
</p>
</div>
<hr>

##  Quick Links
- [ Quick Links](#-quick-links)
- [ Overview](#-overview)
- [ Features](#-features)
- [ Repository Structure](#-repository-structure)
- [ Modules](#modules)
- [ Getting Started](#-getting-started)
    - [ Installation](#-installation)
    - [ Running FrankaEmika_Pick_Place](#-running-FrankaEmika_Pick_Place)
    - [ Tests](#-tests)
- [ Roadmap](#-roadmap)
- [ Contributing](#-contributing)
- [ License](#-license)
- [ Acknowledgments](#-acknowledgments)

---

##  Overview

Error generating summary: HTTPStatusError occurred. See logs for details.

---

##  Features

Error generating summary: HTTPStatusError occurred. See logs for details.

---

##  Repository Structure

```sh
└── FrankaEmika_Pick_Place/
    ├── core/
    │   ├── __init__
    │   ├── interfaces.py
    │   ├── safety.py
    │   └── utils.py
    ├── labs/
    │   ├── Forward_Kinematics/
    │   │   ├── visualize.py
    │   │   └── workspace.py
    │   ├── Inverse_Kinematics/
    │   │   ├── follow.py
    │   │   └── visualize.py
    │   ├── Jacobian_Velocity_FK/
    │   │   ├── follow.py
    │   │   └── visualize.py
    │   ├── Path Planning/
    │   │   ├── potentialField_demo.py
    │   │   └── rrt_demo.py
    │   └── Pick_Place_Final/
    │       ├── Final_Blue.py
    │       ├── Final_Red.py
    │       ├── Ik_new.py
    │       └── translib.py
    ├── lib/
    │   ├── FK_velocity.py
    │   ├── IK_position_null.py
    │   ├── IK_velocity_null.py
    │   ├── Jacobian.py
    │   ├── __init__
    │   ├── calcAngDiff.py
    │   ├── calcManipulability.py
    │   ├── detectCollision.py
    │   ├── forwardKinematics.py
    │   ├── inverseKinematics.py
    │   ├── loadmap.py
    │   ├── potentialFieldPlanner.py
    │   └── rrt.py
    ├── maps/
    │   ├── emptyMap.txt
    │   ├── map1.txt
    │   ├── map2.txt
    │   ├── map3.txt
    │   ├── map4.txt
    │   └── map5.txt
    └── ros/
        └── meam520_labs/
            ├── CMakeLists.txt
            ├── config/
            ├── launch/
            ├── msg/
            ├── scripts/
            ├── urdf/
            └── worlds/

```

---

##  Modules

<details closed><summary>maps</summary>

| File                                                                                             | Summary                                                                   |
| ---                                                                                              | ---                                                                       |
| [emptyMap.txt](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/maps/emptyMap.txt) | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [map4.txt](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/maps/map4.txt)         | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [map5.txt](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/maps/map5.txt)         | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [map1.txt](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/maps/map1.txt)         | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [map2.txt](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/maps/map2.txt)         | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [map3.txt](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/maps/map3.txt)         | Error generating summary: HTTPStatusError occurred. See logs for details. |

</details>

<details closed><summary>labs.Path Planning</summary>

| File                                                                                                                               | Summary                                                                   |
| ---                                                                                                                                | ---                                                                       |
| [potentialField_demo.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/labs/Path Planning/potentialField_demo.py) | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [rrt_demo.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/labs/Path Planning/rrt_demo.py)                       | Error generating summary: HTTPStatusError occurred. See logs for details. |

</details>

<details closed><summary>labs.Forward_Kinematics</summary>

| File                                                                                                                | Summary                                                                   |
| ---                                                                                                                 | ---                                                                       |
| [visualize.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/labs/Forward_Kinematics/visualize.py) | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [workspace.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/labs/Forward_Kinematics/workspace.py) | Error generating summary: HTTPStatusError occurred. See logs for details. |

</details>

<details closed><summary>labs.Jacobian_Velocity_FK</summary>

| File                                                                                                                  | Summary                                                                   |
| ---                                                                                                                   | ---                                                                       |
| [follow.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/labs/Jacobian_Velocity_FK/follow.py)       | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [visualize.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/labs/Jacobian_Velocity_FK/visualize.py) | Error generating summary: HTTPStatusError occurred. See logs for details. |

</details>

<details closed><summary>labs.Pick_Place_Final</summary>

| File                                                                                                                | Summary                                                                   |
| ---                                                                                                                 | ---                                                                       |
| [Ik_new.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/labs/Pick_Place_Final/Ik_new.py)         | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [Final_Blue.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/labs/Pick_Place_Final/Final_Blue.py) | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [Final_Red.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/labs/Pick_Place_Final/Final_Red.py)   | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [translib.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/labs/Pick_Place_Final/translib.py)     | Error generating summary: HTTPStatusError occurred. See logs for details. |

</details>

<details closed><summary>labs.Inverse_Kinematics</summary>

| File                                                                                                                | Summary                                                                   |
| ---                                                                                                                 | ---                                                                       |
| [follow.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/labs/Inverse_Kinematics/follow.py)       | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [visualize.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/labs/Inverse_Kinematics/visualize.py) | Error generating summary: HTTPStatusError occurred. See logs for details. |

</details>

<details closed><summary>ros.meam520_labs</summary>

| File                                                                                                             | Summary                                                                   |
| ---                                                                                                              | ---                                                                       |
| [CMakeLists.txt](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/CMakeLists.txt) | Error generating summary: HTTPStatusError occurred. See logs for details. |

</details>

<details closed><summary>ros.meam520_labs.msg</summary>

| File                                                                                                                                     | Summary                                                                   |
| ---                                                                                                                                      | ---                                                                       |
| [BlockDetection.msg](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/msg/BlockDetection.msg)             | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [BlockDetectionArray.msg](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/msg/BlockDetectionArray.msg)   | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [TransformStampedList.msg](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/msg/TransformStampedList.msg) | Error generating summary: HTTPStatusError occurred. See logs for details. |

</details>

<details closed><summary>ros.meam520_labs.config</summary>

| File                                                                                                                                          | Summary                                                                   |
| ---                                                                                                                                           | ---                                                                       |
| [settings.yaml](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/config/settings.yaml)                         | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [block_detections.rviz](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/config/block_detections.rviz)         | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [lab1.rviz](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/config/lab1.rviz)                                 | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [calibration_camera1.yaml](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/config/calibration_camera1.yaml)   | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [generate_tags.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/config/generate_tags.py)                   | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [turntable_controller.yaml](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/config/turntable_controller.yaml) | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [final.rviz](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/config/final.rviz)                               | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [calibration_camera4.yaml](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/config/calibration_camera4.yaml)   | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [lab2.rviz](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/config/lab2.rviz)                                 | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [lab3.rviz](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/config/lab3.rviz)                                 | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [tags.yaml](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/config/tags.yaml)                                 | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [calibration_camera3.yaml](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/config/calibration_camera3.yaml)   | Error generating summary: HTTPStatusError occurred. See logs for details. |

</details>

<details closed><summary>ros.meam520_labs.urdf</summary>

| File                                                                                                                          | Summary                                                                   |
| ---                                                                                                                           | ---                                                                       |
| [franka_panda.xacro](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/urdf/franka_panda.xacro) | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [cube.xacro](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/urdf/cube.xacro)                 | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [turntable.xacro](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/urdf/turntable.xacro)       | Error generating summary: HTTPStatusError occurred. See logs for details. |

</details>

<details closed><summary>ros.meam520_labs.launch</summary>

| File                                                                                                                                    | Summary                                                                   |
| ---                                                                                                                                     | ---                                                                       |
| [turntable.launch](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/launch/turntable.launch)             | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [vision_pipeline.launch](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/launch/vision_pipeline.launch) | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [lab1.launch](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/launch/lab1.launch)                       | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [lab3.launch](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/launch/lab3.launch)                       | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [final.launch](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/launch/final.launch)                     | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [lab4.launch](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/launch/lab4.launch)                       | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [lab0.launch](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/launch/lab0.launch)                       | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [lab2.launch](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/launch/lab2.launch)                       | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [single.launch](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/launch/single.launch)                   | Error generating summary: HTTPStatusError occurred. See logs for details. |

</details>

<details closed><summary>ros.meam520_labs.worlds</summary>

| File                                                                                                              | Summary                                                                   |
| ---                                                                                                               | ---                                                                       |
| [empty.world](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/worlds/empty.world) | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [map1.world](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/worlds/map1.world)   | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [final.world](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/worlds/final.world) | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [lab2.world](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/worlds/lab2.world)   | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [map3.world](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/worlds/map3.world)   | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [map2.world](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/worlds/map2.world)   | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [lab3.world](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/worlds/lab3.world)   | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [lab1.world](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/worlds/lab1.world)   | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [map4.world](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/worlds/map4.world)   | Error generating summary: HTTPStatusError occurred. See logs for details. |

</details>

<details closed><summary>ros.meam520_labs.scripts</summary>

| File                                                                                                                           | Summary                                                                   |
| ---                                                                                                                            | ---                                                                       |
| [getBlockPose.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/scripts/getBlockPose.py)     | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [block_spawner.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/scripts/block_spawner.py)   | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [worldConverter.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/ros/meam520_labs/scripts/worldConverter.py) | Error generating summary: HTTPStatusError occurred. See logs for details. |

</details>

<details closed><summary>core</summary>

| File                                                                                               | Summary                                                                   |
| ---                                                                                                | ---                                                                       |
| [interfaces.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/core/interfaces.py) | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [utils.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/core/utils.py)           | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [safety.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/core/safety.py)         | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [__init__](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/core/__init__)           | Error generating summary: HTTPStatusError occurred. See logs for details. |

</details>

<details closed><summary>lib</summary>

| File                                                                                                                    | Summary                                                                   |
| ---                                                                                                                     | ---                                                                       |
| [FK_velocity.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/lib/FK_velocity.py)                     | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [loadmap.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/lib/loadmap.py)                             | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [Jacobian.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/lib/Jacobian.py)                           | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [potentialFieldPlanner.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/lib/potentialFieldPlanner.py) | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [inverseKinematics.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/lib/inverseKinematics.py)         | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [IK_position_null.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/lib/IK_position_null.py)           | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [IK_velocity_null.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/lib/IK_velocity_null.py)           | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [forwardKinematics.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/lib/forwardKinematics.py)         | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [calcAngDiff.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/lib/calcAngDiff.py)                     | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [calcManipulability.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/lib/calcManipulability.py)       | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [detectCollision.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/lib/detectCollision.py)             | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [__init__](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/lib/__init__)                                 | Error generating summary: HTTPStatusError occurred. See logs for details. |
| [rrt.py](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/lib/rrt.py)                                     | Error generating summary: HTTPStatusError occurred. See logs for details. |

</details>

---

##  Getting Started

***Requirements***

Ensure you have the following dependencies installed on your system:

* Python: `► INSERT-VERSION-HERE`
* `► ...`
* `► ...`

###  Installation

1. Clone the FrankaEmika_Pick_Place repository:
```sh
git clone https://github.com/Tejendra00/FrankaEmika_Pick_Place
```

2. Change to the project directory:
```sh
cd FrankaEmika_Pick_Place
```

3. Install the dependencies:
```sh
pip install -r requirements.txt
```

###  Running FrankaEmika_Pick_Place
Use the following command to run FrankaEmika_Pick_Place:
```sh
python main.py
```

###  Tests
To execute tests, run:
```sh
pytest
```

---

##  Project Roadmap

- [X] `► INSERT-TASK-1`
- [ ] `► INSERT-TASK-2`
- [ ] `► ...`

---

##  Contributing

Contributions are welcome! Here are several ways you can contribute:

- **[Submit Pull Requests](https://github.com/Tejendra00/FrankaEmika_Pick_Place/blob/main/CONTRIBUTING.md)**: Review open PRs, and submit your own PRs.
- **[Join the Discussions](https://github.com/Tejendra00/FrankaEmika_Pick_Place/discussions)**: Share your insights, provide feedback, or ask questions.
- **[Report Issues](https://github.com/Tejendra00/FrankaEmika_Pick_Place/issues)**: Submit bugs found or log feature requests for FrankaEmika_Pick_Place.

<details closed>
<summary>Contributing Guidelines</summary>

1. **Fork the Repository**: Start by forking the project repository to your GitHub account.
2. **Clone Locally**: Clone the forked repository to your local machine using a Git client.
   ```sh
   git clone <your-forked-repo-url>
   ```
3. **Create a New Branch**: Always work on a new branch, giving it a descriptive name.
   ```sh
   git checkout -b new-feature-x
   ```
4. **Make Your Changes**: Develop and test your changes locally.
5. **Commit Your Changes**: Commit with a clear and concise message describing your updates.
   ```sh
   git commit -m 'Implemented new feature x.'
   ```
6. **Push to GitHub**: Push the changes to your forked repository.
   ```sh
   git push origin new-feature-x
   ```
7. **Submit a Pull Request**: Create a PR against the original project repository. Clearly describe the changes and their motivations.

Once your PR is reviewed and approved, it will be merged into the main branch.

</details>

---

##  License

This project is protected under the [SELECT-A-LICENSE](https://choosealicense.com/licenses) License. For more details, refer to the [LICENSE](https://choosealicense.com/licenses/) file.

---

##  Acknowledgments

- List any resources, contributors, inspiration, etc. here.

[**Return**](#-quick-links)

---
