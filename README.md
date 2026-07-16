# rosi_seed_noid_sim

SEED-NoidのHRI機能を、実機driverを起動せずGazebo上で再現するためのROS Noetic
catkin packageです。

このrepositoryは、上流の
[`seed_r7_ros_pkg`](https://github.com/seed-solutions/seed_r7_ros_pkg)を直接変更せず、
simulation固有のworld、launch、認識・位置推定・動作backendをoverlayとして提供します。

## 対象環境

- Ubuntu 20.04
- ROS Noetic
- Gazebo 11
- `seed_r7_ros_pkg` type-F model

## 現在の実装範囲

Sim Phase S0として、次を提供します。

- 正面3 mにGazebo人物actorを配置したworld
- 上流`seed_r7_gazebo/seed_r7_empty_world.launch`を利用する専用launch
- catkin package metadata
- repository layout test

人物検出、人物同定、位置推定、Approach、Touch、Leave、Navigationのbackendは、
後続phaseでこのpackageへ追加します。

## 起動

```bash
roslaunch rosi_seed_noid_sim seed_noid_person_world.launch \
  robot_model:=typef GUI:=true
```

このlaunchは実機用`seed_r7_bringup`を起動しません。

## 設計境界

HRIコンポーネントは実機・simulationで共通とし、このpackageが実機driverと同じROS
service、topic、actionを提供します。

| HRI境界 | simulation backend予定 |
|---|---|
| `/detection`, `/start`, `/stop`, `/judge_param` | Gazebo RGB画像の人物検出 |
| `/get_position` | RGB-D画像からの人物三次元位置推定 |
| `/seed_robot_action` | Gazebo台車・腕controller |
| `move_base` | map、AMCL、LaserScanによるNavigation |

音声認識、言語理解、健康リスク判定、音声再生は`health_judge` RTCを使用し、この
repositoryには複製しません。

## 再現性

RoSIのsimulation Robot YAMLからこのrepositoryを`collect.git`で取得し、
`dependencies/dependency-lock.yaml`に固定したcommit SHAへcheckoutします。

branchの先端ではなくcommit SHAを使うことで、別workspaceでも同じsourceを取得します。

## テスト

```bash
python3 test/test_repository_layout.py
catkin build rosi_seed_noid_sim
```
