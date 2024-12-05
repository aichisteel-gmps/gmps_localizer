# Overview

The **GMPS Localizer** estimates vehicle pose by GMPS sensor output.
All magnetic markers must be listed in `marker.csv` file.
Data association is executed by comparing magnetic marker position
 predicted from vehicle position with marker table.


## Flowchart
The overall flowchart of gmps_localizer is decribed at the end of this document.

## Requirements
- [gmps_msgs](https://github.com/aichisteel-gmps/gmps_msgs)

## Features
This package includes the following features:

- **Yaw estimation from double markers**, which enables estimating x,y,yaw from two continuous markers with one GMPS sensor.

- **Initial pose estimation with RFID**, which enables data association without previous pose information.


## Launch

The `gmps_localizer` starts with the default parameters with following command.

```sh
ros2 launch gmps_localizer gmps_localizer.launch.xml
```

The parameters and input topic names can be set in the `gmps_localizer.launch.xml` file.


## Nodes
### Subscribed Topics
- in_prev_pose (geometry_msgs/PoseWithCovarianceStamped)  
  Previous pose infromation required for data association, 
  which is typically published from pose fusion filter such as EKF. 

- in_gmps_detect (gmps_msgs/GmpsDetect)  
  GMPS sensor ouput, which means lateral error from detected magnetic marker.

- in_rfid (gmps_msgs/Rfid)  
  RFID tag number written in EPC frame.

- in_velocity (geometry_msgs/TwistWithCovarianceStamped)  
  Vehicle speed.

### Published Topics
- gmps_pose (geometry_msgs/PoseWithCovarianceStamped)  
  Estimated pose with covariance matrix.

- gmps_log (gmps_msgs/GmpsLog)  
  GMPS sensor output and result of data association, for logger.


### Pulished TF
- gmps_link  
  TF from "map" coordinate to estimated pose.


## Functions
#### load marker csv  
Magnetic marker table csv file consists of following columns.

| Column    |Description   | 
| :-------- | :----- | 
|mm_id |User defined number of magnetic markers.|
|tag_id | RFID tag number written in EPC frame.|
|mm_kind | Type of magnetic markers. [not used] |
|pole | Magnetic pole of magnetic markers. |
|x | position of magnetic markers in "map" coordinate.|
|y | position of magnetic markers in "map" coordinate.|

#### data association
- marker_association_by_rfid
- marker_association_by_prev_pose

#### measurement pose
- measurement_double_marker
- measurement_single_marker




## Parameter description

The parameters are set in `launch/gmps_localizer.launch.xml` .


### For pose measurement

| Name      | Type   | Description    | 
| :--------- | :----- | :----------- | 
| enable_pole | bool | Enable/Disable magnetic pole filtering at data association. |
| enable_rfid | bool | Enable/Disable RFID tag_id filtering at data association.  |
| marker_table_csv_name| string | File path of magnetic marker table |
| tf_x | double | tf from baselink to GMPS sensor [m]                       | 
| tf_y | double | tf from baselink to GMPS sensor [m] | 
| tf_yaw| double | tf from baselink to GMPS sensor [rad] | 
|tf_rfid_x | double | tf from baselink to RFID R/W [m]|
| sigma_x_gmps        | double | std_dev of X of gmps_pose [m] |
| sigma_y_gmps        | double | std_dev of Y of gmps_pose [m] |
| sigma_theta_gmps      | double | std_dev of Yaw of gmps_pose [rad] | 

## Algorithm
/* transform from vehicle to marker */

$$\hat{x}_{m} = x_c + e_m\sin(\theta_c) - l_m^\prime \cos(\theta_c)$$  

$$\hat{y}_{m} = y_c - e_m\cos(\theta_c) - l_m^\prime \sin(\theta_c)$$  

/* magnetic marker association */

$$ i = \arg \min _j \sqrt{ (x_m^{(j)} - \hat{x}_m )^2 + (y_m^{(j)}-\hat{y}_m)^2} $$

/* transform from marker to vehicle, including delay */

$$ \tilde{x}_c = x_m^{(i)} - e_m\sin(\theta_c) + l_m^\prime \cos(\theta_c) $$

$$\tilde{y}_c = y_m^{(i)} + e_m\cos(\theta_c) + l_m^\prime \sin(\theta_c)$$

## How to tune parameters
- enable_pole
  - marker.csvの設定が正しく行われていればTrue.そうでなければFalse
- enable_rfid
  - RFIDリーダを搭載しており、marker.csvの設定が正しく行われていればTrue.そうでなければFalse
- marker_table_csv_name
  - 磁気マーカテーブルへのパス
- tf_x
  - baselinkからGMPSセンサ中心までの縦方向のオフセットを計測して設定。GMPSセンサが前にあるときが+
- tf_y
  - baselinkからGMPSセンサ中心までの横方向のオフセットを計測して設定。GMPSセンサが左にあるときが+
- tf_yaw
  - baselinkからGMPSセンサまでのyaw方向のオフセットを計測して設定。CCWが+
- tf_rfid_x
  - baselinkからRFIDリーダまでの縦方向のオフセットを計測して設定。RFIDが前にあるときが+
- th_rfid_detect_range_m
  - 基本的に変える必要はない
- th_association_error_dist_m
  - 基本的に変える必要はない
  - 磁気マーカの設置間隔の半分が目安。2m間隔で敷設していれば1m
  - この値を広くすれば、多少自己位置が怪しくても対応付けをしにいく。ただし誤った対応付けを行う可能性も高まる
- th_association_break_dist_m
  - 基本的に変える必要はない
  - 磁気マーカの対応付けにおいてテーブルの探索を早期に打ち切る際の閾値。磁気マーカの最小間隔の半分が目安。公道タイプは最小1mなので、その半分の0.5m
  - 処理効率を多少改善するための閾値であり、大きくすると誤った対応付けで終了する可能性が高くなる
- th_dist_double_marker_m
  - 基本的に変える必要はない
  - この値を大きくすると、離れたマーカでも二連マーカとしてyaw推定をする。公道タイプは2m間隔を想定しており、3m以上での精度検証はしていない
- th_yaw_diff_double_marker_rad
  - 基本的に変える必要はない
  - この値を大きくすると、直進していなかったとしても二連マーカとしてyaw推定をする。直進の仮定が崩れるほどyaw推定の精度は悪化する
- marker_d_dist_m
  - 基本的に変える必要はない
  - RFIDによる強制紐づけを行いたい二連マーカのマーカ間距離を設定する。通常は2m間隔の2個を1セットとするが、1m間隔や3m間隔にしたい場合は変更が必要
- th_marker_d_dist_m
  - 基本的に変える必要はない
  - marker_d_dist_mに対するtolerance
- th_rfdi_forced_section_change_m
  - 基本的に変える必要はない
  - RFID強制紐づけの発動を早めたい場合は、これを小さくする
- sigma_x_gmps
  - この値を小さくすると、EKFがdead reckoningではなくgmps_pose寄りになる
  - 経験上7cmを設定しているが、特に根拠はない。他の観測やDR側のsigmaと合わせてちょうどいいバランスを手探りで設定している
- sigma_y_gmps
  - xと同じ
- sigma_theta_gmps
  - この値を小さくするとEKFがdead reckoningではなくgmps_pose寄りになる
  - 単体マーカの場合はyaw推定をしないため、かなり大きい値を設定し、dead reckoningによるヨーレートの積分を重視している
  - 二連マーカの場合はyaw推定をしているので小さくすべきなのだが、ソースコードの修正が必要なためできていない

# flowchart by mermaid
```mermaid
---
title: gmps_localizer v2.1.0
---
graph TD;

subgraph callback_detect
a0[["update_mileage( )"]]
a1[["push_marker_queue( )"]]
cond1{rfid_queue != empty}
a2[["marker_association_by_rfid( )"]]
cond2{"rfid_association was failed AND
prev_pose is valid"}
a3[["marker_association_by_prev_pose( )"]]
a4[publish info]

cond3{association was suceeded}
cond4{marker_queue includes two markers}
a5[["check_double_marker( )"]]

cond5{f_double_marker==true}
a6[["measurement_double_marker( )"]]
cond6{prev_pose is valid}
a7[["measurement_single_marker( )"]]
a9[[publish pose]]
a10[/START\]
a11[\END/]

a10 -->a0
a0 --> a1
a1 --> cond1
cond1 --Y--> a2
a2 --> cond2
cond1 --N-->cond2
cond2 --Y-->a3
a3 -->a4
cond2 --N-->a4
a4 --> cond3
cond3 --Y-->cond4
cond3 --N--> a11
cond4 --Y --> a5
cond4 --N -->cond5
a5 --> cond5
cond5 --Y--> a6
cond5 --N--> cond6
a6 -->a9
cond6 --Y-->a7
cond6 --N--> a11
a7 --> a9
a9 --> a11
end

subgraph callback_prevpose
b1[subscribe /prev_pose]
b2[store pose information]
b1 --> b2
end

subgraph callback_rfid
c1[subscribe /rfid]
c2[["update_mileage( )"]]
c3[[push_rfid_queue]]
c1-->c2-->c3
end

callback_prevpose --> callback_detect
callback_rfid --> callback_detect


```
