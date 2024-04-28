# Actuator（作用器）列表

## 直流电机作用器 - type: "DcMotor"

作用于1自由度旋转Joint，模拟直流电机的Actuator。

### 输入自由度（DOFs）：1
- 第1个自由度：PWM，范围0~1。

### 属性
- skeleton_name: 字符串，电机所附属的Skeleton名字
- joint_name: 字符串，附属的Skeleton上需要操纵的Joint名字，注意：需要是1自由度旋转关节。
- name: 字符串，Actuator的唯一名字。用来与其他Actuator区分。
- model: (DcMotorModel)，直流电机模型参数，具体如下：
- DcMotorModel.stall_torque: 浮点数，0转速扭矩
- DcMotorModel.zero_load_rpm: 浮点数，0负载转速
- rotation_direction: 字符串，可选项为“Forward”与“Reversed”。方向请按照左右手系与对应左右手定则选择。

## 关节作用器 - type: "Joint"
作用于任意自由度的Joint。可以直接控制关节的速度等。

### 输入自由度（DOFs）：与作用Joint的自由度相同
- 每个自由度的含义由mode属性决定。

### 属性
- skeleton_name: 字符串，所控制的Joint所在的Skeleton的名字。
- joint_name: 字符串，所控制的Joint的名字。
- name: 字符串，Actuator的唯一名字。用来与其他Actuator区分。
- mode: 字符串，可选项为“Force”，“Velocity”和“Acceleration”。决定了输入值的含义以及作用方法。

## 跟踪作用器 - type: "Follow"

用于自动跟随的Actuator。

### 输入自由度：0

### 属性
- follower_skeleton_name: 字符串，跟踪者Skeleton的名字。
- followed_target_skeleton_name: 字符串，被跟踪的Skeleton的名字。
- name: 字符串，Actuator的唯一名字，用来与其他Actuator区分。
- constraint: 字符换，可选项为“Free”，“Plane”与“Line”。Free就是位置重合跟踪，Plane指在固定平面上跟踪，Line指在固定直线上跟踪。
- plane_origin: 3维浮点数，约束平面上的一点。仅在constraint为Plane时需要。
- plane_normal: 3维浮点数，约束平面的法向量。仅在constraint为Plane时需要。
- line_origin: 3维浮点数，约束直线上的一点。仅在constraint为Line时需要。
- line_direction: 3维浮点数，约束直线的方向。仅在constraint为Line时需要。