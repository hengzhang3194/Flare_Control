# Sensor（传感器）列表

## 体节点位姿传感器 - type: "BodyNode"

用于获取瞬时位姿信息的传感器。

### 输出自由度（DOFs）：12
- 第1个自由度：世界位置x坐标
- 第2个自由度：世界位置y坐标
- 第3个自由度：世界位置z坐标
- 第4个自由度：姿态Yaw角（范围：-Pi到Pi）
- 第5个自由度：姿态Pitch角（范围：-Pi/2到Pi/2）
- 第6个自由度：姿态Roll角（范围：-Pi到Pi）
- 第7个自由度：世界速度x分量
- 第8个自由度：世界速度y分量
- 第9个自由度：世界速度z分量
- 第10个自由度：沿世界x轴角速度
- 第11个自由度：沿世界y轴角速度
- 第12个自由度：沿世界z轴角速度

### 属性
- skeleton_name: 字符串，传感器所附属的Skeleton名字。
- body_node_name: 字符串，传感器所附属的BodyNode（体节点）名字。
- name: 字符串，Sensor的唯一名字。用来与其他Sensor区分。
- up_axis: 字符串，可选项为“XPositive”，“YPositive”，“ZPositive”，“XNegative”，“YNegative”和“ZNegative”。用于计算姿态角的参考上方向。
- forward_axis: 字符串，可选项为“XPositive”，“YPositive”，“ZPositive”，“XNegative”，“YNegative”和“ZNegative”。用于计算姿态角的参考前方向。

## 力传感器 - type: "Force"
用于检测受外力的传感器。注意，目前只能用来测量体节点（BodyNode）受的外力（主要指流体施加的力），无法测量同一个Skeleton内部的力。

有时间平均功能，所以可以避免每一个step力变化过大的问题。

### 输出自由度（DOFs）：1
- 第1个自由度：沿指定轴向受力的值。

### 属性
- skeleton_name: 字符串，传感器所在的Skeleton的名字。
- body_node_name: 字符串，传感器所检测的BodyNode的名字。
- name: 字符串，Sensor的唯一名字。用来与其他Sensor区分。
- averaging_period: 浮点数，时间平均的时间长度。默认值：0秒（不做平均）（不建议）。
- force_axis_local: 3维浮点数，所检测的受力方向。这个方向在BodyNode的本地坐标系下指定。

## 关节传感器 - type: "Joint"

用于检测关节瞬时信息的传感器，如位置、速度等。

### 输出自由度：与所属Joint自由度相同
- 每个自由度的含义由mode属性决定。

### 属性
- skeleton_name: 字符串，传感器所在的Skeleton的名字。
- joint_name: 字符串，传感器所检测的Joint的名字。
- name: 字符串，Sensor的唯一名字，用来与其他Sensor区分。
- mode: 字符串，可选项为“Position”，“Velocity”和“Acceleration”。决定了输出的内容。