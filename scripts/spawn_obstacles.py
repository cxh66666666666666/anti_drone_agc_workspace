#!/usr/bin/env python3
"""
在运行中的Gazebo里加载8字形赛道墙壁
外圈红色墙壁 + 内圈绿色墙壁，形成飞行通道
内圈墙壁在顶部和底部留出通道，让无人机可以通过
"""
import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
import time
import math

def spawn_sdf_model(name, sdf_content, x, y, z):
    """加载SDF模型"""
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        req = SpawnModelRequest()
        req.model_name = name
        req.model_xml = sdf_content
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = z
        req.reference_frame = "world"
        resp = spawn_model(req)
        if resp.success:
            print(f"墙壁 {name} 加载成功 at ({x:.1f}, {y:.1f})")
        else:
            print(f"墙壁 {name} 加载失败: {resp.status_message}")
    except rospy.ServiceException as e:
        print(f"服务调用失败: {e}")

def spawn_wall(x, y, z, length, width, height, yaw, name, color_r=0.8, color_g=0.2, color_b=0.2):
    """生成墙壁SDF，支持自定义颜色"""
    sdf = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <pose>{x} {y} {z} 0 0 {yaw}</pose>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>{length} {width} {height}</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>{length} {width} {height}</size>
          </box>
        </geometry>
        <material>
          <ambient>{color_r} {color_g} {color_b} 1</ambient>
          <diffuse>{color_r} {color_g} {color_b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
    return sdf

def spawn_continuous_arc_walls(center_x, center_y, radius, start_angle, end_angle, 
                               wall_width, wall_height, name_prefix, color_r, color_g, color_b):
    """
    生成连续的弧线墙壁，每段墙壁根据弧长计算长度，确保无缝连接
    """
    walls = []
    arc_length = radius * abs(end_angle - start_angle)
    target_segment_length = 0.5  # 每段目标长度（更短更平滑）
    num_segments = max(int(arc_length / target_segment_length), 4)
    
    angle_step = (end_angle - start_angle) / num_segments
    actual_segment_arc = abs(end_angle - start_angle) / num_segments
    chord_length = 2 * radius * math.sin(actual_segment_arc / 2)
    wall_length = chord_length * 1.25  # 增加25%确保重叠
    
    for i in range(num_segments):
        angle = start_angle + (i + 0.5) * angle_step
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        yaw = angle + math.pi / 2
        
        name = f"{name_prefix}_{i}"
        sdf = spawn_wall(x, y, wall_height/2, wall_length, wall_width, wall_height, 
                        yaw, name, color_r, color_g, color_b)
        walls.append((name, sdf, x, y, wall_height/2))
    
    return walls

def main():
    rospy.init_node('spawn_obstacles')
    print("等待Gazebo启动...")
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    time.sleep(2)

    walls = []
    height = 4.0
    thickness = 0.3
    
    # 8字形参数
    left_center = (-3.0, 0.0)
    right_center = (3.0, 0.0)
    track_radius = 3.0  # 轨迹半径
    margin = 1.5  # 墙壁距离轨迹的安全距离（增加到1.5m）
    
    outer_radius = track_radius + margin + thickness/2  # 外圈半径 = 4.8m
    inner_radius = track_radius - margin - thickness/2  # 内圈半径 = 1.2m
    channel_width = outer_radius - inner_radius - thickness  # 通道宽度
    
    print("生成8字形赛道墙壁...")
    print(f"  轨迹半径: {track_radius}m")
    print(f"  外圈半径: {outer_radius}m")
    print(f"  内圈半径: {inner_radius}m")
    print(f"  通道宽度: {channel_width:.1f}m")
    print(f"  安全距离: {margin}m")
    
    # ========== 外圈红色墙壁（带通道的8字形外圈）==========
    print("\n生成外圈红色墙壁（带通道）...")
    
    # 外圈墙壁角度设置：在交叉处留出通道
    # 2π/3 ≈ 2.09 rad (120°), π/3 ≈ 1.05 rad (60°)
    # 4π/3 ≈ 4.19 rad (240°), 5π/3 ≈ 5.24 rad (300°)
    
    # 右上外弧线：角度从 0 到 2π/3（避开顶部通道）
    walls.extend(spawn_continuous_arc_walls(
        right_center[0], right_center[1], outer_radius,
        0, 2*math.pi/3, thickness, height,
        "wall_outer_right_top", 0.8, 0.2, 0.2
    ))
    
    # 左上外弧线：角度从 π/3 到 π（避开顶部通道）
    walls.extend(spawn_continuous_arc_walls(
        left_center[0], left_center[1], outer_radius,
        math.pi/3, math.pi, thickness, height,
        "wall_outer_left_top", 0.8, 0.2, 0.2
    ))
    
    # 右下外弧线：角度从 4π/3 到 2π（避开底部通道）
    walls.extend(spawn_continuous_arc_walls(
        right_center[0], right_center[1], outer_radius,
        4*math.pi/3, 2*math.pi, thickness, height,
        "wall_outer_right_bottom", 0.8, 0.2, 0.2
    ))
    
    # 左下外弧线：角度从 π 到 5π/3（避开底部通道）
    walls.extend(spawn_continuous_arc_walls(
        left_center[0], left_center[1], outer_radius,
        math.pi, 5*math.pi/3, thickness, height,
        "wall_outer_left_bottom", 0.8, 0.2, 0.2
    ))
    
    # ========== 内圈绿色墙壁（带通道的8字形内圈）==========
    print("生成内圈绿色墙壁（带通道）...")
    
    # 内圈墙壁角度设置：
    # π/3 ≈ 1.05 rad (60°), 2π/3 ≈ 2.09 rad (120°)
    # 4π/3 ≈ 4.19 rad (240°), 5π/3 ≈ 5.24 rad (300°)
    
    # 左上内墙：角度从 π/3 到 2π/3（左侧上半部分，避开顶部通道）
    walls.extend(spawn_continuous_arc_walls(
        left_center[0], left_center[1], inner_radius,
        math.pi/3, 2*math.pi/3, thickness, height,
        "wall_inner_left_top", 0.2, 0.8, 0.2
    ))
    
    # 右上内墙：角度从 π/3 到 2π/3（右侧上半部分，避开顶部通道）
    walls.extend(spawn_continuous_arc_walls(
        right_center[0], right_center[1], inner_radius,
        math.pi/3, 2*math.pi/3, thickness, height,
        "wall_inner_right_top", 0.2, 0.8, 0.2
    ))
    
    # 左下内墙：角度从 4π/3 到 5π/3（左侧下半部分，避开底部通道）
    walls.extend(spawn_continuous_arc_walls(
        left_center[0], left_center[1], inner_radius,
        4*math.pi/3, 5*math.pi/3, thickness, height,
        "wall_inner_left_bottom", 0.2, 0.8, 0.2
    ))
    
    # 右下内墙：角度从 4π/3 到 5π/3（右侧下半部分，避开底部通道）
    walls.extend(spawn_continuous_arc_walls(
        right_center[0], right_center[1], inner_radius,
        4*math.pi/3, 5*math.pi/3, thickness, height,
        "wall_inner_right_bottom", 0.2, 0.8, 0.2
    ))

    print(f"\n加载 {len(walls)} 个墙壁...")
    for name, sdf, x, y, z in walls:
        spawn_sdf_model(name, sdf, x, y, z)
        time.sleep(0.02)

    print("\n========================================")
    print("墙壁加载完成！")
    print("========================================")
    print("8字形赛道说明：")
    print(f"  - 左圆圆心: {left_center}, 右圆圆心: {right_center}")
    print(f"  - 轨迹半径: {track_radius}m")
    print(f"  - 外圈半径: {outer_radius}m (红色墙壁)")
    print(f"  - 内圈半径: {inner_radius}m (绿色墙壁)")
    print(f"  - 通道宽度: {outer_radius - inner_radius - thickness:.1f}m")
    print(f"  - 飞行高度: z = 2m")
    print("  - 红色外圈墙壁 + 绿色内圈墙壁形成飞行通道")
    print("  - 内外圈墙壁在交叉处都留出通道")
    print("========================================")

if __name__ == "__main__":
    main()
