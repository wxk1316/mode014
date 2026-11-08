import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    try:
        pathplan_share = get_package_share_directory('pathplan')
        road_planner_share = get_package_share_directory('road_planner')
    except Exception as e:
        print(f"Error: 找不到功能包 {e}")
        return LaunchDescription([])

    # ================= 核心修改 =================
    # 使用 ExecuteProcess 替代 Node
    # 强制在指定的物理路径下执行 python3 命令，效果等同于你在终端手敲
    plan_points_process = ExecuteProcess(
        cmd=['python3', 'plan_points.py'],
        cwd='/home/nvidia/Desktop/car_0513/src/road_planner/road_planner', # 强制指定工作目录为脚本所在目录
        output='screen'
    )

    # 为这个原生进程包上 10 秒的延时
    delayed_plan_points = TimerAction(
        period=10.0,
        actions=[plan_points_process]
    )
    # ============================================

    road_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(road_planner_share, 'launch', 'run.launch.py')
        )
    )

    pathplan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pathplan_share, 'launch', 'run.launch.py')
        )
    )

    delayed_pathplan = TimerAction(
        period=2.0,
        actions=[pathplan_launch]
    )

    return LaunchDescription([
        road_planner_launch,  
        # delayed_plan_points,  # 10秒后启动 Python 脚本
        delayed_pathplan      
    ])