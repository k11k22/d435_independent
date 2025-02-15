mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make





~/catkin_ws/src/tb3mm_6dof/launch/tb3_sara_gazebo.launch 파일 수정



<launch>
  <arg name="gui" default="true"/>
  <arg name="paused" default="false"/>
  <arg name="use_sim_time" default="true"/>

  <rosparam file="$(find tb3mm_6dof)/config/gazebo_controller.yaml" command="load"/>

  <!-- Gazebo 환경 초기화 -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="paused" value="$(arg paused)"/>
    <arg name="gui" value="$(arg gui)"/>
    <arg name="use_sim_time" value="$(arg use_sim_time)"/>
  </include>

  <!-- 첫 번째 로봇 (sara1) -->
  <group ns="sara1">
    <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find tb3mm_6dof)/urdf/tb3_sara_robot.urdf.xacro'"/>
    <node name="spawn_sara1" pkg="gazebo_ros" type="spawn_model"
          args="-urdf -param robot_description -model sara1 -x 0 -y 0 -z 0.1"/>
  </group>

  <!-- 두 번째 로봇 (sara2) -->
  <group ns="sara2">
    <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find tb3mm_6dof)/urdf/tb3_sara_robot.urdf.xacro'"/>
    <node name="spawn_sara2" pkg="gazebo_ros" type="spawn_model"
          args="-urdf -param robot_description -model sara2 -x 1 -y 0 -z 0.1"/>
  </group>

  <!-- robot_state_publisher -->
  <group ns="sara1">
    <node name="robot_state_publisher_sara1" pkg="robot_state_publisher" type="robot_state_publisher">
      <param name="publish_frequency" type="double" value="50.0" />
    </node>
  </group>

  <group ns="sara2">
    <node name="robot_state_publisher_sara2" pkg="robot_state_publisher" type="robot_state_publisher">
      <param name="publish_frequency" type="double" value="50.0" />
    </node>
  </group>

  <!-- Controller 설정 -->
  <group ns="sara1">
    <rosparam file="$(find tb3mm_6dof)/config/arm_controller.yaml" command="load"/>
    <node name="arm_controller_spawner1" pkg="controller_manager" type="controller_manager" args="spawn arm_controller" respawn="false"/>
  </group>

  <group ns="sara2">
    <rosparam file="$(find tb3mm_6dof)/config/arm_controller.yaml" command="load"/>
    <node name="arm_controller_spawner2" pkg="controller_manager" type="controller_manager" args="spawn arm_controller" respawn="false"/>
  </group>

  <!-- Teleoperation 설정 -->
  <group ns="sara1">
    <node name="teleop_sara1" pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" output="screen"/>
  </group>

  <group ns="sara2">
    <node name="teleop_sara2" pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" output="screen"/>
  </group>
  
  <group ns="d435_camera">
  <!-- d435_independent 패키지 내 d435_camera.urdf.xacro 파일 로드 -->
  <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find d435_independent)/urdf/d435_camera.urdf.xacro'"/>
  <node name="spawn_d435_camera" pkg="gazebo_ros" type="spawn_model"
        args="-urdf -param robot_description -model d435_camera -x 2 -y 0 -z 1.0"/>
  <!-- TF 정보 출력을 위한 robot_state_publisher 추가 (필요시) -->
  <node name="robot_state_publisher_d435" pkg="robot_state_publisher" type="robot_state_publisher">
    <param name="publish_frequency" type="double" value="50.0"/>
  </node>
  
</group>
</launch>

