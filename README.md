# Robot móvil – evasión de obstáculos (ROS 2 Humble + Gazebo 11)

Proyecto de robot diferencial con sensor LIDAR para pruebas de evasión de obstáculos.  
Incluye:

* **URDF/Xacro** del modelo (`urdf/`)
* *Launch* para **RViz 2** (`display.launch.py`)
* *Launch* para **Gazebo + spawn** (`spawn_sim.launch.py`)

---

## 1. Requisitos

| Software                                | Instalación rápida                                             |
|-----------------------------------------|----------------------------------------------------------------|
| ROS 2 Humble *desktop-full*             | `sudo apt install ros-humble-desktop-full`                     |
| Gazebo 11 + plugins ROS 2               | `sudo apt install ros-humble-gazebo-ros-pkgs`                  |
| xacro                                   | `sudo apt install ros-humble-xacro`                            |
| joint_state_publisher                   | `sudo apt install ros-humble-joint-state-publisher`            |
| teleop_twist_keyboard (opcional teleop) | `sudo apt install ros-humble-teleop-twist-keyboard`            |
| colcon (viene con desktop-full)         | –                                                              |

Después de instalar ROS:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 2. Clonar el repositorio

```bash
git clone <URL_DEL_REPO> proyecto_final
cd proyecto_final
```

Estructura esperada:

```
proyecto_final/
├── src/
│   └── lidar_bot_unal/
│       ├── launch/
│       ├── rviz/
│       └── urdf/
└── .gitignore
```

---

## 3. Compilar

```bash
cd proyecto_final
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

> Para no tener que *sourcear* cada vez:  
> `echo "source ~/proyecto_final/install/setup.bash" >> ~/.bashrc`

---

## 4. Visualizar en RViz 2

```bash
ros2 launch lidar_bot_unal display.launch.py
```

Se lanzan automáticamente:

| Nodo                       | Rol                                  |
|----------------------------|--------------------------------------|
| `robot_state_publisher`    | Publica `/robot_description` y `/tf` |
| `joint_state_publisher`    | Publica `/joint_states`              |
| `rviz2`                    | Abre `rviz/my_bot.rviz`              |

---

## 5. Simular en Gazebo

```bash
ros2 launch lidar_bot_unal spawn_sim.launch.py
```

Se abre Gazebo con el robot ya spawneado.  
Plugins activos:

| Plugin                         | Tópicos                                                  |
|--------------------------------|----------------------------------------------------------|
| `libgazebo_ros_diff_drive.so`  | Sub: `/cmd_vel` – Pub: `/odom`, `/tf`                   |
| `libgazebo_ros_ray_sensor.so`  | Pub: `/scan` (`sensor_msgs/LaserScan`)                  |

### Verificación rápida

```bash
# Verifica que los tópicos existen
ros2 topic list | grep -E 'cmd_vel|odom|scan'

# Ver datos del LIDAR
ros2 topic echo --once /scan

# Ver odometría
ros2 topic echo --once /odom
```

---

## 6. Control del robot

### (a) Teclado con `teleop_twist_keyboard`

```bash
sudo apt install ros-humble-teleop-twist-keyboard

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Teclas más usadas:  
`i`/`k` → avanzar/retroceder · `j`/`l` → girar · `space` → detener

### (b) Publicar comandos manuales a `/cmd_vel`

```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.4}}'
```

El robot debe avanzar y girar.

> Asegúrate de que Gazebo esté corriendo y el robot haya sido spawneado.

---

## 7. Flujo de desarrollo

```bash
# 1) Editar Xacro / launch / código
nano src/lidar_bot_unal/urdf/my_bot.urdf.xacro

# 2) Compilar
colcon build --symlink-install
source install/setup.bash

# 3) Probar
ros2 launch lidar_bot_unal display.launch.py     # RViz
#   o
ros2 launch lidar_bot_unal spawn_sim.launch.py   # Gazebo
```

---

## 8. Roadmap

| Semana | Objetivo                                                         | Estado |
|--------|------------------------------------------------------------------|--------|
| 1      | Workspace + RViz mínimo                                          | ✔      |
| 2      | Modelo URDF completo                                             | ✔      |
| 3      | Plugins Gazebo (`diff_drive`, LIDAR `/scan`)                     | ✔      |
| 4      | Nodo `my_bot_nav/avoid.py` para evasión automática de obstáculos | ☐      |
| 5      | Tuning de parámetros, demo final                                 | ☐      |
