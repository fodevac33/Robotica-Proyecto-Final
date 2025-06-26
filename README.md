# Robot móvil - evasión de obstáculos (ROS 2 + Gazebo)

Este proyecto contiene la descripción (URDF/Xacro) y un *launch* que
visualiza el robot en RViz con un solo comando.   
Repositorio pensado para **ROS 2 Humble** (Ubuntu 22.04 / Pop!\_OS 22.04).

---

## 1. Requisitos

| Software                         | Instalación rápida                                    |
| -------------------------------- | ----------------------------------------------------- |
| **ROS 2 Humble desktop-full**    | `sudo apt install ros-humble-desktop-full`           |
| **Gazebo 11 + bindings ROS 2**   | `sudo apt install ros-humble-gazebo-ros-pkgs`        |
| **xacro**                        | `sudo apt install ros-humble-xacro`                  |
| **joint_state_publisher**        | `sudo apt install ros-humble-joint-state-publisher`  |
| **colcon** (viene con desktop)   |                                                      |

> Tras instalar ROS, no olvides:  
> ```bash
> echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
> source ~/.bashrc
> ```

---

## 2. Clonar el repositorio

```bash
git clone <URL_DEL_REPO> proyecto_final
cd proyecto_final
```

Deberías tener esta estructura:

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

## 3. Compilar el proyecto

```bash
# Desde la raíz del workspace
cd proyecto_final

# Limpia posibles restos (opcional pero recomendado)
rm -rf build/ install/ log/

# Compila con enlaces simbólicos (más rápido durante desarrollo)
colcon build --symlink-install

# Cargar el entorno del workspace compilado
source install/setup.bash
```

> Puedes añadir esta línea al final de tu `~/.bashrc` para cargar el overlay automáticamente:
>
> ```bash
> source ~/proyecto_final/install/setup.bash
> ```

---

## 4. Visualizar el robot en RViz

```bash
ros2 launch lidar_bot_unal display.launch.py
```

Este comando lanza automáticamente:

- `robot_state_publisher` – Publica `/robot_description` y `/tf`
- `joint_state_publisher` – Publica `/joint_states` con valores fijos
- `rviz2` – Se abre con la configuración guardada (`rviz/my_bot.rviz`)

Deberías ver un robot con dos ruedas, un cuerpo y un LIDAR montado.

---

## 5. Flujo de trabajo típico

1. Edita archivos (por ejemplo `urdf/my_bot.urdf.xacro`)
2. Compila:

   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

3. Lanza de nuevo:

   ```bash
   ros2 launch lidar_bot_unal display.launch.py
   ```

---

## 6. Próximos pasos

| Semana | Meta                                                                 |
|--------|----------------------------------------------------------------------|
| 3      | Añadir plugins `<gazebo>`: `diff_drive`, `lidar`, `imu`, etc.       |
| 4      | Lanzar Gazebo con `spawn_entity.py`, nodos de evasión, navegación.  |

