🐋 Guía de Instalación ORCA4
Esta guía detalla paso a paso el proceso para configurar el entorno de simulación para el robot submarino ORCA4, incluyendo ArduSub, Gazebo y ROS 2.

Paso 1: Añadir Repositorios Adicionales (APT) 📦
Configuración de las llaves de seguridad de OSRF e instalación de Gazebo Harmonic.

Bash

# 1. Añadir llave de seguridad de OSRF
sudo apt update
sudo apt install curl gnupg lsb-release -y
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

# 2. Añadir el repositorio de Gazebo a las fuentes
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# 3. Actualizar e instalar Gazebo Harmonic
sudo apt update
sudo apt install gz-harmonic

Paso 2: Compilar el Piloto Automático (ArduSub SITL) 🚁
Configuración del firmware de ArduPilot para la simulación.

Bash

# 1. Crear directorio de trabajo
mkdir -p ~/cpr
cd ~/cpr

# 2. Clonar el repositorio
git clone https://github.com/ArduPilot/ardupilot.git

# 3. Configurar y compilar
cd ardupilot
git submodule update --init --recursive
./waf configure --board sitl
./waf sub

Nota: Al finalizar, el ejecutable ardusub se encontrará en: ~/cpr/ardupilot/build/sitl/bin/ardusub.

Paso 3: Instalar TODAS las Dependencias de Compilación 🛠️
Instalación de librerías necesarias para ROS 2, Gazebo y GStreamer.

Bash

sudo apt install -y \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  python3-vcstool \
  rapidjson-dev \
  libprotobuf-dev \
  protobuf-compiler \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  libgz-sim7-dev \
  libgz-transport12-dev \
  libgz-msgs9-dev \
  libgz-plugin-dev

Paso 4: Preparar el Workspace 📂
Configuración del entorno de trabajo de ROS 2 (colcon_ws).

Bash

# 1. Crear y entrar a la carpeta src (asegúrate de haber creado colcon_ws previamente si no existe)
mkdir -p ~/cpr/colcon_ws/src
cd ~/cpr/colcon_ws/src

# 2. Clonar el repositorio ORCA4
git clone https://github.com/clydemcqueen/orca4

# 3. Importar repositorios dependientes
vcs import < orca4/workspace.repos
Paso 5: Instalar Dependencias de ROS (rosdep) y MAVROS 🔗
Bash

# 1. Ir a la raíz del workspace
cd ~/cpr/colcon_ws

# 2. Instalar dependencias de ROS
rosdep update
rosdep install -y --from-paths src --ignore-src

# 3. Instalar datasets geográficos para MAVROS
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
Paso 6: Compilación Final (colcon build) 🏗️
Compilación de todo el proyecto ROS 2.

Bash

cd ~/cpr/colcon_ws
colcon build

Paso 7: Comprobación de la correcta instalación ✅
Para lanzar la simulación, sigue estos pasos en tu terminal:

Vincular el ejecutable de ArduSub: Le decimos al sistema dónde encontrar el binario compilado en el Paso 2.

Bash

export PATH=$PATH:$HOME/cpr/ardupilot/build/sitl/bin

Cargar el entorno de ROS 2:

Bash

source ~/cpr/colcon_ws/install/setup.bash
Lanzar la simulación:

Bash

ros2 launch orca_bringup sim_launch.py

Resultado: Se debería abrir la plataforma de simulación en Gazebo mostrando el robot submarino en el entorno acuático.

Para poder desvincular el puente MAVROS y el control a bajo nivel ArduSub es necesario modificar los archivos en la carpeta "Archivos modificados"
