# Lab de Máquina de Estados do Drone

### Clonar dependências

Abra o terminal na pasta do workspace:
```shell
cd ~/Treinamento/drone_fsm
```

Clone as dependências:

```shell
cd src
git clone --branch release/1.14 --recursive https://github.com/PX4/px4_msgs.git
git clone --branch main https://github.com/Equipe-eVTOL-ITA/fsm.git
git clone https://github.com/Equipe-eVTOL-ITA/custom_msgs.git
```

### Buildar os pacotes

```shell
cd ~/Treinamento/drone_fsm
colcon build --symlink-install
```

### Rodar o pacote

Depois de mudar o código da máquina de estados, você sempre vai precisar buildar o código novamente (mas demora menos de 10s, então não esquenta).

```shell
cd ~/Treinamento/drone_fsm
colcon build --symlink-install --packages-select drone_control
```

Feito isso, vamos abrir a simulação e o agente DDS (ponte entre PX4 e ROS 2):

- Execute a task Simulation
- Execute a task Agent

Com a simulação aberta, vamos rodar o pacote de OffboardControl:

```shell
ros2 run drone_control land_on_bases
```

### Depois de testar a fsm

Antes de fazer mais modificações no seu código, vamos fechar a simulação.

- Rode a task gazebo kill.