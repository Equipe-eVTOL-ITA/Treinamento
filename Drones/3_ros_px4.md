# Aula 3 - Middleware e Robótica: ROS2, PX4 e MAVLink (Sheik e Velha)

## Conteúdo

- [Tópico 1](#tópico-1)
- [Tópico 2](#tópico-2)
- [Tópico 3](#tópico-3)
- [Tópico 4](#tópico-4)
- [Exemplos](#exemplos-de-código-imagens-e-tabelas)
- [Referências](#referências)

## ROS2

### O que é ROS?

ROS (Robot Operating System) é uma coleção de frameworks de software, isto é, uma abstração que une códigos comuns a diversos projetos de software provendo alguma funcionalidade, ou ainda, de maneira mais genérica, um conjunto de bibliotecas de software e ferramentas que auxiliam no desenvolvimento de aplicações para robótica. O link a seguir contém um vídeo explicando o que é ROS de maneira mais detalhada <https://vimeo.com/639236696>. Para instalar o ROS2 acesse o link <https://docs.ros.org/en/foxy/Installation.html>.

### Configurando o ambiente

#### Background

ROS2 se baseia na combinação de workspaces utilizando o shell environment, uma interface entre o usuário e o kernel, um programa no núcleo do sistema operacional que geralmente possui controle sobre tudo no sistema. Worksapce é um termo de ROS para a localização no seu sistema de onde você está desenvolvendo utilizando ROS2. o principal workspace de ROS2 é chamado de underlay enquanto que workspaces subsequentes são chamados de overlay.

### Tasks

#### 1 Source the setup files

Você precisará executar esse comando em cada nova shell que abrir para ter acesso aos comandos do ROS2.

```python
source /opt/ros/foxy/setup.bash
```

#### 2 Add sourcing to your shell startup script

Se você não quiser realizar o comando anterior toda vez que abrir uma nova shell (pular 1), então você pode adicionar o comando ao script de inicialização da shell.

```python
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
```

#### 3 Check environment variables

Sourcing ROS2 setup files irá definir várias variáveis do ambiente necessárias para operar o ROS2. Caso tenha algum problema para achar ou usar os pacotes de ROS2, verifique que o ambiente foi setado corretamente.

```python
printenv | grep -i ROS
```

Confira que as variáveis como ROS_DISTRO e ROS_VERSION estão setadas. Por exemplo, se você está usando o Foxy,você verá:

```python
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=foxy
```

#### 3.1 A variável ```ROS_DOMAIN_ID```

Uma vez que você tenha determinado um único inteiro para o seu grupo de nós de ROS2,você consegue setar as variáveis do ambiente:

```python
export ROS_DOMAIN_ID=<your_domain_id>
```

Para manter essa configuração entre sessões de shell, você pode adicionar o comando ao script de inicialização da shell.

```python
echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc
```

#### 3.2 A variável ```ROS_LOCALHOST_ONLY```

Por definição, a comunicação do ROS2 não é limitada ao host local. A variável ```ROS_LOCALHOST_ONLY``` permite que você restrinja a comunicação ao host local. Isso significa que o seu sistema de ROS2, e seus tópicos, serviços e ações não serão visíveis para outros computadores no network local. 

```python
export ROS_LOCALHOST_ONLY=1
```

Para manter essa configuração entre sessões de shell, você pode adicionar o comando ao script de inicialização da shell.

```python
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
```

### Pré-requisitos

O link a seguir contém o tutorial para instalar os programas que serão utilizados para melhor compreensão dos conceitos de ROS que serão vistos a diante <https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html>

### Entendendo nós

### Background

#### 1 O "gráfico" de ROS2

O gráfico de ROS é um network de elementos de ROS2 processando dados juntos simultaneamente. O gráfico abrange todos os executáveis e todas as conexões entre eles se você fosse mapeá-los e visualzá-los.

#### 2 Nós em ROS2

Cada nó em ROS deve ser responsável por um único propósito, por exemplo, controlar o motor de uma roda ou publicar os dados de um sensor de medição de distância laser. Cada nó pode enviar e receber dados de outros nós via tópicos, serviços, parâmetros e ações.

![image](https://github.com/Equipe-eVTOL-ITA/Treinamento/assets/142051901/c338dcb5-87ee-4600-a9a8-d51ddb9ee428)

### Tasks

##### 1 ros2 run

O comando ```ros2 run``` lança um executável de um pacote.

```python
ros2 run <package_name> <executable_name>
```

Para abrir o turtlesim, abra um novo terminal e insira o seguinte comando:

```python
ros2 run turtlesim turtlesim_node
```

Aqui, o nome do pacote é ```turtlesim``` e o nome do executável é ```turtlesim_node```.

Nós ainda não sabemos o nome do nó, mas podemos descobrir usando ```ros2 node list```.

#### 2 ros2 node list

```ros2 node list``` irá mostrar o nome de todos os nós sendo executados.

Abra um novo terminal enquanto o turtlesim ainda está rodando no outro e insira o seguinte comando:

```python
ros2 node list
```

O terminal irá retornar o nome do nó:

```python
/turtlesim
```

Abra outro terminal e inicie o nó teleop com o comando:

```python
ros2 run turtlesim turtle_teleop_key
```

Aqui, estamos nos referindo ao pacote ```turtlesim``` de novo, mas, dessa vez nós estamos interessados no executável ```turtle_teleop_key```.

Retorne ao terminal em que você rodou ```ros2 node list``` e insira o comando novamente. Você agora verá o nome de dois nós ativos:

```python
/turtlesim
/teleop_turtle
```

#### 2.1 Remapping 

Remapping permite que você redefina propriedades padrão do nó, como o nome do nó, o nome do tópico, o nome do serviço, etc., para valores personalizados. 

Agora, vamos redefinir o nome do nosso nó ```turtlesim```. Em um novo terminal, insira o seguinte comando:

```python
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
```

Como você está inserindo o comando ```ros2 run``` para o turtlesim de novo, outra janela do turtlesim será aberta. No entanto, se você retornar ao terminal em que inseriu ```ros2 node list``` e inserir o comando novamente, você verá o nome de três nós:

```python
/turtlesim
/teleop_turtle
/my_turtle
```

#### 3 ros2 node info

Agora que você sabe o nome dos seus nós, você consegue acessar mais informações sobre eles com:

```python
ros2 node info <node_name>
```

Para investigar seu último nó, ```my_turtle```, insira o seguinte comando:

```python
ros2 node info /my_turtle
```

```ros2 node list``` retorna uma lista de subscribers, publishers, serviços e ações, isto é, o gráfico ROS das conexões que interagem com aquele nó. A saída deve parecer com:

```python
/my_turtle
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Services:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
    /my_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /my_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /my_turtle/get_parameters: rcl_interfaces/srv/GetParameters
    /my_turtle/list_parameters: rcl_interfaces/srv/ListParameters
    /my_turtle/set_parameters: rcl_interfaces/srv/SetParameters
    /my_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:
```

Agora, tente rodar o mesmo comando no nó ```/teleop_turtle``` e veja como suas conexões são diferentes das do ```/my_turtle```.

### Entendendo tópicos

## PX4

## MAVLink

## Exemplos de código, imagens e tabelas

Include code examples and explanations to illustrate key concepts taught during the class or tutorial.

```python
# Example code snippet
def example_function():
    print("Hello, world!")
```

![Example Image](https://example.com/image.jpg)
*Caption: This is an example image*

| Column 1 Header | Column 2 Header | Column 3 Header |
| --------------- | --------------- | --------------- |
| Row 1, Col 1    | Row 1, Col 2    | Row 1, Col 3    |
| Row 2, Col 1    | Row 2, Col 2    | Row 2, Col 3    |
| Row 3, Col 1    | Row 3, Col 2    | Row 3, Col 3    |

## Overview

É a aula sobre como nosso drone funciona de fato. Vamos falar sobre ROS2 (Topics, Subscriptions, Messages), PX4 e MavLINK (mensagens, comunicação, conexão). 
Obs: prof. Vinicius vai auxiliar nessa aula. Mandar o esboço dessa aula quando for concluída para que ele veja e opine.

## Referências

Não precisa ser robusto e colocar TODOS os sites. Basta colocar os mais relevantes, onde a pessoa possa encontrar mais informações ou onde possa ter a referência original da matéria.

[def]: #license

[1] <https://docs.ros.org/en/foxy/Tutorials.html>
[2] <https://pt.wikipedia.org/wiki/Robot_operating_system>
