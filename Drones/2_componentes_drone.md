# Aula 2 - Componentes de um Drone (Noia)

## Overview

Aqui, explicaremos a estrutura geral do drone – frame, hélices, motores brushless, ESCs, placas de distribuição, etc. Qual a função de: Rasp Pi, PixHawk, o que são sensores inerciais, câmeras, o que é odometria, o que é fusão de sensores.

Tópicos: Estrutura geral; Rasperry Pi; PixHawk; Sensores; Motores

## Conteúdo

Alguns conceitos iniciais:

- [Sistemas de Coordenada](#sistemas-de-coordenadas-e-movimentos-do-drone)

Indo das extremidades do drone para dentro, temos:

- [Frame](#frame)
- [Hélices](#hélices)
- [Motores brushless](#motores-brushless)
- [ESCs](#escs)
- [Placa de Distribuição de Energia](#placa-de-distribuição-de-energia)
- [Controladora de Voo](#controladora-de-voo-pixhawk)
- [Computador de Bordo](#computador-de-bordo-raspberry-pi)
- [Telemetria e RC](#antena-de-telemetria-e-controle-de-rádio)
- [Câmeras](#câmeras)
- [Comunicação entre Componentes](#comunicação-entre-componentes-no-drone)
- [Referências](#referências)

## Sistemas de Coordenadas e Movimentos do Drone

Temos três principais sistemas de coordenadas para o drone:

Translation System | Rotation System | Ground System
:-----------------:|:---------------:|:------------:|
<img src=../utils/translation_system.png width=95% height=100%> | <img src=../utils/rotation_system.png width=100% height=100%> | <img src=../utils/ground_system.png width=82% height=80%>

Quanto aos movimentos, temos: Pitch (Arfagem), Roll (Rolagem), Yaw (Guinada) e Throttle (Potência).

Pitch          |  Roll | Yaw | Throttle
:-----------------------:|:-----------------------:|:-------------------------:|:-------------------------:
<img src=../utils/pitch.gif width=100% height=100%> | <img src=../utils/roll.gif width=100% height=100%> | <img src=../utils/yaw.gif width=100% height=100%> | <img src=../utils/pitch.gif width=100% height=100%>


## Frame

O frame é o esqueleto do drone onde todos os componentes são montados, e sua função é proteger os eletrônicos internos. O frame impacta significativamente no desempenho do drone: não apenas devido ao peso e aerodinâmica, mas também impacta na frequência de ressonância e na rigidez.

O frame ideal deve ser forte e prático, mas também o mais leve o possível. Há um _tradeoff_ quanto à diminuição do peso: estruturas menos rígidas estão mais sucetíveis a ressonância: vibrações harmônicas no frame que dificultam o controle de voo e prejudicam a qualidade das imagens.

No geral, a fibra de carbono é o material mais popular para frames, porque apresenta:
- Baixo custo
- Baixo peso
- Razão de rigidez para peso alta
- Altamente customizável

Alguns pontos negativos da fibra de carbono são:
- Conduz eletricidade
- Pode bloquear/atenuar frequências de rádio.

### Posicionamento da bateria

Devemos posicionar as baterias na parte de baixo ou de cima do drone?

As baterias têm um peso relativo ao peso total muito grande. Ao montar as baterias em cima, o centro de massa fica mais próximo ao centro de aceleração (ponto de intersecção das hélices), de modo que o momento de inércia fica menor, facilitando o controle da rotação do drone.

Em contraposição, a montagem das baterias em baixo diminui o uso do espaço superior e permite que o frame do drone seja menor, reduzindo o peso.

## Hélices

### Tamanho

O tamanho de uma hélice é referido como o diâmetro total. 

Uma hélice com diâmetro maior gera mais aceleração, dada uma certa velocidade de rotação. O aumento do diâmetro também pode aumentar a eficiência do drone.

Hélices com diâmetro menor são mais responsivas a comandos.

<img src=../utils/propeller-size.png width=70%>

### Número de pás

Aumentando o número de pás, obtemos:
- Menor eficiência (tempo de voo)
- Menor vibração na hélice
- Maior _thrust_ e maior consumo de energia
- Voo mais suave
- Menos ruído


Visto isso, classificamos as hélices pelo número de pás:

- 1: é a mais eficiente, porém tem o problema de ser desbalanceada.

- 2: opção balanceada com maior eficiência. Consome menos energia para manter um certo RPM. É o design mais comum de hélices

- 3: aumento do _thrust_ gerado mantendo-se o tamanho reduzido. O voo também é mais suave.

- 4-8: apresentam peso elevado e consomem mais energia. No geral, pareiam-se com motores muito potentes em drones com carga útil alta e tempo de voo pequeno.


<img src=../utils/num-blades.png width=80%>

### Sentido de Rotação

O drone se movimenta, em um plano horizontal, por meio do aumento da rotação das hélices contrárias ao movimento e diminuição da rotação das hélices do lado do movimento. Por exemplo, se quisermos que o drone se mova para a esquerda mantendo a mesma posição vertical, diminuímos a rotação das hélices 3 e 4 e aumentamos das hélices 1 e 2.

Por conta disso, mantemos as diagonais com hélices de mesma direção para que o momento angular resultante seja nulo.

CW (horário) e CCW (anti-horário) | Disposição das Hélices |
:-----------------:|:-------------------------------:|
<img src=../utils/direction-rotation.jpg width=50%> | <img src=../utils/propellers.png width=100%>

### Pitch

O pitch é a distância teórica de deslocamento da hélice em uma volta completa de 360º.

Hélices com pitch menor giram mais rápido e consomem menos corrente.

Hélices com pitch maior geram mais _thrust_, têm velocidade limite maior e conosomem mais energia.

### Perfil do Ruído

### Ruído Aerodinâmico

O componente mais significativo do ruído gerado por drones é o ruído aerodinâmico, que ocorre quando o ar é deslocado pelas hélices em movimento. Este tipo de ruído é criado por vários fenômenos:

- Turbulência: As hélices geram vórtices na borda das lâminas, causando variações rápidas de pressão no ar, que são percebidas como som. A intensidade e a frequência dessas variações de pressão dependem da velocidade da hélice e de sua geometria.

- Interferência: Quando o drone possui múltiplas hélices, o fluxo de ar de uma hélice pode interferir com o de outra, criando padrões de interferência que aumentam a complexidade do ruído gerado.
- Variação de Pressão: A passagem das hélices através do ar gera diferenças de pressão acima e abaixo das lâminas, contribuindo para o ruído aerodinâmico.

### Ruído Mecânico

 Este tipo de ruído é produzido pelo próprio motor do drone e pela vibração das estruturas físicas, incluindo o corpo do drone e as próprias hélices. Enquanto o ruído aerodinâmico domina em drones em voo, o ruído mecânico pode ser mais aparente durante a partida, pouso, ou em velocidades mais baixas.

## Motores Brushless

O desempenho de um motor está ligado a muitas variáveis: tamanho, voltagem, eficiência. Deve se analisar também o tamanho do frame e o tamanho das hélices.

O documento "Fundamentos do Motor de Drone" fornece uma visão abrangente sobre os motores brushless usados em drones, focando em vários aspectos que afetam o seu desempenho, incluindo tamanhos de motores, classificação KV, eficiência e mais. Aqui está um resumo detalhado baseado nos requisitos especificados:

### Tipos de Motor e Básicos
- **Motores Brushless vs. Motores Brushed**: Motores brushless são significativamente mais eficientes e duráveis do que motores brushed. Motores brushed têm uma eficiência de 75-80% e brushless, de 85-90%.

<img src=../utils/brush.png width=80%> 


### Tamanho do Motor e Classificação KV

- **Tamanho e Configuração**: O tamanho dos motores brushless é indicado por números como 2207 ou 2306, representando o diâmetro e a altura do estator em milímetros. Um estator mais alto tem mais potência a RPMs altos. Um estator mais largo tem mais torque a RPMs baixos.

- **Classificação KV**: A classificação KV denota a velocidade de rotação do motor por volt aplicado. Por exemplo, para uma bateria 3S (11,1V), a _top speed_ do motor sem hélices com 2300 kv é 2600 x 11,1 = 28860 rpm. Classificações KV mais altas resultam em menor resistência e maior consumo de corrente, mas eficiências mais baixas.

<img src=../utils/motor-size.png width=80%> 

### Fatores de Desempenho
- **Relação Empuxo-Peso**: Somamos o empuxo gerado pelos 4 motores e dividimos pelo peso total do drone. Queremos pelo menos uma relação 2:1, pois:
- - Motores ficam menos eficientes (menos torque e mais calor) a RPMs próximos ao seu limite. Isso diminui a vida útil do motor.
- - Controlabilidade: Todo movimento do drone exige RPMS de alguns dos motores maior que os RPMs de hover. Se nos aproximarmos de uma relação 1:1, é possível que o drone não seja controlável.
- **Torque**: Determinado pelo tamanho do estator, qualidade dos ímãs e detalhes de construção. Um torque maior aumenta a responsividade e agilidade, mas pode causar desafios de oscilação pois o erro é amplificado.
- **Eficiência**: Avaliada através do empuxo produzido por energia consumida (g/w), com uma consideração abrangente em toda a faixa de aceleração sendo crucial para desempenho ótimo.
- **Consumo de Corrente e Temperatura**: Importantes para escolher o ESC correto e garantir a longevidade do motor. O superaquecimento pode desmagnetizar ímãs, reduzindo a eficiência e a vida útil.

### Números N e P

Basicamente, N e P são polos e ímãs. Se você verificar um motor brushless, ele tem especificações como 12N14P. O número antes de N refere-se ao número de eletroímãs no estator e o número antes de P refere-se ao número de ímãs permanentes no sino.

<img src=../utils/12N14P.png width=80%> 


### Guideline geral

<img src=../utils/motor-guideline.png width=80%> 


## ESCs

O Electric Speed Controller (ESC) funciona como ponte entre a bateria e o motor elétrico. Ele usa a corrente direta da bateria e fornece uma corrente tri-fásica que é enviada ao motor.

Por meio das mudanças de fase da corrente tri-fásica, a polaridade da bobina do estator muda na mesma medida em que os rotores giram. Dessa forma, os polos sempre se repelem e o rotor é impulsionado. Por conta disso, o ESC precisa saber a posição dos rotores, o que é determinado pelo microcontrolador e por sensores de Efeito Hall.

A voltagem que é enviada ao motor é um sinal analógico, comumente PWM (Pulse Width Modulation).

<img src=../utils/rotation-dir.png width=80%> 

## Placa de Distribuição de Energia

Não há muito segredo. É Uma placa de distribuição centralizada de energia. Aqui, conectamos nossos 4 ESCs, a controdoladora de voo, o computador de bordo e acessórios. Isso simplifica o cabeamento do drone, poupando peso.

Além disso, a PDB (Power Distribution Board) também tem circuitos com regulador de tensão, convertendo o sinal da bateria (11.1V por exemplo) para 5V ou 12V, que são a tensão utilizada por alguns eletrônicos.

<img src=../utils/PDB.png width=80%>

## Controladora de Voo (PixHawk)

IMUs, linguagem PX4 e ArduPilot

## Computador de Bordo (Raspberry Pi)

Comunicação Wifi (como se conecta ao computador), código de autonomia

## Comunicação entre componentes no Drone

MAVLink, PX4, Micro-UXRCE Agent, SSH, Telemetria, controle RC

## Câmeras

O que é odometria visual, visão computacional, SLAM, Nuvem de pontos, filtro de Kalman

## Referências

Não precisa ser robusto e colocar TODOS os sites. Basta colocar os mais relevantes, onde a pessoa possa encontrar mais informações ou onde possa ter a referência original da matéria.

[def]: #license