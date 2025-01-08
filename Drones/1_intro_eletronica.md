# Aula 1 - Eletrônica Básica e a Salinha (JJ)

## Overview

O que são: Arduino, Rasp Pi, jumpers, motor servo. Também iremos ensinar: como carregar baterias, cores de fios (ground, voltagem, sinal), como medir carga de bateria, como soldar (básico). Iremos terminar essa aula com leitura de sinal digital na Protoboard (PWM) e mostrar a bancada medindo o empuxo do motor.

Tópicos: Baterias; Motores; Solda; PWM; Protoboard.

## PWM

Nesta parte da aula, são passados os conceitos básicos de sinais - analógico e digital - e a esquematização de um circuito em protoboard.

O circuito escolhido para a realização dessa aula é um circuito simples de LED com potenciômetro, sendo exemplificados os conceitos de PWM para:

1. LED piscando (sinal digital);
2. Controle de luminosidade do LED (sinal analógico).

O circuito montado é o seguinte:

![Captura de tela 2024-03-27 154134](https://github.com/Equipe-eVTOL-ITA/Treinamento/assets/129911818/a2008646-61f7-4227-8def-7e171cd740fb)

E os códigos em arudíno são:

1. LED piscando:

```
void setup() {
  pinMode(13, OUTPUT);
  pinMode(A5, INPUT);
}

void loop() {
  unsigned int valorPOT = analogRead(A5);
  digitalWrite(13, HIGH);
  delay(valorPOT);
  digitalWrite(13, LOW);
  delay(valorPOT);
}
```

2. Controle de luminosidade:

```
void setup() {
  pinMode(11, OUTPUT);
  pinMode(A5, INPUT);
}

void loop() {
  unsigned int valorPot = analogRead(A5);
  analogWrite(11, byte(valorPot/4));
}
```
