# Aula 1 - Eletrônica Básica e a Salinha (JJ)

## Overview

Passar conhecimentos básicos sobre eletrônica e sobre a organização da nossa salinha. O que são: Arduino, Rasp Pi, jumpers, motor servo. Também iremos ensinar: como carregar baterias, cores de fios (ground, voltagem, sinal), como medir carga de bateria, como soldar (básico). Iremos terminar essa aula com leitura de sinal digital na Protoboard (PWM) e mostrar a bancada medindo o empuxo do motor.

Explicar o que você vai ensinar na aula.

Tópicos: Baterias; Motores; Solda; PWM; Protoboard.

## Conteúdo

- [Tópico 1](#tópico-1)
- [Tópico 2](#tópico-2)
- [Tópico 3](#tópico-3)
- [PWM e Protoboard](#tópico-4)
- [Exemplos](#exemplos-de-código-imagens-e-tabelas)
- [Referências](#referências)

## Tópico 1

List the main topics that will be covered during the class or tutorial.


## Tópico 2

List the main topics that will be covered during the class or tutorial.


## Tópico 3

List the main topics that will be covered during the class or tutorial.


## Tópico 4

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


## Referências

Não precisa ser robusto e colocar TODOS os sites. Basta colocar os mais relevantes, onde a pessoa possa encontrar mais informações ou onde possa ter a referência original da matéria.

[def]: #license
