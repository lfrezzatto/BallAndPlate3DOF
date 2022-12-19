# Montagem, Modelagem e Simulação de um Sistema Ball and Plate com Manipulador Paralelo de Três Graus de Liberdade

Um sistema ball and plate tem como objetivo equilibrar uma bolinha sobre uma plataforma lisa, utilizando-se de atuadores rotacionais ou prismáticos para causar a inclinação necessária para manter a esfera parada. Tal tipo de planta, além de ser útil para o estudo de controle de sistemas não lineares, pode ser aplicada para simular a marcha humana. Por esses motivos, este projeto tem como principal objetivo preparar um sistema que sirva como ferramenta de aprendizado para outros alunos, descrever em seu texto uma montagem de baixo custo e portátil, utilizar um sistema de visão computacional para realizar a localização da posição da bolinha e identificar o modelo do sistema. Para realização da ação de controle, foram utilizados dois microcontroladores: um Arduíno Mega e um Raspberry Pi, de forma que o primeiro realiza o acionamento dos servomotores enquanto o último executa o algoritmo de visão computacional, realiza o cálculo da ação de controle e envia os resultados para o Arduíno. É esperado que, ao fim do projeto, seja possível replicar a planta, transportá-la e ter todas as ferramentas necessárias para avaliar os melhores métodos de controle para o seu correto funcionamento.

## 🚀 Começando

Aqui são explicados o funcionamento e **[implantação](#-implanta%C3%A7%C3%A3o)** dos principais códigos utilizados para o projeto.

### 📋 Pré-requisitos

Para poder executar todos os códigos, é necessário ter a biblioteca OpenCV instalada no seu Raspberry Pi. Para isso, execute:

```
pip install opencv-python
```

### 🔧 Funcionamento

Aqui são explicados os principais arquivos do projeto, um a um.

[control_main_v3.py](https://github.com/lfrezzatto/BallAndPlate3DOF/blob/main/control_main_v3.py)

Atua como principal algoritmo de controle, executado no Raspberry Pi. Possui duas threads principais, uma para captura de imagem utilizando a câmera e outra para a execução da função principal, de execução do controle. Na versão disponibilizada, é necessário alterar o algoritmo de controle para correto funcionamento.

[prbs_servos.ino](https://github.com/lfrezzatto/BallAndPlate3DOF/blob/main/prbs_servos.ino)

Algoritmo de reset dos servomotores, executado no Arduino. Utilizado quando há necessidade de fazer com que os servos voltem a posição de angulação 0.

[comunicacao_sinal_de_controle.ino](https://github.com/lfrezzatto/BallAndPlate3DOF/blob/main/comunicacao_sinal_de_controle.ino)

Corresponde à continuação da execução do algoritmo de controle, só que do lado do Arduino. O Raspberry envia os ângulos de rotação necessários para cada servomotor via comunicação python-arduino enquanto esse código está sendo executado, de forma a receber a informação com a angulação e aplicá-la nos três servos.

[circulo.m](https://github.com/lfrezzatto/BallAndPlate3DOF/blob/main/circulo.m)

Arquivo de suporte para execução da simulação no Simulink. É executado no Matlab. Tem como principal função, criar vetores de círculos e salvá-los no workspace, para serem utilizados na simulação da planta.

[virtual_v11.slx](https://github.com/lfrezzatto/BallAndPlate3DOF/blob/main/virtual_v11.slx)

Simulação da planta, executada no Simulink. Trata-se a malha de controle desenhada para simulação do sistema, já com a ação de controle.

[monografia.pdf](https://github.com/lfrezzatto/BallAndPlate3DOF/blob/main/monografia.pdf)

Texto que relata a execução do projeto, com manual de uso no final para explicação mais detalhada da montagem da planta e funcionamento dos algoritmos.

## 🛠️ Construído com

Mencione as ferramentas que você usou para criar seu projeto

* [OpenCV](https://opencv.org/) - Biblioteca de visão computacional usada

## ✒️ Autores

Mencione todos aqueles que ajudaram a levantar o projeto desde o seu início

* **Leonardo Torreão Ferreira** - *Trabalho Inicial e documentação*
* **Luciano Antonio Frezzatto Santos** - *Orientação*

---
Feito com base no modelo disponibilizado por [Armstrong Lohãns](https://gist.github.com/lohhans)