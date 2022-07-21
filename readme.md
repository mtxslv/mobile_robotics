# Welcome to Mobile Robotics Github! 
Here you'll find everything you need to know about the projects regarding SISTEMAS ROBOTICOS AUTONOMOS (DCA0440). Below you can find a description of the goals and their deadlines.

# Important Links:
* [Trello](https://trello.com/invite/b/LE1Vd2Gn/085423fc3f98ddc30c34dec30c24bd3e/primeio-projeto-primeira-meta)
* [Calendar](https://calendar.google.com/calendar/u/0?cid=dW8wNGxyY25oZnMxYXY1ZWk5M2I3NDBwbThAZ3JvdXAuY2FsZW5kYXIuZ29vZ2xlLmNvbQ)
* [Slides Simulation Seminar](https://docs.google.com/presentation/d/1Eh-G889aNMEwxEHvTvcuRe9rz5iG43qhoI5tFvpyMJU/edit#slide=id.g8714a43093_3_682)
* [Slides First Project](https://docs.google.com/presentation/d/1vv4ZA68Gq9pY3Shd31ZKyyyXiAz5y6LVSJN09LMnVNY/edit#slide=id.g10066b48594_0_55)
* [Slides Second Project/ROS](https://docs.google.com/presentation/d/126ER057O4IhNq20BbtyoH9c7sfFEpZEJ8YV1HUDyzEc/edit#slide=id.p)
* [Slides 3rd Project](https://docs.google.com/presentation/d/1hwvX2vBSNK5WOi12faF15z5EELcRZ6E4r-z6zT_6B3E/edit#slide=id.p1)

# Primeiro Projeto
## Meta 01 (. - 3 de maio)
Simular no software V-Rep um robô móvel com acionamento diferencial, de maneira a que o mesmo receba os comandos das velocidades de referências para as rodas e retorne a posição e orientação do robô (x,y,theta) em um referencial global. Além do movimento do robô no espaço de trabalho, mostrar os seguintes gráficos: velocidades das rodas (entradas) em função do tempo; configuração do robô (x,y,theta), (saídas), em função do tempo; gráfico das posições (x(t),y(t)) seguidas pelo robô no plano xy. Entregar relatório e vídeo mostrando o a simulação e os gráficos solicitados.
## Meta 02 (4 de maio - 10 de maio)
Implementar gerador de caminho baseado em polinômios interpoladores de 3º grau para robô móvel. Incluir gerador de caminho na simulação. O simulador deve permitir mostrar o caminho gerado sobre a tela do espaço de trabalho do V-Rep. Entregar relatório e vídeo mostrando os resultados obtidos.
## Meta 03 (11 de maio - 17 de maio)
Implementar controladores cinemáticos do robô móvel no simulador: controlador seguidor de trajetória, controlador de posição. Testar o controlador no simulador e obter resultados de simulação (trajetória gerada, trajetória seguida, gráficos das variáveis de entrada e saída em função do tempo, etc.). Entregar relatório e vídeo mostrando o sistema funcionando.
# Segundo Projeto
## Meta 01 (18 de maio - 9 de junho)
Incluir obstáculos poligonais no espaço de trabalho simulado no V-Rep. O simulador deve apresentar na tela os obstáculos no espaço de trabalho. Considerando que o robô seja de forma hexagonal e sua orientação não mude, obter o mapa correspondente em espaço de configuração. Incluir funcionalidade que apresente o caminho do robô no espaço de trabalho e no espaço de configuração. Entregar relatório, e vídeo mostrando as novas funcionalidades implementadas
## Meta 02 (10 de junho - 16 de junho)
Implementar planejador de caminhos baseado em grafos para o robô móvel. A sua implementação deve permitir mostrar o caminho gerado, tanto no espaço de trabalho do V-Rep, como no espaço de configuração. Implementar controlador que permita ao robô seguir o caminho planejado. Entregar relatório e vídeo mostrando o robô seguindo o caminho planejado.
## Meta 03 (17 de junho - 23 de junho)
Implementar planejador de caminhos baseado em campos de potenciais para o robô móvel. A sua implementação deve permitir mostrar o caminho gerado, tanto no espaço de trabalho do V-Rep, como no espaço de configuração. Apresentar gráfico também com a função de potencial calculada. Implementar controlador que permita ao robô seguir o caminho planejado. Entregar relatório e vídeo mostrando o robô seguindo o caminho planejado
# Terceiro Projeto
## Meta 01 (24 de junho - 7 de julho)
Incluir modelo de sensor de alcance (sonar) no robô simulado. Telecomandar o robô através do teclado ou um controlador simples que faça o robô se locomover com velocidade constante e girar aleatoriamente de modo a evitar obstáculos a partir de um limiar especificado de distância do obstáculo detetada. O simulador deve apresentar nova tela com as medições obtidas pelo sensor de alcance a partir dos obstáculos detectados pelo mesmo. Entregar relatório e vídeo mostrando o sistema operando com as novas funcionalidades
## Meta 02 (8 de julho - 14 de julho)
A partir dos dados do sensor de alcance, construir grade de ocupação. Entregar relatório e vídeo apresentando as novas funcionalidades implementadas, particularmente, mostrar a construção progressiva da grade de ocupação.
## Meta 03 (15 de julho - 21 de julho)
Implementar a navegação do robô no ambiente simulado, utilizando controlador implementado na primeira unidade e planejador de caminhos implementado na segunda unidade, e usando como mapa do ambiente a grade de ocupação obtida na meta 2. Executar testes para diferentes configurações de obstáculos. Entregar relatório e vídeo apresentando o funcionamento do sistema
