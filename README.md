Controle de Sustentação de Helicóptero

Estudo de modelagem, análise dinâmica, estimação de estados e projeto de sistemas de controle aplicado a uma dinâmica linear associada à sustentação e à estabilidade de um helicóptero.

Projeto acadêmico desenvolvido no contexto da disciplina Tópicos de Controle Avançado (TCA), do curso de Engenharia de Controle e Automação.

O repositório reúne implementações em Python, Scilab e MATLAB para o estudo de técnicas clássicas de controle moderno, com ênfase na representação em espaço de estados, alocação de polos, observadores de estados e análise de desempenho temporal.

⸻

Sumário

* 1. Sobre o projeto
* 2. Objetivos
* 3. Fundamentação teórica
* 4. Modelo matemático
* 5. Estratégias de controle
* 6. Estrutura do repositório
* 7. Tecnologias utilizadas
* 8. Instalação e execução
* 9. Análise dos resultados
* 10. Limitações e considerações
* 11. Possíveis melhorias
* 12. Contexto acadêmico

⸻

1. Sobre o projeto

O controle de sistemas aeronáuticos exige a análise da dinâmica da planta, a avaliação de distúrbios e a definição de estratégias capazes de produzir respostas estáveis e compatíveis com os requisitos de desempenho estabelecidos.

Neste trabalho, estuda-se um modelo linear de segunda ordem associado a uma dinâmica de controle de helicóptero. O problema envolve a análise dos estados do sistema, o projeto de um observador para estimação de variáveis internas e a aplicação de realimentação de estados para modificar a resposta dinâmica.

O desenvolvimento utiliza diferentes ambientes computacionais para representar o sistema, calcular ganhos de controle e realizar simulações temporais.

As implementações disponibilizadas permitem estudar os conceitos de controle moderno tanto pela formulação matemática quanto pela experimentação computacional.

Escopo: o modelo empregado representa uma dinâmica específica do problema estudado. Ele não deve ser interpretado, sem documentação adicional, como um modelo aerodinâmico completo de um helicóptero.

2. Objetivos

2.1. Objetivo geral

Estudar e aplicar técnicas de controle avançado em um sistema dinâmico linear associado à sustentação de um helicóptero, utilizando modelagem em espaço de estados, estimação de estados e realimentação de estados.

2.2. Objetivos específicos

* Representar a dinâmica do sistema por meio de matrizes de estado.
* Determinar os autovalores e avaliar a dinâmica do modelo original.
* Projetar um observador de estados com polos mais rápidos que os do sistema original.
* Comparar respostas dinâmicas em diferentes configurações.
* Projetar ganhos de realimentação de estados considerando requisitos de sobressinal e tempo de acomodação.
* Investigar os efeitos do posicionamento dos polos sobre o desempenho em malha fechada.
* Implementar e comparar soluções computacionais em Python, Scilab e MATLAB.

3. Fundamentação teórica

3.1. Representação em espaço de estados

Um sistema linear e invariante no tempo pode ser representado por:


$$
\dot{x}(t)=Ax(t)+Bu(t)
$$

$$
y(t)=Cx(t)+Du(t)
$$



Em que:

* $$(x(t))$$ é o vetor de estados;
* $$(u(t))$$ é o vetor de entradas;
* $$(y(t))$$ é o vetor de saídas;
* $$(A)$$ é a matriz de dinâmica do sistema;
* $$(B)$$ relaciona as entradas à dinâmica dos estados;
* $$(C)$$ relaciona os estados às saídas medidas;
* $$(D)$$ representa a transmissão direta entre entrada e saída.

Essa representação permite analisar a estabilidade, a controlabilidade e a observabilidade, além de possibilitar o projeto de controladores e observadores por alocação de polos.

3.2. Análise de estabilidade

A estabilidade assintótica do sistema linear contínuo pode ser avaliada pelos autovalores da matriz $$(A)$$.

Para um sistema linear invariante no tempo, a estabilidade assintótica requer que todos os autovalores tenham parte real estritamente negativa.

A posição dos polos também influencia características como velocidade de resposta, amortecimento e oscilações transitórias.

3.3. Observador de estados

Quando nem todos os estados podem ser medidos diretamente, pode-se utilizar um observador para estimá-los a partir das entradas conhecidas e das saídas disponíveis.

O observador de Luenberger é descrito por:

$$\dot{\hat{x}}=A\hat{x}+Bu+L(y-C\hat{x})$$


Em que:

* $$(\hat{x})$$ é o vetor de estados estimados;
* $$(L)$$ é a matriz de ganho do observador;
* $$(y-C\hat{x})$$ é o erro entre a saída medida e a saída estimada.

Definindo o erro de estimação como (e=x-\hat{x}), obtém-se, para o modelo nominal:

$$
\dot{e}=(A-LC)e
$$

Assim, o posicionamento dos autovalores de $$(A-LC)$$ determina a dinâmica do erro de estimação.

No trabalho, investiga-se o posicionamento dos polos do observador para torná-lo cinco vezes mais rápido, conforme o enunciado implementado no código.

3.4. Realimentação de estados

Uma estratégia de controle por realimentação de estados pode ser expressa por:

$$
u=-Kx+r
$$

Em que $$(K)$$ é a matriz de ganhos e $$(r)$$ representa uma referência ou entrada externa, conforme a estrutura adotada.

Substituindo a lei de controle no modelo da planta, obtém-se:

$$
\dot{x}=(A-BK)x+Br
$$

O posicionamento dos polos de $$(A-BK)$$ permite modificar a dinâmica de malha fechada.

Quando os estados são estimados por um observador, a lei de controle pode utilizar $$(\hat{x})$$ em lugar de $$(x)$$. Essa implementação requer a consideração conjunta do controlador e do observador.

3.5. Requisitos de desempenho

O projeto de controladores pode considerar indicadores como:

* Sobressinal: excedente máximo da resposta em relação ao valor final.
* Tempo de acomodação: tempo necessário para que a resposta permaneça dentro de uma faixa de tolerância em torno do valor final.
* Amortecimento: característica que influencia oscilações e sobressinal.
* Velocidade de resposta: rapidez com que o sistema acompanha a entrada de referência.

Esses indicadores devem ser avaliados sobre a resposta simulada, considerando a configuração efetivamente utilizada e o critério de acomodação adotado.

4. Modelo matemático

O código Python apresenta o seguinte modelo em espaço de estados:

$$
A=\begin{bmatrix}
0 & 1\
-81 & -54
\end{bmatrix}
$$

$$
B=\begin{bmatrix}
0\
1
\end{bmatrix}
$$

$$
C=\begin{bmatrix}
54 & 0{,}9
\end{bmatrix},
\qquad D=\begin{bmatrix}0\end{bmatrix}
$$

Portanto, o modelo é:

$$
\dot{x}=
\begin{bmatrix}
0 & 1;
-81 & -54
\end{bmatrix}x+
\begin{bmatrix}
0\
1
\end{bmatrix}u
$$

$$
y=\begin{bmatrix}54 & 0{,}9\end{bmatrix}x
$$

O vetor de estados é definido no enunciado do código em termos de variáveis associadas à velocidade do vento e à frequência da rajada.

A interpretação física completa, as unidades e a relação detalhada entre a entrada de acionamento e a saída de velocidade angular devem ser consultadas na formulação original do problema.

4.1. Polos do sistema original

Os polos são obtidos a partir dos autovalores de (A), ou, equivalentemente, das raízes de:

$$
\det(sI-A)=0
$$

Para a matriz apresentada, o polinômio característico é:

$$
s^2+54s+81=0
$$

A análise dos polos fornece a referência para o estudo da dinâmica original e para o posicionamento dos polos do observador e do sistema controlado.

5. Estratégias de controle

5.1. Projeto do observador

A implementação Python calcula os autovalores do sistema e define polos desejados para o observador multiplicando os polos originais por cinco.

A matriz de ganho é obtida por alocação de polos, utilizando a função place_poles do módulo scipy.signal sobre o sistema dual:

[
L=\operatorname{place}(A^T,C^T)^T
]

A matriz resultante permite construir a dinâmica do erro de estimação:

[
A_e=A-LC
]

A análise dessa matriz permite verificar se os polos efetivamente calculados correspondem aos requisitos estabelecidos.

5.2. Realimentação de estados

O trabalho também investiga o cálculo de ganhos de realimentação para diferentes requisitos de desempenho.

No código Python, são consideradas condições de projeto com sobressinal máximo de 10% e tempo de acomodação de referência de 1 segundo, além de uma configuração posterior com sobressinal de 5% e tempo de acomodação de referência de 0,5 segundo.

Os polos desejados são calculados a partir dos parâmetros de amortecimento e frequência natural, e os ganhos são obtidos por alocação de polos.

A comparação entre configurações permite investigar como mudanças nos requisitos influenciam os ganhos e a resposta de malha fechada.

Observação metodológica: embora o enunciado mencione a fórmula de Ackermann, a implementação Python apresentada utiliza place_poles. A documentação descreve o procedimento efetivamente implementado, sem afirmar que essa função executa explicitamente a fórmula de Ackermann.

5.3. Simulação e comparação

As simulações são utilizadas para comparar respostas temporais e avaliar os efeitos das alterações nos polos e nos ganhos.

Entre os aspectos de interesse estão:

* diferenças entre a dinâmica original e a dinâmica do observador;
* velocidade de convergência da estimação;
* alterações na resposta em malha fechada;
* atendimento aos requisitos de sobressinal e tempo de acomodação.

Os resultados quantitativos devem ser interpretados a partir das simulações executadas e dos critérios de desempenho utilizados.

6. Estrutura do repositório

A estrutura atualmente identificada é:

ControleSustentationHelicopter/
├── README.md
├── Seminário 1 TCA.py
├── Seminário 1 TCA.sci
└── TCAControSustentation.m

Descrição dos arquivos

Arquivo	Finalidade
README.md	Documentação do projeto, da modelagem e dos procedimentos de execução.
Seminário 1 TCA.py	Implementação em Python, com cálculo de polos, projeto de observador e simulações de resposta dinâmica.
Seminário 1 TCA.sci	Implementação em Scilab das atividades de controle.
TCAControSustentation.m	Implementação em MATLAB relacionada ao problema de controle.

Os três arquivos de código devem ser tratados como implementações distintas até que sua equivalência funcional e numérica seja verificada.

7. Tecnologias utilizadas

Python

A implementação Python utiliza as seguintes bibliotecas:

* NumPy: operações matriciais e cálculo numérico.
* SciPy: sistemas em espaço de estados, análise de respostas e alocação de polos.
* Matplotlib: construção e apresentação de gráficos.
* SciPy Integrate: ferramentas de integração numérica disponíveis para simulações de equações diferenciais.

Scilab

Ambiente de computação numérica utilizado para implementar e analisar sistemas de controle.

MATLAB

Ambiente de computação numérica e engenharia utilizado para modelagem, análise e simulação de sistemas dinâmicos.

8. Instalação e execução

8.1. Clonar o repositório

git clone https://github.com/coelhp/ControleSustentationHelicopter.git
cd ControleSustentationHelicopter

8.2. Executar a versão Python

Recomenda-se utilizar um ambiente virtual para isolar as dependências.

Criação do ambiente:

python -m venv .venv

No Windows:

.venv\Scripts\activate

No Linux ou macOS:

source .venv/bin/activate

Instalação das bibliotecas utilizadas:

python -m pip install numpy scipy matplotlib

Execução do arquivo:

python "Seminário 1 TCA.py"

A implementação utiliza gráficos do Matplotlib e pode abrir janelas para apresentar as respostas simuladas. A execução completa deve ser validada no ambiente local, pois eventuais dependências de interface gráfica e diferenças entre versões podem influenciar o comportamento.

8.3. Executar no Scilab

1. Instale o Scilab.
2. Abra o arquivo Seminário 1 TCA.sci no editor do ambiente.
3. Execute o script.
4. Verifique os resultados numéricos e os gráficos gerados.

8.4. Executar no MATLAB

1. Instale o MATLAB.
2. Abra o arquivo TCAControSustentation.m.
3. Configure o diretório de trabalho para a pasta do repositório.
4. Execute o script e analise as saídas e os gráficos.

A execução em MATLAB e Scilab pode depender de funções e recursos específicos de cada ambiente. Os comandos e as versões compatíveis devem ser confirmados por testes.

9. Análise dos resultados

A avaliação do trabalho deve relacionar os resultados numéricos aos requisitos de projeto.

Recomenda-se organizar a análise em três etapas:

1. Sistema original: determinar os polos e interpretar a dinâmica nominal.
2. Observador: verificar os polos de (A-LC), a dinâmica do erro de estimação e a resposta simulada.
3. Sistema realimentado: verificar os polos de (A-BK), o sobressinal e o tempo de acomodação em cada configuração.

Uma tabela comparativa pode ser utilizada para registrar os resultados:

Configuração	Polos de malha fechada	Sobressinal	Tempo de acomodação
Sistema original	A calcular	A medir	A medir
Sistema com observador	A calcular	A medir	A medir
Realimentação — configuração inicial	A calcular	A medir	A medir
Realimentação — configuração ajustada	A calcular	A medir	A medir

Os campos devem ser preenchidos com valores obtidos por execução dos códigos. Os requisitos de projeto não devem ser apresentados como resultados medidos sem a devida verificação.

10. Limitações e considerações

* O modelo descrito é linear e de segunda ordem, não representando necessariamente todos os fenômenos físicos de um helicóptero real.
* A análise depende das matrizes e das hipóteses fornecidas no problema acadêmico.
* O projeto por alocação de polos depende das propriedades de controlabilidade e observabilidade do modelo.
* A rapidez de um observador não garante, por si só, robustez a ruído de medição ou incertezas de modelagem.
* O desempenho do controlador deve ser validado pelas respostas simuladas, e não apenas pela posição dos polos desejados.
* As implementações em Python, Scilab e MATLAB precisam ser executadas e comparadas antes de se afirmar que apresentam resultados equivalentes.
* Os resultados apresentados neste repositório têm finalidade acadêmica e não constituem validação de um sistema aeronáutico operacional.

11. Possíveis melhorias

Como evolução do projeto, podem ser consideradas as seguintes atividades:

* padronizar a nomenclatura das variáveis e dos arquivos;
* separar a modelagem, o projeto do controlador e as simulações em funções;
* adicionar gráficos específicos para estados reais, estados estimados e erro de estimação;
* calcular automaticamente métricas de desempenho;
* comparar numericamente os resultados entre Python, Scilab e MATLAB;
* adicionar testes para verificar polos, dimensões matriciais e critérios de desempenho;
* documentar as versões dos ambientes e das bibliotecas;
* incluir figuras e tabelas com resultados efetivamente obtidos;
* disponibilizar os enunciados e as referências teóricas pertinentes, respeitando as condições de uso dos materiais originais.

12. Contexto acadêmico

Disciplina: Tópicos de Controle Avançado (TCA)
Curso: Engenharia de Controle e Automação
Tema: modelagem em espaço de estados, observadores e realimentação de estados
Natureza: projeto acadêmico com simulações computacionais

Repositório: github.com/coelhp/ControleSustentationHelicopter

⸻

Desenvolvido para fins de estudo e aplicação de conceitos de controle moderno em sistemas dinâmicos lineares.
