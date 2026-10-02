/*
Seminário TCA
Sistemas de controle de sustentação em helicópteros tem por objetivo manter o direcionamento da aeronave frente a presença de distúrbios,
tendo como entrada o nível de acionamento do pedal (NP) e a saída a velocidade angular no rotor (Wr).
Considerando os estados velocidade do vento (Vv) e frequência da rajada (fv), 𝑥=[𝑉𝑣; 𝑓𝑣], uma parte do movimento específico desse controle possui a dinâmica dada por,
𝐴=[0 1; −81 −54], 𝐵=[0; 1], 𝐶=[54 9/10]
*/
A = [0 1; -81 -54];
B = [0; 1];
C = [54 9/10];
D = 0;

MOB = [C; C*A]
det(MOB)
rank(MOB)
s = %s
PA = det(s*eye(2,2)-A)
polos = roots(PA)
p1 = polos(1);
p2 = polos(2);
polos


/*Resolução letra A: 
Projete um observador de estados capaz de emular o sistema 5x mais rápido. 
Demonstre, de forma comparativa entre o sistema original e o observador, o comportamento de cada estado;
*/

PD = (s - (5*real(p1)+ %i*imag(p1)))*(s - (5*real(p2)+ %i*imag(p2)))
polos_desejados = roots(PD)
/* Aumentando a frequência visando diminuir o período
Multiplicando os polos por 5*/

coef_pd = coeff(PD);
coef_pd

/*Separa os coeficientes do polinômio em um vetor
No polinômio em questão são */

L_1 = (A^2 + coef_pd(2)*A + coef_pd(1)*eye(2,2))* inv(MOB)*[0;1]

/*eye() cria matriz identidade nas dimensões x e y que definirmos
Exemplo (2,2) 
1 0
0 1
*/

AOB = A-L_1*C
GMA = C*inv(s*eye(2,2)-A)*B
GOB = C*inv(s*eye(2,2)-AOB)*B
Xi = [0 0]';
t = 0:0.001:8
sGMA = syslin('c', A, B, C, 0, Xi);
sGOB = syslin('c', AOB, B, C, 0, Xi);
[yMA xMA] = csim('step', t, sGMA, Xi);//Capturando as resposntas ao degrau de GMA
[yOB xOB] = csim('step', t, sGOB, Xi);//Capturando as respostas ao degrau de GOB

/*
Plotando os gráficos do sistema em malha aberta e do observador
*/
plot2d(t', yMA, style = [color("blue")], leg = "Malha Aberta");
mtlb_hold("on");
plot2d(t', yOB, style = [color("red")], leg = "Observador");
figure();

//Comportamento dos Estados

plot2d(t', xMA(1,:), style = [color("blue")], leg = "Malha Aberta");
mtlb_hold("on");
plot2d(t', xOB(1,:), style = [color("red")], leg = "Observador");
figure();

//Comportamento dos Estados_2

plot2d(t', xMA(2,:), style = [color("blue")], leg = "Malha Aberta");
mtlb_hold("on");
plot2d(t', xOB(2,:), style = [color("red")], leg = "Observador");

//Letra B

/*
Através da fórmula de Ackermann, obtenha o vetor de ganhos K capaz de fazer o tempo de acomodação (critério de 2%) ser o mais rápido possível,
com sobressinal limitado a 10%, tanto para o sistema original quando para sistema do observador;
*/

// 1. Parâmetros iniciais (Mp = 0.10, Ts = 1.0s)
Mp_inicial = 0.10;  // Sobressinal máximo de 10%
zeta_inicial = -log(Mp_inicial) / sqrt(%pi^2 + log(Mp_inicial)^2);  // Fator de amortecimento associado a Mp

Ts_inicial = 1.0;  // Tempo de acomodação desejado em segundos
wn_inicial = 4 / (zeta_inicial * Ts_inicial);  // Frequência natural associada

p1_inicial = -zeta_inicial * wn_inicial + %i * wn_inicial * sqrt(1 - zeta_inicial^2);
p2_inicial = -zeta_inicial * wn_inicial - %i * wn_inicial * sqrt(1 - zeta_inicial^2);
polos_desejados_inicial = [p1_inicial, p2_inicial];

// Aplicando Ackermann para obter o K correspondente
K_original_inicial = ppol(A, B, polos_desejados_inicial);

// Para o sistema com observador
L_1_inicial = ppol(A', C', 5 * polos_desejados_inicial)';
AOB_inicial = A - L_1_inicial * C;
K_OB_inicial = ppol(AOB_inicial, B, polos_desejados_inicial);

// Ganhos Novos para os parâmetros (Mp = 0.05, Ts = 0.5s)

// Ajustando o fator de amortecimento para reduzir o Mp
Mp_new = 0.05;  // Reduzir o Mp para 5%
zeta_new = -log(Mp_new) / sqrt(%pi^2 + log(Mp_new)^2);

// Ajustando a frequência natural para reduzir o Ts
Ts_new = 0.5;  // Tempo de acomodação menor
wn_new = 4 / (zeta_new * Ts_new);

// Recalculando os polos desejados
p1_new = -zeta_new * wn_new + %i * wn_new * sqrt(1 - zeta_new^2);
p2_new = -zeta_new * wn_new - %i * wn_new * sqrt(1 - zeta_new^2);
polos_desejados_new = [p1_new, p2_new];

// Aplicação da Fórmula de Ackermann para o sistema original com novos parâmetros
K_original_new = ppol(A, B, polos_desejados_new);

// Aplicação da Fórmula de Ackermann para o sistema com observador
L_1_new = ppol(A', C', 5 * polos_desejados_new)';
A_ob_new = A - L_1_new * C;
K_ob_new = ppol(A_ob_new, B, polos_desejados_new);

/*
Simule o sistema (original e do observador) antes e depois da aplicação do controle, 
excitando-o com um degrau unitário. Obs: 𝐺𝑀𝐴=𝐶(𝑠𝐼−𝐴)−1𝐵 , 𝐺𝑀𝐹=𝐶(𝑠𝐼−𝐻)−1𝐵; onde 𝐻=𝐴−𝐵𝐾;
*/

//Letra C

//Sistema Original
contr(A,B);
MC = [B A*B];
MCi = inv(MC);
PDd = (s - (real(p1_inicial)+ %i*imag(p1_inicial)))*(s - (real(p2_inicial)+ %i*imag(p2_inicial)))
PDa = A^2 + 8*A + 45.784365*eye(2,2)
K = [0 1]*MCi*PDa
Xi = [0;0]
t = 0:0.01:10;
sGMA = syslin('c',A,B,C,0,Xi)
AMF = (A - B*K)
SMF = syslin('c',AMF, B, C, 0, Xi);

[yMA xMA] = csim('step', t, sGMA, Xi);
[yMF xMF] = csim('step', t, SMF, Xi);

plot2d(t', yMA, style = [color("blue")], leg = "Sistema Original")
mtlb_hold("on");
plot2d(t', yMF, style = [color("red")], leg = "Sistema com ganho")

figure();

plot2d(t, xMA(1,:), style = [color("blue")], leg = "Sistema Original")
mtlb_hold("on");
plot2d(t, xMF(1,:), style = [color("red")], leg = "Sistema com ganho")

figure();

plot2d(t, xMA(2,:), style = [color("blue")], leg = "Sistema Original")
mtlb_hold("on");
plot2d(t, xMF(2,:), style = [color("red")], leg = "Sistema com ganho")

figure();

//Sistema do Observador
contr(AOB,B);
MC_OB = [B AOB*B];
MCi_OB = inv(MC_OB);
PDd_OB = (s - (real(p1_inicial)+ %i*imag(p1_inicial)))*(s - (real(p2_inicial)+ %i*imag(p2_inicial)))
PDA_OB = A^2 + 270*A + 2025*eye(2,2)
K_OB = [0 1]*MCi_OB*PDA_OB
sGMA_OB = syslin('c',AOB,B,C,0,Xi);
AMF_OB = AOB - B*K_OB;
sMF_OB = syslin('c',AMF_OB,B,C,0,Xi);

[yMa xMa] = csim('step', t, sGMA_OB, Xi);
[yMF xMF] = csim('step', t, sMF_OB, Xi);

plot2d(t', yMA, style = [color("blue")], leg = "Sistema Original")
mtlb_hold("on");
plot2d(t', yMF, style = [color("red")], leg = "Sistema com ganho")

figure();

plot2d(t, xMA(1,:), style = [color("blue")], leg = "Sistema Original")
mtlb_hold("on");
plot2d(t, xMF(1,:), style = [color("red")], leg = "Sistema com ganho")

figure();

plot2d(t, xMA(2,:), style = [color("blue")], leg = "Sistema Original")
mtlb_hold("on");
plot2d(t, xMF(2,:), style = [color("red")], leg = "Sistema com ganho")

figure();

/*
Obtenha os índices de desempenho no transiente e o erro de regime para o sistema antes e depois da aplicação do controle.
*/

//Letra D
function [peak_time, overshoot, rise_Time, settling_Time] = my_step_info(t, curva)
    [val_max, ind_max] = max(curva);        // Encontra o valor máximo e o índice correspondente
    val_final = curva(length(curva));       // Encontra o valor final da curva
    peak_time = t(ind_max);                 // Tempo de pico
    overshoot = (val_max / val_final - 1) * 100;  // Cálculo do overshoot

    // Inicializa i_10 e i_90
    i_10 = 1;  // Default caso não encontre 10%
    i_90 = length(curva);  // Default para 90% como último índice

    // Encontra o índice onde a curva atinge 10% do valor final
    for i = 1:length(curva) - 1
        if (curva(i) > val_final * 0.10)
            i_10 = i;
            break;
        end
    end

    // Encontra o índice onde a curva atinge 90% do valor final
    for i = 1:length(curva) - 1
        if (curva(i) > val_final * 0.90)
            i_90 = i;
            break;
        end
    end

    // Cálculo do rise time
    rise_Time = t(i_90) - t(i_10);

    // Encontra o tempo de acomodação (settling time)
    for i = (length(curva) - 1):-1:1
        if (abs(curva(i) / val_final) > 1.02) | (abs(curva(i) / val_final) < 0.98)
            qts = i;
            break;
        end
    end

    settling_Time = t(qts);
endfunction

[yMA xMA] = csim('step', t, sGMA, Xi);
[yMF xMF] = csim('step', t, SMF, Xi);

[peak_time, overshoot, rise_Time, settling_Time]=my_step_info(t,csim('step', t, sGMA, Xi))
[peak_time, overshoot, rise_Time, settling_Time]=my_step_info(t,csim('step', t, SMF, Xi))

[yMA_OB xMA_OB] = csim('step', t, sGMA_OB, Xi);
[yMF_OB xMF_OB] = csim('step', t, sMF_OB, Xi);

[peak_time, overshoot, rise_Time, settling_Time]=my_step_info(t,csim('step', t, sGMA_OB, Xi))
[peak_time, overshoot, rise_Time, settling_Time]=my_step_info(t,csim('step', t, sMF_OB, Xi))

/*
Para o sistema de controle original, adote a transformação P diagonal, considerando o estado inicial 𝑥0𝑇=[0 1],
obtenha 𝐽𝑚𝑖𝑛 e os novos ganhos 𝐾=[𝑘 0,5𝑘], para os casos: 𝜆=0.2 e 𝜆=1.4; Observe que, com 𝐻=𝐴−𝐵𝐾, apenas soluções onde k>0 serão estáveis.
*/

//Letra E
//Cálculo das matrizes e do termo J

// 1. Transformação Diagonal P
[A_eigvecs, A_eigvals] = bdiag(A);  // A_eigvecs são os autovetores, A_eigvals são os autovalores
P = A_eigvecs;  // Matriz P de autovetores
P_inv = inv(P);
Lambda = diag(A_eigvals);  // Matriz diagonal dos autovalores
x0 = [0; 1];

// 2. Cálculo de J_min
J_min = x0' * P * x0;

// 3. Cálculo dos Novos Ganhos K para λ = 0.2 e λ = 1.4
lambdas = [0.2, 1.4];
K_values = [];

for lamb = lambdas
    k = lamb;
    K = [k, 0.5 * k];
    H = A - B * K;
    
    // Garantindo que a solução é estável (k > 0 e autovalores negativos)
    eig_H = spec(H);
    if sum(real(eig_H) >= 0) == 0 then
        K_values = [K_values; lamb, K];
    end
end

// Exibindo os resultados
printf("J_min = %.4f\n", J_min);

for i = 1:size(K_values, 1)
    lamb = K_values(i, 1);
    K = K_values(i, 2:3);
    printf("\nλ = %.2f: K = [%.4f, %.4f]\n", lamb, K(1), K(2));
end
/*
Considerando a entrada 𝑢=−𝐾𝑥, determine seu valor para o início da operação em cada caso estável de e);
*/

//Letra F

K1 = [0.2000, 0.1000]
K2 = [1.4000, 0.7000]
X = [0;1]
U0_K1 = -K1*X
U0_K2 = -K2*X

/*
Qual o fator de amortecimento e frequência natural em cada caso estável de e)?
Aplique excitação do tipo degrau nos dois casos e discuta o comportamento dinâmico do sistema em cada caso
*/

//Letra G

AMF_1 = A - B*K1;
sMF1 = syslin('c', AMF_1,B,C,0,Xi);
AMF2 = A - B*K2;
sMF2 = syslin('c',AMF2,B,C,0,Xi);

[yMF1 xMF1] = csim('step', t, sMF1, Xi);
[yMF2 xMF2] = csim('step', t, sMF2, Xi);

plot2d(t', yMF1, style = [color("blue")], leg = "Sistema com o primeiro ganho")
mtlb_hold("on");
plot2d(t', yMF2, style = [color("red")], leg = "Sistema com o segundo ganho")

figure();

plot2d(t, xMF1(1,:), style = [color("blue")], leg = "Sistema com o primeiro ganho")
mtlb_hold("on");
plot2d(t, xMF2(1,:), style = [color("red")], leg = "Sistema com o segundo ganho")

figure();

plot2d(t, xMF1(2,:), style = [color("blue")], leg = "Sistema com o primeiro ganho")
mtlb_hold("on");
plot2d(t, xMF2(2,:), style = [color("red")], leg = "Sistema com o segundo ganho")

figure();

[peak_time, overshoot, rise_Time, settling_Time]=my_step_info(t,csim('step', t, sMF1, Xi))
[peak_time, overshoot, rise_Time, settling_Time]=my_step_info(t,csim('step', t, sMF2, Xi))
