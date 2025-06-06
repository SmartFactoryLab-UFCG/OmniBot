w_data = importdata('experiments_m1\determinar_tau_m1.txt');
Ts = 0.00126;
tempo_wm = (0:length(w_data)-1)'*Ts;
plot(tempo_wm,w_data);

%% DADOS COLETADOS
taum_desacelerando = 0.402;
Va = 6.2;
Ia_vazio = 0.16;
w_vazio = 120.22 * (pi/30);
Ra = 3.87;

Ke = (Va - Ra*Ia_vazio)/w_vazio

B = (Ke*Ia_vazio)/w_vazio

J = taum_desacelerando * B

% Parametros encontrados M1
% Ra = 3.2;
% La = 10e-3;
% Ke = 0.4473;
% J = 0.0016;
% B = 0.0039;
% Ts = 0.01;

% Parametros encontrados M2
% Ra = 3.2;
% La = 10e-3;
% Ke = 0.4473;
% J = 0.0016;
% B = 0.0039;
% Ts = 0.01;

% Parametros encontrados M3
% Ra = 3.2;
% La = 10e-3;
% Ke = 0.4473;
% J = 0.0016;
% B = 0.0039;
% Ts = 0.01;