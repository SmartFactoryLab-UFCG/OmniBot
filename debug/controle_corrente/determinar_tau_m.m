data = importdata('teste_tauw.txt');
i = data(:,1);
w = data(:,2);
 
i_noload = 0.11; %A

% Ea = Ke*w
% Te = Kt*ia

w_data = importdata('experiments_m2\determinar_tau_m.txt');
Ts = 0.00126;
tempo_wm = (0:length(w_data)-1)'*Ts;
plot(tempo_wm,w_data);

%% DADOS COLETADOS
taum_acelerando = 0.044;
taum_desacelerando = 0.419;
Va = 6;
Ia_vazio = 0.11;
w_vazio = 120.8 * (pi/30);
Ra = 3.1;

Ke = (Va - Ra*Ia_vazio)/w_vazio;

B = (Ke*Ia_vazio)/w_vazio;

J = taum_desacelerando * B;

% Parametros encontrados
% Ra = 3.2;
% La = 10e-3;
% Ke = 0.4473;
% J = 0.0016;
% B = 0.0039;
% Ts = 0.01;
