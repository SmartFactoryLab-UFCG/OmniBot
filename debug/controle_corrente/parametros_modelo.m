KT = 0.392;
La = 0.001;
J = 0.1;
Ra = 5.45;
F = 0.1;

data_ia = importdata('experiments_m3\rotor_travado_degraus_7v_2.txt')

data_wm = importdata("experiments\corrente_x_velocidade.txt");
data_ia = importdata("experiments\degrau_tensao_7V.txt");
ia = data_ia(350:end)-0.033;
ia_wm = data_wm(:,1);
wm = data_wm(:,2);

% Parâmetros da tensão
amplitude = 7; % Tensão máxima (12V)
limite_superior = 0.1; % Aciona 12V se Ia > 0.5A
limite_inferior = 0.008; % Aciona 0V se Ia < -0.5A

% Inicializa o vetor de tensão
Vout_wm = zeros(size(ia_wm));
Vout_ia = zeros(size(ia));

% Gera o sinal de tensão baseado na corrente wm
for i = 1:length(ia_wm)
    if ia_wm(i) > limite_superior
        Vout_wm(i) = amplitude;
    elseif ia_wm(i) < limite_inferior
        Vout_wm(i) = 0;
    else
        % Mantém o último estado se dentro da faixa morta
        if i > 1
            Vout_wm(i) = Vout_wm(i-1);
        end
    end
end

% Gera o sinal de tensão baseado na corrente
for i = 1:length(ia)
    if ia(i) > limite_superior
        Vout_ia(i) = amplitude;
    elseif ia(i) < limite_inferior
        Vout_ia(i) = 0;
    else
        % Mantém o último estado se dentro da faixa morta
        if i > 1
            Vout_ia(i) = Vout_ia(i-1);
        end
    end
end


% time vector
Ts = 0.0058;
tempo_iawm = (0:length(ia_wm)-1)'*Ts; 
tempo_wm = (0:length(wm)-1)'*Ts; 
tempo_va = (0:length(Vout)-1)'*Ts;
tempo_ia = (0:length(ia)-1)'*Ts;
