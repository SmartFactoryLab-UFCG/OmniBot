data = importdata("experiments_m3\rotor_sem_roda_degraus_7v_1_exc.txt");
ia_wm = data(:,1);
wm = data(:,2) * 75;

% wm no load = 168 rpm

plot(wm)

Kt = 0.392; % N.m/A
ce = ia_wm * Kt;

Ts = 0.0058;
fs = 1/Ts;
n = length(ia);
f = fs * (0:(n/2)) / n;
Y = fft(ia);
P = abs(Y / n).^2;

figure(2)
plot(f, P(1:n/2+1));
xlabel('Frequência (Hz)');
ylabel('Potência');
title('Espectro de Frequência do Sinal');

fc = 3; % Frequência de corte (ajuste conforme a dinâmica do motor)
[b, a] = butter(4, fc / (fs / 2), 'low'); % Filtro Butterworth de 4ª ordem
filteredCurrent = filtfilt(b, a, ia); % Filtragem bidirecional (sem atraso)

tempo = (0:length(ia)-1)*Ts;

figure(3)
plot(tempo, ia, 'b', tempo, filteredCurrent, 'r', 'LineWidth', 1.0);
xlabel('Tempo (s)');
ylabel('Corrente (A)');
legend('Sinal Bruto', 'Sinal Filtrado');
grid on;

%% Tensão
% Parâmetros do sistema
amplitude = 12; % Tensão máxima (12V)
limite_superior = 0.5; % Aciona 12V se Ia > 0.5A
limite_inferior = -0.5; % Aciona 0V se Ia < -0.5A

% Inicializa o vetor de tensão
Vout_wm = zeros(size(ia_wm));

% Gera o sinal de tensão baseado na corrente
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