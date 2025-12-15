%% SEÑALES DIGITALES
% JORGE ARMANDO CHARLES MICHELLE
% ANDRES ALBERTO ORTEGA REYES

%% Configuración Inicial
clear all;
close all hidden;
clc;

% Inicialización del robotcito
Motores = arduino('COM7', 'Uno', 'Libraries', 'Servo');
ServoX = servo(Motores, 'D9');
ServoY = servo(Motores, 'D10');
posX = 0.5; % 90°
posY = 0.5;
writePosition(ServoX, posX);
writePosition(ServoY, posY);

%% CONTROL PID
step = 1;
Kp = (0.0003)/3;
Ki =  0.0000;
Kd = (0.0009)/3;
prev_eX = 0;
prev_eY = 0;
IntegralX = 0;
IntegralY = 0;

%% Almacenar errores
errX = [];
errY = [];
time = [];

%% Inicialización de la cámara
cam = webcam("FHD Camera");
% Relación de 10.67 horizontal y 6 en vertical pixeles por grado para 1920x1080
cam.Resolution = '640x480';% Relación de 3.56 horizontal y 2.67 en vertical pixeles por grado
pause(2);

%Captura de imagen para seleccionar el color
frame = snapshot(cam);
frame = im2double(frame);
imshow(frame);

%Aplicar ruido
[m,n,o] = size(frame);

%Color
title('Selecciona la región con el color a rastrear');
roi = roipoly; % Selecciona el color

R = frame(:,:,1).*roi;
G = frame(:,:,2).*roi;
B = frame(:,:,3).*roi;
ref(:,:,1) = sum(R(:))/sum(roi(:));
ref(:,:,2) = sum(G(:))/sum(roi(:));
ref(:,:,3) = sum(B(:))/sum(roi(:));
[h, s, v] = rgb2hsv(ref);

umbral = 40/255;

%Filtro
ventana = zeros(m,n);%,o
sig = 1;
for i=1:m
    dx = (i-m/2)/(m/2);
    for j=1:n
        dy = (j-n/2)/(n/2);
        dxy = sqrt(dx^2 + dy^2);
        if dxy<sig
            ventana(i,j) = .5*(cos((pi*dxy)/(sig)) + 1);
        end
    end
end

% Bucle en tiempo real
i=1;
while true
    % Captura frame
    img = snapshot(cam);
    img = im2double(img);
    %Aplicación del ruido
    img1 = img+.2 * randn(m,n,o);
    
    %Transformada de Fourier
    ImgF1 = fftshift(fft2(img1(:,:,1)));
    ImgF2 = fftshift(fft2(img1(:,:,2)));
    ImgF3 = fftshift(fft2(img1(:,:,3)));
    
    %Aplicación del filtro en Fourier
    SalidaF1 = ImgF1.*ventana;
    SalidaF2 = ImgF2.*ventana;
    SalidaF3 = ImgF3.*ventana;

    %Transformada Inversa de Fourier
    Salida1 = abs(ifft2(ifftshift(SalidaF1)));
    Salida2 = abs(ifft2(ifftshift(SalidaF2)));
    Salida3 = abs(ifft2(ifftshift(SalidaF3)));
    
    %Concatenación de los 3 canales para hacer una sola imagen
    ImgSalida = cat(3, Salida1, Salida2, Salida3);

    % Conversión a HSV
    [H, S, V] = rgb2hsv(img);
    
    % Detección de color
    buscH = H > h - umbral & H < h + umbral;
    buscS = S > s - umbral & S < s + umbral;
    buscV = V > v - umbral & V < v + umbral;
    busq = buscH & buscS & buscV;

    % Cálculo del centroide
    [cenx, ceny] = find(busq == 1);
    x = sum(cenx) / sum(busq(:));
    y = sum(ceny) / sum(busq(:));

    Mostrar resultados
    subplot(131);
    imshow(img);
    title('Imagen original')

    subplot(132);
    imshow(img1);
    title('Imagen con ruido');

    subplot(133);
    imshow(busq);
    title('Mascara');

    hold on;
    plot(320, 240, 'x', 'MarkerSize', 5);
    if ~isnan(x)
        plot(y, x, 'xr', 'MarkerSize', 10, 'LineWidth', 2);
    end
    hold off;
    
    %ERROR
    errorX = (m/2) - y;
    errorY = (n/2) - x;
    
    diferencia = sprintf('dx = %d pixeles, dy = %d pixeles', errorX, errorY);
    if exist('hTexto', 'var') && isvalid(hTexto)
        delete(hTexto)
    end
    hTexto = annotation('textbox', [0.4, 0.01, 0.2, 0.05], 'String', diferencia, ...
        'FitBoxToText', 'on', 'EdgeColor', 'none', 'HorizontalAlignment', 'center', ...
        'FontSize', 10, 'Color', 'blue');
    clear X H S V mask rows cols;

    if abs(errorX) < 10
        errorX = 0;
    end
    if abs(errorY) < 10
        errorY = 0;
    end

    %Acción de Control
    if ~isnan(x)
        IntegralX = IntegralX + errorX;
        IntegralY = IntegralY + errorY;

        DerivadaX = errorX - prev_eX;
        DerivadaY = errorY - prev_eY;

        accionX = Kp * errorX + Ki * IntegralX + Kd * DerivadaX;
        accionY = Kp * errorY + Ki * IntegralY + Kd * DerivadaY;

        targetX = posX+accionX;
        targetY = posY+accionY;

        targetX = max(0, min(1, targetX));
        targetY = max(0, min(1, targetY));
        
        %% SEGUIMIENTO DE TRAYECTORIA
        
        for i=1:step
            posX = posX + (targetX - posX) * (i/step);
            posY = posY + (targetY - posY) * (i/step);
            writePosition(ServoX, posX);
            writePosition(ServoY, posY);
            posX = targetX;
            posY = targetY;
            prev_eX = errorX;
            prev_eY = errorY;
        end

        errX(end+1) = errorX;
        errY(end+1) = errorY;
        time(end+1) = i;
    end

    if(get(gcf,'CurrentCharacter') == 'f')
        break;
    end
    i=i+1;
end

% Errores al final
figure;
plot(time, errX, '-r');
hold on;
plot(time, errY, '-b');
xlabel('Tiempo (s)'); ylabel('Error en píxeles (px)');
legend('Error en x', 'Error en y');
title('Error en eje X y eje Y'); grid on;