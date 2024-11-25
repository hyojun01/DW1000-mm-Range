clear all;

%% 아두이노 연결된 포트 목록 (환경에 맞게 변경)
portNames = {'COM3', 'COM18', 'COM5', 'COM4'}; % 각 아두이노의 시리얼 포트 이름
baudRate = 1000000; % 아두이노의 Baud Rate
numCycles = 1000; % 수집할 사이클 수
numArduinos = numel(portNames); % 아두이노 개수
serialObjects = []; % 시리얼 객체를 저장할 배열

%% 각 아두이노의 데이터를 저장할 배열 초기화
phaseMatrix = zeros(numArduinos, numCycles);
distTwrMatrix = zeros(numArduinos, numCycles);
distMatrix = zeros(numArduinos, numCycles);

%% Wave Configuration
c = 3 * 10e8;
freq_CH3 = 2 * (4492.8 * 10e6); % CH3 중심 주파수 사용, 이론식에서 중심 주파수가 2배되는 효과 적용
lambda = c / freq_CH3;

%% 시리얼 포트 열기
for i = 1:numArduinos
    serialObjects{i} = serialport(portNames{i}, baudRate);
    configureTerminator(serialObjects{i}, "LF"); % Terminator 설정
    flush(serialObjects{i}); % 초기 버퍼 비우기
    disp(['Opened serial port: ', portNames{i}]);
end

disp('Start receiving data...');

%% 데이터 수집
for cycle = 1:numCycles
    for i = 1:numArduinos
        while serialObjects{i}.NumBytesAvailable == 0
            pause(0.0001);
        end
        rawData = readline(serialObjects{i}); % 시리얼 통신으로 데이터 읽기
        data = split(rawData, '|'); % '|'를 기준으로 문자열 나누기
        phaseMatrix(i,cycle) = str2double(data{1});
        distTwrMatrix(i,cycle) = str2double(data{2});
    end
    fprintf('Cycle %d/%d completed\n', cycle, numCycles);
end

disp('Data collection complete.');

%% 시리얼 포트 닫기
for i = 1:numArduinos
    clear serialObjects{i};
    disp(['Closed serial port: ', portNames{i}]);
end

%% 거리 계산(후자 방식)
for i = 1:numArduinos
    N = floor(distTwrMatrix(i,:) / lambda);
    distMatrix(i,:) = ((phaseMatrix(i,:) / (2 * pi)) + N) * lambda;
    distMatrix(i,:) = medfilt1(distMatrix(i,:), 40);
end

%% AP의 좌표 (4개의 AP 중 하나를 원점으로 설정)
apCoords = [0, 0.42;  % Responder05
            0, 0;  % Responder06 (원점)
            0.42, 0.42;  % Responder07
            0.42, 0]; % Responder08

z = 1.2; % 두 평면 사이의 거리 (알고 있는 값)

%% 타겟의 좌표 계산
targetCoords = zeros(2, numCycles); % (x, y) 좌표 저장

for cycle = 1:numCycles
    % 원래 측정된 거리 데이터
    d1 = distMatrix(1, cycle);
    d2 = distMatrix(2, cycle);
    d3 = distMatrix(3, cycle);
    d4 = distMatrix(4, cycle);

    % 피타고라스 정리를 이용해 평면 상으로 투사된 거리 계산
    d1_proj = sqrt(d1^2 - z^2);
    d2_proj = sqrt(d2^2 - z^2);
    d3_proj = sqrt(d3^2 - z^2);
    d4_proj = sqrt(d4^2 - z^2);

    % AP 좌표
    x1 = apCoords(1, 1); y1 = apCoords(1, 2);
    x2 = apCoords(2, 1); y2 = apCoords(2, 2);
    x3 = apCoords(3, 1); y3 = apCoords(3, 2);
    x4 = apCoords(4, 1); y4 = apCoords(4, 2);

    % 방정식 구성
    A = [2*(x2-x1), 2*(y2-y1);
         2*(x3-x1), 2*(y3-y1);
         2*(x4-x1), 2*(y4-y1)];
    b = [d1_proj^2 - d2_proj^2 - x1^2 + x2^2 - y1^2 + y2^2;
         d1_proj^2 - d3_proj^2 - x1^2 + x3^2 - y1^2 + y3^2;
         d1_proj^2 - d4_proj^2 - x1^2 + x4^2 - y1^2 + y4^2];

    % 최소자승법으로 좌표 계산
    coord = pinv(A) * b;
    targetCoords(:, cycle) = coord;
end

%% 좌표 플롯
figure;
plot(targetCoords(1, :), targetCoords(2, :), 'b.');
title('Target Position Projected onto 2D Plane');
xlabel('X [m]');
ylabel('Y [m]');
grid on;
