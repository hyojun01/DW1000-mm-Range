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
    disp(['Closed serial prot: ', portNames{i}]);
end

%% 거리 계산(후자 방식)
for i = 1:numArduinos
    N = floor(distTwrMatrix(i,:) / lambda);
    distMatrix(i,:) = ((phaseMatrix(i,:) / (2 * pi)) + N) * lambda;
    distMatrix(i,:) = medfilt1(distMatrix(i,:), 40);
end

%% 수신한 세 개의 위상값 플롯
figure;

subplot(4, 1, 1);
plot(distMatrix(1,:));
title('Distance Responder1');
xlabel('Sample Number');
ylabel('Distance[m]');
ylim([0.8 1.2]);

subplot(4, 1, 2);
plot(distMatrix(2,:));
title('Distance Responder2');
xlabel('Sample Number');
ylabel('Distance[m]');
ylim([0.8 1.2]);

subplot(4, 1, 3);
plot(distMatrix(3,:));
title('Distance Responder3');
xlabel('Sample Number');
ylabel('Distance[m]');
ylim([0.8 1.2]);

subplot(4, 1, 4);
plot(distMatrix(4,:));
title('Distance Responder4');
xlabel('Sample Number');
ylabel('Distance[m]');
ylim([0.8 1.2]);