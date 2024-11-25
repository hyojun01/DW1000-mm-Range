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

%% AP 좌표 설정
AP_coords = [0, 0, 0;  % AP1
             1, 0, 0;  % AP2
             0, 1, 0;  % AP3
             0, 0, 1]; % AP4
         
%% 평면 방정식 설정 (AP 평면 계산)
% 법선 벡터 계산 (AP1, AP2, AP3 사용)
normal_vector = cross(AP_coords(2,:) - AP_coords(1,:), AP_coords(3,:) - AP_coords(1,:));
D = -dot(normal_vector, AP_coords(1,:)); % 평면 상수
plane = [normal_vector, D];

%% 타겟 좌표 계산 및 투영
targetProjected = zeros(3, numCycles); % 평면 투영 좌표 저장

for cycle = 1:numCycles
    % 거리 데이터 사용하여 타겟 3D 좌표 계산
    distances = distMatrix(:, cycle)';
    target3D = findTarget3D(AP_coords, distances); % 타겟 3D 좌표 계산
    
    % 타겟 좌표를 평면으로 투영
    projected = projectToPlane(target3D, plane);
    targetProjected(:, cycle) = projected;
end

disp('Target projection completed.');

%% 플롯
figure;
plot3(targetProjected(1,:), targetProjected(2,:), targetProjected(3,:), 'o');
grid on;
title('Projected Target Coordinates on Responder Plane');
xlabel('X');
ylabel('Y');
zlabel('Z');

%% 타겟 3D 좌표 계산 함수
function target = findTarget3D(AP_coords, distances)
    options = optimoptions('fsolve', 'Display', 'off');
    initial_guess = [0, 0, 0]; % 초기 추정값
    target = fsolve(@(pos) equations(pos, AP_coords, distances), initial_guess, options);
end

function F = equations(pos, AP_coords, distances)
    F = zeros(4, 1);
    for i = 1:4
        F(i) = sqrt((pos(1) - AP_coords(i, 1))^2 + ...
                    (pos(2) - AP_coords(i, 2))^2 + ...
                    (pos(3) - AP_coords(i, 3))^2) - distances(i);
    end
end

%% 평면 투영 함수
function projected = projectToPlane(target, plane)
    A = plane(1); B = plane(2); C = plane(3); D = plane(4);
    d_proj = (A*target(1) + B*target(2) + C*target(3) + D) / sqrt(A^2 + B^2 + C^2);
    projected = target - d_proj * [A, B, C] / sqrt(A^2 + B^2 + C^2);
end
