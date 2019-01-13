%data paths
clear all
testcase = 'runt_utan_pelare'
speed = 'fast'
fileNav = 'dead_navigation.txt'
fileDeadReck ='dead_reckoning.txt'
fileGyroReck ='dead_reckoning_gyro.txt'
gmapFile = strcat('gmapping','.csv')   
hectFile = strcat('hector','.csv')
%testCasePath= 'korridor'
%below this should not need to be modified.
gmapPath = 'csv/gmapping'   
hectPath = 'csv/hector'
testCasePathGmap= fullfile(testcase,speed,gmapPath)
testCasePathHector = fullfile(testcase,speed,hectPath)
fullTestCasePath = fullfile(testcase,speed)
fullvdtPath = fullfile(fullTestCasePath)
gmapPath = fullfile(testCasePathGmap)
hectorPath = fullfile(testCasePathHector)

% import the data from vdt_files and slam runs in ROS
[deadReckX,deadReckY,deadReckTH,deadReckTime] = importDeadReck(fullvdtPath,fileDeadReck,2, 1192);
[deadReckGyroX,deadReckGyroY,deadReckGyroTH,deadReckGyroTime] = importDeadReck(fullvdtPath,fileGyroReck,2, 2380);
[navigationX,navigationY,navigationTH,navigationTime] = importDeadReck(fullvdtPath,fileNav,2, 1190);
[xHector,yHector,thHector,timeHector] = importTfEcho(hectorPath,hectFile, 1, 10);
[xGmap,yGmap,thGmap,timeGmap] = importTfEcho(gmapPath,gmapFile, 1, 10);

%%
rotateGmap = -pi/2 -thGmap(1) - pi/2;
rotateHect = -pi/2   - pi/2;

% convert to vectors for easier hnadling and remove duplicate rows
deadReck =  unique([deadReckTime deadReckX deadReckY deadReckTH],'rows');
deadGyro = unique([deadReckGyroTime deadReckGyroX deadReckGyroY deadReckGyroTH],'rows');
navigation = unique([navigationTime navigationX navigationY navigationTH],'rows');
hectorData = unique([timeHector xHector yHector thHector],'rows');
gmapData = unique([timeGmap xGmap yGmap thGmap],'rows');

% set all datasets to start at zero
deadReck = setStartToZero(deadReck);
deadGyro = setStartToZero(deadGyro);
navigation = setStartToZero(navigation);
hectorData = setStartToZero(hectorData);
gmapData = setStartToZero(gmapData);

%remove outliers from SLAM data
hectorData(isoutlier(hectorData(:,2),'movmedian',10),:)= [];
gmapData(isoutlier(gmapData(:,2),'movmedian',10),:)= [];

%rotate slam runs, hector and gmapping should follow same syntax
R = [cos(rotateHect) -sin(rotateHect); sin(rotateHect) cos(rotateHect)];
hectorData(:,2:3)=(R*[hectorData(:,2) hectorData(:,3)]')';
 
R = [cos(rotateGmap) -sin(rotateGmap); sin(rotateGmap) cos(rotateGmap)];
gmapData(:,2:3)=(R*[gmapData(:,2) gmapData(:,3)]')';


timenav = navigation(1,1)-navigation(end,1)
timedead = deadReck(1,1)-deadReck(end,1)
timegyro = deadGyro(1,1)-deadGyro(end,1)
timehector = hectorData(1,1)-hectorData(end,1)
timegmap = gmapData(1,1)-gmapData(end,1)

% navigation(navigation(:, 1)< hectorData(1,1), :)= [];
% navigation(navigation(:, 1)> hectorData(end,1), :)= [];
% navigation(navigation(:, 1)< gmapData(1,1), :)= [];
% navigation(navigation(:, 1)> gmapData(end,1), :)= [];

% prune the lengths of all data so that the times match up
% navigation(navigation(:, 1)< deadReck(1,1), :)= [];
% navigation(navigation(:, 1)> deadReck(end,1), :)= [];
% navigation(navigation(:, 1)< deadGyro(1,1), :)= [];
% navigation(navigation(:, 1)> deadGyro(end,1), :)= [];
% 
% deadReck(deadReck(:, 1)< navigation(1,1), :)= [];
% deadGyro(deadGyro(:, 1)< navigation(1,1), :)= [];
% deadReck(deadReck(:, 1)> navigation(end,1), :)= [];
% deadGyro(deadGyro(:, 1)> navigation(end,1), :)= [];

% prune length w.r.t start time 
if navigation(1,1) >=  max([deadReck(1,1) deadGyro(1,1) hectorData(1,1) gmapData(1,1)])
    deadReck(deadReck(:, 1)< navigation(1,1), :)= [];
    deadGyro(deadGyro(:, 1)< navigation(1,1), :)= [];
    hectorData(hectorData(:,1) < navigation(1,1), :) = [];
    gmapData(gmapData(:,1) < navigation(1,1), :) = [];
    
elseif deadReck(1,1) >= max([navigation(1,1) deadGyro(1,1) hectorData(1,1) gmapData(1,1)])
    navigation(navigation(:, 1)< deadReck(1,1), :)= [];
    deadGyro(deadGyro(:, 1)< deadReck(1,1), :)= [];
    hectorData(hectorData(:,1) < deadReck(1,1), :) = [];
    gmapData(gmapData(:,1) < deadReck(1,1), :) = [];
    
elseif deadGyro(1,1) >= max([navigation(1,1) deadReck(1,1) hectorData(1,1) gmapData(1,1)])
    navigation(navigation(:, 1)< deadGyro(1,1), :)= [];
    deadReck(deadReck(:, 1)< deadGyro(1,1), :)= [];
    hectorData(hectorData(:,1) < deadGyro(1,1), :) = [];
    gmapData(gmapData(:,1) < deadGyro(1,1), :) = [];
    
elseif hectorData(1,1) >= max([navigation(1,1) deadReck(1,1) deadGyro(1,1) gmapData(1,1)])
    navigation(navigation(:, 1)< hectorData(1,1), :)= [];
    deadReck(deadReck(:, 1)< hectorData(1,1), :)= [];
    deadGyro(deadGyro(:,1) < hectorData(1,1), :) = [];
    gmapData(gmapData(:,1) < hectorData(1,1), :) = [];
    
elseif gmapData(1,1) >= max([navigation(1,1) deadReck(1,1) deadGyro(1,1) hectorData(1,1)])
    navigation(navigation(:, 1)< gmapData(1,1), :)= [];
    deadReck(deadReck(:, 1)< gmapData(1,1), :)= [];
    deadGyro(deadGyro(:,1) < gmapData(1,1), :) = [];
    hectorData(hectorData(:,1) < gmapData(1,1), :) = [];
end

% prune length w.r.t end time 
if navigation(end,1) <= min([deadReck(end,1) deadGyro(end,1) hectorData(end,1) gmapData(end,1)])
    deadReck(deadReck(:, 1)> navigation(end,1), :)= [];
    deadGyro(deadGyro(:, 1)> navigation(end,1), :)= [];
    hectorData(hectorData(:,1) > navigation(end,1), :) = [];
    gmapData(gmapData(:,1) > navigation(end,1), :) = [];
    
elseif deadReck(end,1) <= min([navigation(end,1) deadGyro(end,1) hectorData(end,1) gmapData(end,1)])
    navigation(navigation(:, 1)> deadReck(end,1), :)= [];
    deadGyro(deadGyro(:, 1)> deadReck(end,1), :)= [];
    hectorData(hectorData(:,1) > deadReck(end,1), :) = [];
    gmapData(gmapData(:,1) > deadReck(end,1), :) = [];
    
elseif deadGyro(end,1) <= min([navigation(end,1) deadReck(end,1) hectorData(end,1) gmapData(end,1)])
    navigation(navigation(:, 1)> deadGyro(end,1), :)= [];
    deadReck(deadReck(:, 1)> deadGyro(end,1), :)= [];
    hectorData(hectorData(:,1) > deadGyro(end,1), :) = [];
    gmapData(gmapData(:,1) > deadGyro(end,1), :) = [];
    
elseif hectorData(end,1) <= min([navigation(end,1) deadReck(end,1) deadGyro(end,1) gmapData(end,1)])
    navigation(navigation(:, 1)> hectorData(end,1), :)= [];
    deadReck(deadReck(:, 1)> hectorData(end,1), :)= [];
    deadGyro(deadGyro(:,1) > hectorData(end,1), :) = [];
    gmapData(gmapData(:,1) > hectorData(end,1), :) = [];
    
elseif gmapData(end,1) <= min([navigation(end,1) deadReck(end,1) deadGyro(end,1) hectorData(end,1)])
    navigation(navigation(:, 1)> gmapData(end,1), :)= [];
    deadReck(deadReck(:, 1)> gmapData(end,1), :)= [];
    deadGyro(deadGyro(:,1) > gmapData(end,1), :) = [];
    hectorData(hectorData(:,1) > gmapData(end,1), :) = [];
end

% calculate distance between ground truth and the other vdt methods. 
distDead = diag(pdist2(deadReck(:,2:3),navigation(:,2:3)));
distGyro = diag(pdist2(deadGyro(:,2:3),navigation(:,2:3)));


% downsample nav data and calculate dist Hector
%look more at this but with the correct data
disthector = getDistance(navigation,hectorData);
distGmap = getDistance(navigation, gmapData);


figure
hold on
title('Deviation From Ground Truth')
plot(deadReck(:,1),distDead,'LineWidth',1.2)
plot(deadGyro(:,1),distGyro,'g','LineWidth',1.2)
%plot(rosOdo(:,1)-rosOdo(1,1),distRosOdo)
plot(hectorData(:,1),disthector,'--','LineWidth',1.2)
plot(gmapData(:,1),distGmap,'-.','LineWidth',1.2)
legend('Dead Reckoning','Dead reckoning with IMU','Hector SLAM','Gmapping')


figure()
hold on
title('Path of AGV')
axis ([-18 6 -18 6])
plot(navigation(:,2)-navigation(1,2),navigation(:,3)-navigation(1,3),'k','LineWidth',1.5)
plot(deadReck(:,2)-deadReck(1,2),deadReck(:,3)-deadReck(1,3),'LineWidth',1.2)
plot(deadGyro(:,2)-deadGyro(1,2),deadGyro(:,3)-deadGyro(1,3),'g','LineWidth',1.2)
plot(hectorData(:,2)-hectorData(1,2),hectorData(:,3)-hectorData(1,3),'r--','LineWidth',1.2)
plot(gmapData(:,2)-gmapData(1,2),gmapData(:,3)-gmapData(1,3),'m-.','LineWidth',1.2)
legend('Ground Truth','Dead Reckoning','Dead reckoning with IMU','Hector SLAM','Gmapping')

figure
hold on
title('AGV path in X')
plot(navigation(:,1),navigation(:,2),'k')
plot(deadReck(:,1),deadReck(:,2))
plot(deadGyro(:,1),deadGyro(:,2),'g')
%plot(rosOdo(:,1)-rosOdo(1,1),distRosOdo)
plot(hectorData(:,1),hectorData(:,2),'r--')
plot(gmapData(:,1),gmapData(:,2),'m-.')
legend('Ground Truth','Dead Reckoning','Dead reckoning with IMU','Hector SLAM','Gmapping')

figure
hold on 
title('AGV path in Y')
plot(navigation(:,1),navigation(:,3),'k')
plot(deadReck(:,1),deadReck(:,3))
plot(deadGyro(:,1),deadGyro(:,3),'g')
%plot(rosOdo(:,1)-rosOdo(1,1),distRosOdo)
plot(hectorData(:,1),hectorData(:,3),'r--')
plot(gmapData(:,1),gmapData(:,3),'m-.')
legend('Ground Truth','Dead Reckoning','Dead reckoning with IMU','Hector SLAM','Gmapping')

angleGmap = getAngle(navigation, gmapData);
angleHect = getAngle(navigation, hectorData);

figure
hold on
title('Angle')
plot(navigation(:,1),navigation(:,4),'k')
plot(deadReck(:,1),deadReck(:,4))
plot(deadGyro(:,1),deadGyro(:,4),'g')
%plot(rosOdo(:,1)-rosOdo(1,1),distRosOdo)
plot(hectorData(:,1),hectorData(:,4),'r--')
plot(gmapData(:,1),gmapData(:,4),'m-.')
legend('Ground Truth','Dead Reckoning','Dead reckoning with IMU','Hector SLAM','Gmapping')


%%































