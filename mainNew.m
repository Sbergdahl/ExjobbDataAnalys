%data paths
clear all
testcase = 'korridor'
speed = 'fast'
fileNav = 'dead_navigation.txt'
fileDeadReck ='dead_reckoning.txt'
fileGyroReck ='dead_reckoning_gyro.txt'
gmapFile1 = strcat(speed,'1','.csv')
gmapFile2 = strcat(speed,'2','.csv')
gmapFile3 = strcat(speed,'3','.csv')
gmapFile4 = strcat(speed,'4','.csv')
gmapFile5 = strcat(speed,'5','.csv')

hectFile1 = strcat(speed,'1','.csv')
hectFile2 = strcat(speed,'2','.csv')
hectFile3 = strcat(speed,'3','.csv')
hectFile4 = strcat(speed,'4','.csv')
hectFile5 = strcat(speed,'5','.csv')
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

[xGmap1,yGmap1,thGmap1,timeGmap1] = importTfEcho(gmapPath,gmapFile1, 1, 10);
[xGmap2,yGmap2,thGmap2,timeGmap2] = importTfEcho(gmapPath,gmapFile2, 1, 10);
[xGmap3,yGmap3,thGmap3,timeGmap3] = importTfEcho(gmapPath,gmapFile3, 1, 10);
[xGmap4,yGmap4,thGmap4,timeGmap4] = importTfEcho(gmapPath,gmapFile4, 1, 10);
[xGmap5,yGmap5,thGmap5,timeGmap5] = importTfEcho(gmapPath,gmapFile5, 1, 10);

[xHector1,yHector1,thHector1,timeHector1] = importTfEcho(hectorPath,hectFile1, 1, 10);
[xHector2,yHector2,thHector2,timeHector2] = importTfEcho(hectorPath,hectFile2, 1, 10);
[xHector3,yHector3,thHector3,timeHector3] = importTfEcho(hectorPath,hectFile3, 1, 10);
[xHector4,yHector4,thHector4,timeHector4] = importTfEcho(hectorPath,hectFile4, 1, 10);
[xHector5,yHector5,thHector5,timeHector5] = importTfEcho(hectorPath,hectFile5, 1, 10);


%%

%rotate data to match
rotateGmap1 = -pi/2 -thGmap1(1) - pi/2;
rotateGmap2 = -pi/2 -thGmap2(1) - pi/2;
rotateGmap3 = -pi/2 -thGmap3(1) - pi/2;
rotateGmap4 = -pi/2 -thGmap4(1) - pi/2;
rotateGmap5 = -pi/2 -thGmap5(1) - pi/2;
rotateHect1 = -pi/2   - pi/2;
rotateHect2 = -pi/2   - pi/2;
rotateHect3 = -pi/2   - pi/2;
rotateHect4 = -pi/2   - pi/2;
rotateHect5 = -pi/2   - pi/2;

% convert to vectors for easier hnadling and remove duplicate rows
deadReck =  unique([deadReckTime deadReckX deadReckY deadReckTH],'rows');
deadGyro = unique([deadReckGyroTime deadReckGyroX deadReckGyroY deadReckGyroTH],'rows');
navigation = unique([navigationTime navigationX navigationY navigationTH],'rows');

%%data from slam
hectorData1 = unique([timeHector1 xHector1 yHector1 thHector1],'rows');
hectorData2 = unique([timeHector2 xHector2 yHector2 thHector2],'rows');
hectorData3 = unique([timeHector3 xHector3 yHector3 thHector3],'rows');
hectorData4 = unique([timeHector4 xHector4 yHector4 thHector4],'rows');
hectorData5 = unique([timeHector5 xHector5 yHector5 thHector5],'rows');
gmapData1 = unique([timeGmap1 xGmap1 yGmap1 thGmap1],'rows');
gmapData2 = unique([timeGmap2 xGmap2 yGmap2 thGmap2],'rows');
gmapData3 = unique([timeGmap3 xGmap3 yGmap3 thGmap3],'rows');
gmapData4 = unique([timeGmap4 xGmap4 yGmap4 thGmap4],'rows');
gmapData5 = unique([timeGmap5 xGmap5 yGmap5 thGmap5],'rows');

% set all datasets to start at zero
deadReck = setStartToZero(deadReck);
deadGyro = setStartToZero(deadGyro);
navigation = setStartToZero(navigation);
hectorData1 = setStartToZero(hectorData1);
hectorData2 = setStartToZero(hectorData2);
hectorData3 = setStartToZero(hectorData3);
hectorData4 = setStartToZero(hectorData4);
hectorData5 = setStartToZero(hectorData5);
gmapData1 = setStartToZero(gmapData1);
gmapData2 = setStartToZero(gmapData2);
gmapData3 = setStartToZero(gmapData3);
gmapData4 = setStartToZero(gmapData4);
gmapData5 = setStartToZero(gmapData5);


%remove outliers from SLAM data
hectorData1(isoutlier(hectorData1(:,2),'movmedian',10),:)= [];
hectorData2(isoutlier(hectorData2(:,2),'movmedian',10),:)= [];
hectorData3(isoutlier(hectorData3(:,2),'movmedian',10),:)= [];
hectorData4(isoutlier(hectorData4(:,2),'movmedian',10),:)= [];
hectorData5(isoutlier(hectorData5(:,2),'movmedian',10),:)= [];
gmapData1(isoutlier(gmapData1(:,2),'movmedian',10),:)= [];
gmapData2(isoutlier(gmapData2(:,2),'movmedian',10),:)= [];
gmapData3(isoutlier(gmapData3(:,2),'movmedian',10),:)= [];
gmapData4(isoutlier(gmapData4(:,2),'movmedian',10),:)= [];
gmapData5(isoutlier(gmapData5(:,2),'movmedian',10),:)= [];

%rotate slam runs, hector and gmapping should follow same syntax
R = [cos(rotateHect1) -sin(rotateHect1); sin(rotateHect1) cos(rotateHect1)];
hectorData1(:,2:3)=(R*[hectorData1(:,2) hectorData1(:,3)]')';
R = [cos(rotateHect2) -sin(rotateHect2); sin(rotateHect2) cos(rotateHect2)];
hectorData2(:,2:3)=(R*[hectorData2(:,2) hectorData2(:,3)]')';
R = [cos(rotateHect3) -sin(rotateHect3); sin(rotateHect3) cos(rotateHect3)];
hectorData3(:,2:3)=(R*[hectorData3(:,2) hectorData3(:,3)]')';
R = [cos(rotateHect4) -sin(rotateHect4); sin(rotateHect4) cos(rotateHect4)];
hectorData4(:,2:3)=(R*[hectorData4(:,2) hectorData4(:,3)]')';
R = [cos(rotateHect5) -sin(rotateHect5); sin(rotateHect5) cos(rotateHect5)];
hectorData5(:,2:3)=(R*[hectorData5(:,2) hectorData5(:,3)]')';
 
R = [cos(rotateGmap1) -sin(rotateGmap1); sin(rotateGmap1) cos(rotateGmap1)];
gmapData1(:,2:3)=(R*[gmapData1(:,2) gmapData1(:,3)]')';
R = [cos(rotateGmap2) -sin(rotateGmap2); sin(rotateGmap2) cos(rotateGmap2)];
gmapData2(:,2:3)=(R*[gmapData2(:,2) gmapData2(:,3)]')';
R = [cos(rotateGmap3) -sin(rotateGmap3); sin(rotateGmap3) cos(rotateGmap3)];
gmapData3(:,2:3)=(R*[gmapData3(:,2) gmapData3(:,3)]')';
R = [cos(rotateGmap4) -sin(rotateGmap4); sin(rotateGmap4) cos(rotateGmap4)];
gmapData4(:,2:3)=(R*[gmapData4(:,2) gmapData4(:,3)]')';
R = [cos(rotateGmap5) -sin(rotateGmap5); sin(rotateGmap5) cos(rotateGmap5)];
gmapData5(:,2:3)=(R*[gmapData5(:,2) gmapData5(:,3)]')';


timenav = navigation(1,1)-navigation(end,1)
timedead = deadReck(1,1)-deadReck(end,1)
timegyro = deadGyro(1,1)-deadGyro(end,1)
timehector1 = hectorData1(1,1)-hectorData1(end,1)
timehector2 = hectorData2(1,1)-hectorData2(end,1)
timehector3 = hectorData3(1,1)-hectorData3(end,1)
timehector4 = hectorData4(1,1)-hectorData4(end,1)
timehector5 = hectorData5(1,1)-hectorData5(end,1)
timegmap1 = gmapData1(1,1)-gmapData1(end,1)
timegmap2 = gmapData2(1,1)-gmapData2(end,1)
timegmap3 = gmapData3(1,1)-gmapData3(end,1)
timegmap4 = gmapData4(1,1)-gmapData4(end,1)
timegmap5 = gmapData5(1,1)-gmapData5(end,1)

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
if navigation(1,1) >=  max([deadReck(1,1) deadGyro(1,1) hectorData1(1,1) gmapData1(1,1)])
    deadReck(deadReck(:, 1)< navigation(1,1), :)= [];
    deadGyro(deadGyro(:, 1)< navigation(1,1), :)= [];
    hectorData1(hectorData1(:,1) < navigation(1,1), :) = [];
    hectorData2(hectorData2(:,1) < navigation(1,1), :) = [];
    hectorData3(hectorData3(:,1) < navigation(1,1), :) = [];
    hectorData4(hectorData4(:,1) < navigation(1,1), :) = [];
    hectorData5(hectorData5(:,1) < navigation(1,1), :) = [];
    gmapData1(gmapData1(:,1) < navigation(1,1), :) = [];
    gmapData2(gmapData2(:,1) < navigation(1,1), :) = [];
    gmapData3(gmapData3(:,1) < navigation(1,1), :) = [];
    gmapData4(gmapData4(:,1) < navigation(1,1), :) = [];
    gmapData5(gmapData5(:,1) < navigation(1,1), :) = [];
    
elseif deadReck(1,1) >= max([navigation(1,1) deadGyro(1,1) hectorData1(1,1) gmapData1(1,1)])
    navigation(navigation(:, 1)< deadReck(1,1), :)= [];
    deadGyro(deadGyro(:, 1)< deadReck(1,1), :)= [];
    hectorData1(hectorData1(:,1) < deadReck(1,1), :) = [];
    hectorData2(hectorData2(:,1) < deadReck(1,1), :) = [];
    hectorData3(hectorData3(:,1) < deadReck(1,1), :) = [];
    hectorData4(hectorData4(:,1) < deadReck(1,1), :) = [];
    hectorData5(hectorData5(:,1) < deadReck(1,1), :) = [];
    gmapData1(gmapData1(:,1) < deadReck(1,1), :) = [];
    gmapData2(gmapData2(:,1) < deadReck(1,1), :) = [];
    gmapData3(gmapData3(:,1) < deadReck(1,1), :) = [];
    gmapData4(gmapData4(:,1) < deadReck(1,1), :) = [];
    gmapData5(gmapData5(:,1) < deadReck(1,1), :) = [];
    
elseif deadGyro(1,1) >= max([navigation(1,1) deadReck(1,1) hectorData1(1,1) gmapData1(1,1)])
    navigation(navigation(:, 1)< deadGyro(1,1), :)= [];
    deadReck(deadReck(:, 1)< deadGyro(1,1), :)= [];
    hectorData1(hectorData1(:,1) < deadGyro(1,1), :) = [];
    hectorData2(hectorData2(:,1) < deadGyro(1,1), :) = [];
    hectorData3(hectorData3(:,1) < deadGyro(1,1), :) = [];
    hectorData4(hectorData4(:,1) < deadGyro(1,1), :) = [];
    hectorData5(hectorData5(:,1) < deadGyro(1,1), :) = [];
    gmapData1(gmapData1(:,1) < deadGyro(1,1), :) = [];
    gmapData2(gmapData2(:,1) < deadGyro(1,1), :) = [];
    gmapData3(gmapData3(:,1) < deadGyro(1,1), :) = [];
    gmapData4(gmapData4(:,1) < deadGyro(1,1), :) = [];
    gmapData5(gmapData5(:,1) < deadGyro(1,1), :) = [];    
    
elseif hectorData1(1,1) >= max([navigation(1,1) deadReck(1,1) deadGyro(1,1) gmapData1(1,1)])
    navigation(navigation(:, 1)< hectorData1(1,1), :)= [];
    deadReck(deadReck(:, 1)< hectorData1(1,1), :)= [];
    deadGyro(deadGyro(:,1) < hectorData1(1,1), :) = [];
    hectorData2(hectorData2(:,1) < hectorData2(1,1), :) = [];
    hectorData3(hectorData3(:,1) < hectorData3(1,1), :) = [];
    hectorData4(hectorData4(:,1) < hectorData4(1,1), :) = [];
    hectorData5(hectorData5(:,1) < hectorData5(1,1), :) = [];
    gmapData1(gmapData1(:,1) < hectorData1(1,1), :) = [];
    gmapData2(gmapData2(:,1) < hectorData2(1,1), :) = [];
    gmapData3(gmapData3(:,1) < hectorData3(1,1), :) = [];
    gmapData4(gmapData4(:,1) < hectorData4(1,1), :) = [];
    gmapData5(gmapData5(:,1) < hectorData5(1,1), :) = [];   
    
elseif gmapData1(1,1) >= max([navigation(1,1) deadReck(1,1) deadGyro(1,1) hectorData1(1,1)])
    navigation(navigation(:, 1)< gmapData1(1,1), :)= [];
    deadReck(deadReck(:, 1)< gmapData1(1,1), :)= [];
    deadGyro(deadGyro(:,1) < gmapData1(1,1), :) = [];
    hectorData1(hectorData1(:,1) < gmapData1(1,1), :) = [];
    hectorData2(hectorData2(:,1) < gmapData1(1,1), :) = [];
    hectorData3(hectorData3(:,1) < gmapData1(1,1), :) = [];
    hectorData4(hectorData4(:,1) < gmapData1(1,1), :) = [];
    hectorData5(hectorData5(:,1) < gmapData1(1,1), :) = [];
    gmapData1(gmapData1(:,1) < gmapData1(1,1), :) = [];
    gmapData2(gmapData2(:,1) < gmapData1(1,1), :) = [];
    gmapData3(gmapData3(:,1) < gmapData1(1,1), :) = [];
    gmapData4(gmapData4(:,1) < gmapData1(1,1), :) = [];
    gmapData5(gmapData5(:,1) < gmapData1(1,1), :) = [];    
    
end

% prune length w.r.t end time 
if navigation(end,1) <= min([deadReck(end,1) deadGyro(end,1) hectorData1(end,1) gmapData1(end,1)])
    deadReck(deadReck(:, 1)> navigation(end,1), :)= [];
    deadGyro(deadGyro(:, 1)> navigation(end,1), :)= [];
    hectorData1(hectorData1(:,1) > navigation(end,1), :) = [];
    hectorData2(hectorData2(:,1) > navigation(end,1), :) = [];
    hectorData3(hectorData3(:,1) > navigation(end,1), :) = [];
    hectorData4(hectorData4(:,1) > navigation(end,1), :) = [];
    hectorData5(hectorData5(:,1) > navigation(end,1), :) = [];
    gmapData1(gmapData1(:,1) > navigation(end,1), :) = [];
    gmapData2(gmapData2(:,1) > navigation(end,1), :) = [];
    gmapData3(gmapData3(:,1) > navigation(end,1), :) = [];
    gmapData4(gmapData4(:,1) > navigation(end,1), :) = [];
    gmapData5(gmapData5(:,1) > navigation(end,1), :) = [];
    
elseif deadReck(end,1) <= min([navigation(end,1) deadGyro(end,1) hectorData1(end,1) gmapData1(end,1)])
    navigation(navigation(:, 1)> deadReck(end,1), :)= [];
    deadGyro(deadGyro(:, 1)> deadReck(end,1), :)= [];
    hectorData1(hectorData1(:,1) > deadReck(end,1), :) = [];
    hectorData2(hectorData2(:,1) > deadReck(end,1), :) = [];
    hectorData3(hectorData3(:,1) > deadReck(end,1), :) = [];
    hectorData4(hectorData4(:,1) > deadReck(end,1), :) = [];
    hectorData5(hectorData5(:,1) > deadReck(end,1), :) = [];
    gmapData1(gmapData1(:,1) > deadReck(end,1), :) = [];
    gmapData2(gmapData2(:,1) > deadReck(end,1), :) = [];
    gmapData3(gmapData3(:,1) > deadReck(end,1), :) = [];
    gmapData4(gmapData4(:,1) > deadReck(end,1), :) = [];
    gmapData5(gmapData5(:,1) > deadReck(end,1), :) = [];
    
    
elseif deadGyro(end,1) <= min([navigation(end,1) deadReck(end,1) hectorData1(end,1) gmapData1(end,1)])
    navigation(navigation(:, 1)> deadGyro(end,1), :)= [];
    deadReck(deadReck(:, 1)> deadGyro(end,1), :)= [];
    hectorData1(hectorData1(:,1) > deadGyro(end,1), :) = [];
    hectorData2(hectorData2(:,1) > deadGyro(end,1), :) = [];
    hectorData3(hectorData3(:,1) > deadGyro(end,1), :) = [];
    hectorData4(hectorData4(:,1) > deadGyro(end,1), :) = [];
    hectorData5(hectorData5(:,1) > deadGyro(end,1), :) = [];
    gmapData1(gmapData1(:,1) > deadGyro(end,1), :) = [];
    gmapData2(gmapData2(:,1) > deadGyro(end,1), :) = [];
    gmapData3(gmapData3(:,1) > deadGyro(end,1), :) = [];
    gmapData4(gmapData4(:,1) > deadGyro(end,1), :) = [];
    gmapData5(gmapData5(:,1) > deadGyro(end,1), :) = [];  
    
elseif hectorData1(end,1) <= min([navigation(end,1) deadReck(end,1) deadGyro(end,1) gmapData1(end,1)])
    navigation(navigation(:, 1)> hectorData1(end,1), :)= [];
    deadReck(deadReck(:, 1)> hectorData1(end,1), :)= [];
    deadGyro(deadGyro(:,1) > hectorData1(end,1), :) = [];
    hectorData2(hectorData2(:,1) > hectorData1(end,1), :) = [];
    hectorData3(hectorData3(:,1) > hectorData1(end,1), :) = [];
    hectorData4(hectorData4(:,1) > hectorData1(end,1), :) = [];
    hectorData5(hectorData5(:,1) > hectorData1(end,1), :) = [];
    gmapData1(gmapData1(:,1) > hectorData1(end,1), :) = [];
    gmapData2(gmapData2(:,1) > hectorData1(end,1), :) = [];
    gmapData3(gmapData3(:,1) > hectorData1(end,1), :) = [];
    gmapData4(gmapData4(:,1) > hectorData1(end,1), :) = [];
    gmapData5(gmapData5(:,1) > hectorData1(end,1), :) = [];   
    
    
elseif gmapData(end,1) <= min([navigation(end,1) deadReck(end,1) deadGyro(end,1) hectorData1(end,1)])
    navigation(navigation(:, 1)> gmapData(end,1), :)= [];
    deadReck(deadReck(:, 1)> gmapData(end,1), :)= [];
    deadGyro(deadGyro(:,1) > gmapData(end,1), :) = [];
    hectorData1(hectorData1(:,1) > gmapData1(end,1), :) = [];
    hectorData2(hectorData2(:,1) > gmapData1(end,1), :) = [];
    hectorData3(hectorData3(:,1) > gmapData1(end,1), :) = [];
    hectorData4(hectorData4(:,1) > gmapData1(end,1), :) = [];
    hectorData5(hectorData5(:,1) > gmapData1(end,1), :) = [];
    gmapData1(gmapData1(:,1) > gmapData1(end,1), :) = [];
    gmapData2(gmapData2(:,1) > gmapData1(end,1), :) = [];
    gmapData3(gmapData3(:,1) > gmapData1(end,1), :) = [];
    gmapData4(gmapData4(:,1) > gmapData1(end,1), :) = [];
    gmapData5(gmapData5(:,1) > gmapData1(end,1), :) = [];    
    
end

% calculate distance between ground truth and the other vdt methods. 
distDead = diag(pdist2(deadReck(:,2:3),navigation(:,2:3)));
distGyro = diag(pdist2(deadGyro(:,2:3),navigation(:,2:3)));


% downsample nav data and calculate dist Hector
%look more at this but with the correct data
disthector1 = getDistance(navigation,hectorData1);
disthector2 = getDistance(navigation,hectorData2);
disthector3 = getDistance(navigation,hectorData3);
disthector4 = getDistance(navigation,hectorData4);
disthector5 = getDistance(navigation,hectorData5);
distGmap1 = getDistance(navigation, gmapData1);
distGmap2 = getDistance(navigation, gmapData2);
distGmap3 = getDistance(navigation, gmapData3);
distGmap4 = getDistance(navigation, gmapData4);
distGmap5 = getDistance(navigation, gmapData5);

%% make plots 

figure
hold on
title('Deviation From Ground Truth')
plot(deadReck(:,1),distDead,'LineWidth',1.2)
plot(deadGyro(:,1),distGyro,'g','LineWidth',1.2)
%plot(rosOdo(:,1)-rosOdo(1,1),distRosOdo)
plot(hectorData1(:,1),disthector1,'--','LineWidth',1.2)
plot(gmapData1(:,1),distGmap1,'-.','LineWidth',1.2)
legend('Dead Reckoning','Dead reckoning with IMU','Hector SLAM','Gmapping')


figure()
hold on
title('Path of AGV')
axis ([-18 6 -18 6])
plot(navigation(:,2)-navigation(1,2),navigation(:,3)-navigation(1,3),'k','LineWidth',1.5)
plot(deadReck(:,2)-deadReck(1,2),deadReck(:,3)-deadReck(1,3),'LineWidth',1.2)
plot(deadGyro(:,2)-deadGyro(1,2),deadGyro(:,3)-deadGyro(1,3),'g','LineWidth',1.2)
plot(hectorData1(:,2)-hectorData1(1,2),hectorData1(:,3)-hectorData1(1,3),'r--','LineWidth',1.2)
plot(hectorData2(:,2)-hectorData2(1,2),hectorData2(:,3)-hectorData2(1,3),'r--','LineWidth',1.2)
plot(hectorData3(:,2)-hectorData3(1,2),hectorData3(:,3)-hectorData3(1,3),'r--','LineWidth',1.2)
plot(hectorData4(:,2)-hectorData4(1,2),hectorData4(:,3)-hectorData4(1,3),'r--','LineWidth',1.2)
plot(hectorData5(:,2)-hectorData5(1,2),hectorData5(:,3)-hectorData5(1,3),'r--','LineWidth',1.2)
plot(gmapData1(:,2)-gmapData1(1,2),gmapData1(:,3)-gmapData1(1,3),'c-.','LineWidth',1.2)
plot(gmapData2(:,2)-gmapData2(1,2),gmapData2(:,3)-gmapData2(1,3),'c-.','LineWidth',1.2)
plot(gmapData3(:,2)-gmapData3(1,2),gmapData3(:,3)-gmapData3(1,3),'c-.','LineWidth',1.2)
plot(gmapData4(:,2)-gmapData4(1,2),gmapData4(:,3)-gmapData4(1,3),'c-.','LineWidth',1.2)
plot(gmapData5(:,2)-gmapData5(1,2),gmapData5(:,3)-gmapData5(1,3),'c-.','LineWidth',1.2)
legend('Ground Truth','Dead Reckoning','Dead reckoning with IMU','Hector SLAM','Gmapping')

figure
hold on
title('AGV path in X')
plot(navigation(:,1),navigation(:,2),'k')
plot(deadReck(:,1),deadReck(:,2))
plot(deadGyro(:,1),deadGyro(:,2),'g')
%plot(rosOdo(:,1)-rosOdo(1,1),distRosOdo)
plot(hectorData1(:,1),hectorData1(:,2),'r--')
plot(gmapData1(:,1),gmapData1(:,2),'m-.')
legend('Ground Truth','Dead Reckoning','Dead reckoning with IMU','Hector SLAM','Gmapping')

figure
hold on 
title('AGV path in Y')
plot(navigation(:,1),navigation(:,3),'k')
plot(deadReck(:,1),deadReck(:,3))
plot(deadGyro(:,1),deadGyro(:,3),'g')
%plot(rosOdo(:,1)-rosOdo(1,1),distRosOdo)
plot(hectorData1(:,1),hectorData1(:,3),'r--')
plot(gmapData1(:,1),gmapData1(:,3),'m-.')
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
plot(hectorData1(:,1),hectorData1(:,4),'r--')
plot(gmapData1(:,1),gmapData1(:,4),'m-.')
legend('Ground Truth','Dead Reckoning','Dead reckoning with IMU','Hector SLAM','Gmapping')


%%































