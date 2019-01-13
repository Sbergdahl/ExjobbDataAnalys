file = 'dead_navigation.txt'
gmapFile = 'FAST.csv'
hectFile = 'map_base.csv'
testCasePath= 'C:\2018-12-13-tester\RUNT_MED_PELARE\fast'
testCasePath= 'C:\2018-12-13-tester\RUNT_MED_PELARE\fast'
testCasePathGmap= 'C:\2018-12-13-tester\RUNT_MED_PELARE\GMAPPING'
testCasePathHector = 'C:\2018-12-13-tester\RUNT_MED_PELARE\HECTOR'
%import data exported from VDT
[deadReckX,deadReckY,deadReckTH,deadReckTime] = importDeadReck(testCasePath,'dead_reckoning.txt',2, 1192);
[deadReckGyroX,deadReckGyroY,deadReckGyroTH,deadReckGyroTime] = importDeadReck(testCasePath,'dead_reckoning_gyro.txt',2, 2380);
[navigationX,navigationY,navigationTH,navigationTime] = importDeadReck(testCasePath,'dead_navigation.txt',2, 1190);

%import data from hector SLAM
[xHector,yHector,thHector,timeHector] = importTfEcho(testCasePathHector,hectFile, 1, 10)
hectordata = [xHector yHector thHector timeHector]
%Rotate data from hector to mtach the rest 
R = [cos(0.5*(-pi)) -sin(0.5*(-pi)); sin(0.5*(-pi)) cos(0.5*(-pi))];
tmp1=R*[xHector yHector]'

xHector= tmp1(1,:)'
yHector= tmp1(2,:)'


%Import data exported from 

%gmappnig 
[rosOdoTime,transformTimeGmap,rosOdoX,rosOdoY,rosOdoZ,rosOdoTh] = importRosData(testCasePathGmap,gmapFile, 2, 3187)

dead = unique([deadReckTime deadReckX deadReckY],'rows')
gyro = unique([deadReckGyroTime deadReckGyroX deadReckGyroY],'rows')
nav = unique([navigationTime navigationX navigationY],'rows')
rosOdo = unique([transformTimeGmap*1e-9 rosOdoX-rosOdoX(1) rosOdoY-rosOdoY(1)],'rows')

%create rotation matrix and rotade gmapping vector
R = [cos(0.5*rosOdoTh(1)) -sin(0.5*rosOdoTh(1)); sin(0.5*rosOdoTh(1)) cos(0.5*rosOdoTh(1))];
tmp1=R*rosOdo(:,2:3)'

rosOdo = [rosOdo(:,1) tmp1'];



%OM körningne från vdt är längre i tid än den från ROS 



dead(dead(:, 1)<= rosOdo(1,1), :)= []
gyro(gyro(:, 1)<= rosOdo(1,1), :)= []
nav(nav(:, 1)<= rosOdo(1,1), :)= []
rosOdo(rosOdo(:, 1)>= dead(end,1), :)= []


dead(dead(:, 1)>= rosOdo(end,1), :)= []
gyro(gyro(:, 1)>= rosOdo(end,1), :)= []
nav(nav(:, 1)>= rosOdo(end,1), :)= []

%maybe remove these not sure if really needed
nav=[nav ; nav(end,:)]
dead=[dead ; dead(end,:)]
gyro=[gyro ; gyro(end,:)]

%add offset to SLAM coordinates:
navoffsX= (nav(:,2)-nav(1,2));
navoffsY= (nav(:,3)-nav(1,3));
%gmap(:,2) = abs(gmap(:,2)) - abs(navoffsX)
%gmap(:,3) = abs(gmap(:,3)) - abs(navoffsY)

%distance between dead reckoning and nav
distDead = pdist2([dead(:,2) dead(:,3)],[nav(:,2) nav(:,3)]);
distDead = diag(distDead);

%distance between dead reckoning with gyro and nav
distGyro = pdist2([gyro(:,2) gyro(:,3)],[nav(:,2) nav(:,3)]);
distGyro = diag(distGyro);

%distance between gmappign and nav
distRosOdo = pdist2([rosOdo(:,2) rosOdo(:,3)],[nav(:,2)-nav(1,2) nav(:,3)-nav(1,3)]);
distRosOdo = diag(distRosOdo);

%distance between hector and nav

navDownSampledX =  resample((nav(:,2)-nav(1,2)),length(timeHector), length(nav(:,2)),1)
navDownSampledY =  resample((nav(:,3)-nav(1,3)),length(timeHector), length(nav(:,2)),1)
timeDownSampled = resample((nav(:,1)-nav(1,1)),length(timeHector), length(nav(:,2)),1)

distHector = pdist2([xHector yHector],[navDownSampledX navDownSampledY]);
distHector = diag(distHector);

norm([xHector(100) yHector(100)]'- [nav(end,2)-nav(1,2) nav(end,3)-nav(1,3)]')

clf
figure
hold on 
plot(dead(:,1)-dead(1,1),distDead)
plot(gyro(:,1)-gyro(1,1),distGyro)
plot(rosOdo(:,1)-rosOdo(1,1),distRosOdo)
plot(timeDownSampled,distHector)



hold off


%%
clf
figure()
hold on
plot(deadReckGyroX-deadReckGyroX(1),deadReckGyroY-deadReckGyroY(1))
plot(deadReckX-deadReckX(1),deadReckY-deadReckY(1))
plot((nav(:,2)-nav(1,2)),(nav(:,3)-nav(1,3)),'--')
%plot(rosOdo(:,2)-rosOdo(1,2),rosOdo(:,3)-rosOdo(1,3))



plot(xHector,yHector,'--')






 