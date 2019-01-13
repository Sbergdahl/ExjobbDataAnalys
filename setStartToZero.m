function newData = setStartToZero(data)

newData(:,1) = data(:,1); %-data(1,1);
newData(:,2) = data(:,2)-data(1,2);
newData(:,3) = data(:,3)-data(1,3);
newData(:,4) = data(:,4)-data(1,4);
