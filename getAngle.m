function distHector = getAngle(navigation,data)

navCopy = navigation;
navCopy(navCopy(:, 1)<= data(1,1), :)= [];
navCopy(navCopy(:, 1)>= data(end,1), :)= [];

navDownSampledTH =  resample(navCopy(:,4),length(data(:,1)), length(navCopy(:,4)),1);
navDownSampledTime = resample(navCopy(:,1),length(data(:,1)), length(navCopy(:,4)),1);

navDownSampeledHector= [navDownSampledTime navDownSampledTH];
distHector = diag(pdist2(data(:,4),navDownSampeledHector(:,2)));
