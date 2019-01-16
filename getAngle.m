function distHector = getAngle(navigation,data)

navCopy = navigation;
navCopy(navCopy(:, 1)<= data(1,1), :)= [];
navCopy(navCopy(:, 1)>= data(end,1), :)= [];

navDownSampledTH =  resample(navCopy(:,4),length(data(:,4)), length(navCopy(:,4)),1);
navDownSampledTime = resample(navCopy(:,1),length(data(:,4)), length(navCopy(:,4)),1);

navDownSampeledHector= [navDownSampledTime navDownSampledTH];
%distHector = diag(pdist2(data(:,4),navDownSampeledHector(:,2)));
%distHector = data(:,4)-navDownSampeledHector(:,2);
dsampledTime = linspace(data(1,1),data(end,1),size(data(:,1),1))'
%distHector = [data(:,1) data(:,4)-navDownSampeledHector(:,2)];

distHector = [dsampledTime data(:,4)-navDownSampeledHector(:,2)];
% 
%  figure
%  hold on
%  title('angle check')
%  plot(dsampledTime,navDownSampledTH)
%  plot(dsampledTime,data(:,4))
%  
%  
%  figure
%  hold on
%  plot(distHector)
%  plot(data(1:end,4))


 