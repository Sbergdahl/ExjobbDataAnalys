function distHector = getDistance(navigation,data)

navCopy = navigation;
navCopy(navCopy(:, 1)<= data(1,1), :)= [];
navCopy(navCopy(:, 1)>= data(end,1), :)= [];

navDownSampledX =  resample(navCopy(:,2),length(data(:,1)), length(navCopy(:,2)),1);
navDownSampledY =  resample(navCopy(:,3),length(data(:,1)), length(navCopy(:,2)),1);
navDownSampledTime = resample(navCopy(:,1),length(data(:,1)), length(navCopy(:,2)),1);

hectUpSampledX =  resample(data(:,2),length(navCopy(:,1)), length(data(:,2)),1);
hectUpSampledY =  resample(data(:,3),length(navCopy(:,1)), length(data(:,2)),1);
hectUpSampledTime = resample(data(:,1),length(navCopy(:,1)), length(data(:,2)),1);



%navDownSampeledHector= [navDownSampledTime navDownSampledX navDownSampledY];
navDownSampeledHector= [navDownSampledX navDownSampledY];


% for inti = 1 : length(data)
%    tmp1(inti)= norm(data(inti,2:3)-navDownSampeledHector(inti,2:3))
% end

%distHector = diag(pdist2(data(:,2:3),navDownSampeledHector(:,2:3)));
distHector = diag(pdist2(data(:,2:3),navDownSampeledHector));


%  figure
%  hold on
%  plot(navDownSampledX)
%  plot(data(1:end,2))
%  
%   figure
%  hold on
%  plot(navDownSampledY)
%  plot(data(1:end,3))
%  
% 
