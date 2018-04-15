max_value_hxl_sampled = zeros(1,32,5);  % one column matrix for each sampled set
max_value_hyl_sampled = zeros(1,32,5); % one column matrix for each sampled set
average_value_hxl_sampled = zeros(1,32,5);% one column matrix for each sampled set
average_value_hyl_sampled = zeros(1,32,5);% one column matrix for each sampled set
min_value_hxl_sampled = zeros(1,32,5);
min_value_hyl_sampled = zeros(1,32,5);
for k = 1:5
for u = 1:32
max_value_hxl_sampled(1,u,k) = max(magDirValues(:,1,k,u)); %
end
end

for k = 1:5
for u = 1:32
    max_value_hyl_sampled(1,u,k) = max(magDirValues(:,3,k,u));
end
end

for k = 1:5
for u = 1:32
average_value_hxl_sampled(1,u,k) = mean(magDirValues(:,1,k,u));
end
end

for k = 1:5
for u = 1:32
average_value_hyl_sampled(1,u,k) = mean(magDirValues(:,3,k,u));
end
end

for k = 1:5
for u = 1:32
min_value_hxl_sampled(1,u,k) = min(magDirValues(:,1,k,u)); %
end
end

for k = 1:5
for u = 1:32
min_value_hyl_sampled(1,u,k) = min(magDirValues(:,3,k,u)); %
end
end

v = 0:11.25:348.75; %x axis set
figure(1);
stem(v, max_value_hxl_sampled(:,:,1)); 
ylabel('maxHxlPartial');
ax = gca;
set(gca, 'XTick', v);
figure(2);
stem(v, max_value_hyl_sampled(:,:,4)); 
ylabel('maxHylPartial');
ax = gca;
set(gca, 'XTick', v);
figure(3);
stem(v,average_value_hxl_sampled(:,:,2));
ylabel('averageHxlPartial');
ax = gca;
set(gca, 'XTick', v);
figure(4);
stem(v, average_value_hyl_sampled(:,:,5));
ylabel('averageHxlPartial');
ax = gca;
set(gca, 'XTick', v);
