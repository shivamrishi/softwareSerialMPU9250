mode_value_hxh_sampled = zeros(1,32,5);  % one column matrix for each sampled set
mode_value_hyh_sampled = zeros(1,32,5); % one column matrix for each sampled set
average_value_hxh_sampled = zeros(1,32,5);% one column matrix for each sampled set
average_value_hyh_sampled = zeros(1,32,5);% one column matrix for each sampled set
for k = 1:5
for u = 1:32
mode_value_hxh_sampled(1,u,k) = mode(magDirValues(:,2,k,u)); %
end
end

for k = 1:5
for u = 1:32
    mode_value_hyh_sampled(1,u,k) = mode(magDirValues(:,4,k,u));
end
end

for k = 1:5
for u = 1:32
average_value_hxh_sampled(1,u,k) = mean(magDirValues(:,2,k,u));
end
end

for k = 1:5
for u = 1:32
average_value_hyh_sampled(1,u,k) = mean(magDirValues(:,4,k,u));
end
end
v = 0:11.25:348.75; %x axis set
figure(1);
stem(v, mode_value_hxh_sampled(:,:,1)); 
ylabel('modeHxhPartial');
ax = gca;
set(gca, 'XTick', v);
figure(2);
stem(v, mode_value_hyh_sampled(:,:,4)); 
ylabel('modeHyhPartial');
ax = gca;
set(gca, 'XTick', v);
figure(3);
stem(v,average_value_hxh_sampled(:,:,2));
ylabel('averageHxhPartial');
ax = gca;
set(gca, 'XTick', v);
figure(4);
stem(v, average_value_hyh_sampled(:,:,5));
ylabel('averageHxhPartial');
ax = gca;
set(gca, 'XTick', v);

