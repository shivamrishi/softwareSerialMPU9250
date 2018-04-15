mode_value_hxl = zeros(1,32);
mode_value_hyl = zeros(1,32);
average_value_hxl = zeros(1,32);
average_value_hyl = zeros(1,32);
for u = 1:32
mode_value_hxl(1,u) = mode(magData(:,1,u));
end

for u = 1:32
    mode_value_hyl(1,u) = mode(magData(:,3,u));
end

for u = 1:32
average_value_hxl(1,u) = mean(magData(:,1,u));
end

for u = 1:32
average_value_hyl(1,u) = mean(magData(:,3,u));
end


v = 0:11.25:348.75; %x axis set
figure(1);
stem(v, mode_value_hxl); 
ylabel('modeHxh');
ax = gca;
set(gca, 'XTick', v);
figure(2);
stem(v, mode_value_hyl); 
ylabel('modeHyh');
ax = gca;
set(gca, 'XTick', v);
figure(3);
stem(v,average_value_hxl);
ylabel('averageHxh');
ax = gca;
set(gca, 'XTick', v);
figure(4);
stem(v, average_value_hyl);
ylabel('averageHxh');
ax = gca;
set(gca, 'XTick', v);