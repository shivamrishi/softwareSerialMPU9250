meanvalHXL = zeros(1,32);
meanvalHYL = zeros(1,32);
for i =1:32
maxvalHXL(i) = mean(max_value_hxl_sampled(1,i,:));
maxvalHYL(i) = mean(max_value_hyl_sampled(1,i,:));
end
for i =1:32
meanvalHXL(i) = mean(average_value_hxl_sampled(1,i,:));
meanvalHYL(i) = mean(average_value_hyl_sampled(1,i,:));
end

for i =1:32
minvalHXL(i) = mean(min_value_hxl_sampled(1,i,:));
minvalHYL(i) = mean(min_value_hyl_sampled(1,i,:));
end
v = 0:11.25:348.75;
finoutputHX(1,:) = meanvalHXL(1,:);
finoutputHX(2,:) = maxvalHXL(1,:);
finoutputHX(3,:) = minvalHXL(1,:);
finoutputHX(4,:) = v;
finoutputHY(1,:) = meanvalHYL(1,:);
finoutputHY(2,:) = maxvalHYL(1,:);
finoutputHY(3,:) = minvalHYL(1,:);
finoutputHY(4,:) = v;

finalModeMatrix = zeros(1,32,2);
finalModeMatrix(:,:,1) = meanvalHXL;
finalModeMatrix(:,:,2) = meanvalHYL;
for i = 1:32
productXY(i) = mean(finalModeMatrix(1,i,:));
end

v = 0:11.25:348.75; %x axis set
figure(1);
stem(v, meanvalHXL); 
ylabel('hxh_fin');
ax = gca;
set(gca, 'XTick', v);
figure(2);
stem(v, meanvalHYL); 
ylabel('hyh_fin');
ax = gca;
set(gca, 'XTick', v);
