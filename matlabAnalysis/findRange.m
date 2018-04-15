meanvalHXH = zeros(1,32);
meanvalHYH = zeros(1,32);
for i =1:32
meanvalHXH(i) = mean(average_value_hxh_sampled(1,i,:));
meanvalHYH(i) = mean(average_value_hyh_sampled(1,i,:));
end

finalModeMatrix = zeros(1,32,2);
finalModeMatrix(:,:,1) = meanvalHXH;
finalModeMatrix(:,:,2) = meanvalHYH;
for i = 1:32
productXY(i) = mean(finalModeMatrix(1,i,:));
end

v = ones(1,9);
for i = 1:32
if meanvalHXH(i)<.1
    if meanvalHYH(i) <.1
        rangematrix(1,v(1),1)= i;
        v(1) = v(1)+1;
    end
    if meanvalHYH(i) < 75 && meanvalHYH(i)>0.1
        rangematrix(1,v(2),2) = i;
        v(2) = v(2) +1;
    end
    if meanvalHYH(i) >75
        rangematrix(1,v(3),3) = i;
        v(3) = v(3) +1;
    end
end
if meanvalHXH(i)<75 && meanvalHXH(i)>0.1
    if meanvalHYH(i) <.1
        rangematrix(1,v(4),4)= i;
        v(4) = v(4)+1;
    end
    if meanvalHYH(i) < 75 && meanvalHYH(i)>0.1
        rangematrix(1,v(5),5) = i;
        v(5) = v(5) +1;
    end
    if meanvalHYH(i) >75
        rangematrix(1,v(6),6) = i;
        v(6) = v(6) +1;
    end
end
if meanvalHXH(i)> 75
    if meanvalHYH(i) <.1
        rangematrix(1,v(7),7)= i;
        v(7) = v(7)+1;
    end
    if meanvalHYH(i) < 75 && meanvalHYH(i)>0.1
        rangematrix(1,v(8),8) = i;
        v(8) = v(8) +1;
    end
    if meanvalHYH(i) >75
        rangematrix(1,v(9),9) = i;
        v(9) = v(9) +1;
    end
end
end

        

v = 0:11.25:348.75; %x axis set
figure(1);
stem(v, meanvalHXH); 
ylabel('hxh_fin');
ax = gca;
set(gca, 'XTick', v);
figure(2);
stem(v, meanvalHYH); 
ylabel('hyh_fin');
ax = gca;
set(gca, 'XTick', v);

