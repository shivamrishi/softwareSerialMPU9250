%for north, choose 200 values from 400
index = zeros(200,5,32);% generate a set of random indices to sample main matrix magData
magDirValues = zeros(200,4,5,32); %sampled data across 5 randomly generated indices for each layer or direction
for j = 1:32
for i = 1:5
    index(:,i,j) = randi([1 400],200,1);% generate random indices
end
end
for i = 1:32
for j = 1:5
    magDirValues(:,:,j,i) = magData(index(:,j,i),:,i); %generate sampling matrix
end
end