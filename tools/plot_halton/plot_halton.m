clc;clf;
filename = 'out.txt';
system('plot_halton');
M = csvread(filename);
figure(1); clf;
plot(M(:,2), M(:,3), 'o');

r = rand(length(M),2);
figure(2); clf;
plot(r(:,1),r(:,2),'o');