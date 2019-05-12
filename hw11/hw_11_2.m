Fs=100;
L=length(data(:,2));
n=2^nextpow2(L);
t=[0.01:0.01:L/Fs];
for i=2:1:5
    
   y=data(:,i);
   
   Y=fft(y,n);
   
  f = Fs*(0:(n/2))/n;
  
  P = abs(Y/n);
plot(f,P(1:n/2+1)) 

hold on;
legend('Raw','MAF','FIR','IIR');

end
hold off;

figure;


for i=2:1:5
    
   y=data(:,i);
 hold on;  
plot(t,y) 
legend('Raw','MAF','FIR','IIR');

end