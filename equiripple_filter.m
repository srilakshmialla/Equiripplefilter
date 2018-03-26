%% Minimum phase Equiripple filter
%
% Srilakshmi Alla

%% Clearing memory
clc;
clear all;
close all;

%% Given Specifications
wp=4000; %Passband Frequency
ws=4500; % Stopband Frequency
deltap=0.1; %Passband Ripple
deltas=0.05; % Stopband Ripple
fs = 20000; % Sampling Frequency
f = [wp ws];% vector of band edges 
a = [1 0];% a is a vector specifying the desired amplitude on the bands defined by f
dev = [deltap deltas];%The entries that specify the passband ripple and the stopband attenuation

%% Filter design using Kaiser Window

[n,Wn,beta,ftype] = kaiserord(f,a,dev,fs); % design parameters of the Kaiser Window
n = n + rem(n,2); %making order even
hh = fir1(n,Wn,ftype,kaiser(n+1,beta),'noscale'); % design the filter using the designed parameters

figure(1);
freqz(hh)
title('Frequency response of filter using kaiser window');

figure(2);
hh1=freqz(hh,1,10000,fs);
a1=abs(hh1); %Magnitude Response of filter using Kaiser Window
plot(a1);
title('Magnitude Response of filter using Kaiser Window');
grid on;

%% Equiripple filter

[n1,fo,ao,w] = firpmord(f,a,dev,fs); %parameters to design equiripple filter
n1 = n1 + rem(n1,2); % Making order even
b = firpm(n1,fo,ao,w); % designing filter using designed parameters

figure(3);
freqz(b);
title('frequency response of equiripple filter');

figure(4);
h=freqz(b,1,10000,fs);
a=abs(h);%Magnitude Response of equiripple filter 
plot(a);
title('Magnitude Response of equiripple filter')
grid on;

figure(5);
zplane(b);
title('Pole Zero plot of equiripple filter');

% After making order even the passband and stop band ripple changes

%Maximum Passband ripple
d1=max(a)-1;
disp(d1);

%Maximum Stopband ripple
d2=max(a(4501));
disp(d2);

%% Minimum Phase Equiripple filter


m=1:10000;

%Shifting the signal by d2(Maximum stopband ripple)

b1=b;
l=length(b1);
% Since this is a type-1 filter length will be odd and order will be even
l=l-rem(l,2); % making even
mid=((l/2)+1); % the index starts from 1 rather than 0 so 1 is added while calculating.
disp(mid);
b1(1,mid)=b(1,mid)+0.0491;% select middle value of impulse response(b) and add maximum stopband ripple (d2) to it

figure(6);
zplane(b1)%Pole Zero plot after adding shift of d2
title('Pole Zero plot after adding shift of d2');

% Roots

r1=sort(roots(b1),2); %finding zeros of shifted impulse response
r2=abs(r1); % Magnitude of Zeros
lm=(r2<=1);% gives logical matrix
r3=r1(lm); % new root matrix with roots less than or equal ro 1
b2=poly(r3); % polynomial formed from roots obtained

% The polynomial or impulse response obtained in about step has all roots inside the circle making it minimum phase filter.

figure(7);
zplane(b2);
title('Pole Zero plot of minimum phase equiripple filter');

figure(8);
freqz(b2);
h1=freqz(b2,1,10000,fs);
h2=(abs(h1));
plot(h2);
grid on;
title('Magnitude Response of minimum phase filter');


% Scaling h2 to unit gain
h3(:,:)=h2(1:4000);

% Maximum Passband Ripple in h2
p=max(h3);
disp(p);

% Minimum Passband Ripple in h2
q=min(h3);
disp(q);

% The Magnitude Response of Minimum Phase filter is scaled by factor of (2/(p+q))

h4=((2*h2)/(p+q));

figure(9);
plot(h4);
grid on;
title('Magnitude response of minimum phase equiripple filter after scaling');


%% Conclusion
%
%  Figures 10 and 11 shows frequency response and zplane of both linear  phase and minimum phase filter

figure(10);
plot(m,a,'b',m,h4,'r');
title('Magnitude response of both equiripple filter and  minimum phase equiripple filter');
grid on;


figure(11);
subplot(1,2,1);
zplane(b);
subplot(1,2,2);
zplane(b2);
title('Pole zero plot of linear phase filter and minimum phase filter');