function [Yf1,f1]=Time_frequency(x,fs)
%% 绘制时域和频率图形
% figure();
% plot(IAD(1:length(x)),x);
% xlabel('IAD (angle)');
% ylabel('IAS (angle/s)');
% set(gca,'Fontsize',16);set(gca,'FontWeight');
% set(gca,'Fontname','Times new roman');

x=x-mean(x);
% II=x;
II=abs(hilbert(x));
NFFT = 2^nextpow2(length(II));
Yf = 2*abs(fft(II,NFFT))/NFFT;
Yf1 = Yf(1:NFFT/2);
f1= fs/2 * linspace(0,1,NFFT/2);
% figure()
% plot(f1(1:length(Yf1)),Yf1);
% xlim([1 10])
% set(gca,'Fontsize',16);set(gca,'FontWeight');
% set(gca,'Fontname','Times new roman');
% xlabel('Order(Hz)');ylabel('Amplitude');
% xlim([0.5 30]);
end

