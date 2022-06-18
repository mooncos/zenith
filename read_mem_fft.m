fid = fopen('mem_dump_50ohm_load.hex');
data = fread(fid, 1024, 'float32');
fclose(fid);

I = data(1:2:end);
Q = data(2:2:end);

f=0:1:511;

signal = (I + 1j*Q)*1e-3;

signal_fft = fft(signal);
signal_fft = signal_fft(1:end/2)/length(signal_fft);
signal_fft(2:end) = signal_fft(2:end)*2;
signal_fft = 10*log10(abs(signal_fft).^2/50)+30;

plot(signal_fft);
xlabel("frequency (Hz)");
ylabel("amplitude (dBm)");

hold off;
figure;