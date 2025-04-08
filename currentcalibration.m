current = [565 295 153 0 -151.25 -297 -563 ];
ADC_counts = [654 584 548 512 470 440 361 ];

mdl = fitlm(ADC_counts, current)
figure()
plot(ADC_counts,  current);
title('Measured current in mA as a function of ADC counts')
xlabel('ADC_counts')
ylabel('Current (mA)')