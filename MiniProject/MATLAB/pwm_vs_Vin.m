Vin = 0:.1:7.5;
pwm = Vin./(7.5)*256;
plot(Vin, pwm);
title("AnalogWrite Value vs Input Voltage");
xlabel("Input Voltage");
ylabel("AnalogWrite Value");