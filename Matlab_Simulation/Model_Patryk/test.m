
tf_edf = tf([2.3], [0.15 1], "OutputDelay", 0.7);
t = 0:(9.963/337):9.963;
lsim(tf_edf, data_throttle, t)
hold on;
plot(data_thrust)
hold off;