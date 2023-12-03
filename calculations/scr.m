K = 7.79; a = 1/7; d = 60e-3;

H_s = tf([-1*K*a],[1,0,0], 'InputDelay', d)

sys = feedback(C*H_s,1);

step(sys)