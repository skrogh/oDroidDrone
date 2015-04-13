%close all

q = 1:4;
G_p = 5:7;
G_v = 8:10;
b_g = 11:13;
b_a = 14:16;
s_q = 17:19;
s_G_p = 20:22;
s_G_v = 23:25;
s_b_g = 26:28;
s_b_a = 29:31;
determinant = 32;
meanSigma = 33;
sumSymSigma = 34;


log_odroid = load( 'log.csv' );

if e == 0
   e = size( log_odroid, 1 ); 
end

t = (0:e-1)/400;

subplot(2,3,1)
plot( t, log_odroid(1:e,G_p) )
subplot(2,3,2)
plot( t, log_odroid(1:e,G_v) )
subplot(2,3,3)
plot( t, log_odroid(1:e,b_a) )
subplot(2,3,4)
plot( t, log_odroid(1:e,s_G_p) )
subplot(2,3,5)
plot( t, log_odroid(1:e,s_G_v) )
subplot(2,3,6)
plot( t, log_odroid(1:e,s_b_a) )

figure;
subplot(2,2,1)
plot( t, log_odroid(1:e,q) );
%plot( quaternVectRotate( repmat( [0,0,1], length(log_odroid) ), quaternConj(log_odroid(:,q)) ) );
subplot(2,2,2)
plot( t, log_odroid(1:e,b_g) );
subplot(2,2,3)
plot( t, log_odroid(1:e,s_q) );
subplot(2,2,4)
plot( t, log_odroid(1:e,s_b_g) );

figure;
plot( t(3:e), log_odroid(3:e,determinant) );


figure;
plot( t(3:e), log_odroid(3:e,meanSigma) );

figure;
plot( t(3:e), log_odroid(3:e,sumSymSigma) );

