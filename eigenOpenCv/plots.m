close all

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

load( 'log_.csv' );
subplot(2,3,1)
plot( log(:,G_p) )
subplot(2,3,2)
plot( log(:,G_v) )
subplot(2,3,3)
plot( log(:,b_a) )
subplot(2,3,4)
plot( log(:,s_G_p) )
subplot(2,3,5)
plot( log(:,s_G_v) )
subplot(2,3,6)
plot( log(:,s_b_a) )

figure;
subplot(2,2,1)
plot( log(:,q) );
%plot( quaternVectRotate( repmat( [0,0,1], length(log) ), quaternConj(log(:,q)) ) );
subplot(2,2,2)
plot( log(:,b_g) );
subplot(2,2,3)
plot( log(:,s_q) );
subplot(2,2,4)
plot( log(:,s_b_g) );


