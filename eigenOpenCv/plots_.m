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
%plot( t, log_odroid(1:e,q) );
rotMat = quatern2rotMat(log_odroid(1:e,q));
plot( t, squeeze( rotMat(:,1,1:e) ) );
subplot(2,2,2)
plot( t, log_odroid(1:e,b_g) );
subplot(2,2,3)
plot( t, log_odroid(1:e,s_q) );
subplot(2,2,4)
plot( t, log_odroid(1:e,s_b_g) );

figure;
plot3( log_odroid(1:e,G_p(1)), log_odroid(1:e,G_p(2)), log_odroid(1:e,G_p(3)) );
axis equal
hold on
% plot orientation
for i = 1:200:e
    p = log_odroid(i,G_p)';
    mat = squeeze(rotMat(:,:,i))' * 0.2;
    h = plot3( [p(1), p(1) + mat(1,1)], [p(2), p(2) + mat(2,1)], [p(3), p(3) + mat(3,1)] );
    h.Color = 'red';
    h = plot3( [p(1), p(1) + mat(1,2)], [p(2), p(2) + mat(2,2)], [p(3), p(3) + mat(3,2)] );
    h.Color = 'green';
    h = plot3( [p(1), p(1) + mat(1,3)], [p(2), p(2) + mat(2,3)], [p(3), p(3) + mat(3,3)] );
    h.Color = 'blue';
end
% figure;
% plot( t(3:e), log_odroid(3:e,determinant) );
% 
% figure;
% plot( t(3:e), log_odroid(3:e,meanSigma) );
% 
% figure;
% plot( t(3:e), log_odroid(3:e,sumSymSigma) );


