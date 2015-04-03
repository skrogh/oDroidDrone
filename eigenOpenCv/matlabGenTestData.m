%% set calib file
calib.o_x = 300.8859;
calib.o_y = 222.5206;
calib.f_x = 411.1170;
calib.f_y = 409.9516;
calib.k1 = -0.3453;
calib.k2 = 0.1012;   
calib.k3 = 0;
calib.t1 = -0.0003;
calib.t2 = 0.0014;

calib.CI_q = axisAngle2quatern( [0,1,-1], pi)'; % rotation from inertial frame to camera
calib.CI_q = quaternProd( calib.CI_q',...
    axisAngle2quatern( [1,0,0], -pi/4)...
     )';
calib.C_p_I = [0, 0.0, -0.056]';%position of camera in inertial frame

calib.g = 9.82;
calib.Delta_t = 1/400;
calib.sigma_gd = 0.01;
calib.sigma_ad = 0.01;
calib.sigma_wgd = 0.05 * sqrt( calib.Delta_t );
calib.sigma_wad = 0.1 * sqrt( calib.Delta_t );

calib.sigma_gc = calib.sigma_gd * sqrt( calib.Delta_t );
calib.sigma_ac = calib.sigma_ad * sqrt( calib.Delta_t );
calib.sigma_wgc = calib.sigma_wgd / sqrt( calib.Delta_t );
calib.sigma_wac = calib.sigma_wad / sqrt( calib.Delta_t );

calib.sigma_dc = 0.05;

calib.sigma_Im = 40;

calib.crop_x1 = 0;
calib.crop_x2 = 640;
calib.crop_y1 = 0;
calib.crop_y2 = 480;

calib.maxFrame = 5;

calib.imageOffset = 0.033; %delay of camera with respect to imu in s


x_Ci = [ [ 0, 1, 0, 0 ] [ 0, 0, 0.90 ] [ 0, 0, 0 ] ]';


z = [320;240];
[ G_p_f, f, J_h, theta_log, G_p_f_log ] = triangulate( z, x_Ci, calib );
G_p_f = G_p_f

z = [0;0];
[ G_p_f, f, J_h, theta_log, G_p_f_log ] = triangulate( z, x_Ci, calib );
G_p_f = G_p_f

z = [640;0];
[ G_p_f, f, J_h, theta_log, G_p_f_log ] = triangulate( z, x_Ci, calib );
G_p_f = G_p_f

z = [0;480];
[ G_p_f, f, J_h, theta_log, G_p_f_log ] = triangulate( z, x_Ci, calib );
G_p_f = G_p_f

z = [640;480];
[ G_p_f, f, J_h, theta_log, G_p_f_log ] = triangulate( z, x_Ci, calib );
G_p_f = G_p_f