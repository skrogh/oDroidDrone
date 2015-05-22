%% Client for recieving data from drone.
tcpipClient = udp('10.16.160.137',55000);
set(tcpipClient, 'inputbuffersize', 2^15 )
fclose(tcpipClient)
fopen(tcpipClient)                          % connect
fwrite(tcpipClient,0)
% init plots
hold off
lHandle = plot3(nan, nan, nan);             % get line handle
hold on
xHandle = plot3(nan, nan, nan);             % get headding handle
yHandle = plot3(nan, nan, nan);             % get headding handle
zHandle = plot3(nan, nan, nan);             % get headding handle
view([0,90]);                               %set 2d view
xlim([-1,1]);                               %set xlimit
ylim([-1,1]);                               %set ylimit
axis equal
%% Main loop
while 1                                     %keep going
    rawData = uint8( fread( tcpipClient, 8*10 ) );                %read data from buffer
    state = typecast( rawData, 'double' )'
    
    
    X = get( lHandle, 'XData' );            %get X data in plot
    Y = get( lHandle, 'YData' );            %get Y data in plot
    Z = get( lHandle, 'ZData' );            %get Z data in plot
    
    X = [X state(5)];                         %update X data
    Y = [Y state(6)];                         %update Y data
    Z = [Z state(7)];                         %update Z data
    set( lHandle, 'XData', X, 'YData', Y, 'ZData', Z ); %notify plot of update
    
    %%
    q = state(1:4);
    IG_R = [ q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2, 2*(q(1)*q(2) + q(3)*q(4)), 2*(q(1)*q(3) - q(2)*q(4))
        2*(q(1)*q(2) - q(3)*q(4)), -q(1)^2 + q(2)^2 - q(3)^2 + q(4)^2, 2*(q(2)*q(3) + q(1)*q(4))
        2*(q(1)*q(3) + q(2)*q(4)), 2*(q(2)*q(3) - q(1)*q(4)), -q(1)^2 - q(2)^2 + q(3)^2 + q(4)^2 ];
    GI_R = IG_R';

    x=[1,0,0]';
    g_x = GI_R*x*0.15;
    y=[0,1,0]';
    g_y = GI_R*y*0.15;
    z=[0,0,1]';
    g_z = GI_R*z*0.15;
    
    X = [state(5), state(5)+g_x(1)];                         %update X data
    Y = [state(6), state(6)+g_x(2)];                         %update Y data
    Z = [state(7), state(7)+g_x(3)];                         %update Z data
    set( xHandle, 'XData', X, 'YData', Y, 'ZData', Z ); %notify plot of update
    X = [state(5), state(5)+g_y(1)];                         %update X data
    Y = [state(6), state(6)+g_y(2)];                         %update Y data
    Z = [state(7), state(7)+g_y(3)];                         %update Z data
    set( yHandle, 'XData', X, 'YData', Y, 'ZData', Z ); %notify plot of update
    X = [state(5), state(5)+g_z(1)];                         %update X data
    Y = [state(6), state(6)+g_z(2)];                         %update Y data
    Z = [state(7), state(7)+g_z(3)];                         %update Z data
    set( zHandle, 'XData', X, 'YData', Y, 'ZData', Z ); %notify plot of update
    drawnow;                                %force plot update to animate
end
fclose(tcpipClient)                         %close connection
