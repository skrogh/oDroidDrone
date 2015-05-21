%% Client for recieving data from drone.
tcpipClient = udp('10.16.162.96',55000);
set(tcpipClient, 'inputbuffersize', 2^15 ) 
fclose(tcpipClient)
fopen(tcpipClient)                          % connect
fwrite(tcpipClient,0)
% init plots
hold off
lHandle = plot3(nan, nan, nan);             % get line handle
hold on
hHandle = plot3(nan, nan, nan);             % get headding handle
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
    %%
    X = get( hHandle, 'XData' );            %get X data in plot
    Y = get( hHandle, 'YData' );            %get Y data in plot
    Z = get( hHandle, 'ZData' );            %get Z data in plot
    
    X = [state(5) state(5)];                         %update X data
    Y = [state(6) state(5)];                         %update Y data
    Z = [state(7) state(5)];                         %update Z data
    
    set( lHandle, 'XData', X, 'YData', Y, 'ZData', Z ); %notify plot of update
    drawnow;                                %force plot update to animate
end
fclose(tcpipClient)                         %close connection
