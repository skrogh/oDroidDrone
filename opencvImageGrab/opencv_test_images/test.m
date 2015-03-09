close all
figure;


%get all png files
files = dir('*.png');

%fencepost
points = []; % colums of feature positions
ids = []; % index in points of vpts_old
uid = [];
vpts = cornerPoints;
feature = binaryFeatures(uint8(zeros(0,64)));



for i = 1:length(files)
    display( [ num2str(i) ' / ' num2str(length(files))] )
    feature_old = feature;
    vpts_old = vpts;
    
    %load new image
    image = imread( files(i).name );
    image = rot90( image,2 );
    
    % detect points in new image
    poi = detectFASTFeatures( image, 'MinQuality', 0.004 );
    
    % calculate descriptors
    [feature, vpts] = extractFeatures( image, poi );
    
    % mach points
    index_pairs = matchFeatures( feature_old, feature );
    
    
    % add new empty row to feature list
    points = [ points; -ones( 2, size( points, 2 ) ) ];
    
    ids_old = ids;
    uid_old = uid;
    ids = -ones( length( vpts ), 1 );
    uid = -ones( length( vpts ), 1 );
    % for each feature
    for j = 1:size(vpts.Location,1)
        % find index in poins for this point
        points_index = ids_old( index_pairs( find( index_pairs(:,2) == j, 1 ), 1 ) );
        % check if empty
        if isempty( points_index )
            % point does not have an index, it is new. Make space for it
            points = [ points, -ones( size( points, 1 ), 1 ) ];
            points( end-1, end ) = vpts.Location(j,1);
            points( end, end ) =  vpts.Location(j,2);
            ids( j ) = size( points, 2 );
            if isempty(uid_old)
                uid( j ) = 1;
            else
                uid( j ) = max( uid_old ) + 1;
            end
        else
            % point has an index, we can add it to that
            points( end-1, points_index ) = vpts.Location(j,1);
            points( end, points_index ) =  vpts.Location(j,2);
            % update ids:
            ids( j ) = points_index;
            uid( j ) = uid_old( index_pairs( find( index_pairs(:,2) == j, 1 ), 1 ) );
        end
    end
    
    % remove features with no matches, and not just found
    index = 1:size( points, 2 );
    index = index( ( points( end,: ) == -1 ) & ( sum( points ~= -1 ) < 3*2 ) );
    
    % for all ids over index(j), decrease id width one
    for j = 1:length(index)
        ids( ids > index(j) ) = ids( ids > index(j) ) - 1;
    end
    
    % remove all points(index)
    points( :, index ) = [];
    
  
    
    % get matching points
    %matched_pts = vpts(index_pairs(:, 1));
    %matched_pts_old = vpts_old(index_pairs(:, 2));
    
    % show it
    %showMatchedFeatures(I1,I2,matched_pts1,matched_pts2,'blend');
    
    %title('Putative point matches');
    %legend('matchedPts1','matchedPts2');
    
    %drawnow;
    %pause;
end