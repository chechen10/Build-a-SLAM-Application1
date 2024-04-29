cameraCalibrator
imds = imageDatastore('C:\Users\evanh\OneDrive\Documents\MATLAB');
reset(imds);  
disp(['Number of images in the datastore: ' num2str(numel(imds.Files))]);
if numel(imds.Files) == 0
    error('No images found in the specified directory.');
end
firstImage = readimage(imds, 1);
imageSize = size(firstImage);
imagePlayer = vision.VideoPlayer('Position', [100 100 [imageSize(2), imageSize(1)]+30]);
grayImage = rgb2gray(firstImage);
points = detectMinEigenFeatures(grayImage, 'MinQuality', 0.01);
tracker = vision.PointTracker('MaxBidirectionalError', 2);
initialize(tracker, points.Location, grayImage);
while hasdata(imds)
    frame = read(imds);  
    grayFrame = rgb2gray(frame);
    
    
    [points, validity] = step(tracker, grayFrame);
    
    
    out = insertMarker(frame, points(validity, :), '+');
    step(imagePlayer, out);
    
    
    if nnz(validity) < 10
        release(tracker);
        points = detectMinEigenFeatures(grayFrame, 'MinQuality', 0.01);
        initialize(tracker, points.Location, grayFrame);
    end
end
image1 = readimage(imds, 1);
image2 = readimage(imds, 2);
points1 = detectSURFFeatures(rgb2gray(image1));
points2 = detectSURFFeatures(rgb2gray(image2));
[features1, validPoints1] = extractFeatures(rgb2gray(image1), points1);
[features2, validPoints2] = extractFeatures(rgb2gray(image2), points2);
indexPairs = matchFeatures(features1, features2);
matchedPoints1 = validPoints1(indexPairs(:, 1), :);
matchedPoints2 = validPoints2(indexPairs(:, 2), :);
[fMatrix, epipolarInliers] = estimateFundamentalMatrix(...
    matchedPoints1, matchedPoints2, 'Method', 'RANSAC', ...
    'NumTrials', 10000, 'DistanceThreshold', 0.1);

inlierPoints1 = matchedPoints1(epipolarInliers, :);
inlierPoints2 = matchedPoints2(epipolarInliers, :);

[orientation, location] = relativeCameraPose(fMatrix, cameraParams, ...
    inlierPoints1, inlierPoints2);
cameraPose = rigid3d(orientation, location);
figure;
plotCamera('Location', location, 'Orientation', orientation, 'Size', 0.1);
hold on;

