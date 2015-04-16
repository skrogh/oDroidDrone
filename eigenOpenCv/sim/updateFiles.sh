rm fileOfFiles.txt
ls opencv_test_images > fileOfFiles.txt
sed -i -e 's\.*\sim/opencv_test_images/&\' fileOfFiles.txt 
