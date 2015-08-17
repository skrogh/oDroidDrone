git pull
make
ar rcs libenc.a args.o encoder.o in_demo.o out_file.o mfc.o io_dev.o func_dev.o v4l_dev.o
g++ -c -Wall -g -mfpu=neon  encoderWrapperTest.cpp -I/usr/local/include/opencv -I/usr/local/include -pthread
g++ -Wall -g -mfpu=neon -o cppTest encoderWrapperTest.o -L. -lenc -lm -lrt -L/usr/local/lib -lopencv_shape -lopencv_stitching -lopencv_objdetect -lopencv_superres -lopencv_videostab -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_video -lopencv_photo -lopencv_ml -lopencv_imgproc -lopencv_flann -lopencv_core -lopencv_hal -pthread
