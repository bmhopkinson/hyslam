hyslam is a hybrid SLAM/SfM system designed for mapping. 
The systems is based on ORB-SLAM2 and Colmap.

Dependencies:
pangolin
DBoW2
opencv

Installation
    1. in Thirdparty, compile and install g2o:
   cmake ..
   make -jX
    sudo make install 
    sudo ldconfig
    2. compile and install hyslam
    in main CMakeLists set opencv directory
    cmake ..
    make -jX
    sudo make install
    4. build binary vocabulary: ./tools/bin_vocabulary
