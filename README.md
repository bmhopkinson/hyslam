hyslam is a hybrid SLAM/SfM system designed for mapping. 
The systems is based on ORB-SLAM2 and Colmap.


Installation
    1. in Thirdparty, compile and install g2o:
   cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
   make -jX
    sudo make install 
    2. in Third party, compile DBoW2. currently making static library
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
    make -jX
    headers must be handled manually. collect headers (retaining directory structure)  and put in /usr/local/Thirdparty
    3. compile and install hyslam
    cmake ..
    make -jX
    sudo make install
    4. build binary vocabulary: ./tools/bin_vocabulary