#include <Initializer.h>

namespace ORB_SLAM2 {

void InitializerData::clear() {
    mvIniMatches.clear();
    mvbPrevMatched.clear();
    mvIniP3D.clear();
    mInitialFrame = Frame();
}


void Initializer::clear(){
    init_data.clear();
}

}//end namespace