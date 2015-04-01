#include "UGVNavigation.h"

int main(){
    double angles [] = {-.3,-.2,-.1,0,.1,.2,.3};
    double distances [] = {50,50,50,100,100,50,50};
    std::vector<double> res = obstacleRecognition(distances,angles,7);
    for (int i = 0;i<res.size();i++){
        std::cout<<res[i]<<"---";
    }
    return 0;
    
}
