#include "io.h"
#include <iostream>
int main(){
    const int result=vd2_offline_check();
    std::cout << "Native contract: " << (result?"FAIL":"PASS")
              << " (no DDS, UDP, camera or motion initialized)\n";
    return result;
}
