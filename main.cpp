#include <iostream>
#include <sstream>
#include <fstream>
using namespace std;
#include "VLPGrabber.h"

void pointcloudProcess(std::vector<Point> &current_xyzi)
{
    stringstream filename;
    static int sweep_counter;
    filename<<"pointcloud"<<++sweep_counter<<".bin";
    ofstream file;
    file.open(filename.str().c_str(),ios::binary);
    file.write((char*)&current_xyzi[0],current_xyzi.size()*sizeof(Point));
    cout<<"get a pointcloud"<<endl;
}

int main()
{
    VLPGrabber grabber;
    grabber.registeCb(pointcloudProcess);
    grabber.start();

    int i;
    std::cin>>i;
    return 0;
}

