#include "VLPGrabber.h"
extern "C"
{
#include <string.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
}
#include <cmath>
#include <limits>
#include <sstream>
#include <fstream>

#ifdef __QNX__
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <sys/neutrino.h>
#include <sys/socket.h>
#include <netinet/in.h>
//we use c++0x on qnx platform and there is no std::isnan on qnx6.5
namespace std{
int isnan(float a)
{return a==a;};
}
#endif

//producer thread,receiving data and storing buf pointer to an queue
void VLPGrabber::udp_receive()
{
    int sockfd;
    struct sockaddr_in local_addr;
    struct sockaddr_in vlp_addr;
#ifdef WIN32
	int addr_len = sizeof(struct sockaddr_in);
	WSADATA wsa;  
	WSAStartup(MAKEWORD(1,1),&wsa);
#else
	socklen_t addr_len = sizeof(struct sockaddr_in);
#endif
    if ((sockfd = socket(AF_INET,SOCK_DGRAM,0)) == -1) {
        perror("socket create failed!");
        exit(1);
    }
    memset(&local_addr,0,sizeof(struct sockaddr_in));
    local_addr.sin_family= AF_INET;
    local_addr.sin_port= htons(udp_port);
    local_addr.sin_addr.s_addr =INADDR_ANY;
    if (::bind(sockfd, (struct sockaddr*)&local_addr, sizeof(struct sockaddr)) == -1) {
        perror("bind error!");
        exit(1);
    }

    unsigned char buf[1500];
    while(true)
    {
        int ret=recvfrom(sockfd,(char*)buf,1500,0,(struct sockaddr*)&vlp_addr, &addr_len);
        if(ret==1206)
        {
            unsigned char *dup = static_cast<unsigned char *> (malloc (ret * sizeof(unsigned char)));
            memcpy (dup,buf, ret * sizeof(unsigned char));
            std::unique_lock<std::mutex> lock(mutex_);
            vlp_data.push(dup);
            cond_.notify_one();
        }
        else
            cerr<<"length error:"<<ret<<endl;
    }
}

//consumer thread,decoding data queue according velodyne data sheet
void VLPGrabber::decode()
{
    static uint32_t scan_counter = 0;
    static uint32_t sweep_counter = 0;
    if (sizeof(VLPLaserReturn) != 3)
    {
        cerr<<"size of laser return error!"<<endl;
        exit(-1);
    }
    VLPDataPacket* dataPacket;
    while(true)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        while(vlp_data.empty())
        {
            cond_.wait(lock);
        }
        unsigned char*data=vlp_data.front();
        vlp_data.pop();

        dataPacket=reinterpret_cast<VLPDataPacket*>(data);
        time_t system_time;
        time (&system_time);
        time_t velodyne_time = (system_time & 0x00000000ffffffffl) << 32 | dataPacket->gpsTimestamp;
        scan_counter++;
        double interpolated_azimuth_delta;

        int index = 1;
        if (dataPacket->mode == VLP_DUAL_MODE)
        {
            index = 2;
        }
        if (dataPacket->firingData[index].rotationalPosition < dataPacket->firingData[0].rotationalPosition)
        {
            interpolated_azimuth_delta = ( (dataPacket->firingData[index].rotationalPosition + 36000) - dataPacket->firingData[0].rotationalPosition) / 2.0;
        }
        else
        {
            interpolated_azimuth_delta = (dataPacket->firingData[index].rotationalPosition - dataPacket->firingData[0].rotationalPosition) / 2.0;
        }

        for (int i = 0; i < VLP_FIRING_PER_PKT; ++i)
        {
            VLPFiringData firing_data = dataPacket->firingData[i];

            for (int j = 0; j < VLP_LASER_PER_FIRING; j++)
            {
                double current_azimuth = firing_data.rotationalPosition;
                if (j > VLP_MAX_NUM_LASERS)
                {
                    current_azimuth += interpolated_azimuth_delta;
                }
                if (current_azimuth > 36000)
                {
                    current_azimuth -= 36000;
                }
                if (current_azimuth <last_azimuth_)
                {
                    if (current_sweep_xyzi_.size () > 0)
                    {
                        sweep_counter++;
                        swap(current_sweep_xyzi_,complete_sweep_xyzi_);
                        {
                            std::unique_lock<std::mutex> lock(pointcloudmutex_);
                            pointcloudcond_.notify_one();
                        }
                    }
                    current_sweep_xyzi_.clear();
                }

                Point xyzi;
                Point dual_xyzi;
                computeXYZI (xyzi, current_azimuth, firing_data.laserReturns[j], laser_corrections_[j % VLP_MAX_NUM_LASERS]);

                if (dataPacket->mode == VLP_DUAL_MODE)
                {
                    computeXYZI (dual_xyzi, current_azimuth, dataPacket->firingData[i + 1].laserReturns[j],
                            laser_corrections_[j % VLP_MAX_NUM_LASERS]);
                }

                if (! (std::isnan (xyzi.x) || std::isnan (xyzi.y) || std::isnan (xyzi.z)))
                {
                    current_sweep_xyzi_.push_back (xyzi);
                    last_azimuth_ = current_azimuth;
                }
                if (dataPacket->mode == VLP_DUAL_MODE)
                {
                    if ( (dual_xyzi.x != xyzi.x || dual_xyzi.y != xyzi.y|| dual_xyzi.z != xyzi.z)
                         && ! (std::isnan (dual_xyzi.x) || (std::isnan (dual_xyzi.y) ) || (std::isnan (dual_xyzi.z))))
                    {
                        current_sweep_xyzi_.push_back (dual_xyzi);
                    }
                }
            }
            if (dataPacket->mode == VLP_DUAL_MODE)
            {
                i++;
            }
        }
        delete data;
    }
}

//init the look up table and load vlp16 corrections
void VLPGrabber::initialize ()
{
    if (cos_lookup_table_ == NULL && sin_lookup_table_ == NULL)
    {
        cos_lookup_table_ = static_cast<double *> (malloc (VLP_NUM_ROT_ANGLES * sizeof (*cos_lookup_table_)));
        sin_lookup_table_ = static_cast<double *> (malloc (VLP_NUM_ROT_ANGLES * sizeof (*sin_lookup_table_)));
        for (int i = 0; i < VLP_NUM_ROT_ANGLES; i++)
        {
            double rad = (M_PI / 180.0) * (static_cast<double> (i) / 100.0);
            cos_lookup_table_[i] = std::cos (rad);
            sin_lookup_table_[i] = std::sin (rad);
        }
    }
    loadVLP16Corrections ();

    for (int i = 0; i < VLP_MAX_NUM_LASERS; i++)
    {
        VLPLaserCorrection correction = laser_corrections_[i];
        laser_corrections_[i].sinVertOffsetCorrection = correction.verticalOffsetCorrection * correction.sinVertCorrection;
        laser_corrections_[i].cosVertOffsetCorrection = correction.verticalOffsetCorrection * correction.cosVertCorrection;
    }
}

void VLPGrabber::loadVLP16Corrections ()
{
    double vlp16_vertical_corrections[] = { -15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15};
    for (int i = 0; i < VLP_MAX_NUM_LASERS; i++)
    {
        VLPGrabber::laser_corrections_[i].azimuthCorrection = 0.0;
        VLPGrabber::laser_corrections_[i].distanceCorrection = 0.0;
        VLPGrabber::laser_corrections_[i].horizontalOffsetCorrection = 0.0;
        VLPGrabber::laser_corrections_[i].verticalOffsetCorrection = 0.0;
        VLPGrabber::laser_corrections_[i].verticalCorrection = vlp16_vertical_corrections[i];
        VLPGrabber::laser_corrections_[i].sinVertCorrection = std::sin (VLP_Grabber_toRadians(vlp16_vertical_corrections[i]));
        VLPGrabber::laser_corrections_[i].cosVertCorrection = std::cos (VLP_Grabber_toRadians(vlp16_vertical_corrections[i]));
    }
}

//convert point from polar coordinate to cartesian coordinate
void VLPGrabber::computeXYZI (Point &point,int azimuth,VLPLaserReturn laserReturn,
                              VLPLaserCorrection correction)
{
    double cos_azimuth, sin_azimuth;
    double distanceM = laserReturn.distance * 0.002;

    point.intensity = static_cast<float> (laserReturn.intensity);
    if (distanceM < min_distance_threshold_ || distanceM > max_distance_threshold_)
    {
        point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
        return;
    }
    if (correction.azimuthCorrection == 0)
    {
        cos_azimuth = cos_lookup_table_[azimuth];
        sin_azimuth = sin_lookup_table_[azimuth];
    }
    else
    {
        double azimuthInRadians = VLP_Grabber_toRadians( (static_cast<double> (azimuth) / 100.0) - correction.azimuthCorrection);
        cos_azimuth = std::cos (azimuthInRadians);
        sin_azimuth = std::sin (azimuthInRadians);
    }

    distanceM += correction.distanceCorrection;

    double xyDistance = distanceM * correction.cosVertCorrection;

    point.x = static_cast<float> (xyDistance * sin_azimuth - correction.horizontalOffsetCorrection * cos_azimuth);
    point.y = static_cast<float> (xyDistance * cos_azimuth + correction.horizontalOffsetCorrection * sin_azimuth);
    point.z = static_cast<float> (distanceM * correction.sinVertCorrection + correction.verticalOffsetCorrection);
    if (point.x == 0 && point.y == 0 && point.z == 0)
    {
        point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
    }
}

//user function thread,call the callback function to process pointcloud
void VLPGrabber::dataProcess()
{
    vector<Point> pointcloud;
    while(1)
    {
        {
            std::unique_lock<std::mutex> lock(pointcloudmutex_);
            pointcloudcond_.wait(lock);
            swap(pointcloud,complete_sweep_xyzi_);
        }
        if(pointcloud.size()!=0)
        {
            funptr(pointcloud);
        }
    }
}

VLPGrabber::VLPGrabber(uint32_t port):
    udp_port(port),last_azimuth_(65000),
    min_distance_threshold_ (0.0),
    max_distance_threshold_ (10000.0),
    cos_lookup_table_(NULL),
    sin_lookup_table_(NULL),
    udp_receive_thread(NULL),
    decode_receive_thread(NULL),
    data_process_thread(NULL),
    mutex_(),cond_(),funptr(NULL)
{
    initialize();
}

int VLPGrabber::start()
{
    if(funptr==NULL)
    {
        cerr<<"no data process callback function registered!"<<endl;
        exit(-1);
    }
    udp_receive_thread=new std::thread(&VLPGrabber::udp_receive,this);
    decode_receive_thread=new std::thread(&VLPGrabber::decode,this);
    data_process_thread=new std::thread(&VLPGrabber::dataProcess,this);
    return 0;
}

void VLPGrabber::registeCb(void (*funptr)(vector<Point> &))
{
    this->funptr=funptr;
}

