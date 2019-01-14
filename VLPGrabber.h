/* author：Han Zhengyong
 * date:2016/11/20
 * descripe:it's driver of velodyne16 lidar adapted from PCL
*/

#ifndef VLPGRABBER_H
#define VLPGRABBER_H

extern "C"
{
#include <stdint.h>
#include <sys/types.h>
#ifdef WIN32
#include <WinSock2.h>
#pragma comment(lib,"ws2_32.lib")
#else
#ifdef __MINGW32__
#include <winsock2.h>
#else
#include <netinet/in.h>
#include <sys/socket.h>
#include <semaphore.h>
#endif
#endif
}
#include <iostream>
#include <queue>
#include <vector>
#include <thread>
#include <condition_variable>
#include <mutex>
using namespace std;

#define DEFAULR_PORT 2368
#define VLP_NUM_ROT_ANGLES  36001
#define VLP_LASER_PER_FIRING 32
#define VLP_MAX_NUM_LASERS 16
#define VLP_FIRING_PER_PKT 12
#define VLP_DUAL_MODE 0x39
# define M_PI		3.14159265358979323846
#define VLP_Grabber_toRadians(x) ((x) * M_PI / 180.0)

/*strucure of original lidar data from udp stream*/
#pragma pack(push, 1)
typedef struct VLPLaserReturn
{
    unsigned short distance;
    unsigned char intensity;
} VLPLaserReturn;

struct VLPFiringData
{
    unsigned short blockIdentifier;
    unsigned short rotationalPosition;
    VLPLaserReturn laserReturns[VLP_LASER_PER_FIRING];
};

struct VLPDataPacket
{
    VLPFiringData firingData[VLP_FIRING_PER_PKT];
    unsigned int gpsTimestamp;
    unsigned char mode;
    unsigned char sensorType;
};

/*structure of a single point*/
typedef struct
{
    float x;
    float y;
    float z;
    float intensity;
}Point;
#pragma pack(pop)

struct VLPLaserCorrection
{
    double azimuthCorrection;
    double verticalCorrection;
    double distanceCorrection;
    double verticalOffsetCorrection;
    double horizontalOffsetCorrection;
    double sinVertCorrection;
    double cosVertCorrection;
    double sinVertOffsetCorrection;
    double cosVertOffsetCorrection;
};

/*class of lidar grabber*/
class VLPGrabber
{
private:
    uint32_t udp_port;
    float min_distance_threshold_;
    float max_distance_threshold_;
    uint32_t last_azimuth_;
    VLPLaserCorrection laser_corrections_[VLP_MAX_NUM_LASERS];
    vector<Point > current_sweep_xyzi_;
    vector<Point > complete_sweep_xyzi_;
    //lookup table of cos and sin function
    double *cos_lookup_table_;
    double *sin_lookup_table_;
    /*use producer and consumer mode
     to convert data from udp receive thread and protocal decode thread*/
    queue <unsigned char*> vlp_data;
    std::mutex mutex_;
    std::condition_variable cond_;
    /*mutex to ensure the completeness of a pointcloud when sending data from
     protocal decode thread to user function*/
    std::mutex pointcloudmutex_;
    std::condition_variable pointcloudcond_;
    std::thread *udp_receive_thread;
    std::thread *decode_receive_thread;
    std::thread *data_process_thread;
    void (*funptr)(vector<Point> &);

    void loadVLP16Corrections ();
    void initialize ();
    void udp_receive();
    void decode();
    void computeXYZI (Point &point, int azimuth, VLPLaserReturn laserReturn, VLPLaserCorrection correction);
    void dataProcess();
public:
    VLPGrabber(uint32_t port=DEFAULR_PORT);
    int start();
    void registeCb( void (*funptr)(vector<Point> &));
};
#endif // VLPGRABBER_H
