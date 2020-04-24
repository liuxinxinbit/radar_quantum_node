#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <netdb.h>
#include <errno.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <algorithm>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;
// size_t global_radar_resolution = 500;
// cv::Mat radar_img_final, tmp, radarcolorimg;
// cv::Mat radar_img = cv::Mat(250, 500, CV_8UC1, cv::Scalar(0));


// QUANTUM_SPOKE_PKT_HDR_T
struct QUANTUM_SPOKE_PKT_HDR_T
{
    uint16_t SequenceNumber;   // incrementing sequence number, wraps at 65535
    uint8_t CompressionFormat; // 0- None, 1-RLE, etc.
    uint8_t SpokesInPacket;    // Number of spokes in this packet 1-64.
};

// QUANTUM_SPOKE_DATA_HDR_T
struct QUANTUM_SPOKE_DATA_HDR_T
{
    uint16_t SamplesPerSpoke;   //!< count of samples in each spoke
    uint16_t SpokesPerScan;     //!< count of spokes in each scan
    uint8_t BitsPerSample;      //!< 8 = 8 bits max
    uint8_t Channel;            //!< 0-Default, 1- 2nd Range Dual Range Mode
    uint16_t InstrumentedRange; //!< sample offset for instrumented range
    uint16_t Bearing;           //!< 0-249 (max spokes per scan)
};

// QUANTUM_SPOKE_HEADER_T
struct QUANTUM_SPOKE_HEADER_T
{
    QUANTUM_SPOKE_PKT_HDR_T PktHdr;
    QUANTUM_SPOKE_DATA_HDR_T DataHdr;
    uint16_t DataLength; //!< length of spoke data field
};

// QUANTUM_SPOKE_T
struct QUANTUM_SPOKE_T
{
    QUANTUM_SPOKE_HEADER_T SpokeHeader;
    uint8_t SpokeData[1];
};

// QUANTUM_SPOKE_PACKET_T
struct SpokeData_t
{
    uint32_t MessageId;
    QUANTUM_SPOKE_T Spoke;
};


// QUANTUM_MODE_T
typedef uint8_t QUANTUM_MODE_T;
const QUANTUM_MODE_T QUANTUM_MODE_STANDBY         = 0;
const QUANTUM_MODE_T QUANTUM_MODE_TRANSMITTING    = 1; //!< Transmitting Mode
const QUANTUM_MODE_T QUANTUM_MODE_POWER_DOWN      = 3; //!< Powering Down Mode
const QUANTUM_MODE_T QUANTUM_MODE_TIMED_TX        = 4; //!< Timed Transmit Mode
const QUANTUM_MODE_T QUANTUM_MODE_SLEEP           = 5; //!< Rotation has stalled
const QUANTUM_MODE_T QUANTUM_MODE_STALLED         = 7; //!< Self test failed
const QUANTUM_MODE_T QUANTUM_MODE_SELFTEST_FAILED = 10; // QUANTUM_TX_FREQ_T

typedef uint8_t QUANTUM_TX_FREQ_T;
const QUANTUM_TX_FREQ_T QUANTUM_TX_FREQ_NOMINAL = 0;
const QUANTUM_TX_FREQ_T QUANTUM_TX_FREQ_LOW = 1;
const QUANTUM_TX_FREQ_T QUANTUM_TX_FREQ_HIGH = 2;

// QUANTUM_GAIN_MODE_T
typedef uint8_t QUANTUM_GAIN_MODE_T;
const QUANTUM_GAIN_MODE_T QUANTUM_GAIN_MODE_MANUAL = 0;
const QUANTUM_GAIN_MODE_T QUANTUM_GAIN_MODE_AUTO   = 1;

// QUANTUM_COLOUR_GAIN_MODE_T
typedef uint8_t QUANTUM_COLOUR_GAIN_MODE_T;
const QUANTUM_COLOUR_GAIN_MODE_T QUANTUM_COLOUR_GAIN_MODE_MANUAL = 0;
const QUANTUM_COLOUR_GAIN_MODE_T QUANTUM_COLOUR_GAIN_MODE_AUTO   = 1;

// QUANTUM_SEA_MODE_T
typedef uint8_t QUANTUM_SEA_MODE_T;
const QUANTUM_SEA_MODE_T QUANTUM_SEA_MODE_MANUAL = 0;
const QUANTUM_SEA_MODE_T QUANTUM_SEA_MODE_AUTO  = 1;

// QuantumSeaClutterCurve_t
typedef uint8_t QuantumSeaClutterCurve_t;
const QuantumSeaClutterCurve_t cQuantumSeaCurve_R4      = 0;
const QuantumSeaClutterCurve_t cQuantumSeaCurve_R5_5    = 1;
const QuantumSeaClutterCurve_t cQuantumSeaCurve_Default = cQuantumSeaCurve_R5_5;
const QuantumSeaClutterCurve_t cQuantumSeaCurve_Min     = cQuantumSeaCurve_R4;
const QuantumSeaClutterCurve_t cQuantumSeaCurve_Max     = cQuantumSeaCurve_R5_5;

// QUANTUM_RAIN_MODE_T
typedef uint8_t QUANTUM_RAIN_MODE_T;
const QUANTUM_RAIN_MODE_T QUANTUM_RAIN_MODE_OFF    = 0;
const QUANTUM_RAIN_MODE_T QUANTUM_RAIN_MODE_MANUAL = 1;

// QUANTUM_TARGET_EXPANSION_MODE_T
typedef uint8_t QUANTUM_TARGET_EXPANSION_MODE_T;
const QUANTUM_TARGET_EXPANSION_MODE_T QUANTUM_TARGET_EXPANSION_MODE_OFF = 0;
const QUANTUM_TARGET_EXPANSION_MODE_T QUANTUM_TARGET_EXPANSION_MODE_ON  = 1;

// QUANTUM_INTERFERENCE_REJECTION_MODE_T
typedef uint8_t QUANTUM_INTERFERENCE_REJECTION_MODE_T;
const QUANTUM_INTERFERENCE_REJECTION_MODE_T QUANTUM_INTERFERENCE_REJECTION_MODE_OFF = 0;
const QUANTUM_INTERFERENCE_REJECTION_MODE_T QUANTUM_INTERFERENCE_REJECTION_MODE_ON  = 1;
const QUANTUM_INTERFERENCE_REJECTION_MODE_T QUANTUM_INTERFERENCE_REJECTION_MODE_1 = 1;
const QUANTUM_INTERFERENCE_REJECTION_MODE_T QUANTUM_INTERFERENCE_REJECTION_MODE_2 = 2;
const QUANTUM_INTERFERENCE_REJECTION_MODE_T QUANTUM_INTERFERENCE_REJECTION_MODE_3 = 3;
// QUANTUM_INTERFERENCE_REMarpaTargetStatus_t
const QUANTUM_INTERFERENCE_REJECTION_MODE_T QUANTUM_INTERFERENCE_REJECTION_MODE_4 = 4;
const QUANTUM_INTERFERENCE_REJECTION_MODE_T QUANTUM_INTERFERENCE_REJECTION_MODE_5 = 5;

// QUANTUM_SCANNER_SPEED_T
typedef uint8_t QUANTUM_SCANNER_SPEED_T;
const QUANTUM_SCANNER_SPEED_T QUANTUM_SCANNER_SPEED_NORMAL = 0;
const QUANTUM_SCANNER_SPEED_T QUANTUM_SCANNER_SPEED_AUTO = 1;

struct QUANTUM_ZONE_T {
    uint32_t                                     m_StartDistance;
    uint32_t                                     m_EndDistance;
    uint16_t                                     m_StartAngle;
    uint16_t                                     m_EndAngle;
    bool                                         m_Enable;
    uint8_t                                      m_Spare[3];
};

struct QUANTUM_BLANK_SECTOR_T {
    uint16_t m_StartAngle;
    uint16_t m_EndAngle;
    bool                         m_Enable;
    uint8_t                      m_Spare[3];
};

// QUANTUM_PRESET_DATA_T
struct QUANTUM_PRESET_DATA_T {
    QUANTUM_GAIN_MODE_T        GainMode;
    uint8_t                    GainValue;
    QUANTUM_COLOUR_GAIN_MODE_T ColourGainMode;
    uint8_t                    ColourGainValue;
    QUANTUM_SEA_MODE_T         SeaClutterMode;
    uint8_t                    SeaClutterValue;
    QUANTUM_RAIN_MODE_T        RainClutterMode;
    uint8_t                    RainClutterValue;
};

// QUANTUM_PRESET_MODE_T
typedef uint8_t QUANTUM_PRESET_MODE_T;
const QUANTUM_PRESET_MODE_T cPresetModeHarbour   = 0;
const QUANTUM_PRESET_MODE_T cPresetModeCoastal   = 1;
const QUANTUM_PRESET_MODE_T cPresetModeOffshore  = 2;
const QUANTUM_PRESET_MODE_T cPresetModeWeather   = 3;
const QUANTUM_PRESET_MODE_T cPresetModeFirst     = cPresetModeHarbour;
const QUANTUM_PRESET_MODE_T cPresetModeLast      = cPresetModeWeather;
// QuantumMainBangSuppression_t
typedef uint8_t QuantumMainBangSuppression_t;
const QuantumMainBangSuppression_t cQuantumMainBangSuppressionOff = 0;
const QuantumMainBangSuppression_t cQuantumMainBangSuppressionOn  = 1;

// QUANTUM_RANGE_STATUS_T
const uint8_t QUANTUM_PRESET_MODES = 4;

struct QUANTUM_RANGE_STATUS_T {
    uint8_t                         RangeIndex;
    QUANTUM_PRESET_MODE_T           PresetMode;
    QUANTUM_PRESET_DATA_T           PresetData[QUANTUM_PRESET_MODES];
    QUANTUM_TARGET_EXPANSION_MODE_T TargetExpansionMode;
    QuantumSeaClutterCurve_t        SeaCurve;
    uint16_t                        ZeroRange;
    uint8_t                         STCKnee;
    QuantumMainBangSuppression_t    MainBangSuppression;
};

// QUANTUM_STATUS_T
const uint8_t QUANTUM_CHANNELS = 2;
const uint8_t QUANTUM_GUARD_ZONES = 2;
const uint8_t QUANTUM_BLANK_SECTORS = 2;
const uint8_t QUANTUM_MAX_RANGES = 20;
const uint8_t QUANTUM_AUTO_ACQUIRE_ZONES = 2;

struct QUANTUM_STATUS_T {
    QUANTUM_MODE_T                               RadarMode;
    uint8_t                                      Spare;
    uint8_t                                      StandbyTimeRemainingMins;
    uint8_t                                      StandbyTimeRemainingSecs;
    uint8_t                                      StandbyTime_Mins;
    uint8_t                                      TransmitCount_Scans;
    QUANTUM_TX_FREQ_T                            TxFreq;
    uint8_t                                      Unused;
    bool                                         DualRangeMode;
    QUANTUM_SCANNER_SPEED_T                      ScannerSpeed;
    int16_t                                      BearingAlignment;
    uint8_t                                      GuardZoneSensitivity;
    QUANTUM_INTERFERENCE_REJECTION_MODE_T        InterferenceRejectionMode;
    bool                                         StcEnable;
    bool                                         FtcEnable;
    QUANTUM_RANGE_STATUS_T                       RangeInfo[QUANTUM_CHANNELS];
    QUANTUM_ZONE_T                               GuardZone[QUANTUM_GUARD_ZONES];
    QUANTUM_BLANK_SECTOR_T                       BlankSector[QUANTUM_BLANK_SECTORS];
    uint32_t                                     CustomRanges[QUANTUM_MAX_RANGES];
    QUANTUM_ZONE_T                               AutoAcquireZone[QUANTUM_AUTO_ACQUIRE_ZONES];
};

struct Status_t {
    uint32_t m_MessageCode;
    QUANTUM_STATUS_T m_Status;
};

struct SetRadarMode_t {
    uint32_t m_MessageCode; //!< Message Id
    QUANTUM_MODE_T m_RadarMode; //!< Requested Mode for Radar
};

std::size_t RLE_DecodeSize(const std::vector<uint8_t> data)
{
    std::size_t len = 0;
    int32_t i = 0;
    while (i < data.size())
    {
        if (data[i] == 0x5C)
        {
            len += data[i + 1];
            i += 3;
        }
        else
        {
            ++len;
            ++i;
        }
    }
    return len;
}
std::vector<uint8_t> RLE_Decoding(const std::vector<uint8_t> data)
{
    std::vector<uint8_t> decode(RLE_DecodeSize(data));
    int i = 0;
    int index = 0;
    while (i < data.size())
    {
        if (data[i] == 0x5C)
        {
            memset(decode.data() + index, data[i + 2], data[i + 1]);
            index += data[i + 1];
            i += 3;
        }
        else
        {
            if (data[i] == 0x5B)
            {
                decode[index] = 0x5C;
                ++index;
            }
            else
            {
                decode[index] = data[i];
                ++index;
            }
            ++i;
        }
    }
    return std::move(decode);
}
std::vector<uint8_t> array2vector(const uint8_t *data, std::size_t size)
{
    std::vector<uint8_t> tmp(data, data + size);
    return tmp;
}
std::vector<uint8_t> RLE_Encoding(const std::vector<uint8_t> data)
{
    std::vector<uint8_t> encode(data);
    std::size_t data_size = data.size();
    if (data_size == 1)
        return encode;
    uint8_t last = data[0]; // last data
    uint8_t curr = 0;       // current data
    int32_t rlen = 1;       // run length
    int32_t alen = 0;       // all length
    for (int32_t i = 1; i < data_size; ++i)
    {
        curr = data[i];
        if (curr == last)
        {
            ++rlen;
        }
        else
        {
            if (rlen == 1)
            {
                if (last == 0x5C)
                {
                    encode[alen++] = 0x5B;
                }
                else
                {
                    encode[alen++] = last;
                }
            }
            else if (rlen < 4)
            {
                for (int i = 0; i < rlen; ++i)
                    encode[alen++] = last;
                rlen = 1;
            }
            else
            {
                encode[alen++] = 0x5C;
                encode[alen++] = rlen;
                encode[alen++] = last;
                rlen = 1;
            }
            last = curr;
        }
    }
    if (rlen == 1)
    {
        encode[alen++] = last;
    }
    else
    {
        encode[alen++] = 0x5C;
        encode[alen++] = rlen;
        encode[alen++] = last;
    }
    return std::move(
        std::vector<uint8_t>(encode.cbegin(), encode.cbegin() + alen));
}
std::string htoi(const std::string &hStr)
{
    return std::to_string(std::stoi(hStr, nullptr, 16));
}
std::vector<uint8_t> vector_crop(std::vector<uint8_t> v, size_t b, size_t e)
{
    std::vector<uint8_t> result;
    for (size_t i = b; i < e; i++)
        result.push_back(v.at(i));
    return result;
}
int64_t getCurrentTime() //直接调用这个函数就行了，返回值最好是int64_t，long long应该也可以
{
    struct timeval tv;
    gettimeofday(&tv, NULL); //该函数在sys/time.h头文件中
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}
class SetRadarMode
{
public:
    int send_num;
    double time_now;
    SetRadarMode();
    void send();

private:
    int sock_fd;
    struct sockaddr_in addr_serv;
    int len;
    SetRadarMode_t RadarMode;
};
SetRadarMode::SetRadarMode(void)
{
    sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0)
    {
        perror("socket");
    }
    memset(&addr_serv, 0, sizeof(addr_serv));
    addr_serv.sin_family = AF_INET;
    addr_serv.sin_addr.s_addr = inet_addr("192.168.2.155");
    addr_serv.sin_port = htons(2575);
    len = sizeof(addr_serv);
    time_now = getCurrentTime();
    RadarMode.m_MessageCode=0x00280010;
    RadarMode.m_RadarMode =QUANTUM_MODE_TRANSMITTING;
}
void SetRadarMode::send()
{
    send_num = sendto(sock_fd, (char *)&RadarMode, sizeof(RadarMode), 0, (struct sockaddr *)&addr_serv, len);
    if (send_num < 0)
    {
        perror("sendto error:");
    }
}
SetRadarMode SetRadarMode;
class radar_receiver
{
public:
    double time_now;
    size_t global_radar_resolution = 500;
    cv::Mat radar_img_final, tmp;
    uint show_radar_img = 0;
    cv::Mat radar_img = cv::Mat(250, 500, CV_8UC1, cv::Scalar(0));
    cv::Mat radarcolorimg = cv::Mat(700, 700, CV_8UC1, cv::Scalar(0));

    radar_receiver();
    void myrecvfrom();
    ~radar_receiver();

private:
    char group_ip[20];
    int group_port;
    int n;
    int socket_fd;
    struct sockaddr_in group_addr; //group address
    struct sockaddr_in local_addr; //local address
    struct ip_mreq mreq;
    socklen_t addr_len;
    u_int yes;
};
radar_receiver::radar_receiver(void)
{
    time_now = getCurrentTime();
    socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    strcpy(group_ip, "232.1.123.1");
    addr_len = sizeof(group_addr);
    ;
    group_port = 2574;
    if (socket_fd < 0)
    {
        perror("socket multicast!");
    }

    if (setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) < 0)
    {
        perror("Reusing ADDR failed");
    }
    memset(&group_addr, 0, sizeof(struct sockaddr_in));
    group_addr.sin_family = AF_INET;
    group_addr.sin_port = htons(group_port);
    group_addr.sin_addr.s_addr = inet_addr("230.1.1.1");
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    local_addr.sin_port = htons(group_port); //this port must be the group port
    /*bind local address*/
    if (bind(socket_fd, (struct sockaddr *)&local_addr, sizeof(local_addr)) == -1)
    {
        perror("Binding the multicast!");
    }
    /*use the setsocketopt() to request joining the multicast group*/

    mreq.imr_multiaddr.s_addr = inet_addr(group_ip);
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    if (setsockopt(socket_fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0)
    {
        perror("setsockopt multicast!");
    }
    /*loop to send or recieve*/
}
radar_receiver::~radar_receiver()
{
    close(socket_fd);
}
void radar_receiver::myrecvfrom()
{
    vector<uint8_t> recmsg(256, 0);
    n = recvfrom(socket_fd, recmsg.data(), recmsg.size(), 0,
                 (struct sockaddr *)&group_addr, &addr_len);
    if (n < 0)
    {
        printf("recvfrom err in udptalk!\n");
    }
    else
    {
        SpokeData_t *SpokeData = reinterpret_cast<SpokeData_t *>(recmsg.data());
        if (SpokeData->MessageId == 0x00280003)
        {
            for (;;)
            {
                if (recmsg.back() == 0x00 && recmsg.at(recmsg.size() - 3) != 0x5C)
                {
                    recmsg.pop_back();
                }
                else
                {
                    break;
                }
            }

            if (recmsg.size() > 20)
            {

                std::vector<uint8_t> data_vector, data_vector2;
                data_vector = vector_crop(recmsg, 20, recmsg.size());
                cv::Rect rect;
                data_vector2 = RLE_Decoding(data_vector);
                if (global_radar_resolution < data_vector2.size())
                {
                    radar_img = cv::Mat(250, 500, CV_8UC1, cv::Scalar(0));
                    global_radar_resolution = data_vector2.size();
                }
                else if (global_radar_resolution != data_vector2.size() && global_radar_resolution - data_vector2.size() > 5)
                {
                    radar_img = cv::Mat(250, 500, CV_8UC1, cv::Scalar(0));
                    global_radar_resolution = data_vector2.size();
                }
                else if (global_radar_resolution != data_vector2.size() && global_radar_resolution - data_vector2.size() == 1)
                {
                    data_vector2.push_back(0);
                }
                else if (global_radar_resolution != data_vector2.size() && global_radar_resolution - data_vector2.size() == 2)
                {
                    data_vector2.push_back(0);
                    data_vector2.push_back(0);
                }
                else if (global_radar_resolution != data_vector2.size() && global_radar_resolution - data_vector2.size() == 3)
                {
                    data_vector2.push_back(0);
                    data_vector2.push_back(0);
                    data_vector2.push_back(0);
                }

                if (global_radar_resolution != data_vector2.size())
                    ROS_INFO("%5d %5d", data_vector2.size(), global_radar_resolution);
                memcpy(radar_img.ptr<uint8_t>() + SpokeData->Spoke.SpokeHeader.DataHdr.Bearing * 500,
                       data_vector2.data(),
                       data_vector2.size());
                rect = cv::Rect(0, 0, data_vector2.size(), 250);
                tmp = radar_img(rect);
                cv::resize(tmp, tmp, cv::Size(700, 700));
                cv::linearPolar(tmp, radar_img_final, cv::Point(350, 350), 348,
                                CV_WARP_FILL_OUTLIERS | CV_INTER_LINEAR | CV_WARP_INVERSE_MAP);
                cv::applyColorMap(radar_img_final, radarcolorimg, cv::COLORMAP_JET);
                
                if (show_radar_img==1)
                {
                    cv::imshow("Lidar window", radarcolorimg);
                    cv::waitKey(1);
                }
            }
        }
        else if(SpokeData->MessageId == 0x00280002)
        {
            Status_t *Status = reinterpret_cast<Status_t *>(recmsg.data());
            if((uint)Status->m_Status.RadarMode==0)
            {
                double radarmode_timenow = getCurrentTime();
                if(radarmode_timenow-SetRadarMode.time_now>2000) SetRadarMode.send();
                
            }
        }
    }
}
struct Heartbeat_t
{
    uint32_t MessageCode;
    char HeartbeatText[8];
};
Heartbeat_t MakeHeartBeat()
{
    Heartbeat_t heartbeat;
    heartbeat.MessageCode = 0x00280000;
    memset(heartbeat.HeartbeatText, 0, sizeof(heartbeat.HeartbeatText));
    sprintf(heartbeat.HeartbeatText, "Radar");
    return heartbeat;
}
class Heartbeat_sender
{
public:
    int send_num;
    double time_now;
    Heartbeat_sender();
    void send();

private:
    int sock_fd;
    struct sockaddr_in addr_serv;
    int len;
    Heartbeat_t hb = MakeHeartBeat();
};
Heartbeat_sender::Heartbeat_sender(void)
{
    sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0)
    {
        perror("socket");
    }
    memset(&addr_serv, 0, sizeof(addr_serv));
    addr_serv.sin_family = AF_INET;
    addr_serv.sin_addr.s_addr = inet_addr("192.168.2.155");
    addr_serv.sin_port = htons(2575);
    len = sizeof(addr_serv);
    time_now = getCurrentTime();
}
void Heartbeat_sender::send()
{
    if (getCurrentTime() - time_now >= 1000)
    {
        send_num = sendto(sock_fd, (char *)&hb, 12, 0, (struct sockaddr *)&addr_serv, len);
        if (send_num < 0)
        {
            perror("sendto error:");
        }
        time_now = getCurrentTime();
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;
    radar_receiver radar_receiver;
    Heartbeat_sender Heartbeat_sender;
    if(argc>1) radar_receiver.show_radar_img = atoi(argv[1]);
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("radar/image", 1);
    
    // ros::Rate loop_rate(1);
    int64_t time_now = getCurrentTime();
    while (ros::ok())
    {
        cout<<getCurrentTime()<<endl;
        radar_receiver.myrecvfrom();
        Heartbeat_sender.send();
        if(getCurrentTime()-time_now>3600)
        {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", radar_receiver.radar_img_final).toImageMsg();
            pub.publish(msg);
        }
        // loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}