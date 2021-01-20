// -*- C++ -*-
/*!
 * @file  Velodyne3D.cpp
 * @brief Velodyne3D Ranger
 * @date $Date$
 *
 * $Id$
 */

#define _USE_MATH_DEFINES
#include <math.h>
#include <boost/bind.hpp>
#include "Velodyne3D.h"

const double verticalCorrectionsHDL32[] = {
  -30.67, -9.3299999, -29.33, -8, -28,
  -6.6700001, -26.67, -5.3299999, -25.33, -4, -24, -2.6700001, -22.67,
  -1.33, -21.33, 0, -20, 1.33, -18.67, 2.6700001, -17.33, 4, -16, 5.3299999,
  -14.67, 6.6700001, -13.33, 8, -12, 9.3299999, -10.67, 10.67 };

const double verticalCorrectionsVLP16[] = {
  -15.0, 1.0, -13.0, 3.0, -11.0,
  5.0, -9.0, 7.0, -7.0, 9.0, 
  -5.0, 11.0, -3.0, 13.0, -1.0,
  15.0,
};

const uint32_t verticalIndexVLP16[] = {
  0, 8, 1, 9, 2,
  10, 3, 11, 4, 12,
  5, 13, 6, 14, 7,
  15
};

static const double* verticalCorrections = verticalCorrectionsVLP16;

static const uint32_t LASER_PER_FIRING = 16;

static double cos_lookup_table[HDL_NUM_ROT_ANGLES];
static double sin_lookup_table[HDL_NUM_ROT_ANGLES];
static double sinVerticalCorrection[LASER_PER_FIRING];

void initTables()
{
  for (unsigned int i = 0; i < HDL_NUM_ROT_ANGLES; i++) {
    double rad = HDL_Grabber_toRadians(i / 100.0);
    cos_lookup_table[i] = std::cos(rad);
    sin_lookup_table[i] = std::sin(rad);
  }

  for (int i = 0; i < LASER_PER_FIRING; i++) {
    sinVerticalCorrection[i] = std::sin(HDL_Grabber_toRadians(verticalCorrections[i]));
  }
}

// Module specification
// <rtc-template block="module_spec">
static const char* velodyne3d_spec[] =
  {
    "implementation_id", "Velodyne3D",
    "type_name",         "Velodyne3D",
    "description",       "Velodyne3D Ranger",
    "version",           "1.0.0",
    "vendor",            "SUGAR SWEET ROBOTICS",
    "category",          "Sensor",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debug_level", "0",
    "conf.default.correctionsFilePath", "none",

    // Widget
    "conf.__widget__.debug_level", "text",
    "conf.__widget__.correctionsFilePath", "text",
    // Constraints

    "conf.__type__.debug_level", "int",
    "conf.__type__.correctionsFilePath", "string",

    ""
  };
// </rtc-template>

void GetPacketCallback(Velodyne3D* pRTC, const boost::system::error_code& error, std::size_t num_bytes, std::string* data, unsigned int* data_length)
{
    std::cout << "Packet Receive...." << std::endl;
    (*data).assign(pRTC->_rx_buffer, num_bytes);
    *data_length = (unsigned int)num_bytes;
    return;
}

void velodyne_routine(Velodyne3D* pRTC) {
    pRTC->endflag_ = false;
    pRTC->readyFlag_[0] = pRTC->readyFlag_[1] = false;
    int now = 0;

    try {
        pRTC->_socket->async_receive(boost::asio::buffer(pRTC->_rx_buffer, 1500),
            boost::bind(GetPacketCallback, pRTC, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred, pRTC->data_, pRTC->dataLength_));
        pRTC->_io_service.reset();
        pRTC->_io_service.run();

    }
    catch (std::exception& e) {
        std::cout << "PacketDriver: Error receiving packet - " << e.what() << "." << std::endl;
        return;
    }


    while (!pRTC->endflag_) {
        try {
            PacketDecoder::HDLFrame latest_frame;

            /// pRTC->driver_.GetPacket(pRTC->data_, pRTC->dataLength_);

            pRTC->_socket->async_receive(boost::asio::buffer(pRTC->_rx_buffer, 1500),
                boost::bind(GetPacketCallback, pRTC, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred, pRTC->data_, pRTC->dataLength_));
            pRTC->_io_service.run();

            if (*pRTC->dataLength_ != 1206) {
                std::cout << "PacketDecoder: Warning, data packet is not 1206 bytes" << std::endl;
                continue;//  return RTC::RTC_OK;
            }

            // decoder_.DecodePacket(data_, dataLength_);



            unsigned char* data_char = const_cast<unsigned char*>(reinterpret_cast<const unsigned char*>(pRTC->data_->c_str()));
            HDLDataPacket* dataPacket = reinterpret_cast<HDLDataPacket*>(data_char);
            for (int i = 0; i < HDL_FIRING_PER_PKT; ++i) {
                HDLFiringData firingData = dataPacket->firingData[i];


                int offset = (firingData.blockIdentifier == BLOCK_0_TO_31) ? 0 : 32;



                if (firingData.rotationalPosition < pRTC->last_azimuth_) {
                    //std::cout << "packet_count_per_frame = " << packet_count_per_frame_ << "/" << last_azimuth_ << std::endl;
                    //packet_count_per_frame_ = 0;
                    pRTC->last_azimuth_ = 0;
                    
                    int next = now == 0 ? 1 : 0;
                    pRTC->readyFlag_[now] = true;
                    now = next;


                    for (int i = 0; i < HDL_NUM_ROT_ANGLES; i++) {
                        //memset(&(), 0, sizeof(double)*LASER_PER_FIRING);
                        for (int j = 0; j < LASER_PER_FIRING; j++) {
                            pRTC->buffer_[next].frames[i].ranges[j] = 0;//;.length(LASER_PER_FIRING);
                            pRTC->buffer_[next].frames[i].intensities[j] = 0;//.length(LASER_PER_FIRING);
                        }
                    }

                    continue;// return RTC::RTC_OK;
                }

                //packet_count_per_frame_++;
                pRTC->last_azimuth_ = firingData.rotationalPosition;
                //std::cout << "last_azimuth_ = " << last_azimuth_ << std::endl;
                //double azimuthInRadians = HDL_Grabber_toRadians((static_cast<double> (firingData.rotationalPosition) / 100.0));
                auto frameIndex = firingData.rotationalPosition + 18000;
                if (frameIndex >= HDL_NUM_ROT_ANGLES) {
                    frameIndex -= HDL_NUM_ROT_ANGLES;
                }
                //auto frameIndex = firingData.rotationalPosition;
                if (firingData.rotationalPosition >= HDL_NUM_ROT_ANGLES) break;
                for (int j = 0; j < LASER_PER_FIRING; j++) {
                    const int index = verticalIndexVLP16[j];
                    const double distanceM = firingData.laserReturns[j].distance * 0.002;
                    const unsigned char intensity = firingData.laserReturns[j].intensity;
                    if (firingData.rotationalPosition >= 0 && firingData.rotationalPosition < HDL_NUM_ROT_ANGLES) {
                        if (!pRTC->readyFlag_[now]) {
                            pRTC->buffer_[now].frames[frameIndex].ranges[index] = distanceM;
                            pRTC->buffer_[now].frames[frameIndex].intensities[index] = intensity;
                        }
                    }
                    else {
                        std::cout << "[Velodyne3D] " << "rotationPosition: " << firingData.rotationalPosition << std::endl;
                        break;
                    }
                }
            }
        } catch (std::exception& ex) {
            std::cout << "Exception: " << ex.what() << std::endl;
        }
    }

    pRTC->endflag_ = false;
}

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
Velodyne3D::Velodyne3D(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_range3dOut("range3d", m_range3d)

    // </rtc-template>
{
  
  initTables();
}

/*!
 * @brief destructor
 */
Velodyne3D::~Velodyne3D()
{
}



RTC::ReturnCode_t Velodyne3D::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  addOutPort("range3d", m_range3dOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debug_level", m_debug_level, "0");
  bindParameter("correctionsFilePath", m_correctionsFilePath, "none");
  // </rtc-template>

  data_ = new std::string();
  dataLength_ = new unsigned int();
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Velodyne3D::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Velodyne3D::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Velodyne3D::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

static void init_3d_data(LiDAR3D::RangeData3D* data) {

    /// SEtting for VLP-16
    data->frames.length(HDL_NUM_ROT_ANGLES);
    for (int i = 0; i < HDL_NUM_ROT_ANGLES; i++) {
        data->frames[i].ranges.length(LASER_PER_FIRING);
        data->frames[i].intensities.length(LASER_PER_FIRING);
        //data->frames[i].azimuthAngle = HDL_Grabber_toRadians((static_cast<double> (i) / 100.0));
    }
    data->config.minAltitudeAngle = HDL_Grabber_toRadians(-15.0);
    data->config.maxAltitudeAngle = HDL_Grabber_toRadians(15.0);
    data->config.altitudeAngularRes = HDL_Grabber_toRadians(2.0);

    data->config.minAzimuthAngle = HDL_Grabber_toRadians(-180);
    data->config.maxAzimuthAngle = HDL_Grabber_toRadians(359.0 - 180);
    data->config.azimuthAngularRes = HDL_Grabber_toRadians(1 / 100.0);

    data->config.minRange = 0.3;
    data->config.maxRange = 100.0;
    data->config.rangeRes = 0.002;
    data->config.frequency = 0;

    data->geometry.size.l = 0;
    data->geometry.size.w = 0;
    data->geometry.size.h = 0;
    data->geometry.pose.position.x = 0;
    data->geometry.pose.position.y = 0;
    data->geometry.pose.position.z = 0;
    data->geometry.pose.orientation.r = 0;
    data->geometry.pose.orientation.p = 0;
    data->geometry.pose.orientation.y = 0;

}

RTC::ReturnCode_t Velodyne3D::onActivated(RTC::UniqueId ec_id)
{

  last_azimuth_ = 0;
  packet_count_per_frame_ = 0;

  //driver_.InitPacketDriver(DATA_PORT);
  //decoder_.SetCorrectionsFile("32db.xml");
  //data_ = new std::string();
  //dataLength_ = new unsigned int();
  
  boost::asio::ip::udp::endpoint destination_endpoint(boost::asio::ip::address_v4::any(), DATA_PORT);

  try {
      _socket = boost::shared_ptr<boost::asio::ip::udp::socket>(new boost::asio::ip::udp::socket(_io_service));
      _socket->open(destination_endpoint.protocol());
      _socket->bind(destination_endpoint);
  }
  catch (std::exception& e) {
      std::cout << "PacketDriver: Error binding to socket - " << e.what() << ". Trying once more..." << std::endl;
      try {
          destination_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::any(), DATA_PORT);
          _socket = boost::shared_ptr<boost::asio::ip::udp::socket>(new boost::asio::ip::udp::socket(_io_service));
          _socket->open(destination_endpoint.protocol());
          _socket->bind(destination_endpoint);
      }
      catch (std::exception& e) {
          std::cout << "PacketDriver: Error binding to socket - " << e.what() << ". Failed!" << std::endl;
          return RTC::RTC_ERROR; 
      }
  }

  std::cout << "PacketDriver: Success binding to Velodyne socket!" << std::endl;
  
  
  
  init_3d_data(&m_range3d);
  init_3d_data(buffer_);
  init_3d_data(buffer_ + 1);


  thread_ = new std::thread(velodyne_routine, this);
  return RTC::RTC_OK;
}


RTC::ReturnCode_t Velodyne3D::onDeactivated(RTC::UniqueId ec_id)
{
    endflag_ = true;
    thread_->join();
    delete thread_;
  delete data_;
  delete dataLength_;


  _socket->close();
  _io_service.stop();

  return RTC::RTC_OK;
}


/*
void PacketDecoder::PushFiringData(unsigned char laserId, unsigned short azimuth, unsigned int timestamp, HDLLaserReturn laserReturn, HDLLaserCorrection correction)
{
  double cosAzimuth, sinAzimuth;
  if (correction.azimuthCorrection == 0) {
    cosAzimuth = cos_lookup_table[azimuth];
    sinAzimuth = sin_lookup_table[azimuth];
  } else {
    double azimuthInRadians = HDL_Grabber_toRadians((static_cast<double> (azimuth) / 100.0) - correction.azimuthCo
    
    rrection);
    cosAzimuth = std::cos(azimuthInRadians);
    sinAzimuth = std::sin(azimuthInRadians);
  }

  double distanceM = laserReturn.distance * 0.002 + correction.distanceCorrection;
  double xyDistance = distanceM * correction.cosVertCorrection - correction.sinVertOffsetCorrection;

  double x = (xyDistance * sinAzimuth - correction.horizontalOffsetCorrection * cosAzimuth);
  double y = (xyDistance * cosAzimuth + correction.horizontalOffsetCorrection * sinAzimuth);
  double z = (distanceM * correction.sinVertCorrection + correction.cosVertOffsetCorrection);
  unsigned char intensity = laserReturn.intensity;

  _frame->x.push_back(x);
  _frame->y.push_back(y);
  _frame->z.push_back(z);
  _frame->intensity.push_back(intensity);
  _frame->laser_id.push_back(laserId);
  _frame->azimuth.push_back(azimuth);
  _frame->distance.push_back(distanceM);
  _frame->ms_from_top_of_hour.push_back(timestamp);
}

*/
RTC::ReturnCode_t Velodyne3D::onExecute(RTC::UniqueId ec_id)
{
    for (int i = 0; i < 2; i++) {
        if (readyFlag_[i]) {

//            m_range3d = buffer_[i];
            setTimestamp(m_range3d);
            m_range3dOut.write(buffer_[i]);
            readyFlag_[i] = false;
        }
    }
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Velodyne3D::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Velodyne3D::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t Velodyne3D::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Velodyne3D::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Velodyne3D::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void Velodyne3DInit(RTC::Manager* manager)
  {
    coil::Properties profile(velodyne3d_spec);
    manager->registerFactory(profile,
                             RTC::Create<Velodyne3D>,
                             RTC::Delete<Velodyne3D>);
  }
  
};


