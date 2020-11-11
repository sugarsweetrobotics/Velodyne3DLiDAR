// -*- C++ -*-
/*!
 * @file  Velodyne3DTest.cpp
 * @brief Velodyne3D Ranger
 * @date $Date$
 *
 * $Id$
 */

#include "Velodyne3DTest.h"

// Module specification
// <rtc-template block="module_spec">
static const char* velodyne3d_spec[] =
  {
    "implementation_id", "Velodyne3DTest",
    "type_name",         "Velodyne3DTest",
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

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
Velodyne3DTest::Velodyne3DTest(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_range3dOut("range3d", m_range3d)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
Velodyne3DTest::~Velodyne3DTest()
{
}



RTC::ReturnCode_t Velodyne3DTest::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("range3d", m_range3dIn);
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debug_level", m_debug_level, "0");
  bindParameter("correctionsFilePath", m_correctionsFilePath, "none");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Velodyne3DTest::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Velodyne3DTest::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Velodyne3DTest::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t Velodyne3DTest::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t Velodyne3DTest::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t Velodyne3DTest::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Velodyne3DTest::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Velodyne3DTest::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t Velodyne3DTest::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Velodyne3DTest::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Velodyne3DTest::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void Velodyne3DTestInit(RTC::Manager* manager)
  {
    coil::Properties profile(velodyne3d_spec);
    manager->registerFactory(profile,
                             RTC::Create<Velodyne3DTest>,
                             RTC::Delete<Velodyne3DTest>);
  }
  
};


