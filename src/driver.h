#pragma once

#include <mc_control/mc_controller.h>

#include "falcon/core/FalconDevice.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconCLIBase.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/stamper/StamperUtils.h"
#include "falcon/kinematic/FalconKinematicStamper.h"
#include "falcon/core/FalconGeometry.h"
#include "falcon/gmtl/gmtl.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"

using namespace libnifalcon;
//using namespace std;
using namespace StamperKinematicImpl;

class Falcon_Driver
{
public:
    Falcon_Driver(int NoFalcon);
    ~Falcon_Driver();
    Eigen::Vector3d Get_Pos();
private:
    FalconDevice m_falconDevice;
};
