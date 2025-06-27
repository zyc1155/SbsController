//////////////////////////////////////////////////////////
// ROSfalcon Driver. Publishes and subscribes to falconMsgs for Novint Falcon.
//
// Using LibniFalcon
// Steven Martin
// Based on barrow_mechanics example by Alistair Barrow

#include "driver.h"

/**********************************************
This function initialises the novint falcon controller

NoFalcons is the index of the falcon which you wish to initialise
Index 0 is first falcon.
**********************************************/
Falcon_Driver::Falcon_Driver(int NoFalcon)
{
	m_falconDevice.setFalconFirmware<FalconFirmwareNovintSDK>(); // Set Firmware
	try
	{
		m_falconDevice.open(NoFalcon);
	}
	catch (const std::exception &e)
	{
		mc_rtc::log::error_and_throw<std::runtime_error>("Failed to find falcon");
	}
	mc_rtc::log::success("Falcon {} Found", NoFalcon);

	// There's only one kind of firmware right now, so automatically set that.
	m_falconDevice.setFalconFirmware<FalconFirmwareNovintSDK>();
	// Next load the firmware to the device

	bool skip_checksum = false;
	// See if we have firmware
	bool firmware_loaded = false;
	firmware_loaded = m_falconDevice.isFirmwareLoaded();
	if (!firmware_loaded)
	{
		// cout << "Loading firmware" << endl;
		uint8_t *firmware_block;
		long firmware_size;
		{

			firmware_block = const_cast<uint8_t *>(NOVINT_FALCON_NVENT_FIRMWARE);
			firmware_size = NOVINT_FALCON_NVENT_FIRMWARE_SIZE;

			for (int i = 0; i < 20; ++i)
			{
				if (!m_falconDevice.getFalconFirmware()->loadFirmware(skip_checksum, NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t *>(NOVINT_FALCON_NVENT_FIRMWARE)))

				{
					mc_rtc::log::info("Firmware loading try failed");
				}
				else
				{
					firmware_loaded = true;
					break;
				}
			}
		}
	}
	else if (!firmware_loaded)
	{
		mc_rtc::log::error_and_throw<std::runtime_error>("No firmware loaded to device, and no firmware specified to load (--nvent_firmware, --test_firmware, etc...). Cannot continue");
	}
	if (!firmware_loaded || !m_falconDevice.isFirmwareLoaded())
	{
		mc_rtc::log::error_and_throw<std::runtime_error>("No firmware loaded to device, cannot continue");
	}
	// mc_rtc::log::success("Firmware loaded");

	m_falconDevice.getFalconFirmware()->setHomingMode(true); // Set homing mode (keep track of encoders !needed!)
	// mc_rtc::log::info("Homing Set");
	std::array<int, 3> forces = {0, 0, 0};
	m_falconDevice.getFalconFirmware()->setForces(forces);
	m_falconDevice.runIOLoop(); // read in data
	{
		bool stop = false;
		bool homing = false;
		bool homing_reset = false;
		usleep(100000);
		int tryLoad = 0;
		while (!stop) //&& tryLoad < 100
		{
			if (!m_falconDevice.runIOLoop())
				continue;
			if (!homing)
			{
				if (!m_falconDevice.getFalconFirmware()->isHomed())
				{
					m_falconDevice.getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::RED_LED);
					mc_rtc::log::info("Falcon {} not currently homed. Move control all the way out then push straight all the way in.", NoFalcon);
				}

				homing = true;
			}

			if (homing && m_falconDevice.getFalconFirmware()->isHomed())
			{
				m_falconDevice.getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::BLUE_LED);
				mc_rtc::log::info("Falcon {} homed.", NoFalcon);
				homing_reset = true;
				stop = true;
			}
			tryLoad++;
		}
		while (!m_falconDevice.runIOLoop())
			;
	}

	m_falconDevice.setFalconKinematic<libnifalcon::FalconKinematicStamper>();
}

Falcon_Driver::~Falcon_Driver()
{
	m_falconDevice.close();
}

Eigen::Vector3d Falcon_Driver::Get_Pos()
{
	std::array<double, 3> Pos;
	Eigen::Vector3d r_Pos;
	m_falconDevice.runIOLoop();
	Pos = m_falconDevice.getPosition();

	r_Pos << Pos[0], Pos[1], Pos[2];

	return r_Pos;
}