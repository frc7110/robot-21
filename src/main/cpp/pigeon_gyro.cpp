// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "pigeon_gyro.h"

#include <hal/FRCUsageReporting.h>

#include "frc/DriverStation.h"
#include "frc/Timer.h"
#include "frc/smartdashboard/SendableRegistry.h"

using namespace frc;

static constexpr auto kSamplePeriod = 0.0005_s;
static constexpr double kCalibrationSampleTime = 5.0;
static constexpr double kDegreePerSecondPerLSB = 0.0125;

// static constexpr int kRateRegister = 0x00;
// static constexpr int kTemRegister = 0x02;
// static constexpr int kLoCSTRegister = 0x04;
// static constexpr int kHiCSTRegister = 0x06;
// static constexpr int kQuadRegister = 0x08;
// static constexpr int kFaultRegister = 0x0A;
static constexpr int kPIDRegister = 0x0C;
// static constexpr int kSNHighRegister = 0x0E;
// static constexpr int kSNLowRegister = 0x10;

//pigeon_gyro::pigeon_gyro() : pigeon_gyro(SPI::kOnboardCS0) {}

pigeon_gyro::pigeon_gyro(int can_id)
    : m_can_id(can_id), m_angle(0), m_rate(0), m_simDevice("Gyro:CTRE_pigeon", can_id) {
  if (m_simDevice) {
//  m_simAngle =
//      m_simDevice.CreateDouble("angle_x", hal::SimDevice::kInput, 0.0);
//  m_simRate = m_simDevice.CreateDouble("rate_x", hal::SimDevice::kInput, 0.0);
  }

  if (!m_simDevice) {
      _pidgey = new PigeonIMU(can_id); /* Pigeon is on CANBus (powered from ~12V, and has a device ID of zero */
      _pidgey->ConfigFactoryDefault();

    // Validate the part ID
//  if ((ReadRegister(kPIDRegister) & 0xff00) != 0x5200) {
//    DriverStation::ReportError("could not find ADXRS450 gyro");
//    return;
//  }

//  m_spi.InitAccumulator(kSamplePeriod, 0x20000000u, 4, 0x0c00000eu,
//                        0x04000000u, 10u, 16u, true, true);

    Calibrate();
  }

  HAL_Report(HALUsageReporting::kResourceType_ADXRS450, m_can_id + 1);

  SendableRegistry::GetInstance().AddLW(this, "pigeon_gyro", m_can_id);
}

double pigeon_gyro::GetAngle() const {
  if (m_simAngle) {
    return m_simAngle.Get();
  }

  bool angleIsGood = (_pidgey->GetState() == PigeonIMU::Ready) ? true : false;

  if (angleIsGood)
  {
    PigeonIMU::FusionStatus stat;
    _pidgey->GetFusedHeading(stat);
    m_angle = stat.heading;
  }

  return m_angle;
}

double pigeon_gyro::GetRate() const {
  if (m_simRate) {
    return m_simRate.Get();
  }

  bool angleIsGood = (_pidgey->GetState() == PigeonIMU::Ready) ? true : false;

  if (angleIsGood)
  {
    double xyz_dps[3];
    _pidgey->GetRawGyro(xyz_dps);
    m_rate = xyz_dps[2];
  }

  return m_rate;
}

void pigeon_gyro::Reset() {
  if (m_simAngle) {
//  m_simAngle.Reset();
  }

  /* nonzero to block the config until success, zero to skip checking */
    const int kTimeoutMs = 30;
    _pidgey->SetFusedHeading(0.0, kTimeoutMs); /* reset heading, angle measurement wraps at plus/minus 23,040 degrees (64 rotations) */
    m_angle = m_rate = 0;
}

void pigeon_gyro::Calibrate() {
  Wait(0.1);

//m_spi.SetAccumulatorIntegratedCenter(0);
//m_spi.ResetAccumulator();

  Wait(kCalibrationSampleTime);

//m_spi.SetAccumulatorIntegratedCenter(m_spi.GetAccumulatorIntegratedAverage());
//m_spi.ResetAccumulator();
}

int pigeon_gyro::GetPort() const {
    return m_can_id;
}
