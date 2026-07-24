#pragma once

/// Conceptual layering (no concrete vendor types in core):
/// Transport: CAN / EtherCAT / Serial / TCP
/// DeviceDriver: Motor / IO / Battery / Gripper
/// Upper layers see device semantics (set_velocity, grasp), never sendFrame()/registers.

namespace robot_hardware_api
{
namespace transport
{

class ITransport
{
public:
  virtual ~ITransport() = default;
};

}  // namespace transport

namespace device
{

class IDeviceDriver
{
public:
  virtual ~IDeviceDriver() = default;
};

}  // namespace device
}  // namespace robot_hardware_api
