#ifndef CONTROLLER_BASE_H
#define CONTROLLER_BASE_H

#include <vector>
#include <stdint.h>

#include "libcdpr/inc/types.h"
#include "libgrabec/inc/slaves/goldsolowhistledrive.h"
#include "types.h"

template <typename T> using vect = std::vector<T>;

class ControllerBase
{
public:
  ControllerBase() {}
  explicit ControllerBase(const uint8_t motor_id);
  explicit ControllerBase(const vect<uint8_t>& motors_id);
  virtual ~ControllerBase();

  void SetMotorID(const uint8_t motor_id);
  void SetMotorsID(const vect<uint8_t>& motors_id);
  void SetMode(const int8_t mode);
  void SetMode(const uint8_t motor_id, const int8_t mode);

  vect<uint8_t> GetMotorsID() const { return motors_id_; }
  int8_t GetMode(const uint8_t motor_id) const;
  vect<int8_t> GetModes() const { return modes_; }

  virtual vect<MotorStatus> CalcCableSetPoint(const grabcdpr::Vars& robot_status) = 0;

protected:
  vect<uint8_t> motors_id_;
  vect<int8_t> modes_;
};

#endif // CONTROLLER_BASE_H
