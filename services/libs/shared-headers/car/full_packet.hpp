#pragma once

#include "fsm/machine_component.hpp"
#include "motors/motors_component.hpp"
#include "sensors/sensors_telemetry.hpp"
#include "services/services_telemetry.hpp"
#include "servo/servo_telemetry.hpp"
#include <stdint.h>

namespace shared::car{
    struct CarTelemetry{
        fsm::FSMTelemetry fsm;
        motors::MotorsComponentTelemetry motors;
        sensors::SensorsTelemetry sensors;
        services::ServiceTelemetry services;
        servos::ServoTelemetry servo;
    };
}