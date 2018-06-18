/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "FallingRelax.h"

#include <chrono>
#include <cmath>

#include "extension/Configuration.h"
#include "extension/Script.h"

#include "message/behaviour/ServoCommand.h"
#include "message/input/Sensors.h"

#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"
#include "utility/support/eigen_armadillo.h"

namespace module {
namespace behaviour {
    namespace skills {

        // internal only callback messages to start and stop our action
        struct Falling {};
        struct KillFalling {};

        using extension::Configuration;
        using extension::ExecuteScriptByName;

        using message::input::Sensors;

        using utility::behaviour::ActionPriorites;
        using utility::behaviour::RegisterAction;
        using LimbID  = utility::input::LimbID;
        using ServoID = utility::input::ServoID;

        FallingRelax::FallingRelax(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment))
            , id(size_t(this) * size_t(this) - size_t(this))
            , falling(false)
            , COM(0.0f)
            , falling_threshold(0.0f)
            , recovery_acceleration({0.0f, 0.0f})
            , priority(0.0f)
            , print_logs(false) {

            // do a little configurating
            on<Configuration>("FallingRelax.yaml").then([this](const Configuration& config) {
                COM               = config["COM"].as<float>();
                falling_threshold = config["falling_threshold"].as<float>();

                // Once the acceleration has stabalized, we are no longer falling
                recovery_acceleration = config["recovery_acceleration"].as<std::vector<float>>();

                priority = config["priority"].as<float>();

                print_logs = config["print_logs"].as<bool>();
            });

            on<Last<5, Trigger<Sensors>>, Single>().then([this](
                                                             const std::list<std::shared_ptr<const Sensors>>& sensors) {
                if (!sensors.empty()) {
                    double falling_angle = 0.0;
                    double acc_magnitude = 0.0;
                    double gyro_mag      = 0.0;

                    if (!falling) {
                        for (const auto& sensor : sensors) {
                            gyro_mag += sensor->gyroscope.squaredNorm();
                        }
                        gyro_mag = gyro_mag / sensors.size();

                        for (const auto& sensor : sensors) {
                            falling_angle += sensor->world(2, 2);
                        }
                        falling_angle = std::acos(std::abs(falling_angle / sensors.size()));

                        if (gyro_mag > ((2.0f * 9.8f / COM) * (1.0f - std::cos(falling_angle - falling_threshold)))) {
                            if (print_logs) {
                                log("FALLING! :",
                                    ((2.0f * 9.8f / COM) * (1.0f - std::cos(falling_angle - falling_threshold))),
                                    "<",
                                    gyro_mag);
                            }
                            falling = true;
                            updatePriority(priority);
                        }
                    }
                    else if (falling) {
                        for (const auto& sensor : sensors) {
                            acc_magnitude += sensor->accelerometer.norm();
                        }
                        acc_magnitude /= sensors.size();

                        // See if we recover
                        if (acc_magnitude > recovery_acceleration[0] && acc_magnitude < recovery_acceleration[1]) {
                            if (print_logs) {
                                log("NOT FALLING! :",
                                    acc_magnitude,
                                    recovery_acceleration[0],
                                    recovery_acceleration[1]);
                            }
                            falling = false;
                            updatePriority(0);
                        }
                    }
                }
            });

            on<Trigger<Falling>>().then([this] {
                emit(std::make_unique<ExecuteScriptByName>(id, "Relax.yaml"));
                // id, "Relax.yaml", NUClear::clock::now() + std::chrono::seconds(2)));
            });

            on<Trigger<KillFalling>>().then([this] {
                emit(std::make_unique<ExecuteScriptByName>(
                    id, "Relax.yaml", NUClear::clock::now() + std::chrono::seconds(2)));
                falling = false;
                updatePriority(0);
            });

            emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction{
                id,
                "Falling Relax",
                {std::pair<float, std::set<LimbID>>(
                    0, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM, LimbID::HEAD})},
                [this](const std::set<LimbID>&) { emit(std::make_unique<Falling>()); },
                [this](const std::set<LimbID>&) { emit(std::make_unique<KillFalling>()); },
                [this](const std::set<ServoID>&) {
                    // Ignore
                }}));
        }

        void FallingRelax::updatePriority(const float& priority) {
            emit(std::make_unique<ActionPriorites>(ActionPriorites{id, {priority}}));
        }

    }  // namespace skills
}  // namespace behaviour
}  // namespace module
