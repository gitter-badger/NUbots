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

#include <cmath>

#include "SimpleWalkPathPlanner.h"


#include "extension/Configuration.h"

#include <cmath>

#include "message/behaviour/KickPlan.h"
#include "message/behaviour/MotionCommand.h"
#include "message/behaviour/Subsumption.h"
#include "message/input/Sensors.h"
#include "message/localisation/FieldObject.h"
#include "message/motion/KickCommand.h"
#include "message/motion/WalkCommand.h"
#include "message/vision/VisionObjects.h"

#include "utility/behaviour/Action.h"
#include "utility/behaviour/MotionCommand.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"
#include "utility/localisation/transform.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/matrix/Transform3D.h"

#include "utility/nubugger/NUhelpers.h"
#include "utility/support/eigen_armadillo.h"

#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"


namespace module {
namespace behaviour {
    namespace planning {

        using extension::Configuration;

        using LimbID  = utility::input::LimbID;
        using ServoID = utility::input::ServoID;
        using message::input::Sensors;

        using message::motion::WalkCommand;
        using message::behaviour::KickPlan;
        using message::behaviour::MotionCommand;
        using message::motion::KickFinished;
        using message::motion::StopCommand;
        using message::behaviour::WantsToKick;
        using utility::localisation::transform::RobotToWorldTransform;
        using utility::localisation::transform::WorldToRobotTransform;
        using utility::math::matrix::Transform2D;
        using utility::math::matrix::Transform3D;
        using utility::nubugger::graph;
        using utility::nubugger::drawSphere;

        using utility::behaviour::RegisterAction;
        using utility::behaviour::ActionPriorites;

        using message::motion::WalkStopped;
        using message::motion::EnableWalkEngineCommand;
        using message::motion::DisableWalkEngineCommand;

        using message::localisation::Self;
        using message::vision::Ball;


        SimpleWalkPathPlanner::SimpleWalkPathPlanner(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment))
            , latestCommand(utility::behaviour::StandStill())
            , subsumptionId(size_t(this) * size_t(this) - size_t(this))
            , currentTargetPosition(arma::fill::zeros)
            , currentTargetHeading(arma::fill::zeros)
            , targetHeading(Eigen::Vector2d::Zero(), KickPlan::KickType::SCRIPTED)
            , timeBallLastSeen(NUClear::clock::now()) {

            // do a little configurating
            on<Configuration>("SimpleWalkPathPlanner.yaml").then([this](const Configuration& file) {

                turnSpeed            = file.config["turnSpeed"].as<float>();
                forwardSpeed         = file.config["forwardSpeed"].as<float>();
                sideSpeed            = file.config["sideSpeed"].as<float>();
                a                    = file.config["a"].as<float>();
                b                    = file.config["b"].as<float>();
                search_timeout       = file.config["search_timeout"].as<float>();
                robot_ground_space   = file.config["robot_ground_space"].as<bool>();
                ball_approach_dist   = file.config["ball_approach_dist"].as<float>();
                slowdown_distance    = file.config["slowdown_distance"].as<float>();
                useLocalisation      = file.config["useLocalisation"].as<bool>();
                slow_approach_factor = file.config["slow_approach_factor"].as<float>();

                emit(std::make_unique<WantsToKick>(false));
            });

            emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction{
                subsumptionId,
                "Simple Walk Path Planner",
                {
                    // Limb sets required by the walk engine:
                    std::pair<double, std::set<LimbID>>(0, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG}),
                    std::pair<double, std::set<LimbID>>(0, {LimbID::LEFT_ARM, LimbID::RIGHT_ARM}),
                },
                [this](const std::set<LimbID>& givenLimbs) {
                    if (givenLimbs.find(LimbID::LEFT_LEG) != givenLimbs.end()) {
                        // Enable the walk engine.
                        emit<Scope::DIRECT>(std::move(std::make_unique<EnableWalkEngineCommand>(subsumptionId)));
                    }
                },
                [this](const std::set<LimbID>& takenLimbs) {
                    if (takenLimbs.find(LimbID::LEFT_LEG) != takenLimbs.end()) {
                        // Shut down the walk engine, since we don't need it right now.
                        emit<Scope::DIRECT>(std::move(std::make_unique<DisableWalkEngineCommand>(subsumptionId)));
                    }
                },
                [this](const std::set<ServoID>&) {
                    // nothing
                }}));

            on<Trigger<WalkStopped>>().then([this] {
                emit(std::make_unique<ActionPriorites>(ActionPriorites{subsumptionId, {0, 0}}));
            });

            // on<Trigger<std::vector<Ball>>>().then([this]{
            //     log("std::vector<Ball>");
            // });
            // on<Trigger<std::vector<Self>>>().then([this]{
            //     log("std::vector<Self>");
            // });
            // on<Trigger<KickPlan>>().then([this]{
            //     log("KickPlan");
            // });
            // on<Trigger<WantsToKick>>().then([this]{
            //     log("WantsToKick");
            // });
            // on<Trigger<Sensors>>().then([this]{
            //     log("Sensors");
            // });

            on<Every<20, Per<std::chrono::seconds>>,
               With<std::vector<Ball>>,
               With<std::vector<Self>>,
               With<Sensors>,
               With<WantsToKick>,
               With<KickPlan>,
               Sync<SimpleWalkPathPlanner>>()
                .then([this](const std::vector<Ball>& ball,
                             const std::vector<Self>& selfs,
                             const Sensors& sensors,
                             const WantsToKick& wantsTo,
                             const KickPlan& kickPlan) {
                    if (wantsTo.kick) {
                        emit(std::make_unique<StopCommand>(subsumptionId));
                        return;
                    }

                    if (latestCommand.type == message::behaviour::MotionCommand::Type::StandStill) {


                        emit(std::make_unique<StopCommand>(subsumptionId));
                        // emit(std::make_unique<ActionPriorites>(ActionPriorites { subsumptionId, { 40, 11 }}));

                        return;
                    }
                    else if (latestCommand.type == message::behaviour::MotionCommand::Type::DirectCommand) {
                        // TO DO, change to Bezier stuff
                        std::unique_ptr<WalkCommand> command =
                            std::make_unique<WalkCommand>(subsumptionId, latestCommand.walkCommand);
                        emit(std::move(command));
                        emit(std::make_unique<ActionPriorites>(ActionPriorites{subsumptionId, {40, 11}}));
                        return;
                    }

                    Transform3D Htw = convert<double, 4, 4>(sensors.world);
                    auto now        = NUClear::clock::now();
                    float timeSinceBallSeen =
                        std::chrono::duration_cast<std::chrono::nanoseconds>(now - timeBallLastSeen).count()
                        * (1 / std::nano::den);

                    // TODO: support non-ball targets
                    if (!robot_ground_space) {
                        if (ball.size() > 0) {
                            arma::vec2 rBWw_vec2 = convert<double, 2>(ball[0].position.head<2>());
                            rBWw[0]              = rBWw_vec2[0];
                            rBWw[1]              = rBWw_vec2[1];

                            timeBallLastSeen = now;
                            // log("ball seen");
                        }
                        else {
                            rBWw = timeSinceBallSeen < search_timeout ? rBWw :  // Place last seen
                                       Htw.x() + Htw.translation();             // In front of the robot
                        }
                        arma::vec3 position3d = Htw.transformPoint(rBWw);
                        position[0]           = position3d[0];
                        position[1]           = position3d[1];
                    }
                    else {
                        if (ball.size() > 0) {
                            position         = convert<double, 3>(ball[0].torsoSpacePosition);
                            timeBallLastSeen = now;
                        }
                        else {
                            position = timeSinceBallSeen < search_timeout ? position :  // Place last seen
                                           arma::vec2({1, 0});                          // In front of the robot
                        }
                    }

                    // Hack Planner:
                    float headingChange = 0;
                    float sideStep      = 0;
                    float speedFactor   = 1;
                    if (useLocalisation) {
                        arma::vec2 kick_target =
                            WorldToRobotTransform(convert<double, 2>(selfs.front().locObject.position),
                                                  convert<double, 2>(selfs.front().heading),
                                                  convert<double, 2>(kickPlan.target));
                        // //approach point:
                        arma::vec2 ballToTarget = arma::normalise(kick_target - position);
                        arma::vec2 kick_point   = position - ballToTarget * ball_approach_dist;

                        if (arma::norm(position) > slowdown_distance) {
                            position = kick_point;
                        }
                        else {
                            speedFactor   = slow_approach_factor;
                            headingChange = std::atan2(ballToTarget[1], ballToTarget[0]);
                            sideStep      = 1;
                        }
                    }
                    // arma::vec2 ball_world_position = WorldToRobotTransform(selfs.front().position,
                    // selfs.front().heading, position);


                    float angle = std::atan2(position[1], position[0]) + headingChange;
                    // log("ball bearing", angle);
                    angle = std::min(turnSpeed, std::max(angle, -turnSpeed));
                    // log("turnSpeed", turnSpeed);
                    // log("ball bearing", angle);
                    // log("ball position", position);
                    // log("loc position", selfs.front().position.t());
                    // log("loc heading", selfs.front().heading);

                    // Euclidean distance to ball
                    float scaleF            = 2.0 / (1.0 + std::exp(-a * std::fabs(position[0]) + b)) - 1.0;
                    float scaleF2           = angle / M_PI;
                    float finalForwardSpeed = speedFactor * forwardSpeed * scaleF * (1.0 - scaleF2);

                    float scaleS         = 2.0 / (1.0 + std::exp(-a * std::fabs(position[1]) + b)) - 1.0;
                    float scaleS2        = angle / M_PI;
                    float finalSideSpeed = -speedFactor * ((0 < position[1]) - (position[1] < 0)) * sideStep * sideSpeed
                                           * scaleS * (1.0 - scaleS2);
                    // log("forwardSpeed1", forwardSpeed);
                    // log("scale", scale);
                    // log("distanceToBall", distanceToBall);
                    // log("forwardSpeed2", finalForwardSpeed);


                    std::unique_ptr<WalkCommand> command =
                        std::make_unique<WalkCommand>(subsumptionId, convert<double, 3>(Transform2D({0, 0, 0})));
                    command->command = convert<double, 3>(Transform2D({finalForwardSpeed, finalSideSpeed, angle}));

                    // TODO: delete this?!?!?
                    arma::vec2 ball_world_position =
                        RobotToWorldTransform(convert<double, 2>(selfs.front().locObject.position),
                                              convert<double, 2>(selfs.front().heading),
                                              position.rows(0, 1));
                    arma::vec2 kick_target =
                        2 * ball_world_position - convert<double, 2>(selfs.front().locObject.position);
                    emit(drawSphere("kick_target",
                                    arma::vec3({kick_target[0], kick_target[1], 0.0}),
                                    0.1,
                                    arma::vec3({1, 0, 0}),
                                    0));
                    // log("kick_target", kick_target[0], kick_target[1]);

                    emit(std::make_unique<KickPlan>(
                        KickPlan(convert<double, 2>(kick_target), KickPlan::KickType::SCRIPTED)));

                    emit(std::move(command));
                    emit(std::make_unique<ActionPriorites>(ActionPriorites{subsumptionId, {40, 11}}));
                });

            on<Trigger<MotionCommand>, Sync<SimpleWalkPathPlanner>>().then([this](const MotionCommand& cmd) {
                // save the plan
                latestCommand = cmd;

            });
        }

    }  // namespace planning
}  // namespace behaviour
}  // namespace module
