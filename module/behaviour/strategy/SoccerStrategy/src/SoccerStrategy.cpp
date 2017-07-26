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

#include "SoccerStrategy.h"

#include "extension/Configuration.h"

#include "message/behaviour/Look.h"
#include "message/behaviour/MotionCommand.h"
#include "message/behaviour/Nod.h"
#include "message/behaviour/SoccerObjectPriority.h"
#include "message/input/Sensors.h"
#include "message/localisation/ResetRobotHypotheses.h"
#include "message/motion/DiveCommand.h"
#include "message/motion/GetupCommand.h"
#include "message/platform/darwin/DarwinSensors.h"
#include "message/support/FieldDescription.h"
#include "message/vision/VisionObjects.h"

#include "utility/behaviour/MotionCommand.h"
#include "utility/math/geometry/Circle.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/time/time.h"

namespace module {
namespace behaviour {
    namespace strategy {

        using extension::Configuration;

        using message::behaviour::Behaviour;
        using message::behaviour::FieldTarget;
        using message::behaviour::Look;
        using message::behaviour::KickPlan;
        using KickType = message::behaviour::KickPlan::KickType;
        using message::behaviour::MotionCommand;
        using message::behaviour::Nod;
        using message::behaviour::SoccerObjectPriority;
        using SearchType = message::behaviour::SoccerObjectPriority::SearchType;
        using message::input::GameEvents;
        using message::input::GameState;
        using Phase          = message::input::GameState::Data::Phase;
        using Penalisation   = message::input::GameEvents::Penalisation;
        using Unpenalisation = message::input::GameEvents::Unpenalisation;
        using GameMode       = message::input::GameState::Data::Mode;
        using message::input::Sensors;
        using VisionBall = message::vision::Ball;
        using message::vision::Goal;
        using message::localisation::Ball;
        using message::localisation::Field;
        using message::localisation::ResetRobotHypotheses;
        using message::motion::DiveCommand;
        using message::motion::DiveFinished;
        using message::motion::ExecuteGetup;
        using message::motion::KillGetup;
        using message::platform::darwin::ButtonMiddleDown;
        using message::platform::darwin::ButtonLeftDown;
        using message::support::FieldDescription;

        using utility::time::durationFromSeconds;
        using utility::math::geometry::Circle;
        using utility::math::matrix::Rotation3D;
        using utility::math::matrix::Transform2D;
        using utility::math::matrix::Transform3D;

        SoccerStrategy::SoccerStrategy(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment))
            , cfg_()
            , walkTarget()
            , lookTarget()
            , kickType()
            , ballSearchStartTime()
            , goalLastMeasured() {

            on<Configuration>("SoccerStrategy.yaml").then([this](const Configuration& config) {

                cfg_.ball_last_seen_max_time = durationFromSeconds(config["ball_last_seen_max_time"].as<double>());
                cfg_.goal_last_seen_max_time = durationFromSeconds(config["goal_last_seen_max_time"].as<double>());

                cfg_.localisation_interval = durationFromSeconds(config["localisation_interval"].as<double>());
                cfg_.localisation_duration = durationFromSeconds(config["localisation_duration"].as<double>());
                cfg_.localisation_ball_covariance_threshold =
                    config["localisation_ball_covariance_threshold"].as<double>();
                cfg_.localisation_field_covariance_threshold =
                    config["localisation_field_covariance_threshold"].as<double>();

                cfg_.start_position_offensive = config["start_position_offensive"].as<arma::vec2>();
                cfg_.start_position_defensive = config["start_position_defensive"].as<arma::vec2>();

                cfg_.is_goalie = config["goalie"].as<bool>();

                // Use configuration here from file GoalieWalkPlanner.yaml
                cfg_.goalie_command_timeout           = config["goalie_command_timeout"].as<float>();
                cfg_.goalie_rotation_speed_factor     = config["goalie_rotation_speed_factor"].as<float>();
                cfg_.goalie_max_rotation_speed        = config["goalie_max_rotation_speed"].as<float>();
                cfg_.goalie_translation_speed_factor  = config["goalie_translation_speed_factor"].as<float>();
                cfg_.goalie_max_translation_speed     = config["goalie_max_translation_speed"].as<float>();
                cfg_.goalie_side_walk_angle_threshold = config["goalie_side_walk_angle_threshold"].as<float>();

                cfg_.ball_proximity_threshold = config["ball_proximity_threshold"].as<float>();

                cfg_.alwaysPowerKick      = config["always_power_kick"].as<bool>();
                cfg_.forcePlaying         = config["force_playing"].as<bool>();
                cfg_.forcePenaltyShootout = config["force_penalty_shootout"].as<bool>();

            });

            // TODO: unhack
            emit(std::make_unique<KickPlan>(KickPlan(Eigen::Vector2d(4.5, 0), KickType::SCRIPTED)));


            // For checking last seen times
            on<Trigger<std::vector<VisionBall>>>().then([this](const std::vector<VisionBall>& balls) {
                if (!balls.empty()) {
                    ballLastMeasured = NUClear::clock::now();
                }
            });

            on<Trigger<std::vector<Goal>>>().then([this](const std::vector<Goal>& goals) {
                if (!goals.empty()) {
                    goalLastMeasured = NUClear::clock::now();
                }
            });

            // TODO: remove this horrible code
            // Check to see if we are currently in the process of getting up.
            on<Trigger<ExecuteGetup>>().then([this] { isGettingUp = true; });

            // Check to see if we have finished getting up.
            on<Trigger<KillGetup>>().then([this] { isGettingUp = false; });

            // Check to see if we are currently in the process of diving.
            on<Trigger<DiveCommand>>().then([this] { isDiving = true; });

            // Check to see if we have finished diving.
            on<Trigger<DiveFinished>>().then([this] { isDiving = false; });

            on<Trigger<Penalisation>>().then([this](const Penalisation& selfPenalisation) {
                if (selfPenalisation.context == GameEvents::Context::SELF) {
                    selfPenalised = true;
                }
            });

            on<Trigger<Unpenalisation>, With<FieldDescription>>().then(
                [this](const Unpenalisation& selfPenalisation, const FieldDescription& fieldDescription) {

                    if (selfPenalisation.context == GameEvents::Context::SELF) {
                        selfPenalised = false;

                        // TODO: isSideChecking = true;
                        // TODO: only do this once put down
                        unpenalisedLocalisationReset(fieldDescription);
                    }
                });


            on<Trigger<ButtonMiddleDown>, Single>().then([this] {

                if (!cfg_.forcePlaying) {
                    NUClear::log("Force playing started.");
                    emit(std::make_unique<Nod>(true));
                    cfg_.forcePlaying = true;
                }

            });

            on<Trigger<ButtonLeftDown>, Single, With<Sensors>>().then([this](const Sensors& sensors) {
                NUClear::log("Localisation Orientation reset. Localisation resets will now orient this as forwards.");
                manualOrientationReset = true;
                emit(std::make_unique<Nod>(true));
                manualOrientation = Rotation3D(Transform3D(convert<double, 4, 4>(-sensors.world)).rotation()).yaw();


            });

            // Main Loop
            // TODO: ensure a reasonable state is emitted even if gamecontroller is not running
            on<Every<30, Per<std::chrono::seconds>>,
               With<Sensors>,
               With<GameState>,
               With<Phase>,
               With<FieldDescription>,
               With<Field>,
               With<Ball>,
               Single>()
                .then([this](const Sensors& sensors,
                             const GameState& gameState,
                             const Phase& phase,
                             const FieldDescription& fieldDescription,
                             const Field& field,
                             const Ball& ball) {

                    try {

                        Behaviour::State previousState = currentState;

                        auto mode = cfg_.forcePenaltyShootout ? GameMode::PENALTY_SHOOTOUT : gameState.data.mode.value;

                        // auto& phase = gameState.phase;

                        // TODO: fix ik kick
                        kickType = KickType::SCRIPTED;
                        // kickType = mode == GameMode::PENALTY_SHOOTOUT || cfg_.alwaysPowerKick ? KickType::SCRIPTED :
                        // KickType::IK_KICK;

                        if (cfg_.forcePlaying) {
                            play(field, ball, fieldDescription, mode, sensors);
                        }
                        else if (pickedUp(sensors)) {
                            // TODO: stand, no moving
                            standStill();
                            currentState = Behaviour::State::PICKED_UP;
                        }
                        else {
                            if (mode == GameMode::NORMAL || mode == GameMode::OVERTIME
                                || mode == GameMode::PENALTY_SHOOTOUT) {
                                if (phase == Phase::INITIAL) {
                                    standStill();
                                    find({FieldTarget(FieldTarget::Target::SELF)}, field, ball, sensors);
                                    initialLocalisationReset(fieldDescription);
                                    currentState = Behaviour::State::INITIAL;
                                }
                                else if (phase == Phase::READY) {
                                    if (gameState.data.our_kick_off) {
                                        walkTo(fieldDescription, cfg_.start_position_offensive);
                                    }
                                    else {
                                        walkTo(fieldDescription, cfg_.start_position_defensive);
                                    }
                                    find({FieldTarget(FieldTarget::Target::SELF)}, field, ball, sensors);
                                    currentState = Behaviour::State::READY;
                                }
                                else if (phase == Phase::SET) {
                                    standStill();
                                    find({FieldTarget(FieldTarget::Target::BALL)}, field, ball, sensors);
                                    if (mode == GameMode::PENALTY_SHOOTOUT) {
                                        penaltyLocalisationReset();
                                    }
                                    else {
                                        initialLocalisationReset(fieldDescription);
                                    }
                                    currentState = Behaviour::State::SET;
                                }
                                else if (phase == Phase::TIMEOUT) {
                                    standStill();
                                    find({FieldTarget(FieldTarget::Target::SELF)}, field, ball, sensors);
                                    currentState = Behaviour::State::TIMEOUT;
                                }
                                else if (phase == Phase::FINISHED) {
                                    standStill();
                                    find({FieldTarget(FieldTarget::Target::SELF)}, field, ball, sensors);
                                    currentState = Behaviour::State::FINISHED;
                                }
                                else if (phase == Phase::PLAYING) {
                                    play(field, ball, fieldDescription, mode, sensors);
                                }
                            }
                        }

                        if (currentState != previousState) {
                            emit(std::make_unique<Behaviour::State>(currentState));
                        }
                    }
                    catch (std::runtime_error err) {
                        log(err.what());
                        log("Runtime exception.");
                    }
                });

            on<Trigger<Field>, With<FieldDescription>>().then(
                [this](const Field& field, const FieldDescription& fieldDescription) {
                    auto kickTarget = convert<double, 2>(getKickPlan(field, fieldDescription));
                    emit(std::make_unique<KickPlan>(KickPlan(kickTarget, kickType)));
                    emit(utility::nubugger::drawCircle(
                        "SocStrat_kickTarget", Circle(0.05, convert<double, 2>(kickTarget)), 0.3, {0, 0, 0}));
                });
        }

        void SoccerStrategy::play(const Field& field,
                                  const Ball& ball,
                                  const FieldDescription& fieldDescription,
                                  const GameMode& mode,
                                  const Sensors& sensors) {
            if (penalised() && !cfg_.forcePlaying) {  // penalised
                standStill();
                find({FieldTarget(FieldTarget::Target::SELF)}, field, ball, sensors);
                currentState = Behaviour::State::PENALISED;
            }
            else if (cfg_.is_goalie) {  // goalie
                find({FieldTarget(FieldTarget::Target::BALL)}, field, ball, sensors);
                goalieWalk(field, ball);
                currentState = Behaviour::State::GOALIE_WALK;
            }
            else {
                /*if (NUClear::clock::now() - lastLocalised > cfg_.localisation_interval) {
                standStill();
                find({FieldTarget(FieldTarget::Target::BALL)}, sensors);
                if (NUClear::clock::now() - lastLocalised > cfg_.localisation_interval + cfg_.localisation_duration) {
                    lastLocalised = NUClear::clock::now();
                }
                currentState = Behaviour::State::LOCALISING;
            }
            else*/ if (NUClear::clock::now() - ballLastMeasured
                       < cfg_.ball_last_seen_max_time) {  // ball has been seen recently
                    find({FieldTarget(FieldTarget::Target::BALL)}, field, ball, sensors);
                    walkTo(fieldDescription, FieldTarget::Target::BALL);
                    currentState = Behaviour::State::WALK_TO_BALL;
                }
                else {  // ball has not been seen recently
                    if (mode != GameMode::PENALTY_SHOOTOUT
                        && (Eigen::Vector2d(field.position[0], field.position[1]).norm()
                            > 1)) {  // a long way away from centre
                        // walk to centre of field
                        find({FieldTarget(FieldTarget::Target::BALL)}, field, ball, sensors);
                        walkTo(fieldDescription, arma::vec2({0, 0}));
                        currentState = Behaviour::State::MOVE_TO_CENTRE;
                    }
                    else {
                        find({FieldTarget(FieldTarget::Target::BALL)}, field, ball, sensors);
                        walkTo(fieldDescription, FieldTarget::Target::BALL);
                        // spinWalk();

                        currentState = Behaviour::State::SEARCH_FOR_BALL;
                    }
                }
            }
        }

        void SoccerStrategy::initialLocalisationReset(const FieldDescription& fieldDescription) {

            auto reset = std::make_unique<ResetRobotHypotheses>();

            ResetRobotHypotheses::Self selfSideBaseLine;
            selfSideBaseLine.position << -fieldDescription.dimensions.field_length * 0.5
                                             + fieldDescription.dimensions.goal_area_length,
                0;
            selfSideBaseLine.position_cov = Eigen::Vector2d::Constant(0.1).asDiagonal();
            selfSideBaseLine.heading      = 0;
            selfSideBaseLine.heading_var  = 0.005;
            if (manualOrientationReset) {
                selfSideBaseLine.heading     = manualOrientation;
                selfSideBaseLine.heading_var = 0.00005;
                selfSideBaseLine.absoluteYaw = true;
            }

            reset->hypotheses.push_back(selfSideBaseLine);
            emit(std::move(reset));
        }

        void SoccerStrategy::penaltyLocalisationReset() {

            auto reset = std::make_unique<ResetRobotHypotheses>();

            ResetRobotHypotheses::Self selfSideBaseLine;
            selfSideBaseLine.position << 2.0, 0.0;
            selfSideBaseLine.position_cov = Eigen::Vector2d::Constant(0.1).asDiagonal();
            selfSideBaseLine.heading      = 0;
            selfSideBaseLine.heading_var  = 0.005;
            if (manualOrientationReset) {
                selfSideBaseLine.heading     = manualOrientation;
                selfSideBaseLine.heading_var = 0.00005;
                selfSideBaseLine.absoluteYaw = true;
            }
            reset->hypotheses.push_back(selfSideBaseLine);

            emit(std::move(reset));
        }

        void SoccerStrategy::unpenalisedLocalisationReset(const FieldDescription& fieldDescription) {

            auto reset = std::make_unique<ResetRobotHypotheses>();
            ResetRobotHypotheses::Self selfSideLeft;
            selfSideLeft.position << -fieldDescription.penalty_robot_start,
                fieldDescription.dimensions.field_width * 0.5;
            selfSideLeft.position_cov = Eigen::Vector2d::Constant(0.1).asDiagonal();
            selfSideLeft.heading      = -M_PI_2;
            selfSideLeft.heading_var  = 0.005;
            if (manualOrientationReset) {
                selfSideLeft.heading     = manualOrientation;
                selfSideLeft.heading_var = 0.00005;
                selfSideLeft.absoluteYaw = true;
            }
            reset->hypotheses.push_back(selfSideLeft);

            ResetRobotHypotheses::Self selfSideRight;
            selfSideRight.position << -fieldDescription.penalty_robot_start,
                -fieldDescription.dimensions.field_width * 0.5;
            selfSideRight.position_cov = Eigen::Vector2d::Constant(0.1).asDiagonal();
            selfSideRight.heading      = M_PI_2;
            selfSideRight.heading_var  = 0.005;
            if (manualOrientationReset) {
                selfSideRight.heading     = manualOrientation;
                selfSideRight.heading_var = 0.00005;
                selfSideRight.absoluteYaw = true;
            }
            reset->hypotheses.push_back(selfSideRight);

            ResetRobotHypotheses::Self selfSideBaseLine;
            selfSideBaseLine.position << -fieldDescription.dimensions.field_length * 0.5
                                             + fieldDescription.dimensions.goal_area_length,
                0;
            selfSideBaseLine.position_cov = Eigen::Vector2d::Constant(0.1).asDiagonal();
            selfSideBaseLine.heading      = 0;
            selfSideBaseLine.heading_var  = 0.005;
            if (manualOrientationReset) {
                selfSideBaseLine.heading     = manualOrientation;
                selfSideBaseLine.heading_var = 0.00005;
                selfSideBaseLine.absoluteYaw = true;
            }
            reset->hypotheses.push_back(selfSideBaseLine);

            emit(std::move(reset));
        }

        void SoccerStrategy::searchWalk() {}

        void SoccerStrategy::standStill() {
            emit(std::make_unique<MotionCommand>(utility::behaviour::StandStill()));
        }

        void SoccerStrategy::walkTo(const FieldDescription& fieldDescription, const FieldTarget::Target& target) {
            if (target != FieldTarget::Target::BALL) {
                throw std::runtime_error("SoccerStrategy::walkTo: Only FieldTarget::Target::BALL is supported.");
            }

            arma::vec2 enemyGoal = {fieldDescription.dimensions.field_length * 0.5, 0};

            emit(std::make_unique<MotionCommand>(utility::behaviour::BallApproach(enemyGoal)));
        }

        void SoccerStrategy::walkTo(const FieldDescription& fieldDescription, arma::vec position) {

            arma::vec2 enemyGoal = {fieldDescription.dimensions.field_length * 0.5, 0};

            auto goalState = Transform2D::lookAt(position, enemyGoal);
            emit(std::make_unique<MotionCommand>(utility::behaviour::WalkToState(goalState)));
        }

        bool SoccerStrategy::pickedUp(const Sensors& sensors) {

            bool feetOffGround = !sensors.leftFootDown && !sensors.rightFootDown;
            return false && feetOffGround && !isGettingUp && !isDiving && sensors.world(2, 2) < 0.92
                   && sensors.world(2, 2) > 0.88;
        }

        bool SoccerStrategy::penalised() {
            return selfPenalised;
        }

        bool SoccerStrategy::ballDistance(const Ball& ball) {
            return ball.position.norm();
        }

        void SoccerStrategy::find(const std::vector<FieldTarget>& fieldObjects,
                                  const Field& locField,
                                  const Ball& locBall,
                                  const Sensors& sensors) {
            // Create the soccer object priority pointer and initialise each value to 0.
            auto soccerObjectPriority  = std::make_unique<SoccerObjectPriority>();
            soccerObjectPriority->ball = 0;
            soccerObjectPriority->goal = 0;
            soccerObjectPriority->line = 0;
            for (auto& fieldObject : fieldObjects) {
                switch (fieldObject.target.value) {
                    case FieldTarget::Target::SELF: {
                        soccerObjectPriority->goal       = 1;
                        soccerObjectPriority->searchType = SearchType::GOAL_SEARCH;
                        break;
                    }
                    case FieldTarget::Target::BALL: {
                        soccerObjectPriority->ball       = 1;
                        soccerObjectPriority->searchType = SearchType::LOST;
                        break;
                    }
                    default: throw std::runtime_error("Soccer strategy attempted to find a bad object");
                }
            }

            // When weve seen both field targets
            Transform3D Htw = convert<double, 4, 4>(sensors.world);
            if (arma::norm(Htw.transformPoint(arma::vec3{locBall.position[0], locBall.position[1], 0.0}).cols(0, 1))
                < cfg_.ball_proximity_threshold) {
                soccerObjectPriority->ball = 1;
                soccerObjectPriority->goal = 0;
            }
            if (soccerObjectPriority->ball && soccerObjectPriority->goal) {
                // Check field message and localisation ball message
                // Check trace of their covariances are bigger than configurable threshold
                // If the ball covariance track is larger, always look at ball
                if (locBall.covariance.trace() > cfg_.localisation_ball_covariance_threshold) {
                    //      If within certain distance, focus on ball
                    soccerObjectPriority->ball = 1;
                    soccerObjectPriority->goal = 0;
                }
                // If covariance ball is below certain threshold and not too close
                //  and covariance on self is above certain threshold, look for goals
                else if (locField.covariance.trace() > cfg_.localisation_field_covariance_threshold) {
                    soccerObjectPriority->ball = 0;
                    soccerObjectPriority->goal = 1;
                }
            }
            emit(std::move(soccerObjectPriority));
        }

        void SoccerStrategy::spinWalk() {
            emit(std::make_unique<MotionCommand>(utility::behaviour::DirectCommand({0, 0, 1})));
        }

        arma::vec2 SoccerStrategy::getKickPlan(const Field& field, const FieldDescription& fieldDescription) {

            // Defines the box within in which the kick target is changed from the centre
            // of the oppposition goal to the perpendicular distance from the robot to the goal

            float maxKickRange =
                0.6;  // TODO: make configurable, only want to change at the last kick to avoid smart goalies
            float xTakeOverBox = maxKickRange;
            size_t error       = 0.05;
            size_t buffer      = error + 2 * fieldDescription.ball_radius;             // 15cm
            float yTakeOverBox = fieldDescription.dimensions.goal_width / 2 - buffer;  // 90-15 = 75cm
            float xRobot       = field.position[0];
            float yRobot       = field.position[1];
            arma::vec2 newTarget;

            if ((fieldDescription.dimensions.field_length / 2) - xTakeOverBox < xRobot && -yTakeOverBox < yRobot
                && yRobot < yTakeOverBox) {
                // Aims for behind the point that gives the shortest distance
                newTarget[0] =
                    fieldDescription.dimensions.field_length / 2 + fieldDescription.dimensions.goal_depth / 2;
                newTarget[1] = yRobot;
            }
            else {
                // Aims for the centre of the goal
                newTarget[0] = fieldDescription.dimensions.field_length / 2;
                newTarget[1] = 0;
            }
            return newTarget;
        }

        void SoccerStrategy::goalieWalk(const Field& field, const Ball& ball) {
            std::unique_ptr<MotionCommand> motionCommand;

            float timeSinceBallSeen =
                std::chrono::duration_cast<std::chrono::microseconds>(NUClear::clock::now() - ballLastMeasured).count()
                * 1e-6;
            if (timeSinceBallSeen < cfg_.goalie_command_timeout) {

                float fieldBearing = field.position[2];
                int signBearing    = fieldBearing > 0 ? 1 : -1;
                float rotationSpeed =
                    -signBearing * std::fmin(std::fabs(cfg_.goalie_rotation_speed_factor * fieldBearing),
                                             cfg_.goalie_max_rotation_speed);

                int signTranslation = ball.position[1] > 0 ? 1 : -1;
                float translationSpeed =
                    signTranslation * std::fmin(std::fabs(cfg_.goalie_translation_speed_factor * ball.position[1]),
                                                cfg_.goalie_max_translation_speed);

                motionCommand =
                    std::make_unique<MotionCommand>(utility::behaviour::DirectCommand({0, 0, rotationSpeed}));
                if (std::fabs(fieldBearing) < cfg_.goalie_side_walk_angle_threshold) {
                    motionCommand->walkCommand.y() = translationSpeed;
                }
            }
            else {
                motionCommand = std::make_unique<MotionCommand>(utility::behaviour::DirectCommand({0, 0, 0}));
            }
            emit(std::move(motionCommand));
        }

    }  // namespace strategy
}  // namespace behaviour
}  // namespace module
