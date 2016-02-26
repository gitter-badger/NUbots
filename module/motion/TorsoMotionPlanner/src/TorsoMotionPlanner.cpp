/*
 * This file is part of NUbots Codebase.
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
 * Copyright 2016 NUbots <nubots@nubots.net>
 */
/*===========================================================================================================*/
/*----------------------------------------CONSTANTS AND DEFINITIONS------------------------------------------*/
/*===========================================================================================================*/
//      INCLUDE(S)
/*===========================================================================================================*/
#include "TorsoMotionPlanner.h"

#include <algorithm>
#include <armadillo>
#include <chrono>
#include <cmath>

#include "message/behaviour/ServoCommand.h"
#include "message/support/Configuration.h"
#include "message/motion/WalkCommand.h"
#include "message/motion/ServoTarget.h"
#include "message/motion/Script.h"
#include "message/behaviour/FixedWalkCommand.h"
#include "message/localisation/FieldObject.h"
#include "message/input/PushDetection.h"

#include "utility/motion/Balance.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/motion/ForwardKinematics.h"
#include "utility/motion/RobotModels.h"
#include "utility/math/angle.h"
#include "utility/math/matrix/Rotation3D.h"
/*===========================================================================================================*/
//      NAMESPACE(S)
/*===========================================================================================================*/
namespace module 
{
namespace motion 
{
	/*=======================================================================================================*/
    //      UTILIZATION REFERENCE(S)
    /*=======================================================================================================*/
    using message::input::PushDetection;
    using message::input::ServoID;
    using message::input::Sensors;
    using message::input::LimbID;
    using message::behaviour::ServoCommand;
    using message::behaviour::WalkOptimiserCommand;
    using message::behaviour::WalkConfigSaved;
    // using message::behaviour::RegisterAction;
    // using message::behaviour::ActionPriorites;
    using message::input::LimbID;
    using message::motion::WalkCommand;
    using message::motion::WalkStartCommand;
    using message::motion::WalkStopCommand;
    using message::motion::WalkStopped;
    using message::motion::EnableModularWalkEngineCommand;
    using message::motion::DisableModularWalkEngineCommand;
    using message::motion::ServoTarget;
    using message::motion::Script;
    using message::support::SaveConfiguration;
    using message::support::Configuration;

    using extension::Configuration;

    using utility::motion::kinematics::calculateLegJointsTeamDarwin;
    using utility::motion::kinematics::DarwinModel;
    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;
    using utility::math::matrix::Rotation3D;
    using utility::math::angle::normalizeAngle;
    using utility::nubugger::graph;
    using utility::support::Expression;
    /*=======================================================================================================*/
    //      NAME: TorsoMotionPlanner
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    TorsoMotionPlanner::TorsoMotionPlanner(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) 
    {
        //Configure foot motion planner...
        on<Configuration>(CONFIGURATION_PATH).then([this] (const Configuration& config) 
        {
            configure(config.config);
        });

        //Transform analytical torso positions in accordance with the stipulated targets...
        updateHandle = on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, With<Sensors>, Single, Priority::HIGH>()
        .then([this](const Sensors& sensors) 
        {
            updateTorsoPosition(sensors);
        }).disable();

        //In the event of a new foot step target specified by the foot placement planning module...
        on<Trigger<FootStepTarget>>().then([this] 
        {
<<<<<<< e7199040444bd454e68ee7a37c6bfb4e3c9debc2
<<<<<<< f10da9c4b0232f8ba743f03d26325ddc863f1f48
<<<<<<< cdf96fbdd5a07a2529fcd01d4c07dd1dfe64510f
            torsoZMP();
=======
            zmpTorsoCoefficients();
>>>>>>> Message Headers, emit structs, and further encapsulation of
        });
<<<<<<< 96dd3deaa26010585080bb841a4e4f1773925448
        
            zmpTorsoCoefficients();
=======
            if(supportFoot)
            {
                setLeftFootDestination(targetDestination);
            }
            else
            {
                setRightFootDestination(targetDestination);
            }
            setDestinationTime(targetTime);  
            zmpTorsoCoefficients();  
        });
<<<<<<< 183df72fb88459adef7436f0515b768fde100df7:module/motion/ModularWalkEngine/src/TorsoMotionPlanner.cpp

>>>>>>> Configuration Management for Modular Design
        }
<<<<<<< f10da9c4b0232f8ba743f03d26325ddc863f1f48
>>>>>>> Added messages between modules
=======
>>>>>>> Message Headers, emit structs, and further encapsulation of
=======
>>>>>>> Adding emits and triggers for flow of data
=======
>>>>>>> Further Modularization in development hierarchy, reorganised directory structure:module/motion/TorsoMotionPlanner/src/TorsoMotionPlanner.cpp
    }
    /*=======================================================================================================*/
    //      NAME: getTime
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    double TorsoMotionPlanner::getTime() 
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(NUClear::clock::now().time_since_epoch()).count() * 1E-6;
    }
     /*=======================================================================================================*/
    //      NAME: getLeftFootDestination
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    double TorsoMotionPlanner::getLeftFootDestination()
    {
        setNewStepReceived(false);
        return (leftFootDestination.front());
    }
    /*=======================================================================================================*/
    //      NAME: setLeftFootDestination
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    void TorsoMotionPlanner::setLeftFootDestination(double inLeftFootDestination)
    {
        setNewStepReceived(true);
        leftFootDestination.push(inLeftFootDestination);
    }
    /*=======================================================================================================*/
    //      NAME: getRightFootDestination
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    double TorsoMotionPlanner::getRightFootDestination()
    {
        setNewStepReceived(false);
        return (rightFootDestination.front());
    }
    /*=======================================================================================================*/
    //      NAME: setRightFootDestination
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    void TorsoMotionPlanner::setRightFootDestination(double inRightFootDestination)
    {
        setNewStepReceived(true);
        rightFootDestination.push(inRightFootDestination);
    }
    /*=======================================================================================================*/
    //      NAME: updateTorsoPosition
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    void TorsoMotionPlanner::updateTorsoPosition()
    {
<<<<<<< 64307f428050a70febbcdd787f038b25be0bf9ed
        torso.uTorso = zmpTorsoCompensation(phase, zmpTorsoCoefficients, zmpParams, stepTime, zmpTime, phase1Single, phase2Single, uSupport, uLeftFootDestination, uLeftFootSource, uRightFootDestination, uRightFootSource);
        Transform2D uTorsoActual = uTorso.localToWorld({-DarwinModel::Leg::HIP_OFFSET_X, 0, 0});
        Transform3D torso.torso = arma::vec6({uTorsoActual.x(), uTorsoActual.y(), bodyHeight, 0, bodyTilt, uTorsoActual.angle()});
        emit(std:make_unique<TorsoUpdate>(uTorso, torso); //uTorso is needed by motionArms and torso is needed for feet position
                             //could also move calculation of torso to response
=======
        uTorso = zmpTorsoCompensation(phase, zmpTorsoCoefficients, zmpParams, stepTime, zmpTime, phase1Single, phase2Single, uSupport, uLeftFootSource, uRightFootSource);
>>>>>>> Modifications to TorsoMotionPlanner
    }
    /*=======================================================================================================*/
    //      NAME: stepTorso
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    Transform2D TorsoMotionPlanner::stepTorso(Transform2D uLeftFoot, Transform2D uRightFoot, double shiftFactor) 
    {
        Transform2D uLeftFootSupport  = uLeftFoot.localToWorld({-footOffset[0], -footOffset[1], 0});
        Transform2D uRightFootSupport = uRightFoot.localToWorld({-footOffset[0], footOffset[1], 0});
        return uLeftFootSupport.interpolate(shiftFactor, uRightFootSupport);
    }

    /*=======================================================================================================*/
    //      NAME: zmpTorsoCoefficients
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
<<<<<<< f10da9c4b0232f8ba743f03d26325ddc863f1f48
<<<<<<< cdf96fbdd5a07a2529fcd01d4c07dd1dfe64510f
    void TorsoMotionPlanner::torsoZMP() //originally part of CalculateNewStep
=======
    void ModularWalkEngine::zmpCoefficients() //originally part of CalculateNewStep
>>>>>>> Added messages between modules
=======
    void TorsoMotionPlanner::zmpTorsoCoefficients() //originally part of CalculateNewStep
>>>>>>> Message Headers, emit structs, and further encapsulation of
    {
        uTorsoDestination = stepTorso(getLeftFootDestination(), getRightFootDestination(), 0.5);

        // compute ZMP coefficients
        zmpParams = 
        {
            (uSupport.x() - uTorso.x()) / (stepTime * phase1Single),
            (uTorsoDestination.x() - uSupport.x()) / (stepTime * (1 - phase2Single)),
            (uSupport.y() - uTorso.y()) / (stepTime * phase1Single),
            (uTorsoDestination.y() - uSupport.y()) / (stepTime * (1 - phase2Single)),
        };

        zmpTorsoCoefficients.rows(0,1) = zmpSolve(uSupport.x(), uTorsoSource.x(), uTorsoDestination.x(), uTorsoSource.x(), uTorsoDestination.x(), phase1Single, phase2Single, stepTime, zmpTime);
        zmpTorsoCoefficients.rows(2,3) = zmpSolve(uSupport.y(), uTorsoSource.y(), uTorsoDestination.y(), uTorsoSource.y(), uTorsoDestination.y(), phase1Single, phase2Single, stepTime, zmpTime);
    }
    /*=======================================================================================================*/
    //      NAME: zmpSolve
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
<<<<<<< f10da9c4b0232f8ba743f03d26325ddc863f1f48
<<<<<<< cdf96fbdd5a07a2529fcd01d4c07dd1dfe64510f
    arma::vec2 TorsoMotionPlanner::zmpSolve(double zs, double z1, double z2, double x1, double x2, double phase1Single, double phase2Single, double stepTime, double zmpTime) 
=======
    arma::vec2 ModularWalkEngine::zmpTorsoCompensation(double zs, double z1, double z2, double x1, double x2, double phase1Single, double phase2Single, double stepTime, double zmpTime) 
>>>>>>> Added messages between modules
=======
    arma::vec2 TorsoMotionPlanner::zmpSolve(double zs, double z1, double z2, double x1, double x2, double phase1Single, double phase2Single, double stepTime, double zmpTime) 
>>>>>>> Message Headers, emit structs, and further encapsulation of
    {
        /*
        Solves ZMP equations.
        The resulting form of x is
        x(t) = z(t) + aP*exp(t/zmpTime) + aN*exp(-t/zmpTime) - zmpTime*mi*sinh((t-Ti)/zmpTime)
        where the ZMP point is piecewise linear:
        z(0) = z1, z(T1 < t < T2) = zs, z(stepTime) = z2
        */
        double T1 = stepTime * phase1Single;
        double T2 = stepTime * phase2Single;
        double m1 = (zs - z1) / T1;
        double m2 = -(zs - z2) / (stepTime - T2);

        double c1 = x1 - z1 + zmpTime * m1 * std::sinh(-T1 / zmpTime);
        double c2 = x2 - z2 + zmpTime * m2 * std::sinh((stepTime - T2) / zmpTime);
        double expTStep = std::exp(stepTime / zmpTime);
        double aP = (c2 - c1 / expTStep) / (expTStep - 1 / expTStep);
        double aN = (c1 * expTStep - c2) / (expTStep - 1 / expTStep);
        return {aP, aN};
    }
    /*=======================================================================================================*/
    //      NAME: zmpTorsoCompensation
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    Transform2D TorsoMotionPlanner::zmpTorsoCompensation(double phase, arma::vec4 zmpTorsoCoefficients, arma::vec4 zmpParams, double stepTime, double zmpTime, double phase1Single, double phase2Single, Transform2D uSupport, Transform2D uLeftFootSource, Transform2D uRightFootSource) 
    {
        //Note that phase is the only variable updated during a step
        Transform2D com = {0, 0, 0};
        double expT = std::exp(stepTime * phase / zmpTime);
        com.x() = uSupport.x() + zmpTorsoCoefficients[0] * expT + zmpTorsoCoefficients[1] / expT;
        com.y() = uSupport.y() + zmpTorsoCoefficients[2] * expT + zmpTorsoCoefficients[3] / expT;
        if (phase < phase1Single) 
        {
            com.x() += zmpParams[0] * stepTime * (phase - phase1Single) -zmpTime * zmpParams[0] * std::sinh(stepTime * (phase - phase1Single) / zmpTime);
            com.y() += zmpParams[1] * stepTime * (phase - phase1Single) -zmpTime * zmpParams[1] * std::sinh(stepTime * (phase - phase1Single) / zmpTime);
        } 
        else if (phase > phase2Single) 
        {
            com.x() += zmpParams[2] * stepTime * (phase - phase2Single) -zmpTime * zmpParams[2] * std::sinh(stepTime * (phase - phase2Single) / zmpTime);
            com.y() += zmpParams[3] * stepTime * (phase - phase2Single) -zmpTime * zmpParams[3] * std::sinh(stepTime * (phase - phase2Single) / zmpTime);
        }
        // com[2] = .5 * (uLeftFoot[2] + uRightFoot[2]);
        // Linear speed turning
        com.angle() = phase * (getLeftFootDestination().angle() + getRightFootDestination().angle()) / 2 + (1 - phase) * (uLeftFootSource.angle() + uRightFootSource.angle()) / 2;
        return com;
    }

}  // motion
}  // modules
