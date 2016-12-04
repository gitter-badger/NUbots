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
#include "FootMotionPlanner.h"
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
    using message::input::LimbID;
    using NewStepTargetInfo  = message::motion::NewStepTargetInfo;
    using NewFootTargetInfo  = message::motion::NewFootTargetInfo;
    using NextFootTargetInfo = message::motion::NextFootTargetInfo;
    using message::motion::FootMotionUpdate;
    using message::motion::EnableFootMotion;
    using message::motion::DisableFootMotion;
    using message::motion::FootStepRequested;
    using message::motion::FootStepCompleted;
    using message::support::Configuration;

    using utility::support::Expression;
    using message::motion::kinematics::KinematicsModel;
    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;
    using utility::nubugger::graph;
/*=======================================================================================================*/
//      NUCLEAR METHOD: FootMotionPlanner
/*=======================================================================================================*/
    FootMotionPlanner::FootMotionPlanner(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment))
        , DEBUG(false), DEBUG_ITER(0)
        , emitFootPosition(false)
        , updateHandle()
        , leftFootPositionTransform(), rightFootPositionTransform()
        , activeLimbSource(), activeLimbDestination()
        , activeForwardLimb(), newStepInfoSets()
        , activeLimbInitial(LimbID::LEFT_LEG)
        , stepTime(0.0), stepHeight(0.0)
        , step_height_slow_fraction(0.0f), step_height_fast_fraction(0.0f)
        , ankle_pitch_lift(0.0), ankle_pitch_fall(0.0)
        , stepLimits(arma::fill::zeros), footOffsetCoefficient(arma::fill::zeros)
        , INITIAL_STEP(false), newStepStartTime(0.0), destinationTime(), lastVeloctiyUpdateTime()
        , velocityHigh(0.0), accelerationTurningFactor(0.0), velocityLimits(arma::fill::zeros)
        , accelerationLimits(arma::fill::zeros), accelerationLimitsHigh(arma::fill::zeros)
        , velocityCurrent()
        , phase1Single(0.0), phase2Single(0.0)
        , kinematicsModel()
    {    	
        //Configure foot motion planner...
        on<Configuration>("WalkEngine.yaml").then("Foot Motion Planner - Configure", [this] (const Configuration& config) 
        {             
            configure(config.config);          
        });

        //Define kinematics model for physical calculations...
        on<Trigger<KinematicsModel>>().then("WalkEngine - Update Kinematics Model", [this](const KinematicsModel& model)
        {
            kinematicsModel = model;
        });

        //Transform analytical foot positions in accordance with the stipulated targets...
        updateHandle = on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, Single, Priority::HIGH>()
        .then("Foot Motion Planner - Update Foot Position", [this]
        {
            double motionPhase = getMotionPhase();
            if(DEBUG) { log<NUClear::TRACE>("Messaging: Foot Motion Planner - Update Foot Position(0)"); }                     
                updateFootPosition(motionPhase, getActiveLimbSource(), getActiveForwardLimb(), getActiveLimbDestination());         
            if(DEBUG) { log<NUClear::TRACE>("Messaging: Foot Motion Planner - Update Foot Position(1)"); }

            //DEBUG: Printout of motion phase function...
            emit(graph("FMP Synchronising Motion Phase", arma::vec({motionPhase, (getActiveForwardLimb() == LimbID::LEFT_LEG ? 1  : 0)})));
            emit(graph("FMP Synchronising Data Queues", arma::vec({
                                                                        destinationTime.size(), 
                                                                        velocityCurrent.size(), 
                                                                        activeLimbSource.size(), 
                                                                        activeForwardLimb.size(), 
                                                                        activeLimbDestination.size(), 
                                                                        newStepInfoSets.size()
                                                                      })));   
        }).disable();

        //In the event of a new foot step target specified by the foot placement planning module...
        on<Trigger<NewFootTargetInfo>, With<NewStepTargetInfo>>().then("Foot Motion Planner - Received Target Foot Position", [this] (
            const NewFootTargetInfo& nft,
            const NewStepTargetInfo& nst) 
        {      
            // Step Target Data queued evaluation...
            if(DEBUG) { log<NUClear::TRACE>("Messaging: Foot Motion Planner - Received Target Foot Position(0)"); }
                setDestinationTime(nst.targetTime);                      //Queued    : FPP
                setVelocityCurrent(nst.velocityCurrent);                 //Queued    : FPP             
            if(DEBUG) { log<NUClear::TRACE>("Messaging: Foot Motion Planner - Received Target Foot Position(1)"); }

            // Foot Target Data queued evaluation...
            if(DEBUG) { log<NUClear::TRACE>("Messaging: Foot Motion Planner - Received Target Foot Position(0)"); }                      
                if(nst.activeForwardLimb == LimbID::LEFT_LEG)
                {  
                    setActiveLimbSource(nft.leftFootSource);             //Queued    : FPP
                    setActiveLimbDestination(nft.leftFootDestination);   //Queued    : FPP          
                }
                else
                {       
                    setActiveLimbSource(nft.rightFootSource);            //Queued    : FPP
                    setActiveLimbDestination(nft.rightFootDestination);  //Queued    : FPP      
                }                         
                setActiveForwardLimb(nst.activeForwardLimb);             //Queued    : FPP            
            if(DEBUG) { log<NUClear::TRACE>("Messaging: Foot Motion Planner - Received Target Foot Position(1)"); }
            
            // Next step data queued forwarding...
            if(DEBUG) { log<NUClear::TRACE>("Messaging: Foot Motion Planner - Generate Foot Position Set(0)"); }
                NewStepInfo nextStep = NewStepInfo();
                    nextStep.lFootSource = nft.leftFootSource;
                    nextStep.rFootSource = nft.rightFootSource;
                    nextStep.sMass = nft.supportMass;
                    nextStep.lFootDestination = nft.leftFootDestination;
                    nextStep.rFootDestination = nft.rightFootDestination;
                newStepInfoSets.push(nextStep);
            if(DEBUG) { log<NUClear::TRACE>("Messaging: Foot Motion Planner - Generate Foot Position Set(1)"); }
        });

        //If foot motion is requested, enable updating...
        on<Trigger<EnableFootMotion>>().then([this]
        {            
            postureInitialize(); // Reset stance as we don't know where our limbs are.
            updateHandle.enable();
        });

        //If foot motion no longer requested, cease updating...
        on<Trigger<DisableFootMotion>>().then([this] 
        {
            updateHandle.disable(); 
        });
    }
/*=======================================================================================================*/
//      METHOD: updateFootPosition
/*=======================================================================================================*/
    void FootMotionPlanner::updateFootPosition(double inPhase, const Transform2D& inActiveLimbSource, const LimbID& inActiveForwardLimb, const Transform2D& inActiveLimbDestination) 
    {       
        //Instantiate unitless phases for x(=0), y(=1) and z(=2) foot motion...
        arma::vec3 getFootPhases = getFootPhase(inPhase, phase1Single, phase2Single);

        //Lift foot by amount depending on walk speed
        if(DEBUG) { log<NUClear::TRACE>("Messaging: Foot Motion Planner - getFootPhase limits and calculations"); }
        auto& limit = (getVelocityCurrent().x() > velocityHigh ? accelerationLimitsHigh : accelerationLimits); // TODO: use a function instead
        float speed = std::min(1.0, std::max(std::abs(getVelocityCurrent().x() / limit[0]), std::abs(getVelocityCurrent().y() / limit[1])));
        float scale = (step_height_fast_fraction - step_height_slow_fraction) * speed + step_height_slow_fraction;
        getFootPhases[2] *= scale;
        if(DEBUG) { log<NUClear::TRACE>("Messaging: Foot Motion Planner - Interpolate Transform2D"); }           
        
        //Interpolate Transform2D from start to destination - deals with flat resolved movement in (x,y) coordinates     
        if (inActiveForwardLimb == LimbID::RIGHT_LEG) 
        {         
            //TODO: Vector field function??
            setRightFootPosition(inActiveLimbSource.interpolate(getFootPhases[0], inActiveLimbDestination));
        }
        else
        {         
            //TODO: Vector field function??
            setLeftFootPosition(inActiveLimbSource.interpolate(getFootPhases[0],  inActiveLimbDestination));
        }
        
        if(DEBUG) { log<NUClear::TRACE>("Messaging: Foot Motion Planner - Instantiate FootLocal Variables"); }
        //Translates foot motion into z dimension for stepping in three-dimensional space...
        Transform3D leftFootLocal  = getLeftFootPosition();
        Transform3D rightFootLocal = getRightFootPosition();       

        if(DEBUG) { log<NUClear::TRACE>("Messaging: Foot Motion Planner - Translate Z for support foot"); }
        // If the intended footstep is unchanged, cease z-translation to conserve energy and stop, otherwise proceed...
        if(!isTargetStepUnchanged())
        {
            // Manipulate(update) z component of foot position to action movement with a varying altitude locus...
            if (inActiveForwardLimb == LimbID::RIGHT_LEG) 
            {
                rightFootLocal = rightFootLocal.translateZ(stepHeight * getFootPhases[2]); //TODO: Vector field function??
                rightFootLocal = rightFootLocal.rotateY(calculateAnklePitch(inPhase)); // Rotate ankle pitch...
            }
            else
            {
                leftFootLocal  = leftFootLocal.translateZ(stepHeight  * getFootPhases[2]); //TODO: Vector field function??
                leftFootLocal  = leftFootLocal.rotateY(calculateAnklePitch(inPhase)); // Rotate ankle pitch...
            }     
        }

        //DEBUGGING: Emit relative feet position phase with respect to robot state... 
        if (emitFootPosition)
        {
            emit(graph("FMP Synchronising Foot Motion", stepHeight * getFootPhases[2]));
        }

        if(DEBUG) { log<NUClear::TRACE>("Messaging: Foot Motion Planner - Emit FootMotionUpdate"); }
        //Broadcast struct of updated foot motion data at corresponding phase identity...
        emit(std::make_unique<FootMotionUpdate>(inPhase, inActiveForwardLimb, getLeftFootPosition(), getRightFootPosition(), leftFootLocal, rightFootLocal));              
        // Notify whenever a subsequent foot step is promoted...
        emit(std::make_unique<NextFootTargetInfo>(  
                                                    newStepInfoSets.front().lFootSource,
                                                    newStepInfoSets.front().rFootSource,
                                                    newStepInfoSets.front().sMass,
                                                    newStepInfoSets.front().lFootDestination,
                                                    newStepInfoSets.front().rFootDestination
                                                 ));
    }
/*=======================================================================================================*/
//      METHOD: Foot Phase
/*=======================================================================================================*/
    arma::vec3 FootMotionPlanner::getFootPhase(double phase, double phase1Single, double phase2Single) 
    {
        // Computes relative x,z motion of foot during single support phase
        // phSingle = 0: x=0, z=0, phSingle = 1: x=1,z=0
        double phaseSingle = std::min(std::max(phase - phase1Single, 0.0) / (phase2Single - phase1Single), 1.0);
        double phaseSingleSkew = std::pow(phaseSingle, 0.8) - 0.17 * phaseSingle * (1 - phaseSingle);
        double xf = 0.5 * (1 - std::cos(M_PI * phaseSingleSkew));
        double zf = 0.5 * (1 - std::cos(2 * M_PI * phaseSingleSkew));
        return {xf, phaseSingle, zf};
    }
/*=======================================================================================================*/
//      METHOD: Calculate Ankle Pitch
/*=======================================================================================================*/
    double FootMotionPlanner::calculateAnklePitch(double inPhase)
    {
        //Instantiate ankle pitch rotation parameter...
        double anklePitch = 0;
        if(inPhase <= 0.1)
        {
            // Scale 0.0 to 0.1 -> 0.0 to 1.0 for proportionality...
            anklePitch = (10.0 * inPhase) * ankle_pitch_lift; 
        }
        else if(inPhase <= 0.5)
        {
            // Simplified ((0 - 1)/(0.5 - 0.1)) * (inPhase - 0.5) + 0...
            anklePitch = (1.25 - (2.5 * inPhase)) * ankle_pitch_lift; 
        }
        else if(inPhase <= 0.9)
        {
            // Simplified ((1 - 0)/(0.9 - 0.5)) * (inPhase - 0.9) + 1...
            anklePitch = (1.25 - (2.5 * inPhase)) * ankle_pitch_fall; 
        }
        else if(inPhase <= 1.0)
        {
            // Scale 0.9 to 1.0 -> -1.0 to 0.0 for proportionality...
            anklePitch = (10 * (inPhase - 1.0)) * ankle_pitch_fall; 
        }
        return (anklePitch);
    }
/*=======================================================================================================*/
//      METHOD: Reset The Stance of the Humanoid to Initial Valid Stance
/*=======================================================================================================*/
    void FootMotionPlanner::postureInitialize() 
    {    
        // Default Initial Torso Position...
        Transform2D uTorso = Transform2D({-getFootOffsetCoefficient(0), 0, 0});
        // Default Initial Left  Foot Position...
        setLeftFootPosition(uTorso.localToWorld({getFootOffsetCoefficient(0), kinematicsModel.Leg.HIP_OFFSET_Y - getFootOffsetCoefficient(1), 0}));        
        // Default Initial Right Foot Position...
        setRightFootPosition(uTorso.localToWorld({getFootOffsetCoefficient(0), -kinematicsModel.Leg.HIP_OFFSET_Y + getFootOffsetCoefficient(1), 0}));               
    }    
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Time
/*=======================================================================================================*/
    double FootMotionPlanner::getTime() 
    {
        if(DEBUG) { log<NUClear::TRACE>("System Time:%f\n\r", double(NUClear::clock::now().time_since_epoch().count()) * (1.0 / double(NUClear::clock::period::den))); }
        return (double(NUClear::clock::now().time_since_epoch().count()) * (1.0 / double(NUClear::clock::period::den)));
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: New Step Start Time
/*=======================================================================================================*/
    double FootMotionPlanner::getNewStepStartTime()
    {
        return (newStepStartTime);
    }
    void FootMotionPlanner::setNewStepStartTime(double inNewStartTime)
    {
        newStepStartTime = inNewStartTime;
    }    
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Destination Time
/*=======================================================================================================*/
    double FootMotionPlanner::getDestinationTime()
    {
        if(destinationTime.size() > 0)
        {
            return (destinationTime.front());
        }
        else
        {            
            return (stepTime); //DEBUGGING: blank value            
        }
    }
    void FootMotionPlanner::setDestinationTime(double inDestinationTime)
    {
        destinationTime.push(inDestinationTime);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Foot Offset Coefficient
/*=======================================================================================================*/
    double FootMotionPlanner::getFootOffsetCoefficient(int index)
    {
        return (footOffsetCoefficient[index]);
    }
    void FootMotionPlanner::setFootOffsetCoefficient(const arma::vec2& inFootOffsetCoefficient)
    {
        footOffsetCoefficient = inFootOffsetCoefficient;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setFootOffsetCoefficient
/*=======================================================================================================*/
    void FootMotionPlanner::setFootOffsetCoefficient(int index, double inValue)
    {
        footOffsetCoefficient[index] = inValue;
    }  
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Velocity
/*=======================================================================================================*/
    Transform2D FootMotionPlanner::getVelocityCurrent() 
    {
        if(velocityCurrent.size() > 0)
        {
            return (velocityCurrent.front());
        }
        else
        {          
            return (Transform2D({0, 0, 0})); //DEBUGGING: blank value
        }
    }
    void FootMotionPlanner::setVelocityCurrent(Transform2D inVelocityCurrent) 
    {
        velocityCurrent.push(inVelocityCurrent);
    }       
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Left Foot Position
/*=======================================================================================================*/
    Transform2D FootMotionPlanner::getLeftFootPosition()
    {
        return (leftFootPositionTransform);
    }
    void FootMotionPlanner::setLeftFootPosition(const Transform2D& inLeftFootPosition)
    {
        leftFootPositionTransform = inLeftFootPosition;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Right Foot Position
/*=======================================================================================================*/
    Transform2D FootMotionPlanner::getRightFootPosition()
    {
        return (rightFootPositionTransform);
    }
    void FootMotionPlanner::setRightFootPosition(const Transform2D& inRightFootPosition)
    {
        rightFootPositionTransform = inRightFootPosition;
    }    
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Active Limb Source
/*=======================================================================================================*/
    Transform2D FootMotionPlanner::getActiveLimbSource()
    {
        if(activeLimbSource.size() > 0)
        {
            return (activeLimbSource.front());
        }
        else
        {          
            return (Transform2D({0, 0, 0})); //DEBUGGING: blank value
        }
    }
    void FootMotionPlanner::setActiveLimbSource(const Transform2D& inActiveLimbSource)
    {
        activeLimbSource.push(inActiveLimbSource);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Active Limb Destination
/*=======================================================================================================*/
    Transform2D FootMotionPlanner::getActiveLimbDestination()
    {
        if(activeLimbDestination.size() > 0)
        {
            return (activeLimbDestination.front());
        }
        else
        {
            return (Transform2D({0, 0, 0})); //DEBUGGING: blank value
        }
    }
    void FootMotionPlanner::setActiveLimbDestination(const Transform2D& inActiveLimbDestination)
    {
        activeLimbDestination.push(inActiveLimbDestination);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Active Forward Limb
/*=======================================================================================================*/
    LimbID FootMotionPlanner::getActiveForwardLimb()
    {
        if(activeForwardLimb.size() > 0)
        {
            return (activeForwardLimb.front());
        }
        else
        {                   
            return (LimbID::INVALID); //DEBUGGING: blank value
        }
    }
    void FootMotionPlanner::setActiveForwardLimb(const LimbID& inActiveForwardLimb)
    {
        activeForwardLimb.push(inActiveForwardLimb);
    }    
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Is New Step the Same as Previous
/*=======================================================================================================*/    
    bool FootMotionPlanner::isTargetStepUnchanged()
    {
        return  (
                    (getActiveLimbSource().x() == getActiveLimbDestination().x())       &&
                    (getActiveLimbSource().y() == getActiveLimbDestination().y())       &&
                    (getActiveLimbSource().angle() == getActiveLimbDestination().angle())
                );
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: New Step Available
/*=======================================================================================================*/
    bool FootMotionPlanner::isNewStepAvailable()
    {    
        return (
                    (destinationTime.size() > MIN_QUEUE_SIZE)        && 
                    (velocityCurrent.size() > MIN_QUEUE_SIZE)        && 
                    (activeLimbSource.size() > MIN_QUEUE_SIZE)       && 
                    (activeForwardLimb.size() > MIN_QUEUE_SIZE)      &&
                    (activeLimbDestination.size() > MIN_QUEUE_SIZE)                         
               );
    }    
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: New Step Received
/*=======================================================================================================*/
    bool FootMotionPlanner::isNewStepReceived()
    {         
        return (
                    (destinationTime.size() > MIN_QUEUE_SIZE)                   && 
                    (destinationTime.size() == velocityCurrent.size())          && 
                    (velocityCurrent.size() == activeLimbSource.size())         && 
                    (activeLimbSource.size() == activeForwardLimb.size())       &&
                    (activeForwardLimb.size() == activeLimbDestination.size())                         
               );
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Motion Phase
/*=======================================================================================================*/
    double FootMotionPlanner::getMotionPhase()
    {
        // Obtain current system time...
        double currentTime = getTime();
        // The percentage completed of the current step, range: [0,1]...    
        double motionPhase = 1.0 - (((getNewStepStartTime() + getDestinationTime()) - currentTime) / stepTime);         
        // Bind phase value to range [0,1], emit status if step completed...
        if (motionPhase > 1)
        {
            motionPhase = std::fmod(motionPhase, 1);
            // Consume completed step instructions, only once an entire new set has been received...
            if(isNewStepReceived())
            {                    
                if (destinationTime.size() > MIN_QUEUE_SIZE)         { destinationTime.pop();        }
                if (velocityCurrent.size() > MIN_QUEUE_SIZE)         { velocityCurrent.pop();        }
                if (activeLimbSource.size() > MIN_QUEUE_SIZE)        { activeLimbSource.pop();       }
                if (activeForwardLimb.size() > MIN_QUEUE_SIZE)       { activeForwardLimb.pop();      }
                if (activeLimbDestination.size() > MIN_QUEUE_SIZE)   { activeLimbDestination.pop();  }
            }
            if (newStepInfoSets.size() > MIN_QUEUE_SIZE)   { newStepInfoSets.pop();  } // TODO
            // Increment relative step motionphase...
            setNewStepStartTime(getTime());   
            // If there has already been an updated instruction, then process before requesting new data...
            if(!isNewStepAvailable()) 
            {
                // Notify helper modules of completed footstep (trigger request for new step instruction)...
                emit(std::make_unique<FootStepRequested>(true)); 
            }
        }
        return (motionPhase);
    }
/*=======================================================================================================*/
//      METHOD: Configuration
/*=======================================================================================================*/
    void FootMotionPlanner::configure(const YAML::Node& config)
    {         
        if(DEBUG) { log<NUClear::TRACE>("Configure FootMotionPlanner - Start"); }      
        auto& wlk = config["walk_engine"];
        auto& fmp = config["foot_motion_planner"];

        auto& debug = fmp["debugging"];
        DEBUG = debug["enabled"].as<bool>();
        emitFootPosition = debug["emit_foot_position"].as<bool>();
        
        auto& stance = wlk["stance"];
        setFootOffsetCoefficient(stance["foot_offset"].as<arma::vec>());

        auto& fmp_walkCycle = fmp["walk_cycle"];
        ankle_pitch_lift = fmp_walkCycle["lift_ankle_pitch"].as<Expression>();
        ankle_pitch_fall = fmp_walkCycle["fall_ankle_pitch"].as<Expression>();

        auto& wlk_walkCycle = wlk["walk_cycle"];
        stepTime    = wlk_walkCycle["step_time"].as<Expression>();
        stepHeight  = wlk_walkCycle["step"]["height"].as<Expression>();
        stepLimits  = wlk_walkCycle["step"]["limits"].as<arma::mat::fixed<3,2>>();

        step_height_slow_fraction = wlk_walkCycle["step"]["height_slow_fraction"].as<float>();
        step_height_fast_fraction = wlk_walkCycle["step"]["height_fast_fraction"].as<float>();

        auto& velocity = wlk_walkCycle["velocity"];
        velocityLimits = velocity["limits"].as<arma::mat::fixed<3,2>>();
        velocityHigh   = velocity["high_speed"].as<Expression>();

        auto& acceleration = wlk_walkCycle["acceleration"];
        accelerationLimits          = acceleration["limits"].as<arma::vec>();
        accelerationLimitsHigh      = acceleration["limits_high"].as<arma::vec>();
        accelerationTurningFactor   = acceleration["turning_factor"].as<Expression>();

        phase1Single = wlk_walkCycle["single_support_phase"]["start"].as<Expression>();
        phase2Single = wlk_walkCycle["single_support_phase"]["end"].as<Expression>();         
        if(DEBUG) { log<NUClear::TRACE>("Configure FootMotionPlanner - Finish"); }               
    }
}  // motion
}  // modules