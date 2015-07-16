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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_BEHAVIOUR_STRATEGY_SOCCERSTRATEGGY_H
#define MODULES_BEHAVIOUR_STRATEGY_SOCCERSTRATEGGY_H

#include <nuclear>
#include <armadillo>

#include "messages/behaviour/FieldTarget.h"
#include "messages/behaviour/proto/Behaviour.pb.h"
#include "messages/localisation/FieldObject.h"
#include "messages/input/Sensors.h"
#include "messages/support/FieldDescription.h"

namespace modules {
namespace behaviour {
namespace strategy {

    class SoccerStrategy : public NUClear::Reactor {
    private:
        
        struct Config {
            NUClear::clock::duration ball_last_seen_max_time;
            NUClear::clock::duration goal_last_seen_max_time;

            arma::vec2 start_position_offensive;
            arma::vec2 start_position_defensive;
            bool is_goalie;

            float goalie_command_timeout;
            float goalie_rotation_speed_factor;
            float goalie_max_rotation_speed;
            float goalie_translation_speed_factor;
            float goalie_max_translation_speed;
        } cfg_;
        
        messages::behaviour::FieldTarget walkTarget;

        std::vector<messages::behaviour::FieldTarget> lookTarget;

        // TODO: remove horrible
        bool isGettingUp = false;
        bool isDiving = false;
        bool selfPenalised = false;
        messages::behaviour::proto::Behaviour::State currentState = messages::behaviour::proto::Behaviour::INIT;

        time_t ballLastMeasured;
        time_t selfLastMeasured;
        void initialLocalisationReset(const messages::support::FieldDescription& fieldDescription);
        void penaltyLocalisationReset();
        void unpenalisedLocalisationReset(const messages::support::FieldDescription& fieldDescription);

        void standStill();
        void searchWalk();
        void walkTo(const messages::support::FieldDescription& fieldDescription, const messages::behaviour::FieldTarget& object);
        void walkTo(const messages::support::FieldDescription& fieldDescription, arma::vec position);
        void find(const std::vector<messages::behaviour::FieldTarget>& objects);
        void spinWalk();
        bool pickedUp(const messages::input::Sensors& sensors);
        bool penalised();
        bool ballDistance(const messages::localisation::Ball& ball);
        void goalieWalk(const std::vector<messages::localisation::Self>& selfs, const std::vector<messages::localisation::Ball>& balls);
        arma::vec2 getKickPlan(const std::vector<messages::localisation::Self>& selfs, const messages::support::FieldDescription& fieldDescription);
    
    public:
        static constexpr const char* CONFIGURATION_PATH = "SoccerStrategy.yaml";

        explicit SoccerStrategy(std::unique_ptr<NUClear::Environment> environment);
    };

}  // strategy
}  // behaviours
}  // modules

#endif  // MODULES_BEHAVIOUR_STRATEGY_SOCCERSTRATEGGY_H

