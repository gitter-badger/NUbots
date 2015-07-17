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

#include "NUbugger.h"

#include "messages/support/nubugger/proto/Message.pb.h"
#include "messages/behaviour/Action.h"
#include "messages/input/LimbID.h"

#include "utility/time/time.h"

namespace modules {
namespace support {
    using messages::support::nubugger::proto::Message;
    using utility::time::getUtcTimestamp;

    using messages::behaviour::ActionStart;
    using messages::behaviour::ActionKill;
    using messages::behaviour::RegisterAction;
    using messages::behaviour::ActionPriorites;
    using messages::behaviour::proto::Subsumption;

    using messages::input::LimbID;

    inline Subsumption::Limb getLimb(const LimbID& limb) {
        switch (limb) {
            case LimbID::LEFT_LEG:
                return Subsumption::LEFT_LEG;
            case LimbID::RIGHT_LEG:
                return Subsumption::RIGHT_LEG;
            case LimbID::LEFT_ARM:
                return Subsumption::LEFT_ARM;
            case LimbID::RIGHT_ARM:
                return Subsumption::RIGHT_ARM;
            case LimbID::HEAD:
                return Subsumption::HEAD;
            default:
                throw std::runtime_error("Invalid Limb");
        }
    }

    void NUbugger::provideSubsumption() {

        handles[Message::SUBSUMPTION].push_back(on<Trigger<ActionStart>>([this](const ActionStart& actionStart) {

            Message message = createMessage(Message::SUBSUMPTION);

            auto* subsumption = message.mutable_subsumption();
            
            auto* actionStateChange = subsumption->add_action_state_change();
            actionStateChange->set_state(Subsumption::ActionStateChange::START);
            actionStateChange->set_name(actionStart.name);
            
            for (auto& limbID : actionStart.limbs) {
                actionStateChange->add_limbs(getLimb(limbID));
            }

            send(message);

        }));

        handles[Message::SUBSUMPTION].push_back(on<Trigger<ActionKill>>([this](const ActionKill& actionKill) {

            Message message = createMessage(Message::SUBSUMPTION);

            auto* subsumption = message.mutable_subsumption();
            
            auto* actionStateChange = subsumption->add_action_state_change();
            actionStateChange->set_state(Subsumption::ActionStateChange::KILL);
            actionStateChange->set_name(actionKill.name);
            
            for (auto& limbID : actionKill.limbs) {
                actionStateChange->add_limbs(getLimb(limbID));
            }

            send(message);

        }));

        handles[Message::SUBSUMPTION].push_back(on<Trigger<RegisterAction>>([this] (const RegisterAction& action) {

            Message message = createMessage(Message::SUBSUMPTION);

            auto* subsumption = message.mutable_subsumption();
            
            auto* actionRegister = subsumption->add_action_register();
            uint id = action.id;
            actionRegister->set_id(id);
            actionRegister->set_name(action.name);
            
            for (const auto& set : action.limbSet) {
                auto* limbSet = actionRegister->add_limb_set();
                limbSet->set_priority(set.first);
                for (auto& limbID : set.second) {
                    limbSet->add_limbs(getLimb(limbID));
                }
            }

            actionRegisters.insert(std::make_pair(id, *actionRegister));
            send(message);

        }));

        handles[Message::SUBSUMPTION].push_back(on<Trigger<ActionPriorites>>([this] (const ActionPriorites& action) {
            
            Message message = createMessage(Message::SUBSUMPTION);

            auto* subsumption = message.mutable_subsumption();
            
            auto* actionPriorityChange = subsumption->add_action_priority_change();
            uint id = action.id;
            actionPriorityChange->set_id(id);

            Subsumption::ActionRegister actionRegister = actionRegisters.find(id)->second;

            size_t index = 0;
            for (const auto& priority : action.priorities) {
                Subsumption::LimbSet limbSet = actionRegister.limb_set(index);
                actionPriorityChange->add_priorities(priority);
                limbSet.set_priority(priority);
                index++;
            }

            send(message);

        }));
    }

    void NUbugger::sendSubsumption() {

        Message message = createMessage(Message::SUBSUMPTION);

        for (const auto& actionRegister : actionRegisters) {
            auto* objSubsumption = message.mutable_subsumption();
            *objSubsumption->add_action_register() = actionRegister.second;
        }

        send(message);

    }

}
}
