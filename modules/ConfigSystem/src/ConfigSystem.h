/*
 * This file is part of ConfigSystem.
 *
 * ConfigSystem is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ConfigSystem is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ConfigSystem.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Trent Houliston <trent@houliston.me>
 */

#ifndef MODULES_CONFIGSYSTEM_H_
#define MODULES_CONFIGSYSTEM_H_

#include <NUClear.h>
#include <vector>
#include <set>
#include <map>

#include "messages/Configuration.h"

namespace modules {

	class ConfigSystem : public NUClear::Reactor {

	private:
        static constexpr const char* BASE_CONFIGURATION_PATH = "config/";
        
        std::set<std::type_index> loaded;
		std::map<std::string, std::vector<std::function<void (NUClear::Reactor*, messages::ConfigurationNode*)>>> handler;

        volatile bool running;
        void run();
        void kill();

		int watcherFd;
        int killFd;
	public:
		explicit ConfigSystem(NUClear::PowerPlant* plant);
	};
}
#endif

