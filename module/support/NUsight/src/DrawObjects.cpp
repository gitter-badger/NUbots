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

#include "NUsight.h"

#include "message/support/nusight/DrawObjects.h"
#include "utility/time/time.h"

namespace module {
namespace support {
    using utility::time::getUtcTimestamp;

    using message::support::nusight::DrawObject;
    using message::support::nusight::DrawObjects;

    void NUsight::provideDrawObjects() {

        handles["draw_objects"].push_back(
            on<Trigger<DrawObjects>, Priority::LOW>().then([this](std::shared_ptr<const DrawObjects> drawObjects) {
                powerplant.emit_shared<Scope::NETWORK>(std::move(drawObjects), "nusight", false);
            }));

        handles["draw_objects"].push_back(
            on<Trigger<DrawObject>, Priority::LOW>().then([this](std::shared_ptr<const DrawObject> drawObject) {
                powerplant.emit_shared<Scope::NETWORK>(std::move(drawObject), "nusight", false);
            }));
    }
}  // namespace support
}  // namespace module
