/*
 * This file is part of DarwinPlatform.
 *
 * DarwinPlatform is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * DarwinPlatform is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with DarwinPlatform.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Trent Houliston <trent@houliston.me>
 */

#include "DarwinDevice.h"

Darwin::DarwinDevice::DarwinDevice(UART& coms, int id) : coms(coms), id(id) {}

bool Darwin::DarwinDevice::ping() {

    // Ping and get the result
    CommandResult result = coms.execute(PingCommand(id));

    // Check if there was an error code
    return result.header.errorcode == ErrorCode::NONE;
}