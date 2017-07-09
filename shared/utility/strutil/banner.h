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

#ifndef UTILITY_STRUTIL_BANNER_H
#define UTILITY_STRUTIL_BANNER_H

#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>

namespace utility {

namespace strutil {
    const char* BANNER_GLYPHS[8][255] = {{R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"(     )",      R"( _ )",
                                          R"( _ _ )",      R"(   _  _   )",
                                          R"(  _  )",      R"( _   __)",
                                          R"(        )",   R"( _ )",
                                          R"(  __)",       R"(__  )",
                                          R"(    _    )",  R"(       )",
                                          R"(   )",        R"(        )",
                                          R"(   )",        R"(     __)",
                                          R"(  ___  )",    R"( __ )",
                                          R"( ___  )",     R"( ____  )",
                                          R"( _  _   )",   R"( _____ )",
                                          R"(   __  )",    R"( ______ )",
                                          R"(  ___  )",    R"(  ___  )",
                                          R"(   )",        R"(   )",
                                          R"(   __)",      R"(        )",
                                          R"(__   )",      R"( ___  )",
                                          R"(         )",  R"(          )",
                                          R"( ____  )",    R"(  _____ )",
                                          R"( _____  )",   R"( ______ )",
                                          R"( ______ )",   R"(  _____ )",
                                          R"( _    _ )",   R"( _____ )",
                                          R"(      _ )",   R"( _  __)",
                                          R"( _      )",   R"( __  __ )",
                                          R"( _   _ )",    R"(  ____  )",
                                          R"( _____  )",   R"(  ____  )",
                                          R"( _____  )",   R"(  _____ )",
                                          R"( _______ )",  R"( _    _ )",
                                          R"(__      __)", R"(__          __)",
                                          R"(__   __)",    R"(__     __)",
                                          R"( ______)",    R"( ___ )",
                                          R"(__     )",    R"( ___ )",
                                          R"( /\ )",       R"(        )",
                                          R"( _ )",        R"(       )",
                                          R"( _     )",    R"(      )",
                                          R"(     _ )",    R"(      )",
                                          R"(  __ )",      R"(       )",
                                          R"( _     )",    R"( _ )",
                                          R"(   _ )",      R"( _    )",
                                          R"( _ )",        R"(           )",
                                          R"(       )",    R"(       )",
                                          R"(       )",    R"(       )",
                                          R"(      )",     R"(     )",
                                          R"( _   )",      R"(       )",
                                          R"(       )",    R"(          )",
                                          R"(      )",     R"(       )",
                                          R"(     )",      R"(   __)",
                                          R"( _ )",        R"(__   )",
                                          R"(  /\/|)",     R"()"},
                                         {R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"(     )",      R"(| |)",
                                          R"(( | ))",      R"( _| || |_ )",
                                          R"( | | )",      R"((_) / /)",
                                          R"(  ___   )",   R"(( ))",
                                          R"( / /)",       R"(\ \ )",
                                          R"( /\| |/\ )",  R"(   _   )",
                                          R"(   )",        R"(        )",
                                          R"(   )",        R"(    / /)",
                                          R"( / _ \ )",    R"(/_ |)",
                                          R"(|__ \ )",     R"(|___ \ )",
                                          R"(| || |  )",   R"(| ____|)",
                                          R"(  / /  )",    R"(|____  |)",
                                          R"( / _ \ )",    R"( / _ \ )",
                                          R"( _ )",        R"( _ )",
                                          R"(  / /)",      R"( ______ )",
                                          R"(\ \  )",      R"(|__ \ )",
                                          R"(   ____  )",  R"(    /\    )",
                                          R"(|  _ \ )",    R"( / ____|)",
                                          R"(|  __ \ )",   R"(|  ____|)",
                                          R"(|  ____|)",   R"( / ____|)",
                                          R"(| |  | |)",   R"(|_   _|)",
                                          R"(     | |)",   R"(| |/ /)",
                                          R"(| |     )",   R"(|  \/  |)",
                                          R"(| \ | |)",    R"( / __ \ )",
                                          R"(|  __ \ )",   R"( / __ \ )",
                                          R"(|  __ \ )",   R"( / ____|)",
                                          R"(|__   __|)",  R"(| |  | |)",
                                          R"(\ \    / /)", R"(\ \        / /)",
                                          R"(\ \ / /)",    R"(\ \   / /)",
                                          R"(|___  /)",    R"(|  _|)",
                                          R"(\ \    )",    R"(|_  |)",
                                          R"(|/\|)",       R"(        )",
                                          R"(( ))",        R"(       )",
                                          R"(| |    )",    R"(      )",
                                          R"(    | |)",    R"(      )",
                                          R"( / _|)",      R"(       )",
                                          R"(| |    )",    R"((_))",
                                          R"(  (_))",      R"(| |   )",
                                          R"(| |)",        R"(           )",
                                          R"(       )",    R"(       )",
                                          R"(       )",    R"(       )",
                                          R"(      )",     R"(     )",
                                          R"(| |  )",      R"(       )",
                                          R"(       )",    R"(          )",
                                          R"(      )",     R"(       )",
                                          R"(     )",      R"(  / /)",
                                          R"(| |)",        R"(\ \  )",
                                          R"( |/\/ )",     R"()"},
                                         {R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"(     )",      R"(| |)",
                                          R"( V V )",      R"(|_  __  _|)",
                                          R"(/ __))",      R"(   / / )",
                                          R"( ( _ )  )",   R"(|/ )",
                                          R"(| | )",       R"( | |)",
                                          R"( \ ` ' / )",  R"( _| |_ )",
                                          R"(   )",        R"( ______ )",
                                          R"(   )",        R"(   / / )",
                                          R"(| | | |)",    R"( | |)",
                                          R"(   ) |)",     R"(  __) |)",
                                          R"(| || |_ )",   R"(| |__  )",
                                          R"( / /_  )",    R"(    / / )",
                                          R"(| (_) |)",    R"(| (_) |)",
                                          R"((_))",        R"((_))",
                                          R"( / / )",      R"(|______|)",
                                          R"( \ \ )",      R"(   ) |)",
                                          R"(  / __ \ )",  R"(   /  \   )",
                                          R"(| |_) |)",    R"(| |     )",
                                          R"(| |  | |)",   R"(| |__   )",
                                          R"(| |__   )",   R"(| |  __ )",
                                          R"(| |__| |)",   R"(  | |  )",
                                          R"(     | |)",   R"(| ' / )",
                                          R"(| |     )",   R"(| \  / |)",
                                          R"(|  \| |)",    R"(| |  | |)",
                                          R"(| |__) |)",   R"(| |  | |)",
                                          R"(| |__) |)",   R"(| (___  )",
                                          R"(   | |   )",  R"(| |  | |)",
                                          R"( \ \  / / )", R"( \ \  /\  / / )",
                                          R"( \ V / )",    R"( \ \_/ / )",
                                          R"(   / / )",    R"(| |  )",
                                          R"( \ \   )",    R"(  | |)",
                                          R"(    )",       R"(        )",
                                          R"( \|)",        R"(  __ _ )",
                                          R"(| |__  )",    R"(  ___ )",
                                          R"(  __| |)",    R"(  ___ )",
                                          R"(| |_ )",      R"(  __ _ )",
                                          R"(| |__  )",    R"( _ )",
                                          R"(   _ )",      R"(| | __)",
                                          R"(| |)",        R"( _ __ ___  )",
                                          R"( _ __  )",    R"(  ___  )",
                                          R"( _ __  )",    R"(  __ _ )",
                                          R"( _ __ )",     R"( ___ )",
                                          R"(| |_ )",      R"( _   _ )",
                                          R"(__   __)",    R"(__      __)",
                                          R"(__  __)",     R"( _   _ )",
                                          R"( ____)",      R"( | | )",
                                          R"(| |)",        R"( | | )",
                                          R"(      )",     R"()"},
                                         {R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"(     )",      R"(| |)",
                                          R"(     )",      R"( _| || |_ )",
                                          R"(\__ \)",      R"(  / /  )",
                                          R"( / _ \/\)",   R"(   )",
                                          R"(| | )",       R"( | |)",
                                          R"(|_     _|)",  R"(|_   _|)",
                                          R"(   )",        R"(|______|)",
                                          R"(   )",        R"(  / /  )",
                                          R"(| | | |)",    R"( | |)",
                                          R"(  / / )",     R"( |__ < )",
                                          R"(|__   _|)",   R"(|___ \ )",
                                          R"(| '_ \ )",    R"(   / /  )",
                                          R"( > _ < )",    R"( \__, |)",
                                          R"(   )",        R"(   )",
                                          R"(< <  )",      R"( ______ )",
                                          R"(  > >)",      R"(  / / )",
                                          R"( / / _` |)",  R"(  / /\ \  )",
                                          R"(|  _ < )",    R"(| |     )",
                                          R"(| |  | |)",   R"(|  __|  )",
                                          R"(|  __|  )",   R"(| | |_ |)",
                                          R"(|  __  |)",   R"(  | |  )",
                                          R"( _   | |)",   R"(|  <  )",
                                          R"(| |     )",   R"(| |\/| |)",
                                          R"(| . ` |)",    R"(| |  | |)",
                                          R"(|  ___/ )",   R"(| |  | |)",
                                          R"(|  _  / )",   R"( \___ \ )",
                                          R"(   | |   )",  R"(| |  | |)",
                                          R"(  \ \/ /  )", R"(  \ \/  \/ /  )",
                                          R"(  > <  )",    R"(  \   /  )",
                                          R"(  / /  )",    R"(| |  )",
                                          R"(  \ \  )",    R"(  | |)",
                                          R"(    )",       R"(        )",
                                          R"(   )",        R"( / _` |)",
                                          R"(| '_ \ )",    R"( / __|)",
                                          R"( / _` |)",    R"( / _ \)",
                                          R"(|  _|)",      R"( / _` |)",
                                          R"(| '_ \ )",    R"(| |)",
                                          R"(  | |)",      R"(| |/ /)",
                                          R"(| |)",        R"(| '_ ` _ \ )",
                                          R"(| '_ \ )",    R"( / _ \ )",
                                          R"(| '_ \ )",    R"( / _` |)",
                                          R"(| '__|)",     R"(/ __|)",
                                          R"(| __|)",      R"(| | | |)",
                                          R"(\ \ / /)",    R"(\ \ /\ / /)",
                                          R"(\ \/ /)",     R"(| | | |)",
                                          R"(|_  /)",      R"(/ /  )",
                                          R"(| |)",        R"(  \ \)",
                                          R"(      )",     R"()"},
                                         {R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"(     )",      R"(|_|)",
                                          R"(     )",      R"(|_  __  _|)",
                                          R"((   /)",      R"( / / _ )",
                                          R"(| (_>  <)",   R"(   )",
                                          R"(| | )",       R"( | |)",
                                          R"( / , . \ )",  R"(  |_|  )",
                                          R"( _ )",        R"(        )",
                                          R"( _ )",        R"( / /   )",
                                          R"(| |_| |)",    R"( | |)",
                                          R"( / /_ )",     R"( ___) |)",
                                          R"(   | |  )",   R"( ___) |)",
                                          R"(| (_) |)",    R"(  / /   )",
                                          R"(| (_) |)",    R"(   / / )",
                                          R"( _ )",        R"( _ )",
                                          R"( \ \ )",      R"(|______|)",
                                          R"( / / )",      R"( |_|  )",
                                          R"(| | (_| |)",  R"( / ____ \ )",
                                          R"(| |_) |)",    R"(| |____ )",
                                          R"(| |__| |)",   R"(| |____ )",
                                          R"(| |     )",   R"(| |__| |)",
                                          R"(| |  | |)",   R"( _| |_ )",
                                          R"(| |__| |)",   R"(| . \ )",
                                          R"(| |____ )",   R"(| |  | |)",
                                          R"(| |\  |)",    R"(| |__| |)",
                                          R"(| |     )",   R"(| |__| |)",
                                          R"(| | \ \ )",   R"( ____) |)",
                                          R"(   | |   )",  R"(| |__| |)",
                                          R"(   \  /   )", R"(   \  /\  /   )",
                                          R"( / . \ )",    R"(   | |   )",
                                          R"( / /__ )",    R"(| |  )",
                                          R"(   \ \ )",    R"(  | |)",
                                          R"(    )",       R"(        )",
                                          R"(   )",        R"(| (_| |)",
                                          R"(| |_) |)",    R"(| (__ )",
                                          R"(| (_| |)",    R"(|  __/)",
                                          R"(| |  )",      R"(| (_| |)",
                                          R"(| | | |)",    R"(| |)",
                                          R"(  | |)",      R"(|   < )",
                                          R"(| |)",        R"(| | | | | |)",
                                          R"(| | | |)",    R"(| (_) |)",
                                          R"(| |_) |)",    R"(| (_| |)",
                                          R"(| |   )",     R"(\__ \)",
                                          R"(| |_ )",      R"(| |_| |)",
                                          R"( \ V / )",    R"( \ V  V / )",
                                          R"( >  < )",     R"(| |_| |)",
                                          R"( / / )",      R"(\ \  )",
                                          R"(| |)",        R"(  / /)",
                                          R"(      )",     R"()"},
                                         {R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"(     )",      R"((_))",
                                          R"(     )",      R"(  |_||_|  )",
                                          R"( |_| )",      R"(/_/ (_))",
                                          R"( \___/\/)",   R"(   )",
                                          R"(| | )",       R"( | |)",
                                          R"( \/|_|\/ )",  R"(       )",
                                          R"(( ))",        R"(        )",
                                          R"((_))",        R"(/_/    )",
                                          R"( \___/ )",    R"( |_|)",
                                          R"(|____|)",     R"(|____/ )",
                                          R"(   |_|  )",   R"(|____/ )",
                                          R"( \___/ )",    R"( /_/    )",
                                          R"( \___/ )",    R"(  /_/  )",
                                          R"((_))",        R"(( ))",
                                          R"(  \_\)",      R"(        )",
                                          R"(/_/  )",      R"( (_)  )",
                                          R"( \ \__,_|)",  R"(/_/    \_\)",
                                          R"(|____/ )",    R"( \_____|)",
                                          R"(|_____/ )",   R"(|______|)",
                                          R"(|_|     )",   R"( \_____|)",
                                          R"(|_|  |_|)",   R"(|_____|)",
                                          R"( \____/ )",   R"(|_|\_\)",
                                          R"(|______|)",   R"(|_|  |_|)",
                                          R"(|_| \_|)",    R"( \____/ )",
                                          R"(|_|     )",   R"( \___\_\)",
                                          R"(|_|  \_\)",   R"(|_____/ )",
                                          R"(   |_|   )",  R"( \____/ )",
                                          R"(    \/    )", R"(    \/  \/    )",
                                          R"(/_/ \_\)",    R"(   |_|   )",
                                          R"(/_____|)",    R"(| |_ )",
                                          R"(    \_\)",    R"( _| |)",
                                          R"(    )",       R"(        )",
                                          R"(   )",        R"( \__,_|)",
                                          R"(|_.__/ )",    R"( \___|)",
                                          R"( \__,_|)",    R"( \___|)",
                                          R"(|_|  )",      R"( \__, |)",
                                          R"(|_| |_|)",    R"(|_|)",
                                          R"(  | |)",      R"(|_|\_\)",
                                          R"(|_|)",        R"(|_| |_| |_|)",
                                          R"(|_| |_|)",    R"( \___/ )",
                                          R"(| .__/ )",    R"( \__, |)",
                                          R"(|_|   )",     R"(|___/)",
                                          R"( \__|)",      R"( \__,_|)",
                                          R"(  \_/  )",    R"(  \_/\_/  )",
                                          R"(/_/\_\)",     R"( \__, |)",
                                          R"(/___|)",      R"( | | )",
                                          R"(| |)",        R"( | | )",
                                          R"(      )",     R"()"},
                                         {R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"(     )",      R"(   )",
                                          R"(     )",      R"(          )",
                                          R"(     )",      R"(       )",
                                          R"(        )",   R"(   )",
                                          R"( \_\)",       R"(/_/ )",
                                          R"(         )",  R"(       )",
                                          R"(|/ )",        R"(        )",
                                          R"(   )",        R"(       )",
                                          R"(       )",    R"(    )",
                                          R"(      )",     R"(       )",
                                          R"(        )",   R"(       )",
                                          R"(       )",    R"(        )",
                                          R"(       )",    R"(       )",
                                          R"(   )",        R"(|/ )",
                                          R"(     )",      R"(        )",
                                          R"(     )",      R"(      )",
                                          R"(  \____/ )",  R"(          )",
                                          R"(       )",    R"(        )",
                                          R"(        )",   R"(        )",
                                          R"(        )",   R"(        )",
                                          R"(        )",   R"(       )",
                                          R"(        )",   R"(      )",
                                          R"(        )",   R"(        )",
                                          R"(       )",    R"(        )",
                                          R"(        )",   R"(        )",
                                          R"(        )",   R"(        )",
                                          R"(         )",  R"(        )",
                                          R"(          )", R"(              )",
                                          R"(       )",    R"(         )",
                                          R"(       )",    R"(|___|)",
                                          R"(       )",    R"(|___|)",
                                          R"(    )",       R"( ______ )",
                                          R"(   )",        R"(       )",
                                          R"(       )",    R"(      )",
                                          R"(       )",    R"(      )",
                                          R"(     )",      R"(  __/ |)",
                                          R"(       )",    R"(   )",
                                          R"( _/ |)",      R"(      )",
                                          R"(   )",        R"(           )",
                                          R"(       )",    R"(       )",
                                          R"(| |    )",    R"(    | |)",
                                          R"(      )",     R"(     )",
                                          R"(     )",      R"(       )",
                                          R"(       )",    R"(          )",
                                          R"(      )",     R"(  __/ |)",
                                          R"(     )",      R"(  \_\)",
                                          R"(| |)",        R"(/_/  )",
                                          R"(      )",     R"()"},
                                         {R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"()",           R"()",
                                          R"(     )",      R"(   )",
                                          R"(     )",      R"(          )",
                                          R"(     )",      R"(       )",
                                          R"(        )",   R"(   )",
                                          R"(    )",       R"(    )",
                                          R"(         )",  R"(       )",
                                          R"(   )",        R"(        )",
                                          R"(   )",        R"(       )",
                                          R"(       )",    R"(    )",
                                          R"(      )",     R"(       )",
                                          R"(        )",   R"(       )",
                                          R"(       )",    R"(        )",
                                          R"(       )",    R"(       )",
                                          R"(   )",        R"(   )",
                                          R"(     )",      R"(        )",
                                          R"(     )",      R"(      )",
                                          R"(         )",  R"(          )",
                                          R"(       )",    R"(        )",
                                          R"(        )",   R"(        )",
                                          R"(        )",   R"(        )",
                                          R"(        )",   R"(       )",
                                          R"(        )",   R"(      )",
                                          R"(        )",   R"(        )",
                                          R"(       )",    R"(        )",
                                          R"(        )",   R"(        )",
                                          R"(        )",   R"(        )",
                                          R"(         )",  R"(        )",
                                          R"(          )", R"(              )",
                                          R"(       )",    R"(         )",
                                          R"(       )",    R"(     )",
                                          R"(       )",    R"(     )",
                                          R"(    )",       R"(|______|)",
                                          R"(   )",        R"(       )",
                                          R"(       )",    R"(      )",
                                          R"(       )",    R"(      )",
                                          R"(     )",      R"( |___/ )",
                                          R"(       )",    R"(   )",
                                          R"(|__/ )",      R"(      )",
                                          R"(   )",        R"(           )",
                                          R"(       )",    R"(       )",
                                          R"(|_|    )",    R"(    |_|)",
                                          R"(      )",     R"(     )",
                                          R"(     )",      R"(       )",
                                          R"(       )",    R"(          )",
                                          R"(      )",     R"( |___/ )",
                                          R"(     )",      R"(     )",
                                          R"(|_|)",        R"(     )",
                                          R"(      )",     R"()"}};

    std::string banner(const std::string& input) {

        std::stringstream output;

        // Loop through each of our glyph rows
        for (int row = 0; row < 8; ++row) {
            // Loop through each character of our input
            for (unsigned char c : input) {
                output << BANNER_GLYPHS[row][uint(c)];
            }
            output << std::endl;
        }

        return output.str();
    }
}
}

#endif
