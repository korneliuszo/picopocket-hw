/*
 * SPDX-FileCopyrightText: Korneliusz Osmenda <korneliuszo@gmail.com>
 *
 * SPDX-License-Identifier: MIT-0
 */
#pragma once

#include <jmpcoro.hpp>

using  Thread = JmpCoro::TCB<JmpCoro::RRSched>;
template<int STACK_LEN>
using  StaticThread = JmpCoro::StaticThread<Thread,STACK_LEN>;
