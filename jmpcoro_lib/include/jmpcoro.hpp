/*
 * SPDX-FileCopyrightText: Korneliusz Osmenda <korneliuszo@gmail.com>
 *
 * SPDX-License-Identifier: MIT-0
 */
#pragma once

#include <cstddef>
#include <csetjmp>
#include <iostream>

namespace JmpCoro {

template<typename CRTP>
class Ring {
public:
	CRTP *next;
	CRTP *prev;
	Ring() : next(static_cast<CRTP*>(this)), prev(static_cast<CRTP*>(this))
	{
	};
	void append_to(CRTP *ring) {
		next = ring->next;
		prev = ring;
		ring->next = static_cast<CRTP*>(this);
		next->prev = static_cast<CRTP*>(this);
	};
	void disconnect() {
		next->prev = prev;
		prev->next = next;
	}
};

template<typename TCB>
class RRSched : public Ring<TCB>
{
protected:
	RRSched() : Ring<TCB>(){};

	TCB* next_in_schedule()
	{
		return this->next;
	}
	void prepare_run(TCB * from)
	{
		this->append_to(from);
	}
	TCB* finalize_run__provide_next()
	{
		this->disconnect();
		return this->next;
	}
};

template<template <typename> class Sched>
struct TCB : public Sched<TCB<Sched>> {
public:
	typedef void (*ENTRY)(TCB *tcb);
private:
	std::jmp_buf * volatile spc;
	static void switchsp(TCB *from, TCB *to) {
		std::jmp_buf sp;
		from->spc = &sp;
		if (setjmp(sp)) {
			return;
		}
		longjmp(*to->spc, 1);
	}
public:
	TCB() :
			spc(nullptr)
	{};
	void yield()
	{
		switchsp(this,Sched<TCB<Sched>>::next_in_schedule());
	}
protected:
	void run(TCB * from, volatile int * _sp, ENTRY _entry)
	{
		std::jmp_buf sp;
		from->spc = &sp;
		if(setjmp(sp)) {
			return;
		}
		this->prepare_run(from);
		TCB * to_register = this;
		asm volatile (
				"mov sp, %[SP]\n\t"
				"mov r0, %[TO_PTR]\n\t"
				"blx %[ENTRY]"
			:[TO_PTR]"+l" (to_register)
			:[SP]"l" (_sp),
			 [ENTRY]"l"(_entry)
			:"memory", "r0", "r1", "r2", "r3", "ip", "lr" //call clobbers that
		);
		longjmp(*to_register->finalize_run__provide_next()->spc,1); //as we cannot guarantee as from thread still works
	};
};


template<class Thread, int STACK_LEN>
class StaticThread: public Thread {
    volatile int stack[STACK_LEN];
public:
    template<class ... ARGS>
    StaticThread(ARGS ... args) : Thread(args ...){};

    void run(Thread * from, typename Thread::ENTRY _entry)
	{
    	Thread::run(from, &stack[STACK_LEN-1], _entry);
	}
};

};
