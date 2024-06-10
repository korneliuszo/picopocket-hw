#pragma once

#include <cstdint>

namespace {
extern "C" const int16_t Jingle[];
extern "C" void * _sizeof_Jingle;

template <typename T>
static constexpr inline auto ptrToAddr(T *pointer) noexcept
{
  return reinterpret_cast<std::uintptr_t>(pointer);
}
}

const int16_t *Jingle_ptr = Jingle;
const std::size_t Jingle_size = ptrToAddr(&_sizeof_Jingle);
