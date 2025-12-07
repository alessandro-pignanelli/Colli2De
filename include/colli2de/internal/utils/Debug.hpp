#pragma once

#include <print>

#ifdef DBG
#define DEBUG_ONLY(code) code
#else
#define DEBUG_ONLY(code)
#endif

#if defined(DBG) || defined(ENABLE_ASSERT)

#define _C2D_DEBUG_ASSERT_2_ARGS(condition, message)                                                                   \
    do                                                                                                                 \
    {                                                                                                                  \
        if (!(condition))                                                                                              \
        {                                                                                                              \
            throw std::runtime_error("Assertion failed: " + std::string(message));                                     \
        }                                                                                                              \
    } while (0)

#define _C2D_DEBUG_ASSERT_1_ARG(condition)                                                                             \
    do                                                                                                                 \
    {                                                                                                                  \
        if (!(condition))                                                                                              \
        {                                                                                                              \
            throw std::runtime_error("Assertion failed.");                                                             \
        }                                                                                                              \
    } while (0)

#define _C2D_DEBUG_ASSERT_GET_MACRO(_1, _2, NAME, ...) NAME
#define C2D_DEBUG_ASSERT(...)                                                                                          \
    _C2D_DEBUG_ASSERT_GET_MACRO(__VA_ARGS__, _C2D_DEBUG_ASSERT_2_ARGS, _C2D_DEBUG_ASSERT_1_ARG)(__VA_ARGS__)

#else
#define C2D_DEBUG_ASSERT(...)
#endif
