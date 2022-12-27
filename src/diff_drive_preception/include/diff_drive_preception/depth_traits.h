#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace depth_traits
{
    template <typename T>
    struct DepthTraits {};

    template <>
    struct DepthTraits<uint16_t>
    {
        static inline bool valid(uint16_t depth)
        {
            return depth != 0;
        }
        static inline float toMeters(uint16_t depth)
        {
            return static_cast<float>(depth) * 0.001f;
        }
        static inline uint16_t fromMeters(float depth)
        {
            return std::lround(depth * 1000.0f);
        }
        static inline void initializeBuffer(std::vector<uint8_t> &buffer)
        {
            (void)buffer;
        }
    };

    template <>
    struct DepthTraits<float>
    {
        static inline bool valid(float depth)
        {
            return std::isfinite(depth);
        }
        static inline float toMeters(float depth)
        {
            return depth;
        }
        static inline float fromMeters(float depth)
        {
            return depth;
        }
        static inline void initializeBuffer(std::vector<uint8_t> &buffer)
        {
            auto start = reinterpret_cast<float *>(&buffer[0]);
            auto end = reinterpret_cast<float *>(&buffer[0] + buffer.size());
            std::fill(start, end, std::numeric_limits<float>::quiet_NaN());
        }
    };

}