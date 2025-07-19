#include <chrono>
#include <fstream>
#include <filesystem>
#include <numeric>
#include <string>
#include <vector>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>

#include "utils/Print.hpp"

using namespace std::chrono;
using namespace std::literals::chrono_literals;
using namespace Catch;

#ifdef NDEBUG
    
#define __BENCHMARK_FUNCTION(name, threshold, func, reset, ...) \
{ \
    if (std::string(name).contains(c2d::test::filterBenchmark)) \
    { \
        std::vector<microseconds> elapsedTimes; \
        BENCHMARK(name) \
        { \
            (reset)(); \
            const auto start = high_resolution_clock::now(); \
            const auto result = func(); \
            const auto end = high_resolution_clock::now(); \
            elapsedTimes.push_back(duration_cast<microseconds>(end - start)); \
            return result; \
        }; \
        const auto elapsedAvg = std::accumulate(elapsedTimes.begin(), elapsedTimes.end(), microseconds(0)) / elapsedTimes.size(); \
        CHECK(elapsedAvg <= threshold); \
        c2d::test::printElapsed(elapsedAvg, threshold); \
        c2d::test::storeBenchmark(name, elapsedTimes, threshold); \
    } \
}

#else

#define __BENCHMARK_FUNCTION(name, threshold, func, reset, ...) \
{ \
    if (std::string(name).contains(c2d::test::filterBenchmark)) \
    { \
        c2d::test::println("Running benchmark only once: \"{}\"", name); \
        (reset)(); \
        const auto result = (func)(); \
    } \
}

#endif

#define BENCHMARK_FUNCTION(...) __BENCHMARK_FUNCTION(__VA_ARGS__, [](){})

namespace c2d
{
namespace test
{

inline std::string filterBenchmark = "";

struct BenchmarkResult
{
    std::string name;
    float threshold;
    float minTime;
    float maxTime;
    float avgTime;

    BenchmarkResult(const std::string& name, const microseconds& threshold, const std::vector<microseconds>& elapsed)
        : name(name), threshold(threshold.count() / 1000.0f), minTime(0.0f), maxTime(0.0f), avgTime(0.0f)
    {
        this->threshold = threshold.count() / 1000.0f;
        if (!elapsed.empty())
        {
            const size_t size = elapsed.size();
            const size_t exclude = 0;
            const auto begin = elapsed.begin() + exclude;
            const auto end = elapsed.end() - exclude;
            const size_t newSize = size - 2 * exclude;

            minTime = std::min_element(begin, end)->count() / 1000.0f;
            maxTime = std::max_element(begin, end)->count() / 1000.0f;
            avgTime = std::accumulate(begin, end, microseconds(0)).count() / 1000.0f / newSize;
        }
    }
};

inline std::vector<BenchmarkResult> benchmarkResults;
inline microseconds maxTime = 0us;

inline void storeBenchmark(const std::string& name, const std::vector<microseconds>& elapsed, const microseconds& threshold)
{
    benchmarkResults.emplace_back(name, threshold, elapsed);
    maxTime = std::max(maxTime, *std::max_element(elapsed.begin(), elapsed.end()));
    maxTime = std::max(maxTime, threshold);
}

namespace
{
    std::error_code createDirectoryRecursive(const std::string& dirName)
    {
        std::error_code err;

        if (std::filesystem::exists(dirName))
            return err;

        std::filesystem::create_directories(dirName, err);
        c2d::test::println("Error: {}", err.message());

        return err;
    }
}

inline void exportBenchmarks(const std::filesystem::path& filePath)
{
    if (benchmarkResults.empty())
            return;
    std::sort(benchmarkResults.begin(),
              benchmarkResults.end(),
              [](const BenchmarkResult& a, const BenchmarkResult& b) { return a.avgTime < b.avgTime; });

    const auto fileDirectory = filePath.parent_path();
    if (createDirectoryRecursive(fileDirectory.string()))
    {
        throw std::runtime_error("Failed to create directory for benchmark results: " + fileDirectory.string());
    }

    std::ofstream file(filePath.string());
    if (!file.is_open())
    {
        throw std::runtime_error("Failed to open benchmark results file: " + filePath.string());
    }

    int maxBenchmarkNameLength = 0;
    for (const auto& result : benchmarkResults)
        maxBenchmarkNameLength = std::max(maxBenchmarkNameLength, static_cast<int>(result.name.length()));
    const std::string benchmarkNameStr = "Benchmark Name";
    maxBenchmarkNameLength = std::max(maxBenchmarkNameLength, static_cast<int>(benchmarkNameStr.size()));

    #define println2(...) \
        _printlnFile(file, __VA_ARGS__); \
        c2d::test::println(__VA_ARGS__)

    // Header
    println2("{:<{}} | Status | Threshold (ms) | Avg (ms) | Min (ms) | Max (ms) |",
             benchmarkNameStr, maxBenchmarkNameLength);

    // Separator
    println2("{} | {} | {} | {} | {} | {} |", 
             std::string(maxBenchmarkNameLength, '-'),
             std::string(6, '-'),
             std::string(14, '-'),
             std::string(8, '-'),
             std::string(8, '-'),
             std::string(8, '-'));

    // Results
    const auto resultIntDigits = static_cast<int>(std::to_string(static_cast<int>(maxTime.count() / 1000.0f)).size());
    const auto resultTotalDigits = resultIntDigits + 4; // 3 decimal places + 1 for the dot
    for (const auto& result : benchmarkResults)
    {
        println2("{:<{}} | {:<6} | {:<14} | {:<8} | {:<8} | {:<8} |",
                 result.name, maxBenchmarkNameLength,
                 (result.avgTime <= result.threshold) ? "PASS" : "FAIL",
                 std::format("{:{}.3f}", result.threshold, resultTotalDigits),
                 std::format("{:{}.3f}", result.avgTime, resultTotalDigits),
                 std::format("{:{}.3f}", result.minTime, resultTotalDigits),
                 std::format("{:{}.3f}", result.maxTime, resultTotalDigits));
    }

    #undef println2
}

}
}
