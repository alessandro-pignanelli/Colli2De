#include <chrono>
#include <format>
#include <iostream>
#include <queue>
#include <catch2/catch_session.hpp>

#include "utils/Performance.hpp"

void parseCustomArgs(int& argc, char* argv[])
{
    std::queue<int> toRemove;
    for (int i = 1; i < argc; ++i)
        if (std::string(argv[i]) == "--filter-benchmark" || std::string(argv[i]) == "-fb")
        {
            if (i + 1 < argc)
            {
                c2d::test::filterBenchmark = argv[++i];
            }
            else
            {
                std::cerr << "Error: --filter-benchmark requires a benchmark name." << std::endl;
                exit(1);
            }
            toRemove.push(i);
            toRemove.push(i + 1);
        }

    int copyFrom = 0;
    for (int i = 0; i < argc; i++)
    {
        if (toRemove.front() == i)
        {
            toRemove.pop();
            copyFrom++;
        }
        if (i + copyFrom >= argc)
            break;
        argv[i] = argv[i + copyFrom];
    }
    argc -= copyFrom;
}

int main(int argc, char* argv[])
{
    parseCustomArgs(argc, argv);
    int result = Catch::Session().run(argc - 1, argv);

    const auto dateTime = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now());
    const auto dateTimeStr = std::format("{:%Y-%m-%d--%H-%M-%S}", dateTime);
    const auto fileName = std::format("benchmarks--{}.md", dateTimeStr);
    const auto folder = argv[argc - 1];
    c2d::test::exportBenchmarks(std::filesystem::path(folder) / fileName);

    return result;
}