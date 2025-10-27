#include "utils/Performance.hpp"

#include <catch2/catch_session.hpp>
#include <chrono>
#include <format>
#include <iostream>

void parseCustomArgs(int& argc, char* argv[])
{
    for (int i = 1; i < argc; ++i)
        if (std::string(argv[i]) == "--filter-benchmark" || std::string(argv[i]) == "-fb")
        {
            if (i + 1 < argc)
            {
                c2d::test::filterBenchmark = argv[i + 1];
            }
            else
            {
                std::cerr << "Error: --filter-benchmark requires a benchmark name." << std::endl;
                exit(1);
            }

            // Remove the argument from the list
            for (int j = i; j < argc - 2; ++j)
                argv[j] = argv[j + 2];
            argc -= 2;

            ++i;
        }
}

int main(int argc, char* argv[])
{
    parseCustomArgs(argc, argv);
    c2d::test::benchmarkSystem();

    int result = Catch::Session().run(argc - 1, argv);

    const auto dateTime = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now());
    const auto dateTimeStr = std::format("{:%Y-%m-%d--%H-%M-%S}", dateTime);
    const auto fileName = std::format("benchmarks--{}.md", dateTimeStr);
    const auto folder = argv[argc - 1];
    c2d::test::exportBenchmarks(std::filesystem::path(folder) / fileName);

    return result;
}
