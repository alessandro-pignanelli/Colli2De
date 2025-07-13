#include <chrono>
#include <format>
#include <catch2/catch_session.hpp>

#include "utils/Performance.hpp"

int main(int argc, char* argv[])
{
    int result = Catch::Session().run(argc - 1, argv);

    const auto dateTime = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now());
    const auto dateTimeStr = std::format("{:%Y-%m-%d--%H-%M-%S}", dateTime);
    const auto fileName = std::format("benchmarks--{}.md", dateTimeStr);
    const auto folder = argv[argc - 1];
    c2d::test::exportBenchmarks(std::filesystem::path(folder) / fileName);

    return result;
}