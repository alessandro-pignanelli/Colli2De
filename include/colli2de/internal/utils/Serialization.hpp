#pragma once

#ifdef C2D_USE_CEREAL
#include <cereal/cereal.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/optional.hpp>
#include <cereal/types/set.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/variant.hpp>
#include <cereal/types/vector.hpp>
#endif

#include <istream>
#include <ostream>

namespace c2d
{

class Writer
{
  public:
    Writer(std::ostream& out) : mOut(out) {}

    template <typename T>
    Writer& write(const T& v)
    {
        mOut.write(std::bit_cast<const char*>(&v), sizeof(T));
        return *this;
    }

    template <typename T>
    Writer& operator<<(const T& v)
    {
        write(v);
        return *this;
    }

    template <typename T>
    Writer& operator()(const T& v)
    {
        write(v);
        return *this;
    }

  private:
    std::ostream& mOut;
};

class Reader
{
  public:
    Reader(std::istream& in) : mIn(in) {}

    template <typename T>
    Reader& read(T& v)
    {
        mIn.read(std::bit_cast<char*>(&v), sizeof(T));
        return *this;
    }

    template <typename T>
    Reader& operator>>(T& v)
    {
        read(v);
        return *this;
    }

    template <typename T>
    Reader& operator()(T& v)
    {
        read(v);
        return *this;
    }

  private:
    std::istream& mIn;
};

#ifdef C2D_USE_CEREAL
template <typename T>
concept IsCerealArchive =
    std::derived_from<T, cereal::detail::OutputArchiveBase> || std::derived_from<T, cereal::detail::InputArchiveBase>;
#endif

}; // namespace c2d
