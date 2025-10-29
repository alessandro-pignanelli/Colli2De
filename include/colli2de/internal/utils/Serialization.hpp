#pragma once

#include <istream>
#include <ostream>

class Writer
{
  public:
    Writer(std::ostream& out) : out(out) {}

    template <typename T>
    Writer& write(const T& v)
    {
        out.write(std::bit_cast<const char*>(&v), sizeof(T));
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
    std::ostream& out;
};

class Reader
{
  public:
    Reader(std::istream& in) : in(in) {}

    template <typename T>
    Reader& read(T& v)
    {
        in.read(std::bit_cast<char*>(&v), sizeof(T));
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
    std::istream& in;
};
