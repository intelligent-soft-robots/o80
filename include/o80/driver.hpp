#pragma once

namespace o80
{
template <class IN, class OUT>
class Driver
{
public:
    typedef IN DRIVER_IN;
    typedef OUT DRIVER_OUT;

public:
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void set(const IN& in) = 0;
    virtual OUT get() = 0;
};

}  // namespace o80
