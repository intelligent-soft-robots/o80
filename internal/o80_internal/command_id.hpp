// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#pragma once

namespace o80
{
class CommandId
{
public:
    CommandId() : value(-1)
    {
    }
    CommandId(int _value) : value(_value)
    {
    }
    CommandId(const CommandId& other)
    {
        value = other.value;
    }
    CommandId(CommandId&& other) noexcept
    {
        value = other.value;
    }
    CommandId& operator=(const CommandId& other)
    {
        value = other.value;
        return *this;
    }
    CommandId& operator=(CommandId&& other) noexcept
    {
        value = other.value;
        return *this;
    }

    int value;

    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(value);
    }

    int get_id() const
    {
        static int id = -1;
        id++;
        return id;
    }
};
}  // namespace o80
