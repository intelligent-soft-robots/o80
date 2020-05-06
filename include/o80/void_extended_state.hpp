// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#pragma once

namespace o80 {

    class VoidExtendedState
    {
    public:
	VoidExtendedState()
	{
	}
	VoidExtendedState(const VoidExtendedState& other)
	{
	}
	VoidExtendedState(VoidExtendedState&& other) noexcept
	{
	}
	VoidExtendedState& operator=(const VoidExtendedState& other)
	{
	    return *this;
	}
	VoidExtendedState& operator=(VoidExtendedState&& other) noexcept
	{
	    return *this;
	}
	void console() const
	{
	    std::cout << "empty extended state" << std::endl;
	}
	template <class Archive>
	void serialize(Archive& archive)
	{
	    archive(foo);
	}
	char foo;
    };

}
