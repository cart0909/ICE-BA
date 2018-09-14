#pragma once
#include <memory>

template<class T>
using UPtr = std::unique_ptr<T>;

template<class T>
using SPtr = std::shared_ptr<T>;

template<class T>
using WPtr = std::weak_ptr<T>;
