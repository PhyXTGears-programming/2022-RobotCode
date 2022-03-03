// Copyright 2020 Robby Sammelson
// Used with permission

#pragma once

#include <ostream>

namespace swervedrive {

template <class T>
class vector2 {
    public:
        vector2 () : x(T()), y(T()) {}
        vector2 (T xVal, T yVal) : x(xVal), y(yVal) {}

        T get_x () const { return x; }
        T get_y () const { return y; }

        vector2<T> operator+ (const vector2<T> b) const {
            return vector2<T> {get_x() + b.get_x(), get_y() + b.get_y()};
        }

        T operator* (const vector2<T> b) const {
            return get_x() * b.get_x() + get_y() * b.get_y();
        }
        
        template<class O>
        auto operator* (const O b) const -> vector2<decltype(T()*O())> {
            return {get_x() * b, + get_y() * b};
        }
        
        template<class O>
        auto operator/ (const O b) const -> vector2<decltype(T()/O())> {
            return {get_x() / b, + get_y() / b};
        }
    
    private:
        T x, y;
};

}

template <class T>
std::ostream& operator<< (std::ostream& os, const swervedrive::vector2<T>& vector) {
    os << "(" << vector.get_x() << ", " << vector.get_y() << ")";
    return os;
}
