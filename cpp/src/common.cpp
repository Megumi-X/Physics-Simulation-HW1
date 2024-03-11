#include "include/common.hpp"

namespace backend {

void Assert(const bool condition, const std::string& location, const std::string& message) {
    if (!condition) {
        std::stringstream ss;
        ss << "[" << location << "]: " << message << std::endl;
        std::cerr << ss.str();
        throw std::runtime_error(ss.str());
    }
}

template<>
const VectorXr FromSkewSymmetricMatrix<2>(const Matrix2r& A) {
    // [0, -w]
    // [w,  0]
    VectorXr a = VectorXr::Zero(1);
    a(0) = A(1, 0);
    return a;
}

template<>
const VectorXr FromSkewSymmetricMatrix<3>(const Matrix3r& A) {
    VectorXr a = VectorXr::Zero(3);
    a(0) = A(2, 1);
    a(1) = A(0, 2);
    a(2) = A(1, 0);
    return a;
}

template<>
const Matrix2r ToSkewSymmetricMatrix<2>(const VectorXr& a) {
    Assert(static_cast<integer>(a.size()) == 1, "ToSkewSymmetricMatrix", "Inconsistent a size.");
    Matrix2r A;
    A << 0, -a(0),
        a(0), 0;
    return A;
}

template<>
const Matrix3r ToSkewSymmetricMatrix<3>(const VectorXr& a) {
    Assert(static_cast<integer>(a.size()) == 3, "ToSkewSymmetricMatrix", "Inconsistent a size.");
    Matrix3r A;
    A << 0, -a(2), a(1),
        a(2), 0, -a(0),
        -a(1), a(0), 0;
    return A;
}

template<>
const VectorXr Cross<2>(const Vector2r& a, const Vector2r& b) {
    Vector1r c;
    c(0) = a.x() * b.y() - a.y() * b.x();
    return c;
}

template<>
const VectorXr Cross<3>(const Vector3r& a, const Vector3r& b){
    const Vector3r axb = a.cross(b);
    return axb;
}

}