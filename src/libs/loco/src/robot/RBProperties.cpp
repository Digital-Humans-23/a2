#include "loco/robot/RBProperties.h"

namespace crl::loco {

RRBCollsionShape::~RRBCollsionShape() = default;

void RBProperties::setMOI(double moi00, double moi11, double moi22, double moi01, double moi02, double moi12) {
    MOI_local << moi00, moi01, moi02, moi01, moi11, moi12, moi02, moi12, moi22;
}

void RBProperties::offsetMOI(double x, double y, double z) {
    setMOI(MOI_local(0, 0) + mass * (y * y + z * z), MOI_local(1, 1) + mass * (x * x + z * z), MOI_local(2, 2) + mass * (x * x + y * y),
           MOI_local(0, 1) - mass * x * y, MOI_local(0, 2) - mass * x * z, MOI_local(1, 2) - mass * y * z);
}

void RBProperties::rotateMOI(double q, double x, double y, double z) {
    Quaternion quat(q, x, y, z);
    MOI_local = quat * MOI_local * quat.inverse();
}

Matrix3x3 RBProperties::getMOI(const Matrix3x3& RToWorld) const {
    return RToWorld * MOI_local * RToWorld.transpose();
}

Matrix3x3 RBProperties::getMOI(const Quaternion& q) const {
    return getMOI(q.toRotationMatrix());
}

}  // namespace crl::loco