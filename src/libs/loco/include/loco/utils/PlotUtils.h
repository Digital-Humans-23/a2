//
// Created by Dongho Kang on 12.12.21.
//

#ifndef CRL_LOCO_PLOTUTILS_H
#define CRL_LOCO_PLOTUTILS_H

#include <string>
#include <vector>

#include "crl-basic/gui/plots.h"
#include "crl-basic/utils/mathDefs.h"

namespace crl::loco {

template <typename T>
class RealTimeLinePlot2D : public crl::gui::RealTimeLinePlot2D<T> {
public:
    explicit RealTimeLinePlot2D(const std::string &title, const std::string &xlabel, const std::string &ylabel, int maxSize = 1000)
        : crl::gui::RealTimeLinePlot2D<T>(title, xlabel, ylabel, maxSize) {}
};

}  // namespace crl::loco

#endif  //CRL_LOCO_PLOTUTILS_H
