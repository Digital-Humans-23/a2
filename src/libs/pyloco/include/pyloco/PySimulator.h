//
// Created by Dongho Kang on 11.09.22.
//

#ifndef PYLOCO_PYBARESIMULATOR_H
#define PYLOCO_PYBARESIMULATOR_H

#include <pybind11/pybind11.h>

#include "pylocobase/sim/Simulator.h"

namespace py = pybind11;

namespace pyloco {

/**
 * This is a "Trampoline" class for binding pure virtual functions.
 * see https://pybind11.readthedocs.io/en/stable/advanced/classes.html for reference.
 */
class PySimulator : public Simulator {
public:
    using Simulator::Simulator;

    void reset() override {
        PYBIND11_OVERRIDE_PURE(void,      /* Return type */
                               Simulator, /* Parent class */
                               reset      /* Name of function in C++ (must match Python name) */
        );
    }

    void step(const crl::dVector &jointTarget) override {
        PYBIND11_OVERRIDE_PURE(void,       /* Return type */
                               Simulator,  /* Parent class */
                               step,       /* Name of function in C++ (must match Python name) */
                               jointTarget /* Argument(s) */
        );
    }
};

}  // namespace pyloco

#endif  //PYLOCO_PYBARESIMULATOR_H
