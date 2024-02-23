#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;
using namespace pybind11::literals;

#include "osqp_api_functions.h"
#include "osqp_api_types.h"
#include "osqp_data_workspace.h"

py::tuple solve() {
    py::gil_scoped_release release;
    OSQPInt status = osqp_solve(&osqp_data_solver);
    py::gil_scoped_acquire acquire;

    if (status != 0) throw std::runtime_error("Solve failed");

    OSQPInt m;
    OSQPInt n;
    osqp_get_dimensions(&osqp_data_solver, &m, &n);

    auto x = py::array_t<OSQPFloat>({n}, {sizeof(OSQPFloat)}, (&osqp_data_solver)->solution->x);
    auto y = py::array_t<OSQPFloat>({m}, {sizeof(OSQPFloat)}, (&osqp_data_solver)->solution->y);

    py::tuple results = py::make_tuple(x, y, status, (&osqp_data_solver)->info->iter, (&osqp_data_solver)->info->run_time);
    return results;
}

OSQPInt update_data_vec(py::object q, py::object l, py::object u) {
    OSQPFloat* _q;
    OSQPFloat* _l;
    OSQPFloat* _u;

    if (q.is_none()) {
        _q = NULL;
    } else {
        _q = (OSQPFloat *)py::array_t<OSQPFloat>(q).data();
    }
    if (l.is_none()) {
        _l = NULL;
    } else {
        _l = (OSQPFloat *)py::array_t<OSQPFloat>(l).data();
    }
    if (u.is_none()) {
        _u = NULL;
    } else {
        _u = (OSQPFloat *)py::array_t<OSQPFloat>(u).data();
    }

    return osqp_update_data_vec(&osqp_data_solver, _q, _l, _u);
}

#if OSQP_EMBEDDED_MODE == 2
OSQPInt update_data_mat(py::object P_x, py::object P_i, py::object A_x, py::object A_i) {
    OSQPFloat* _P_x;
    OSQPInt* _P_i;
    OSQPInt _P_n = 0;
    OSQPFloat* _A_x;
    OSQPInt* _A_i;
    OSQPInt _A_n = 0;

    if (P_x.is_none()) {
        _P_x = NULL;
    } else {
        auto _P_x_array = py::array_t<OSQPFloat>(P_x);
        _P_x = (OSQPFloat *)_P_x_array.data();
        _P_n = _P_x_array.size();
    }

    if (P_i.is_none()) {
        _P_i = NULL;
    } else {
        auto _P_i_array = py::array_t<OSQPInt>(P_i);
        _P_i = (OSQPInt *)_P_i_array.data();
        _P_n = _P_i_array.size();
    }

    if (A_x.is_none()) {
        _A_x = NULL;
    } else {
        auto _A_x_array = py::array_t<OSQPFloat>(A_x);
        _A_x = (OSQPFloat *)_A_x_array.data();
        _A_n = _A_x_array.size();
    }

    if (A_i.is_none()) {
        _A_i = NULL;
    } else {
        auto _A_i_array = py::array_t<OSQPInt>(A_i);
        _A_i = (OSQPInt *)_A_i_array.data();
        _A_n = _A_i_array.size();
    }

    return osqp_update_data_mat(&osqp_data_solver, _P_x, _P_i, _P_n, _A_x, _A_i, _A_n);
}
#endif

PYBIND11_MODULE(pyosqp, m) {
    m.def("solve", &solve);
    m.def("update_data_vec", &update_data_vec, "Update q/l/u", py::arg("q") = py::none(), py::arg("l") = py::none(), py::arg("u") = py::none());
#if OSQP_EMBEDDED_MODE == 2
    m.def("update_data_mat", &update_data_mat, "Update P/A", py::arg("P_x") = py::none(), py::arg("P_i") = py::none(), py::arg("A_x") = py::none(), py::arg("A_i") = py::none());
#endif
}