#include <iostream>
#include <string>

#include <pybind11/pybind11.h>

#include <dwa_planner_py/dwa_planner_py.h>


namespace py = pybind11;


void GridMap::meow() const {std::cout << "meow!" << std::endl; }


PYBIND11_MODULE(cat, m) {
    py::class_<Cat>(m, "Cat")
        .def(py::init<>());
}
