// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use pyo3::{prelude::*, types::PyDict};

#[macro_use]
mod macros;
mod base;
mod geometric;

#[pymodule]
fn oxmpl_py(py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    let sys_modules = py
        .import("sys")?
        .getattr("modules")?
        .downcast_into::<PyDict>()?;
    let base_module = base::create_module(py)?;
    m.add_submodule(&base_module)?;
    sys_modules.set_item("oxmpl_py.base", base_module)?;

    let geometric_module = geometric::create_module(py)?;
    m.add_submodule(&geometric_module)?;
    sys_modules.set_item("oxmpl_py.geometric", geometric_module)?;

    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    m.add(
        "__doc__",
        "A Rust-powered motion planning library for Python, inspired by OMPL.",
    )?;

    Ok(())
}
