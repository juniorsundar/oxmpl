// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

mod prm;
mod rrt;
mod rrt_connect;
mod rrt_star;
pub use prm::PyPrm;
pub use rrt::PyRrt;
pub use rrt_connect::PyRrtConnect;
pub use rrt_star::PyRrtStar;

use pyo3::prelude::*;
use pyo3::types::PyDict;

pub fn create_module(py: Python<'_>) -> PyResult<Bound<'_, PyModule>> {
    let geometric_module = PyModule::new(py, "geometric")?;

    let sys_modules = py
        .import("sys")?
        .getattr("modules")?
        .downcast_into::<PyDict>()?;
    geometric_module.add_class::<PyRrt>()?;
    geometric_module.add_class::<PyRrtConnect>()?;
    geometric_module.add_class::<PyRrtStar>()?;
    geometric_module.add_class::<PyPrm>()?;

    sys_modules.set_item("oxmpl_py.geometric", &geometric_module)?;

    Ok(geometric_module)
}
