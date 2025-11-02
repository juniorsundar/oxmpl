use std::rc::Rc;

use oxmpl::base::planner::PlannerConfig as OxmplPlannerConfig;
use pyo3::prelude::*;

#[pyclass(name = "PlannerConfig", unsendable)]
pub struct PyPlannerConfig(pub Rc<OxmplPlannerConfig>);

#[pymethods]
impl PyPlannerConfig {
    #[new]
    #[pyo3(signature = (seed=None))]
    fn new(seed: Option<u64>) -> Self {
        let planner_config = OxmplPlannerConfig { seed };
        Self(Rc::new(planner_config))
    }

    #[getter]
    fn get_seed(&self) -> Option<u64> {
        self.0.seed
    }

    fn __repr__(&self) -> String {
        format!("<PlannerConfig seed={:?}>", self.0.seed)
    }
}
