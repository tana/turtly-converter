use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};

use anyhow::Result;
use argmin::{
    core::{observers::ObserverMode, CostFunction, Executor, Gradient},
    solver::{linesearch::MoreThuenteLineSearch, quasinewton::LBFGS},
};
use argmin_observer_slog::SlogLogger;
use candle_core::{DType, Device, Tensor, Var};
use candle_nn::{Linear, Module, VarBuilder, VarMap};
use nalgebra::{vector, DMatrix, DVector, Vector3};
use serde::{Deserialize, Serialize};

use crate::utils::Mesh;

const DELTA: f64 = 1e-6;
const ELU_ALPHA: f64 = 1.0;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AdaptiveTransform {
    model: Model,
}

impl AdaptiveTransform {
    pub fn apply(&self, point: Vector3<f64>) -> Vector3<f64> {
        vector![point.x, point.y, self.model.evaluate(&point)]
    }

    pub fn apply_inverse(&self, _point: Vector3<f64>) -> Vector3<f64> {
        todo!()
    }

    pub fn jacobian(&self, point: Vector3<f64>) -> f64 {
        todo!()
    }
}

pub fn fit_adaptive(mesh: &Mesh, center: &Vector3<f64>) -> Result<AdaptiveTransform> {
    let targets = target_normals(mesh, center);

    let num_hidden = 100;

    let varmap = VarMap::new();
    let vs = VarBuilder::from_varmap(&varmap, DType::F64, &Device::Cpu);
    let model = TrainableModel::new(vs.clone(), num_hidden)?;

    let problem = Fitting {
        vars: varmap.all_vars(),
        model,
        targets,
    };

    let init_params = problem.get_params()?;

    log::info!("Fitting...");

    let linesearch = MoreThuenteLineSearch::new();
    let solver = LBFGS::new(linesearch, 5).with_tolerance_cost(1e-3)?;
    let fit_result = Executor::new(problem, solver)
        .add_observer(SlogLogger::term(), ObserverMode::Always)
        .configure(|state| state.param(init_params))
        .run()?;

    log::info!("Fitting completed");
    log::debug!("{}", fit_result);

    let problem_after = fit_result.problem.get_problem().unwrap();
    problem_after.set_params(&fit_result.state.best_param.unwrap())?;

    Ok(AdaptiveTransform {
        model: problem_after.model.into(),
    })
}

fn target_normals(mesh: &Mesh, center: &Vector3<f64>) -> Vec<(Vector3<f64>, Vector3<f64>)> {
    log::info!("Generating target normals...");

    let mut target = Vec::with_capacity(mesh.triangles.len());

    // Vertex normals
    let vert_normals = mesh.calc_vert_normals();

    for (pos, normal) in mesh.vertices.iter().zip(vert_normals.iter()) {
        // Overhang angle (positive means overhang)
        let angle = normal.z.acos() - FRAC_PI_2;

        if angle < 0.0 || angle > FRAC_PI_4 {
            continue;
        }

        let side = Vector3::z().cross(&normal).cross(&Vector3::z());
        if side.norm() < std::f64::EPSILON {
            continue;
        }
        let side = side.normalize();

        let target_normal = angle.cos() * Vector3::z() + angle.sin() * side;

        target.push((pos - center, target_normal));
    }

    target
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct Model {
    hidden_weights: DMatrix<f64>,
    hidden_bias: DVector<f64>,
    output_weights: DMatrix<f64>,
    output_bias: DVector<f64>,
}

impl Model {
    fn evaluate(&self, pos: &Vector3<f64>) -> f64 {
        let hidden_val = elu(&(&self.hidden_weights * pos + &self.hidden_bias), ELU_ALPHA);
        let output_val = elu(
            &(&self.output_weights * hidden_val + &self.output_bias),
            ELU_ALPHA,
        );
        pos.z + output_val[0]
    }
}

impl From<TrainableModel> for Model {
    fn from(value: TrainableModel) -> Self {
        let hidden_weights = tensor_to_matrix(value.hidden_layer.weight());
        let hidden_bias = tensor_to_vector(value.hidden_layer.bias().unwrap());
        let output_weights = tensor_to_matrix(value.output_layer.weight());
        let output_bias = tensor_to_vector(value.output_layer.bias().unwrap());

        assert_eq!(hidden_weights.ncols(), 3);
        assert_eq!(hidden_weights.nrows(), hidden_bias.nrows());
        assert_eq!(output_weights.ncols(), hidden_weights.nrows());
        assert_eq!(output_weights.nrows(), output_bias.nrows());
        assert_eq!(output_weights.nrows(), 1);

        Self {
            hidden_weights,
            hidden_bias,
            output_weights,
            output_bias,
        }
    }
}

fn elu(x: &DVector<f64>, alpha: f64) -> DVector<f64> {
    x.map(|c| if c > 0.0 { c } else { alpha * (c.exp() - 1.0) })
}

fn tensor_to_vector(tensor: &Tensor) -> DVector<f64> {
    DVector::from_vec(tensor.to_vec1().unwrap())
}

fn tensor_to_matrix(tensor: &Tensor) -> DMatrix<f64> {
    DMatrix::from_row_iterator(
        tensor.dim(0).unwrap(),
        tensor.dim(1).unwrap(),
        tensor.to_vec2().unwrap().iter().flatten().cloned(),
    )
}

#[derive(Clone, Debug)]
struct TrainableModel {
    pub hidden_layer: Linear,
    pub output_layer: Linear,
}

impl TrainableModel {
    fn new(vs: VarBuilder, num_hidden: usize) -> candle_core::Result<Self> {
        Ok(Self {
            hidden_layer: candle_nn::linear(3, num_hidden, vs.pp("hidden_layer"))?,
            output_layer: candle_nn::linear(num_hidden, 1, vs.pp("output_layer"))?,
        })
    }

    fn forward(&self, pos: &Tensor) -> candle_core::Result<Tensor> {
        let nn_out = self
            .output_layer
            .forward(&self.hidden_layer.forward(pos)?.elu(ELU_ALPHA)?)?
            .elu(ELU_ALPHA)?;

        Ok((pos.get_on_dim(1, 2)? + nn_out.get_on_dim(1, 0)?)?)
    }
}

fn loss_func(
    model: &TrainableModel,
    targets: &[(Vector3<f64>, Vector3<f64>)],
) -> candle_core::Result<Tensor> {
    let target_pos = targets
        .iter()
        .map(|(pos, _)| pos.iter())
        .flatten()
        .cloned()
        .collect();
    let target_pos = Tensor::from_vec(target_pos, (targets.len(), 3), &Device::Cpu)?;
    let target_grad = targets
        .iter()
        .map(|(_, grad)| grad.iter())
        .flatten()
        .cloned()
        .collect();
    let target_grad = Tensor::from_vec(target_grad, (targets.len(), 3), &Device::Cpu)?;

    // Calculate gradient of the model function (finite difference)
    let f = model.forward(&target_pos)?;
    assert_eq!(*f.shape(), targets.len().into());
    let dx = Tensor::new(&[DELTA, 0.0, 0.0], &Device::Cpu)?.repeat((targets.len(), 1))?;
    let dy = Tensor::new(&[0.0, DELTA, 0.0], &Device::Cpu)?.repeat((targets.len(), 1))?;
    let dz = Tensor::new(&[0.0, 0.0, DELTA], &Device::Cpu)?.repeat((targets.len(), 1))?;
    let dfdx = ((model.forward(&(&target_pos + dx)?)? - &f)? / DELTA)?;
    assert_eq!(*dfdx.shape(), targets.len().into());
    let dfdy = ((model.forward(&(&target_pos + dy)?)? - &f)? / DELTA)?;
    assert_eq!(*dfdy.shape(), targets.len().into());
    let dfdz = ((model.forward(&(&target_pos + dz)?)? - &f)? / DELTA)?;
    assert_eq!(*dfdz.shape(), targets.len().into());
    let grad = Tensor::stack(&[dfdx, dfdy, dfdz], 1)?;
    assert_eq!(*grad.shape(), (targets.len(), 3).into());

    // Mean Squared Error
    // candle_nn::loss::mse(&grad, &target_grad)

    // Normalize gradients
    let norm = grad.sqr()?.sum_keepdim(1)?.sqrt()?;
    assert_eq!(*norm.shape(), (targets.len(), 1).into());
    let grad = grad.broadcast_div(&norm)?;

    // Cosine similarity loss
    // Subtracted from 1 to convert maximization into minimization
    // See: https://docs.pytorch.org/docs/stable/generated/torch.nn.CosineEmbeddingLoss.html
    let grad_loss = (1.0 - (grad * target_grad)?.sum(1)?.mean(0)?)?;

    Ok(grad_loss)
}

struct Fitting {
    vars: Vec<Var>,
    model: TrainableModel,
    targets: Vec<(Vector3<f64>, Vector3<f64>)>,
}

impl Fitting {
    fn set_params(&self, params: &Vec<f64>) -> Result<()> {
        let mut p = 0;
        for var in self.vars.iter() {
            let n = var.elem_count();

            var.set(&Tensor::from_slice(
                &params[p..(p + n)],
                var.shape(),
                &Device::Cpu,
            )?)?;

            p += n;
        }

        Ok(())
    }

    fn get_params(&self) -> Result<Vec<f64>> {
        let mut params = Vec::new();
        for var in self.vars.iter() {
            params.append(&mut var.flatten_all()?.to_vec1()?);
        }

        Ok(params)
    }
}

impl CostFunction for Fitting {
    type Param = Vec<f64>;

    type Output = f64;

    fn cost(&self, param: &Self::Param) -> Result<Self::Output> {
        self.set_params(param)?;
        Ok(loss_func(&self.model, &self.targets)?.to_scalar()?)
    }
}

impl Gradient for Fitting {
    type Param = Vec<f64>;

    type Gradient = Vec<f64>;

    fn gradient(&self, param: &Self::Param) -> Result<Self::Gradient> {
        self.set_params(param)?;
        let loss = loss_func(&self.model, &self.targets)?;

        let grad_store = loss.backward()?;

        let mut grad = Vec::new();
        for var in self.vars.iter() {
            let var_grad = grad_store.get(var).expect("Gradient not computed");
            grad.append(&mut var_grad.flatten_all()?.to_vec1()?);
        }

        Ok(grad)
    }
}
