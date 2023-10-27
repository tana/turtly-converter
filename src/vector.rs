use std::ops;

/// Wrapper for `stl_io::Vector` with operators
#[derive(Copy, Clone, Debug)]
pub struct Vector(stl_io::Vector<f32>);

impl Vector {
    pub fn norm_sq(&self) -> f32 {
        let elements: [f32; 3] = Into::<stl_io::Vector<f32>>::into(*self).into();
        elements.iter().map(|e| e * e).sum()
    }

    pub fn norm(&self) -> f32 {
        self.norm_sq().sqrt()
    }
}

impl From<stl_io::Vector<f32>> for Vector {
    fn from(value: stl_io::Vector<f32>) -> Self {
        Self(value)
    }
}

impl Into<stl_io::Vector<f32>> for Vector {
    fn into(self) -> stl_io::Vector<f32> {
        let Self(inner) = self;
        inner
    }
}

impl ops::Add<Vector> for Vector {
    type Output = Self;

    fn add(self, rhs: Vector) -> Self {
        let Self(a) = self;
        let Self(b) = rhs;

        Self(stl_io::Vector::new([
            a[0] + b[0],
            a[1] + b[1],
            a[2] + b[2],
        ]))
    }
}

impl ops::Sub<Vector> for Vector {
    type Output = Self;

    fn sub(self, rhs: Vector) -> Self {
        let Self(a) = self;
        let Self(b) = rhs;

        Self(stl_io::Vector::new([
            a[0] - b[0],
            a[1] - b[1],
            a[2] - b[2],
        ]))
    }
}

impl ops::Mul<f32> for Vector {
    type Output = Self;

    fn mul(self, rhs: f32) -> Self {
        let Self(a) = self;

        Self(stl_io::Vector::new([
            a[0] * rhs,
            a[1] * rhs,
            a[2] * rhs,
        ]))
    }
}

impl ops::Div<f32> for Vector {
    type Output = Self;

    fn div(self, rhs: f32) -> Self {
        let Self(a) = self;

        Self(stl_io::Vector::new([
            a[0] / rhs,
            a[1] / rhs,
            a[2] / rhs,
        ]))
    }
}