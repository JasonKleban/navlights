use core::{f32::consts::PI};
use num_traits::float::Float;

use libm::{cosf, sinf};

pub struct Perlin2d<const HEIGHT: usize, const WIDTH: usize> {
    gradients : [ [ [ f32 ; 2] ; HEIGHT ] ; WIDTH ]
}

impl<const HEIGHT: usize, const WIDTH: usize> Perlin2d<HEIGHT, WIDTH> {
    pub fn new<F : FnMut() -> u32>(mut random : F) -> Self {
        Perlin2d {
            gradients: core::array::from_fn(|_| {
                core::array::from_fn(|_| {
                    let angle = (random() as f32 / u32::MAX as f32) * 2.0 * PI;
                    [ cosf(angle), sinf(angle) ]
                })
            })
        }
    }

    fn dot(&self, cell_x : usize, cell_y : usize, v_x: f32, v_y: f32) -> f32 {
        let w = self.gradients[cell_x][cell_y];
        w[0] * v_x + w[1] * v_y
    }

    fn lerp(&self, a : f32, b : f32, t : f32) -> f32 {
        a + t * (b - a)
    }

    fn s_curve(&self, t : f32) -> f32 {
        t * t * (3.0 - 2.0 * t)
    }

    pub fn value(&self, x : f32, y : f32) -> f32 {
        let gx = x * (WIDTH as f32);
        let gy = y * (HEIGHT as f32);

        let x0 = gx.floor() as usize % WIDTH;
        let y0 = gy.floor() as usize % HEIGHT;
        let x1 = (x0 + 1) % WIDTH;
        let y1 = (y0 + 1) % HEIGHT;

        let gx_fract = gx.fract();
        let gy_fract = gy.fract();

        let s_gx = self.s_curve(gx_fract);
        let s_gy = self.s_curve(gy_fract);

        let v00 = self.dot(x0, y0, gx_fract, gy_fract);
        let v10 = self.dot(x1, y0, gx_fract - 1.0, gy_fract);
        let v01 = self.dot(x0, y1, gx_fract, gy_fract - 1.0);
        let v11 = self.dot(x1, y1, gx_fract - 1.0, gy_fract - 1.0);

        let vx0 = self.lerp(v00, v10, s_gx);
        let vx1 = self.lerp(v01, v11, s_gx);

        self.lerp(vx0, vx1, s_gy)
    }
}