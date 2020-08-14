extern crate kiss3d;
extern crate nalgebra as na;

use kiss3d::camera::{ArcBall, Camera};
use kiss3d::light::Light;
use kiss3d::window::Window;
use na::{Point3, Translation3, Unit, UnitQuaternion, Vector3};
use std::f64::consts::PI;
use std::path::Path;

// https://stackoverflow.com/questions/3684269/component-of-a-quaternion-rotation-around-an-axis
// fn find_orthonomal(normal: Vector3<f32>) -> Vector3<f32> {
// let y_quat = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI as f32 / 2.0);
// let x_quat = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI as f32 / 2.0);
// let mut w = x_quat.transform_vector(&normal);
// let dot = w.dot(&normal);
// if dot.abs() > 0.6 {
// w = y_quat.transform_vector(&normal);
// }

// normal.cross(&w.normalize())
// }

// https://stackoverflow.com/questions/3684269/component-of-a-quaternion-rotation-around-an-axis
// fn find_twist(quat: UnitQuaternion<f32>, axis: Vector3<f32>) -> UnitQuaternion<f32> {
// let normalized = axis.normalize();
// let orthonomal = find_orthonomal(normalized);
// let transformed = quat.transform_vector(&orthonomal);
// let flattened = (transformed - (transformed.dot(&normalized) * normalized)).normalize();

// return UnitQuaternion::from_axis_angle(
// &Unit::new_normalize(normalized),
// orthonomal.dot(&flattened).acos(),
// );
// }

fn quat_to_zyz(quat: UnitQuaternion<f32>) -> Vec<UnitQuaternion<f32>> {
    let matrix = quat.to_rotation_matrix();
    let divisor = (1.0 - matrix[(0, 0)].powf(2.0)).sqrt();
    let a = (matrix[(1, 0)]/divisor).asin();
    let b = matrix[(0, 0)].acos();
    let c = (matrix[(0, 1)]/divisor).asin();
    println!("{}, {}", divisor, a);
    vec![
        UnitQuaternion::from_axis_angle(&Vector3::x_axis(), a),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), b),
        UnitQuaternion::from_axis_angle(&Vector3::x_axis(), c),
    ]
    // let x = quat.coords[0];
    // let y = quat.coords[1];
    // let z = quat.coords[2];
    // let w = quat.coords[3];
    // two_axis_rot(
        // // zyz:
        // // 2.0*(y*z - w*x),
        // // 2.0*(x*z + w*y),
        // // w*w - x*x - y*y + z*z,
        // // 2.0*(y*z + w*x),
        // // -2.0*(x*z - w*y),
        // // zxz:
        // // 2.0*(x*z + w*y),
        // // -2.0*(y*z - w*x),
        // // w*w - x*x - y*y + z*z,
        // // 2.0*(x*z - w*y),
        // // 2.0*(y*z + w*x),
        // // yxy:
        // // 2.0*(x*y - w*z),
        // // 2.0*(y*z + w*x),
        // // w*w - x*x + y*y - z*z,
        // // 2.0*(x*y + w*z),
        // // -2.0*(y*z - w*x),
        // // yzy:
        // // 2.0*(y*z + w*x),
        // // -2.0*(x*y - w*z),
        // // w*w - x*x + y*y - z*z,
        // // 2.0*(y*z - w*x),
        // // 2.0*(x*y + w*z),
        // // xyx:
        // 2.0*(x*y + w*z),
        // -2.0*(x*z - w*y),
        // w*w + x*x - y*y - z*z,
        // 2.0*(x*y - w*z),
        // 2.0*(x*z + w*y),
        // // xzx:
        // // 2.0*(x*z - w*y),
        // // 2.0*(x*y + w*z),
        // // w*w + x*x - y*y - z*z,
        // // 2.0*(x*z + w*y),
        // // -2.0*(x*y - w*z),
        // )
}

fn two_axis_rot(r11: f32, r12: f32, r21: f32, r31: f32, r32: f32) -> Vec<UnitQuaternion<f32>> {
    return vec![
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), r11.atan2(r12) + PI as f32 / 2.0),
        UnitQuaternion::from_axis_angle(&Vector3::x_axis(), r21.acos() + PI as f32 / 2.0),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -1.0 * r31.atan2(r32))
    ];
}

struct Gimbal {
    outer_ring: UnitQuaternion<f32>,
    ring_a: UnitQuaternion<f32>,
    ring_b: UnitQuaternion<f32>,
    inner_ring: UnitQuaternion<f32>,
}

impl Gimbal {
    fn new() -> Gimbal {
        Gimbal {
            outer_ring: UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.0),
            ring_a: UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.0),
            ring_b: UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.0),
            inner_ring: UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.0),
        }
    }

    fn look_at(&mut self, target: UnitQuaternion<f32>) {
        let angles = quat_to_zyz(target / self.outer_ring);
        self.ring_a = self.outer_ring * angles[0];
        self.ring_b = self.outer_ring * angles[0] * angles[1];
        self.inner_ring = self.outer_ring * angles[0] * angles[1] * angles[2];
    }

    fn set_outer(&mut self, quat: UnitQuaternion<f32>) {
        self.outer_ring = quat;
    }
}

fn main() {
    let mut window = Window::new("Kiss3d: cube");
    // let mut c      = window.add_cube(1.0, 1.0, 1.0);
    let mut rings: Vec<kiss3d::scene::SceneNode> = vec![];
    rings.push(window.add_obj(
            Path::new("torus.obj"),
            Path::new("."),
            Vector3::new(1.0, 1.0, 1.0),
    ));
    rings.push(window.add_obj(
            Path::new("torus.obj"),
            Path::new("."),
            Vector3::new(0.8, 0.8, 0.8),
    ));
    rings.push(window.add_obj(
            Path::new("torus.obj"),
            Path::new("."),
            Vector3::new(0.6, 0.6, 0.6),
    ));
    rings.push(window.add_obj(
            Path::new("torus.obj"),
            Path::new("."),
            Vector3::new(0.4, 0.4, 0.4),
    ));

    // torus.set_color(1.0, 0.0, 0.0);

    let mut gimbal = Gimbal::new();

    window.set_light(Light::StickToCamera);
    // let mut point = window.add_sphere(5.0);
    // point.set_local_translation(Translation3::new(5.0, 5.0, 5.0));
    //
    // let mut look_indicator = window.add_cylinder(0.05, 5.0);

    let mut camera = ArcBall::new(Point3::new(-5.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0));
    // look_indicator.set_local_translation(Translation3::from(Vector3::new(0.0, 0.0, 0.0)));

    let mut spin: f32 = 0.0;

    while window.render_with_camera(&mut camera) {
        spin += 0.01;
        if spin >= 1.0 {
            spin = -1.0;
        }
        // gimbal.set_outer(UnitQuaternion::from_axis_angle(&Vector3::y_axis(), spin));
        // gimbal.look_at(UnitQuaternion::face_towards(
                // &camera.eye().coords,
                // &Vector3::y_axis()));
        gimbal.look_at(UnitQuaternion::from_axis_angle(&Vector3::z_axis(), spin));
        rings[0].set_local_rotation(gimbal.outer_ring);
        rings[1].set_local_rotation(gimbal.ring_a);
        rings[2].set_local_rotation(gimbal.ring_b);
        rings[3].set_local_rotation(gimbal.inner_ring);
        // look_indicator.set_local_rotation(gimbal.inner_ring);
    }
}
