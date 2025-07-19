use bincode::{deserialize, serialize};
use nalgebra::{Quaternion, Translation3, Unit, UnitQuaternion, Vector3};
use rapier3d::geometry::{InteractionGroups, TriMeshFlags};
use rapier3d::prelude::*;
use serde::{Deserialize, Serialize};
use std::num::NonZeroUsize;

pub struct World {
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    gravity: Vector3<Real>,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    query_pipeline: QueryPipeline,
    physics_hooks: (),
    event_handler: (),
}

#[derive(Serialize, Deserialize)]
pub struct SerializableWorld {
    gravity: Vector3<f32>,
    integration_parameters: IntegrationParameters,
    islands: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
}

impl World {
    pub fn new(gravity_x: f32, gravity_y: f32, gravity_z: f32) -> Self {
        let integration_parameters = IntegrationParameters::default();
        Self {
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            gravity: Vector3::new(gravity_x, gravity_y, gravity_z),
            integration_parameters: integration_parameters,
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: QueryPipeline::new(),
            physics_hooks: (),
            event_handler: (),
        }
    }

    pub fn bodies(&self) -> &RigidBodySet {
        &self.rigid_body_set
    }

    pub fn create_dynamic_body(&mut self) -> f64 {
        let rigid_body = RigidBodyBuilder::dynamic().build();
        let handle = self.rigid_body_set.insert(rigid_body);
        let (index, generation) = handle.into_raw_parts();
        encode_handle_for_js(index, generation)
    }

    pub fn create_kinematic_body(&mut self) -> f64 {
        let rigid_body = RigidBodyBuilder::kinematic_position_based().build();
        let handle = self.rigid_body_set.insert(rigid_body);
        let (index, generation) = handle.into_raw_parts();
        encode_handle_for_js(index, generation)
    }

    pub fn create_fixed_body(&mut self) -> f64 {
        let rigid_body = RigidBodyBuilder::fixed().build();
        let handle = self.rigid_body_set.insert(rigid_body);
        let (index, generation) = handle.into_raw_parts();
        encode_handle_for_js(index, generation)
    }

    pub fn add_box_collider(
        &mut self,
        handle: f64,
        half_x: f32,
        half_y: f32,
        half_z: f32,
        is_sensor: bool,
        pos_x: Option<f32>,
        pos_y: Option<f32>,
        pos_z: Option<f32>,
        rot_x: Option<f32>,
        rot_y: Option<f32>,
        rot_z: Option<f32>,
    ) -> f64 {
        let collider = ColliderBuilder::cuboid(half_x, half_y, half_z)
            .sensor(is_sensor)
            .translation(Vector3::new(
                pos_x.unwrap_or(0.0),
                pos_y.unwrap_or(0.0),
                pos_z.unwrap_or(0.0),
            ))
            .rotation(Vector3::new(
                rot_x.unwrap_or(0.0),
                rot_y.unwrap_or(0.0),
                rot_z.unwrap_or(0.0),
            ))
            .build();
        let (index, generation) = decode_handle_from_js(handle);
        let handle: RigidBodyHandle = RigidBodyHandle::from_raw_parts(index, generation);
        let handle =
            self.collider_set
                .insert_with_parent(collider, handle, &mut self.rigid_body_set);
        let (index, generation) = handle.into_raw_parts();
        encode_handle_for_js(index, generation)
    }

    pub fn add_cylinder_collider(
        &mut self,
        handle: f64,
        half_height: f32,
        radius: f32,
        is_sensor: bool,
        pos_x: Option<f32>,
        pos_y: Option<f32>,
        pos_z: Option<f32>,
        rot_x: Option<f32>,
        rot_y: Option<f32>,
        rot_z: Option<f32>,
    ) -> f64 {
        let collider = ColliderBuilder::cylinder(half_height, radius)
            .sensor(is_sensor)
            .translation(Vector3::new(
                pos_x.unwrap_or(0.0),
                pos_y.unwrap_or(0.0),
                pos_z.unwrap_or(0.0),
            ))
            .rotation(Vector3::new(
                rot_x.unwrap_or(0.0),
                rot_y.unwrap_or(0.0),
                rot_z.unwrap_or(0.0),
            ))
            .build();
        let (index, generation) = decode_handle_from_js(handle);
        let handle: RigidBodyHandle = RigidBodyHandle::from_raw_parts(index, generation);
        let handle =
            self.collider_set
                .insert_with_parent(collider, handle, &mut self.rigid_body_set);
        let (index, generation) = handle.into_raw_parts();
        encode_handle_for_js(index, generation)
    }

    pub fn add_trimesh_collider(
        &mut self,
        handle: f64,
        vertices: Vec<f32>,
        indices: Vec<u32>,
        is_sensor: bool,
        trimesh_flags: u32,
        pos_x: Option<f32>,
        pos_y: Option<f32>,
        pos_z: Option<f32>,
        rot_x: Option<f32>,
        rot_y: Option<f32>,
        rot_z: Option<f32>,
    ) -> f64 {
        let points: Vec<Point<Real>> = vertices
            .chunks_exact(3)
            .map(|chunk| Point::new(chunk[0], chunk[1], chunk[2]))
            .collect();

        let triangles: Vec<[u32; 3]> = indices
            .chunks_exact(3)
            .map(|chunk| [chunk[0], chunk[1], chunk[2]])
            .collect();

        let mut flags = TriMeshFlags::empty();
        if trimesh_flags & (TriMeshFlags::ORIENTED.bits() as u32) != 0 {
            flags |= TriMeshFlags::ORIENTED;
        }
        if trimesh_flags & (TriMeshFlags::FIX_INTERNAL_EDGES.bits() as u32) != 0 {
            flags |= TriMeshFlags::FIX_INTERNAL_EDGES;
        }
        let collider_builder_result =
            ColliderBuilder::trimesh_with_flags(points.clone(), triangles.clone(), flags);
        if let Ok(collider_builder) = collider_builder_result {
            let collider = collider_builder
                .sensor(is_sensor)
                .translation(Vector3::new(
                    pos_x.unwrap_or(0.0),
                    pos_y.unwrap_or(0.0),
                    pos_z.unwrap_or(0.0),
                ))
                .rotation(Vector3::new(
                    rot_x.unwrap_or(0.0),
                    rot_y.unwrap_or(0.0),
                    rot_z.unwrap_or(0.0),
                ))
                .build();
            let (index, generation) = decode_handle_from_js(handle);
            let handle: RigidBodyHandle = RigidBodyHandle::from_raw_parts(index, generation);
            let handle =
                self.collider_set
                    .insert_with_parent(collider, handle, &mut self.rigid_body_set);
            let (index, generation) = handle.into_raw_parts();
            encode_handle_for_js(index, generation)
        } else {
            0.0
        }
    }

    pub fn add_convex_hull_collider(
        &mut self,
        handle: f64,
        vertices: Vec<f32>,
        is_sensor: bool,
        pos_x: Option<f32>,
        pos_y: Option<f32>,
        pos_z: Option<f32>,
        rot_x: Option<f32>,
        rot_y: Option<f32>,
        rot_z: Option<f32>,
    ) -> f64 {
        let points: Vec<Point<Real>> = vertices
            .chunks_exact(3)
            .map(|chunk| Point::new(chunk[0], chunk[1], chunk[2]))
            .collect();

        let collider_builder_opt = ColliderBuilder::convex_hull(&points);
        if let Some(collider_builder) = collider_builder_opt {
            let collider = collider_builder
                .sensor(is_sensor)
                .translation(Vector3::new(
                    pos_x.unwrap_or(0.0),
                    pos_y.unwrap_or(0.0),
                    pos_z.unwrap_or(0.0),
                ))
                .rotation(Vector3::new(
                    rot_x.unwrap_or(0.0),
                    rot_y.unwrap_or(0.0),
                    rot_z.unwrap_or(0.0),
                ))
                .build();
            let (index, generation) = decode_handle_from_js(handle);
            let handle: RigidBodyHandle = RigidBodyHandle::from_raw_parts(index, generation);
            let handle =
                self.collider_set
                    .insert_with_parent(collider, handle, &mut self.rigid_body_set);
            let (index, generation) = handle.into_raw_parts();
            encode_handle_for_js(index, generation)
        } else {
            0.0
        }
    }

    pub fn create_revolute_joint(
        &mut self,
        body1_handle: f64,
        body2_handle: f64,
        anchor1_x: f32,
        anchor1_y: f32,
        anchor1_z: f32,
        anchor2_x: f32,
        anchor2_y: f32,
        anchor2_z: f32,
        axis_x: f32,
        axis_y: f32,
        axis_z: f32,
        wake_up: bool,
    ) -> f64 {
        let (index, generation) = decode_handle_from_js(body1_handle);
        let handle1: RigidBodyHandle = RigidBodyHandle::from_raw_parts(index, generation);
        let (index, generation) = decode_handle_from_js(body2_handle);
        let handle2: RigidBodyHandle = RigidBodyHandle::from_raw_parts(index, generation);
        if self.rigid_body_set.contains(handle1) && self.rigid_body_set.contains(handle2) {
            let joint = RevoluteJointBuilder::new(Unit::new_normalize(Vector3::new(
                axis_x, axis_y, axis_z,
            )))
            .local_anchor1(Point::new(anchor1_x, anchor1_y, anchor1_z))
            .local_anchor2(Point::new(anchor2_x, anchor2_y, anchor2_z))
            .build();
            let handle = self
                .impulse_joint_set
                .insert(handle1, handle2, joint, wake_up);
            let (index, generation) = handle.into_raw_parts();
            return encode_handle_for_js(index, generation);
        } else {
            return 0.0;
        }
    }

    pub fn create_fixed_joint(
        &mut self,
        body1_handle: f64,
        body2_handle: f64,
        anchor1_x: f32,
        anchor1_y: f32,
        anchor1_z: f32,
        anchor2_x: f32,
        anchor2_y: f32,
        anchor2_z: f32,
        frame1_x: f32,
        frame1_y: f32,
        frame1_z: f32,
        frame1_w: f32,
        frame2_x: f32,
        frame2_y: f32,
        frame2_z: f32,
        frame2_w: f32,
        wake_up: bool,
    ) -> f64 {
        let (index, generation) = decode_handle_from_js(body1_handle);
        let handle1: RigidBodyHandle = RigidBodyHandle::from_raw_parts(index, generation);
        let (index, generation) = decode_handle_from_js(body2_handle);
        let handle2: RigidBodyHandle = RigidBodyHandle::from_raw_parts(index, generation);
        if self.rigid_body_set.contains(handle1) && self.rigid_body_set.contains(handle2) {
            let joint = FixedJointBuilder::new()
                .local_anchor1(Point::new(anchor1_x, anchor1_y, anchor1_z))
                .local_anchor2(Point::new(anchor2_x, anchor2_y, anchor2_z))
                .local_frame1(Isometry::from_parts(
                    Translation3::identity(),
                    UnitQuaternion::from_quaternion(Quaternion::new(
                        frame1_w, frame1_x, frame1_y, frame1_z,
                    )),
                ))
                .local_frame2(Isometry::from_parts(
                    Translation3::identity(),
                    UnitQuaternion::from_quaternion(Quaternion::new(
                        frame2_w, frame2_x, frame2_y, frame2_z,
                    )),
                ))
                .build();
            let handle = self
                .impulse_joint_set
                .insert(handle1, handle2, joint, wake_up);
            let (index, generation) = handle.into_raw_parts();
            return encode_handle_for_js(index, generation);
        } else {
            return 0.0;
        }
    }

    pub fn set_revolute_joint_limits(&mut self, handle: f64, min: f32, max: f32) -> bool {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = ImpulseJointHandle::from_raw_parts(index, generation);
        if let Some(joint) = self.impulse_joint_set.get_mut(handle, true) {
            if let Some(revolute_joint) = joint.data.as_revolute_mut() {
                revolute_joint.set_limits([min, max]);
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

    pub fn configure_revolute_joint_motor(
        &mut self,
        handle: f64,
        target_pos: f32,
        target_vel: f32,
        stiffness: f32,
        damping: f32,
    ) -> bool {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = ImpulseJointHandle::from_raw_parts(index, generation);
        if let Some(joint) = self.impulse_joint_set.get_mut(handle, true) {
            if let Some(revolute_joint) = joint.data.as_revolute_mut() {
                revolute_joint.set_motor(target_pos, target_vel, stiffness, damping);
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

    pub fn get_joint_data(
        &self,
        handle: f64,
    ) -> Option<(f64, f64, f32, f32, f32, f32, f32, f32, f32, f32, f32)> {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = ImpulseJointHandle::from_raw_parts(index, generation);
        if let Some(joint) = self.impulse_joint_set.get(handle) {
            if let Some(revolute_joint) = joint.data.as_revolute() {
                let (index1, generation1) = joint.body1.into_raw_parts();
                let body1 = encode_handle_for_js(index1, generation1);
                let (index2, generation2) = joint.body2.into_raw_parts();
                let body2 = encode_handle_for_js(index2, generation2);
                let anchor1 = revolute_joint.local_anchor1().coords;
                let anchor2 = revolute_joint.local_anchor2().coords;
                let axis = revolute_joint.data.local_axis1();
                Some((
                    body1, body2, anchor1.x, anchor1.y, anchor1.z, anchor2.x, anchor2.y, anchor2.z,
                    axis.x, axis.y, axis.z,
                ))
            } else {
                None
            }
        } else {
            None
        }
    }

    pub fn set_body_next_kinematic_translation(
        &mut self,
        handle: f64,
        x: f32,
        y: f32,
        z: f32,
    ) -> bool {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = RigidBodyHandle::from_raw_parts(index, generation);
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.set_next_kinematic_translation(Vector3::new(x, y, z));
            true
        } else {
            false
        }
    }

    pub fn set_body_translation(
        &mut self,
        handle: f64,
        x: f32,
        y: f32,
        z: f32,
        wake_up: bool,
    ) -> bool {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = RigidBodyHandle::from_raw_parts(index, generation);
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.set_translation(Vector3::new(x, y, z), wake_up);
            true
        } else {
            false
        }
    }

    pub fn set_body_next_kinematic_rotation(
        &mut self,
        handle: f64,
        x: f32,
        y: f32,
        z: f32,
        w: f32,
    ) -> bool {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = RigidBodyHandle::from_raw_parts(index, generation);
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            let rotation = UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z));
            body.set_next_kinematic_rotation(rotation);
            true
        } else {
            false
        }
    }

    pub fn set_body_rotation(
        &mut self,
        handle: f64,
        x: f32,
        y: f32,
        z: f32,
        w: f32,
        wake_up: bool,
    ) -> bool {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = RigidBodyHandle::from_raw_parts(index, generation);
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            let rotation = UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z));
            body.set_rotation(rotation, wake_up);
            true
        } else {
            false
        }
    }

    pub fn set_body_velocity(
        &mut self,
        handle: f64,
        vx: f32,
        vy: f32,
        vz: f32,
        wake_up: bool,
    ) -> bool {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = RigidBodyHandle::from_raw_parts(index, generation);
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            let lin_velocity = Vector3::new(vx, vy, vz);
            body.set_linvel(lin_velocity, wake_up);
            true
        } else {
            false
        }
    }

    pub fn set_body_angular_velocity(
        &mut self,
        handle: f64,
        wx: f32,
        wy: f32,
        wz: f32,
        wake_up: bool,
    ) -> bool {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = RigidBodyHandle::from_raw_parts(index, generation);
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            let ang_velocity = Vector3::new(wx, wy, wz);
            body.set_angvel(ang_velocity, wake_up);
            true
        } else {
            false
        }
    }

    pub fn apply_impulse(&mut self, handle: f64, x: f32, y: f32, z: f32, wake_up: bool) -> bool {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = RigidBodyHandle::from_raw_parts(index, generation);
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            let impulse = Vector3::new(x, y, z);
            body.apply_impulse(impulse, wake_up);
            true
        } else {
            false
        }
    }

    pub fn get_body_translation(&self, handle: f64) -> Option<(f32, f32, f32)> {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = RigidBodyHandle::from_raw_parts(index, generation);
        if let Some(body) = self.rigid_body_set.get(handle) {
            let translation = body.translation();
            Some((translation.x, translation.y, translation.z))
        } else {
            None
        }
    }

    pub fn get_body_rotation(&self, handle: f64) -> Option<(f32, f32, f32, f32)> {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = RigidBodyHandle::from_raw_parts(index, generation);
        if let Some(body) = self.rigid_body_set.get(handle) {
            let rotation = body.rotation();
            Some((rotation.i, rotation.j, rotation.k, rotation.w))
        } else {
            None
        }
    }

    pub fn get_body_velocity(&self, handle: f64) -> Option<(f32, f32, f32)> {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = RigidBodyHandle::from_raw_parts(index, generation);
        if let Some(body) = self.rigid_body_set.get(handle) {
            let linvel = body.linvel();
            Some((linvel.x, linvel.y, linvel.z))
        } else {
            None
        }
    }

    pub fn get_body_angular_velocity(&self, handle: f64) -> Option<(f32, f32, f32)> {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = RigidBodyHandle::from_raw_parts(index, generation);
        if let Some(body) = self.rigid_body_set.get(handle) {
            let angvel = body.angvel();
            Some((angvel.x, angvel.y, angvel.z))
        } else {
            None
        }
    }

    pub fn set_body_enabled_translations(
        &mut self,
        handle: f64,
        enable_x: bool,
        enable_y: bool,
        enable_z: bool,
        wake_up: bool,
    ) -> bool {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = RigidBodyHandle::from_raw_parts(index, generation);
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.set_enabled_translations(enable_x, enable_y, enable_z, wake_up);
            true
        } else {
            false
        }
    }

    pub fn set_body_enabled_rotations(
        &mut self,
        handle: f64,
        enable_x: bool,
        enable_y: bool,
        enable_z: bool,
        wake_up: bool,
    ) -> bool {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = RigidBodyHandle::from_raw_parts(index, generation);
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.set_enabled_rotations(enable_x, enable_y, enable_z, wake_up);
            true
        } else {
            false
        }
    }

    pub fn get_body_mass(&self, handle: f64) -> Option<f32> {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = RigidBodyHandle::from_raw_parts(index, generation);
        if let Some(body) = self.rigid_body_set.get(handle) {
            Some(body.mass() as f32)
        } else {
            None
        }
    }

    pub fn get_body_collider(&self, handle: f64, collider_index: usize) -> Option<f64> {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = RigidBodyHandle::from_raw_parts(index, generation);
        if let Some(body) = self.rigid_body_set.get(handle) {
            if let Some(handle) = body.colliders().get(collider_index) {
                let (index, generation) = handle.into_raw_parts();
                Some(encode_handle_for_js(index, generation))
            } else {
                None
            }
        } else {
            None
        }
    }

    pub fn get_body_num_colliders(&self, handle: f64) -> usize {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = RigidBodyHandle::from_raw_parts(index, generation);
        if let Some(body) = self.rigid_body_set.get(handle) {
            body.colliders().len()
        } else {
            0
        }
    }

    pub fn step(&mut self, dt: f32) {
        self.integration_parameters.dt = dt;

        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            Some(&mut self.query_pipeline),
            &self.physics_hooks,
            &self.event_handler,
        );
    }

    pub fn set_gravity(&mut self, x: f32, y: f32, z: f32) {
        self.gravity = Vector3::new(x, y, z);
    }

    pub fn set_body_enabled(&mut self, handle: f64, enabled: bool) -> bool {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = RigidBodyHandle::from_raw_parts(index, generation);
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.set_enabled(enabled);
            true
        } else {
            false
        }
    }

    pub fn is_body_enabled(&self, handle: f64) -> bool {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = RigidBodyHandle::from_raw_parts(index, generation);
        if let Some(body) = self.rigid_body_set.get(handle) {
            body.is_enabled()
        } else {
            false
        }
    }

    pub fn body_sleep(&mut self, handle: f64) -> bool {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = RigidBodyHandle::from_raw_parts(index, generation);
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.sleep();
            true
        } else {
            false
        }
    }

    pub fn is_body_sleeping(&self, handle: f64) -> bool {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = RigidBodyHandle::from_raw_parts(index, generation);
        if let Some(body) = self.rigid_body_set.get(handle) {
            body.is_sleeping()
        } else {
            false
        }
    }

    pub fn set_body_soft_ccd_prediction(&mut self, handle: f64, prediction: f32) -> bool {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = RigidBodyHandle::from_raw_parts(index, generation);
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.set_soft_ccd_prediction(prediction);
            true
        } else {
            false
        }
    }

    pub fn set_body_ccd_enabled(&mut self, handle: f64, enabled: bool) -> bool {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = RigidBodyHandle::from_raw_parts(index, generation);
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.enable_ccd(enabled);
            true
        } else {
            false
        }
    }

    pub fn set_body_additional_solver_iterations(
        &mut self,
        handle: f64,
        iterations: usize,
    ) -> bool {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = RigidBodyHandle::from_raw_parts(index, generation);
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.set_additional_solver_iterations(iterations);
            true
        } else {
            false
        }
    }

    pub fn set_body_angular_damping(&mut self, handle: f64, damping: f32) -> bool {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = RigidBodyHandle::from_raw_parts(index, generation);
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.set_angular_damping(damping);
            true
        } else {
            false
        }
    }

    pub fn set_body_linear_damping(&mut self, handle: f64, damping: f32) -> bool {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = RigidBodyHandle::from_raw_parts(index, generation);
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.set_linear_damping(damping);
            true
        } else {
            false
        }
    }

    pub fn set_collider_density(&mut self, handle: f64, density: f32) -> bool {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = ColliderHandle::from_raw_parts(index, generation);
        if let Some(collider) = self.collider_set.get_mut(handle) {
            collider.set_density(density);
            true
        } else {
            false
        }
    }

    pub fn set_collider_friction(&mut self, handle: f64, friction: f32) -> bool {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = ColliderHandle::from_raw_parts(index, generation);
        if let Some(collider) = self.collider_set.get_mut(handle) {
            collider.set_friction(friction);
            true
        } else {
            false
        }
    }

    pub fn set_collider_restitution(&mut self, handle: f64, restitution: f32) -> bool {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = ColliderHandle::from_raw_parts(index, generation);
        if let Some(collider) = self.collider_set.get_mut(handle) {
            collider.set_restitution(restitution);
            true
        } else {
            false
        }
    }

    pub fn set_collider_collision_groups(&mut self, handle: f64, groups: u32) -> bool {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = ColliderHandle::from_raw_parts(index, generation);
        if let Some(collider) = self.collider_set.get_mut(handle) {
            collider.set_collision_groups(InteractionGroups::new(
                Group::from_bits_retain((groups >> 16) as u32),
                Group::from_bits_retain((groups & 0x0000_ffff) as u32),
            ));
            true
        } else {
            false
        }
    }

    pub fn set_collider_contact_skin(&mut self, handle: f64, contact_skin: f32) -> bool {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = ColliderHandle::from_raw_parts(index, generation);
        if let Some(collider) = self.collider_set.get_mut(handle) {
            collider.set_contact_skin(contact_skin);
            true
        } else {
            false
        }
    }

    pub fn set_collider_enabled(&mut self, handle: f64, enabled: bool) -> bool {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = ColliderHandle::from_raw_parts(index, generation);
        if let Some(collider) = self.collider_set.get_mut(handle) {
            collider.set_enabled(enabled);
            true
        } else {
            false
        }
    }

    pub fn is_collider_enabled(&self, handle: f64) -> bool {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = ColliderHandle::from_raw_parts(index, generation);
        if let Some(collider) = self.collider_set.get(handle) {
            collider.is_enabled()
        } else {
            false
        }
    }

    pub fn get_collider_shape_type(&self, handle: f64) -> Option<u32> {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = ColliderHandle::from_raw_parts(index, generation);
        if let Some(collider) = self.collider_set.get(handle) {
            let shape = collider.shape();
            if shape.as_cuboid().is_some() {
                Some(1)
            } else if shape.as_cylinder().is_some() {
                Some(10)
            } else if shape.as_trimesh().is_some() {
                Some(6)
            } else if shape.as_convex_polyhedron().is_some() {
                Some(9)
            } else {
                None
            }
        } else {
            None
        }
    }

    pub fn get_collider_parent(&self, handle: f64) -> Option<f64> {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = ColliderHandle::from_raw_parts(index, generation);
        if let Some(collider) = self.collider_set.get(handle) {
            if let Some(parent_handle) = collider.parent() {
                let (index, generation) = parent_handle.into_raw_parts();
                Some(encode_handle_for_js(index, generation))
            } else {
                None
            }
        } else {
            None
        }
    }

    pub fn get_collider_translation(&self, handle: f64) -> Option<(f32, f32, f32)> {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = ColliderHandle::from_raw_parts(index, generation);
        if let Some(collider) = self.collider_set.get(handle) {
            let translation = collider.translation();
            Some((translation.x, translation.y, translation.z))
        } else {
            None
        }
    }

    pub fn get_collider_rotation(&self, handle: f64) -> Option<(f32, f32, f32, f32)> {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = ColliderHandle::from_raw_parts(index, generation);
        if let Some(collider) = self.collider_set.get(handle) {
            let rotation = collider.rotation();
            Some((rotation.i, rotation.j, rotation.k, rotation.w))
        } else {
            None
        }
    }

    pub fn get_collider_vertices(&self, handle: f64) -> Option<Vec<f32>> {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = ColliderHandle::from_raw_parts(index, generation);
        if let Some(collider) = self.collider_set.get(handle) {
            let shape = collider.shape();
            if let Some(trimesh) = shape.as_trimesh() {
                Some(
                    trimesh
                        .vertices()
                        .iter()
                        .flat_map(|v| v.coords.iter().cloned())
                        .collect(),
                )
            } else if let Some(convex) = shape.as_convex_polyhedron() {
                Some(
                    convex
                        .points()
                        .iter()
                        .flat_map(|p| p.coords.iter().cloned())
                        .collect(),
                )
            } else {
                None
            }
        } else {
            None
        }
    }

    pub fn get_collider_indices(&self, handle: f64) -> Option<Vec<u32>> {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = ColliderHandle::from_raw_parts(index, generation);
        if let Some(collider) = self.collider_set.get(handle) {
            let shape = collider.shape();
            if let Some(trimesh) = shape.as_trimesh() {
                Some(
                    trimesh
                        .indices()
                        .iter()
                        .flat_map(|arr| arr.iter().cloned())
                        .collect(),
                )
            } else {
                None
            }
        } else {
            None
        }
    }

    pub fn get_collider_half_extents(&self, handle: f64) -> Option<(f32, f32, f32)> {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = ColliderHandle::from_raw_parts(index, generation);
        if let Some(collider) = self.collider_set.get(handle) {
            if let Some(cuboid) = collider.shape().as_cuboid() {
                Some((
                    cuboid.half_extents.x,
                    cuboid.half_extents.y,
                    cuboid.half_extents.z,
                ))
            } else {
                None
            }
        } else {
            None
        }
    }

    pub fn get_collider_radius(&self, handle: f64) -> Option<f32> {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = ColliderHandle::from_raw_parts(index, generation);
        if let Some(collider) = self.collider_set.get(handle) {
            if let Some(cylinder) = collider.shape().as_cylinder() {
                Some(cylinder.radius)
            } else if let Some(cuboid) = collider.shape().as_cuboid() {
                Some(
                    cuboid
                        .half_extents
                        .x
                        .max(cuboid.half_extents.y)
                        .max(cuboid.half_extents.z),
                )
            } else {
                None
            }
        } else {
            None
        }
    }

    pub fn get_collider_half_height(&self, handle: f64) -> Option<f32> {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = ColliderHandle::from_raw_parts(index, generation);
        if let Some(collider) = self.collider_set.get(handle) {
            if let Some(cylinder) = collider.shape().as_cylinder() {
                Some(cylinder.half_height)
            } else if let Some(cuboid) = collider.shape().as_cuboid() {
                Some(cuboid.half_extents.z)
            } else {
                None
            }
        } else {
            None
        }
    }

    pub fn get_collider_flags(&self, handle: f64) -> Option<u32> {
        let (index, generation) = decode_handle_from_js(handle);
        let handle = ColliderHandle::from_raw_parts(index, generation);
        if let Some(collider) = self.collider_set.get(handle) {
            if let Some(trimesh) = collider.shape().as_trimesh() {
                Some(trimesh.flags().bits() as u32)
            } else {
                None
            }
        } else {
            None
        }
    }

    pub fn take_snapshot(&self) -> Vec<u8> {
        let serializable_world = SerializableWorld {
            gravity: self.gravity,
            integration_parameters: self.integration_parameters.clone(),
            islands: self.island_manager.clone(),
            broad_phase: self.broad_phase.clone(),
            narrow_phase: self.narrow_phase.clone(),
            bodies: self.rigid_body_set.clone(),
            colliders: self.collider_set.clone(),
            impulse_joints: self.impulse_joint_set.clone(),
            multibody_joints: self.multibody_joint_set.clone(),
        };

        serialize(&serializable_world).unwrap_or_else(|_| Vec::new())
    }

    pub fn restore_snapshot(&mut self, snapshot: &[u8]) -> bool {
        match deserialize::<SerializableWorld>(snapshot) {
            Ok(world_data) => {
                self.gravity = world_data.gravity;
                self.integration_parameters = world_data.integration_parameters;
                self.island_manager = world_data.islands;
                self.broad_phase = world_data.broad_phase;
                self.narrow_phase = world_data.narrow_phase;
                self.rigid_body_set = world_data.bodies;
                self.collider_set = world_data.colliders;
                self.impulse_joint_set = world_data.impulse_joints;
                self.multibody_joint_set = world_data.multibody_joints;
                true
            }
            Err(_) => false,
        }
    }
}

static mut WORLD: Option<World> = None;

#[neon::export]
fn init_world(gravity_x: f64, gravity_y: f64, gravity_z: f64) -> bool {
    unsafe {
        WORLD = Some(World::new(
            gravity_x as f32,
            gravity_y as f32,
            gravity_z as f32,
        ));
    }
    true
}

#[neon::export]
fn destroy_world() -> bool {
    unsafe {
        WORLD = None;
    }
    true
}

#[neon::export]
fn set_world_gravity(x: f64, y: f64, z: f64) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.set_gravity(x as f32, y as f32, z as f32);
            true
        } else {
            false
        }
    }
}

#[neon::export]
fn get_world_gravity() -> Vec<f64> {
    unsafe {
        if let Some(ref world) = WORLD {
            vec![
                world.gravity.x as f64,
                world.gravity.y as f64,
                world.gravity.z as f64,
            ]
        } else {
            vec![0.0, 0.0, 0.0]
        }
    }
}

#[neon::export]
fn set_integration_parameters_num_solver_iterations(num_solver_iterations: f64) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.integration_parameters.num_solver_iterations =
                NonZeroUsize::new(num_solver_iterations as usize).unwrap();
            true
        } else {
            false
        }
    }
}

#[neon::export]
fn set_integration_parameters_num_additional_friction_iterations(
    num_additional_friction_iterations: f64,
) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world
                .integration_parameters
                .num_additional_friction_iterations = num_additional_friction_iterations as usize;
            true
        } else {
            false
        }
    }
}

#[neon::export]
fn set_integration_parameters_num_internal_pgs_iterations(
    num_internal_pgs_iterations: f64,
) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.integration_parameters.num_internal_pgs_iterations =
                num_internal_pgs_iterations as usize;
            true
        } else {
            false
        }
    }
}

#[neon::export]
fn set_integration_parameters_min_island_size(min_island_size: f64) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.integration_parameters.min_island_size = min_island_size as usize;
            true
        } else {
            false
        }
    }
}

#[neon::export]
fn get_world_bodies() -> Vec<f64> {
    unsafe {
        if let Some(ref world) = WORLD {
            world
                .rigid_body_set
                .iter()
                .map(|(handle, _)| {
                    let (index, generation) = handle.into_raw_parts();
                    encode_handle_for_js(index, generation)
                })
                .collect()
        } else {
            Vec::new()
        }
    }
}

#[neon::export]
fn get_world_colliders() -> Vec<f64> {
    unsafe {
        if let Some(ref world) = WORLD {
            world
                .collider_set
                .iter()
                .map(|(handle, _)| {
                    let (index, generation) = handle.into_raw_parts();
                    encode_handle_for_js(index, generation)
                })
                .collect()
        } else {
            Vec::new()
        }
    }
}

#[neon::export]
fn get_world_impulse_joints() -> Vec<f64> {
    unsafe {
        if let Some(ref world) = WORLD {
            world
                .impulse_joint_set
                .iter()
                .map(|(handle, _)| {
                    let (index, generation) = handle.into_raw_parts();
                    encode_handle_for_js(index, generation)
                })
                .collect()
        } else {
            Vec::new()
        }
    }
}

#[neon::export]
fn set_timestep(dt: f64) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.integration_parameters.dt = dt as f32;
            true
        } else {
            false
        }
    }
}

#[neon::export]
fn take_snapshot() -> Vec<u8> {
    unsafe {
        if let Some(ref world) = WORLD {
            world.take_snapshot()
        } else {
            Vec::new()
        }
    }
}

#[neon::export]
fn restore_snapshot(snapshot: Vec<u8>) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.restore_snapshot(&snapshot)
        } else {
            false
        }
    }
}

#[neon::export]
fn create_dynamic_body() -> f64 {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.create_dynamic_body()
        } else {
            0.0
        }
    }
}

#[neon::export]
fn create_kinematic_body() -> f64 {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.create_kinematic_body()
        } else {
            0.0
        }
    }
}

#[neon::export]
fn create_fixed_body() -> f64 {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.create_fixed_body()
        } else {
            0.0
        }
    }
}

#[neon::export]
fn add_box_collider(
    handle: f64,
    half_x: f64,
    half_y: f64,
    half_z: f64,
    is_sensor: bool,
    pos_x: Option<f64>,
    pos_y: Option<f64>,
    pos_z: Option<f64>,
    rot_x: Option<f64>,
    rot_y: Option<f64>,
    rot_z: Option<f64>,
) -> f64 {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.add_box_collider(
                handle,
                half_x as f32,
                half_y as f32,
                half_z as f32,
                is_sensor,
                pos_x.map(|v| v as f32),
                pos_y.map(|v| v as f32),
                pos_z.map(|v| v as f32),
                rot_x.map(|v| v as f32),
                rot_y.map(|v| v as f32),
                rot_z.map(|v| v as f32),
            ) as f64
        } else {
            0.0
        }
    }
}

#[neon::export]
fn add_cylinder_collider(
    handle: f64,
    half_height: f64,
    radius: f64,
    is_sensor: bool,
    pos_x: Option<f64>,
    pos_y: Option<f64>,
    pos_z: Option<f64>,
    rot_x: Option<f64>,
    rot_y: Option<f64>,
    rot_z: Option<f64>,
) -> f64 {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.add_cylinder_collider(
                handle,
                half_height as f32,
                radius as f32,
                is_sensor,
                pos_x.map(|v| v as f32),
                pos_y.map(|v| v as f32),
                pos_z.map(|v| v as f32),
                rot_x.map(|v| v as f32),
                rot_y.map(|v| v as f32),
                rot_z.map(|v| v as f32),
            ) as f64
        } else {
            0.0
        }
    }
}

#[neon::export]
fn add_trimesh_collider(
    handle: f64,
    vertices: Vec<f64>,
    indices: Vec<f64>,
    is_sensor: bool,
    trimesh_flags: f64,
    pos_x: Option<f64>,
    pos_y: Option<f64>,
    pos_z: Option<f64>,
    rot_x: Option<f64>,
    rot_y: Option<f64>,
    rot_z: Option<f64>,
) -> f64 {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.add_trimesh_collider(
                handle,
                vertices.into_iter().map(|v| v as f32).collect(),
                indices.into_iter().map(|v| v as u32).collect(),
                is_sensor,
                trimesh_flags as u32,
                pos_x.map(|v| v as f32),
                pos_y.map(|v| v as f32),
                pos_z.map(|v| v as f32),
                rot_x.map(|v| v as f32),
                rot_y.map(|v| v as f32),
                rot_z.map(|v| v as f32),
            ) as f64
        } else {
            0.0
        }
    }
}

#[neon::export]
fn add_convex_hull_collider(
    handle: f64,
    vertices: Vec<f64>,
    is_sensor: bool,
    pos_x: Option<f64>,
    pos_y: Option<f64>,
    pos_z: Option<f64>,
    rot_x: Option<f64>,
    rot_y: Option<f64>,
    rot_z: Option<f64>,
) -> f64 {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.add_convex_hull_collider(
                handle,
                vertices.into_iter().map(|v| v as f32).collect(),
                is_sensor,
                pos_x.map(|v| v as f32),
                pos_y.map(|v| v as f32),
                pos_z.map(|v| v as f32),
                rot_x.map(|v| v as f32),
                rot_y.map(|v| v as f32),
                rot_z.map(|v| v as f32),
            ) as f64
        } else {
            0.0
        }
    }
}

#[neon::export]
fn create_revolute_joint(
    body1_handle: f64,
    body2_handle: f64,
    anchor1_x: f64,
    anchor1_y: f64,
    anchor1_z: f64,
    anchor2_x: f64,
    anchor2_y: f64,
    anchor2_z: f64,
    axis_x: f64,
    axis_y: f64,
    axis_z: f64,
    wake_up: bool,
) -> f64 {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.create_revolute_joint(
                body1_handle,
                body2_handle,
                anchor1_x as f32,
                anchor1_y as f32,
                anchor1_z as f32,
                anchor2_x as f32,
                anchor2_y as f32,
                anchor2_z as f32,
                axis_x as f32,
                axis_y as f32,
                axis_z as f32,
                wake_up,
            )
        } else {
            0.0
        }
    }
}

#[neon::export]
fn create_fixed_joint(
    body1_handle: f64,
    body2_handle: f64,
    anchor1_x: f64,
    anchor1_y: f64,
    anchor1_z: f64,
    anchor2_x: f64,
    anchor2_y: f64,
    anchor2_z: f64,
    frame1_x: f64,
    frame1_y: f64,
    frame1_z: f64,
    frame1_w: f64,
    frame2_x: f64,
    frame2_y: f64,
    frame2_z: f64,
    frame2_w: f64,
    wake_up: bool,
) -> f64 {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.create_fixed_joint(
                body1_handle,
                body2_handle,
                anchor1_x as f32,
                anchor1_y as f32,
                anchor1_z as f32,
                anchor2_x as f32,
                anchor2_y as f32,
                anchor2_z as f32,
                frame1_x as f32,
                frame1_y as f32,
                frame1_z as f32,
                frame1_w as f32,
                frame2_x as f32,
                frame2_y as f32,
                frame2_z as f32,
                frame2_w as f32,
                wake_up,
            )
        } else {
            0.0
        }
    }
}

#[neon::export]
fn set_revolute_joint_limits(handle: f64, min_angle: f64, max_angle: f64) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.set_revolute_joint_limits(handle, min_angle as f32, max_angle as f32)
        } else {
            false
        }
    }
}

#[neon::export]
fn configure_revolute_joint_motor(
    handle: f64,
    target_pos: f64,
    target_vel: f64,
    stiffness: f64,
    damping: f64,
) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.configure_revolute_joint_motor(
                handle,
                target_pos as f32,
                target_vel as f32,
                stiffness as f32,
                damping as f32,
            )
        } else {
            false
        }
    }
}

#[neon::export]
fn get_joint_data(handle: f64) -> Vec<f64> {
    unsafe {
        if let Some(ref world) = WORLD {
            let (index, generation) = decode_handle_from_js(handle);
            let handle = ImpulseJointHandle::from_raw_parts(index, generation);
            if let Some(data) = world.impulse_joint_set.get(handle) {
                if let Some(revolute) = data.data.as_revolute() {
                    let (index1, generation1) = data.body1.into_raw_parts();
                    let (index2, generation2) = data.body2.into_raw_parts();
                    let body1_handle = encode_handle_for_js(index1, generation1);
                    let body2_handle = encode_handle_for_js(index2, generation2);
                    vec![
                        0.0,
                        body1_handle,
                        body2_handle,
                        revolute.local_anchor1().x as f64,
                        revolute.local_anchor1().y as f64,
                        revolute.local_anchor1().z as f64,
                        revolute.local_anchor2().x as f64,
                        revolute.local_anchor2().y as f64,
                        revolute.local_anchor2().z as f64,
                        revolute.data.local_axis1().x as f64,
                        revolute.data.local_axis1().y as f64,
                        revolute.data.local_axis1().z as f64,
                    ]
                } else if let Some(fixed) = data.data.as_fixed() {
                    let (index1, generation1) = data.body1.into_raw_parts();
                    let (index2, generation2) = data.body2.into_raw_parts();
                    let body1_handle = encode_handle_for_js(index1, generation1);
                    let body2_handle = encode_handle_for_js(index2, generation2);
                    vec![
                        1.0,
                        body1_handle,
                        body2_handle,
                        fixed.local_anchor1().x as f64,
                        fixed.local_anchor1().y as f64,
                        fixed.local_anchor1().z as f64,
                        fixed.local_anchor2().x as f64,
                        fixed.local_anchor2().y as f64,
                        fixed.local_anchor2().z as f64,
                        fixed.local_frame1().rotation.i as f64,
                        fixed.local_frame1().rotation.j as f64,
                        fixed.local_frame1().rotation.k as f64,
                        fixed.local_frame1().rotation.w as f64,
                        fixed.local_frame2().rotation.i as f64,
                        fixed.local_frame2().rotation.j as f64,
                        fixed.local_frame2().rotation.k as f64,
                        fixed.local_frame2().rotation.w as f64,
                    ]
                } else {
                    vec![]
                }
            } else {
                vec![]
            }
        } else {
            vec![]
        }
    }
}

#[neon::export]
fn intersection_pairs_with(handle: f64) -> Vec<f64> {
    unsafe {
        if let Some(ref world) = WORLD {
            let (index, generation) = decode_handle_from_js(handle);
            let handle = ColliderHandle::from_raw_parts(index, generation);
            let mut result = Vec::new();
            for contact_pair in world.narrow_phase.contact_pairs() {
                let handle1 = contact_pair.collider1;
                let handle2 = contact_pair.collider2;
                if handle1 == handle || handle2 == handle {
                    let other_handle = if handle1 == handle { handle2 } else { handle1 };
                    let (index, generation) = other_handle.into_raw_parts();
                    result.push(encode_handle_for_js(index, generation));
                }
            }
            for intersection_pair in world.narrow_phase.intersection_pairs() {
                let handle1 = intersection_pair.0;
                let handle2 = intersection_pair.1;
                if handle1 == handle || handle2 == handle {
                    let other_handle = if handle1 == handle { handle2 } else { handle1 };
                    let (index, generation) = other_handle.into_raw_parts();
                    result.push(encode_handle_for_js(index, generation));
                }
            }
            result
        } else {
            vec![]
        }
    }
}

#[neon::export]
fn set_body_translation(handle: f64, x: f64, y: f64, z: f64, wake_up: bool) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.set_body_translation(handle, x as f32, y as f32, z as f32, wake_up)
        } else {
            false
        }
    }
}

#[neon::export]
fn set_body_next_kinematic_translation(handle: f64, x: f64, y: f64, z: f64) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.set_body_next_kinematic_translation(handle, x as f32, y as f32, z as f32)
        } else {
            false
        }
    }
}

#[neon::export]
fn set_body_rotation(handle: f64, x: f64, y: f64, z: f64, w: f64, wake_up: bool) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.set_body_rotation(handle, x as f32, y as f32, z as f32, w as f32, wake_up)
        } else {
            false
        }
    }
}

#[neon::export]
fn set_body_next_kinematic_rotation(handle: f64, x: f64, y: f64, z: f64, w: f64) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.set_body_next_kinematic_rotation(handle, x as f32, y as f32, z as f32, w as f32)
        } else {
            false
        }
    }
}

#[neon::export]
fn set_body_velocity(handle: f64, vx: f64, vy: f64, vz: f64, wake_up: bool) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.set_body_velocity(handle, vx as f32, vy as f32, vz as f32, wake_up)
        } else {
            false
        }
    }
}

#[neon::export]
fn set_body_angular_velocity(handle: f64, wx: f64, wy: f64, wz: f64, wake_up: bool) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.set_body_angular_velocity(handle, wx as f32, wy as f32, wz as f32, wake_up)
        } else {
            false
        }
    }
}

#[neon::export]
fn apply_impulse(handle: f64, x: f64, y: f64, z: f64, wake_up: bool) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.apply_impulse(handle, x as f32, y as f32, z as f32, wake_up)
        } else {
            false
        }
    }
}

#[neon::export]
fn get_body_translation(handle: f64) -> Vec<f64> {
    unsafe {
        if let Some(ref world) = WORLD {
            if let Some((x, y, z)) = world.get_body_translation(handle) {
                vec![x as f64, y as f64, z as f64]
            } else {
                vec![]
            }
        } else {
            vec![]
        }
    }
}

#[neon::export]
fn get_body_rotation(handle: f64) -> Vec<f64> {
    unsafe {
        if let Some(ref world) = WORLD {
            if let Some((x, y, z, w)) = world.get_body_rotation(handle) {
                vec![x as f64, y as f64, z as f64, w as f64]
            } else {
                vec![]
            }
        } else {
            vec![]
        }
    }
}

#[neon::export]
fn get_body_velocity(handle: f64) -> Vec<f64> {
    unsafe {
        if let Some(ref world) = WORLD {
            if let Some((vx, vy, vz)) = world.get_body_velocity(handle) {
                vec![vx as f64, vy as f64, vz as f64]
            } else {
                vec![]
            }
        } else {
            vec![]
        }
    }
}

#[neon::export]
fn get_body_angular_velocity(handle: f64) -> Vec<f64> {
    unsafe {
        if let Some(ref world) = WORLD {
            if let Some((wx, wy, wz)) = world.get_body_angular_velocity(handle) {
                vec![wx as f64, wy as f64, wz as f64]
            } else {
                vec![]
            }
        } else {
            vec![]
        }
    }
}

#[neon::export]
fn is_body_enabled(handle: f64) -> bool {
    unsafe {
        if let Some(ref world) = WORLD {
            world.is_body_enabled(handle)
        } else {
            false
        }
    }
}

#[neon::export]
fn set_body_enabled(handle: f64, enabled: bool) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.set_body_enabled(handle, enabled)
        } else {
            false
        }
    }
}

#[neon::export]
fn set_body_enabled_translations(
    handle: f64,
    enabled_x: bool,
    enabled_y: bool,
    enabled_z: bool,
    wake_up: bool,
) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.set_body_enabled_translations(handle, enabled_x, enabled_y, enabled_z, wake_up)
        } else {
            false
        }
    }
}

#[neon::export]
fn set_body_enabled_rotations(
    handle: f64,
    enabled_x: bool,
    enabled_y: bool,
    enabled_z: bool,
    wake_up: bool,
) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.set_body_enabled_rotations(handle, enabled_x, enabled_y, enabled_z, wake_up)
        } else {
            false
        }
    }
}

#[neon::export]
fn body_sleep(handle: f64) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.body_sleep(handle)
        } else {
            false
        }
    }
}

#[neon::export]
fn is_body_sleeping(handle: f64) -> bool {
    unsafe {
        if let Some(ref world) = WORLD {
            world.is_body_sleeping(handle)
        } else {
            false
        }
    }
}

#[neon::export]
fn set_body_soft_ccd_prediction(handle: f64, precision: f64) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.set_body_soft_ccd_prediction(handle, precision as f32)
        } else {
            false
        }
    }
}

#[neon::export]
fn set_body_ccd_enabled(handle: f64, enabled: bool) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.set_body_ccd_enabled(handle, enabled)
        } else {
            false
        }
    }
}

#[neon::export]
fn set_body_additional_solver_iterations(handle: f64, iterations: f64) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.set_body_additional_solver_iterations(handle, iterations as usize)
        } else {
            false
        }
    }
}

#[neon::export]
fn get_body_mass(handle: f64) -> f64 {
    unsafe {
        if let Some(ref world) = WORLD {
            if let Some(mass) = world.get_body_mass(handle) {
                mass as f64
            } else {
                -1.0
            }
        } else {
            -1.0
        }
    }
}

#[neon::export]
fn set_body_angular_damping(handle: f64, damping: f64) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.set_body_angular_damping(handle, damping as f32)
        } else {
            false
        }
    }
}

#[neon::export]
fn set_body_linear_damping(handle: f64, damping: f64) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.set_body_linear_damping(handle, damping as f32)
        } else {
            false
        }
    }
}

#[neon::export]
fn get_body_num_colliders(handle: f64) -> f64 {
    unsafe {
        if let Some(ref world) = WORLD {
            world.get_body_num_colliders(handle) as f64
        } else {
            -1.0
        }
    }
}

#[neon::export]
fn get_body_collider(handle: f64, collider_index: f64) -> f64 {
    unsafe {
        if let Some(ref world) = WORLD {
            if let Some(handle) = world.get_body_collider(handle, collider_index as usize) {
                handle as f64
            } else {
                -1.0
            }
        } else {
            -1.0
        }
    }
}

#[neon::export]
fn set_collider_density(handle: f64, density: f64) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.set_collider_density(handle, density as f32)
        } else {
            false
        }
    }
}

#[neon::export]
fn set_collider_friction(handle: f64, friction: f64) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.set_collider_friction(handle, friction as f32)
        } else {
            false
        }
    }
}

#[neon::export]
fn set_collider_restitution(handle: f64, restitution: f64) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.set_collider_restitution(handle, restitution as f32)
        } else {
            false
        }
    }
}

#[neon::export]
fn set_collider_collision_groups(handle: f64, groups: f64) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.set_collider_collision_groups(handle, groups as u32)
        } else {
            false
        }
    }
}

#[neon::export]
fn set_collider_contact_skin(handle: f64, contact_skin: f64) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.set_collider_contact_skin(handle, contact_skin as f32)
        } else {
            false
        }
    }
}

#[neon::export]
fn set_collider_enabled(handle: f64, enabled: bool) -> bool {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.set_collider_enabled(handle, enabled)
        } else {
            false
        }
    }
}

#[neon::export]
fn is_collider_enabled(handle: f64) -> bool {
    unsafe {
        if let Some(ref world) = WORLD {
            world.is_collider_enabled(handle)
        } else {
            false
        }
    }
}

#[neon::export]
fn get_collider_shape_type(handle: f64) -> u32 {
    unsafe {
        if let Some(ref world) = WORLD {
            if let Some(shape_type) = world.get_collider_shape_type(handle) {
                shape_type as u32
            } else {
                0
            }
        } else {
            0
        }
    }
}

#[neon::export]
fn get_collider_parent(handle: f64) -> f64 {
    unsafe {
        if let Some(ref world) = WORLD {
            if let Some(parent_handle) = world.get_collider_parent(handle) {
                parent_handle as f64
            } else {
                -1.0
            }
        } else {
            -1.0
        }
    }
}

#[neon::export]
fn get_collider_translation(handle: f64) -> Vec<f64> {
    unsafe {
        if let Some(ref world) = WORLD {
            if let Some((x, y, z)) = world.get_collider_translation(handle) {
                vec![x as f64, y as f64, z as f64]
            } else {
                vec![]
            }
        } else {
            vec![]
        }
    }
}

#[neon::export]
fn get_collider_rotation(handle: f64) -> Vec<f64> {
    unsafe {
        if let Some(ref world) = WORLD {
            if let Some((x, y, z, w)) = world.get_collider_rotation(handle) {
                vec![x as f64, y as f64, z as f64, w as f64]
            } else {
                vec![]
            }
        } else {
            vec![]
        }
    }
}

#[neon::export]
fn get_collider_vertices(handle: f64) -> Vec<f64> {
    unsafe {
        if let Some(ref world) = WORLD {
            if let Some(vertices) = world.get_collider_vertices(handle) {
                vertices.into_iter().map(|v| v as f64).collect()
            } else {
                vec![]
            }
        } else {
            vec![]
        }
    }
}

#[neon::export]
fn get_collider_indices(handle: f64) -> Vec<f64> {
    unsafe {
        if let Some(ref world) = WORLD {
            if let Some(indices) = world.get_collider_indices(handle) {
                indices.into_iter().map(|i| i as f64).collect()
            } else {
                vec![]
            }
        } else {
            vec![]
        }
    }
}

#[neon::export]
fn get_collider_half_extents(handle: f64) -> Vec<f64> {
    unsafe {
        if let Some(ref world) = WORLD {
            if let Some((half_x, half_y, half_z)) = world.get_collider_half_extents(handle) {
                vec![half_x as f64, half_y as f64, half_z as f64]
            } else {
                vec![]
            }
        } else {
            vec![]
        }
    }
}

#[neon::export]
fn get_collider_half_height(handle: f64) -> f64 {
    unsafe {
        if let Some(ref world) = WORLD {
            if let Some(half_height) = world.get_collider_half_height(handle) {
                half_height as f64
            } else {
                -1.0
            }
        } else {
            -1.0
        }
    }
}

#[neon::export]
fn get_collider_radius(handle: f64) -> f64 {
    unsafe {
        if let Some(ref world) = WORLD {
            if let Some(radius) = world.get_collider_radius(handle) {
                radius as f64
            } else {
                -1.0
            }
        } else {
            -1.0
        }
    }
}

#[neon::export]
fn get_collider_flags(handle: f64) -> u32 {
    unsafe {
        if let Some(ref world) = WORLD {
            if let Some(flags) = world.get_collider_flags(handle) {
                flags
            } else {
                0
            }
        } else {
            0
        }
    }
}

#[neon::export]
fn step_simulation(dt: f64) {
    unsafe {
        if let Some(ref mut world) = WORLD {
            world.step(dt as f32);
        }
    }
}

fn encode_handle_for_js(index: u32, generation: u32) -> f64 {
    let combined = ((generation as u64) << 32) | (index as u64);
    f64::from_bits(combined)
}

fn decode_handle_from_js(encoded: f64) -> (u32, u32) {
    let combined = encoded.to_bits();
    let index = (combined & 0xFFFFFFFF) as u32;
    let generation = (combined >> 32) as u32;
    (index, generation)
}
