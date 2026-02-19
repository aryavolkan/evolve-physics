//! Godot 4 GDExtension: PhysicsServer2DExtension backed by Rapier2D
//!
//! This replaces Godot's built-in 2D physics with deterministic Rapier2D physics.

use godot::builtin::{Array, Callable, PackedVector2Array, Rid, Transform2D, Variant, Vector2};
use godot::classes::native::PhysicsServer2DExtensionMotionResult;
use godot::classes::{
    physics_server_2d, IPhysicsDirectBodyState2DExtension, IPhysicsDirectSpaceState2DExtension,
    IPhysicsServer2DExtension, Object, PhysicsDirectBodyState2D, PhysicsDirectBodyState2DExtension,
    PhysicsDirectSpaceState2D, PhysicsDirectSpaceState2DExtension, PhysicsServer2DExtension,
};
use godot::meta::RawPtr;
use godot::prelude::*;

use rapier2d::prelude as rapier;
use std::collections::HashMap;
use std::ffi::c_void;
use std::sync::{Arc, Mutex};

// ── Entry point ──────────────────────────────────────────────────────────────

struct EvolvePhysicsExtension;

#[gdextension]
unsafe impl ExtensionLibrary for EvolvePhysicsExtension {}

// ── Write-back queue for DirectBodyState ─────────────────────────────────────

/// Pending write-back from EvolveDirectBodyState to the physics server
#[derive(Debug)]
enum BodyWriteBack {
    CentralImpulse {
        body: Rid,
        impulse: Vector2,
    },
    Impulse {
        body: Rid,
        impulse: Vector2,
        position: Vector2,
    },
    TorqueImpulse {
        body: Rid,
        impulse: f32,
    },
    CentralForce {
        body: Rid,
        force: Vector2,
    },
    Force {
        body: Rid,
        force: Vector2,
        position: Vector2,
    },
    Torque {
        body: Rid,
        torque: f32,
    },
    LinearVelocity {
        body: Rid,
        velocity: Vector2,
    },
    AngularVelocity {
        body: Rid,
        velocity: f32,
    },
    Transform {
        body: Rid,
        transform: Transform2D,
    },
}

static WRITE_BACK_QUEUE: std::sync::LazyLock<Mutex<Vec<BodyWriteBack>>> =
    std::sync::LazyLock::new(|| Mutex::new(Vec::new()));

// ── Data structures ──────────────────────────────────────────────────────────

/// A full Rapier2D physics space (world)
/// Collects collision and contact force events from Rapier
struct CollisionEventCollector {
    collision_events: Arc<Mutex<Vec<rapier::CollisionEvent>>>,
    contact_force_events: Arc<Mutex<Vec<rapier::ContactForceEvent>>>,
}

impl CollisionEventCollector {
    fn new() -> Self {
        Self {
            collision_events: Arc::new(Mutex::new(Vec::new())),
            contact_force_events: Arc::new(Mutex::new(Vec::new())),
        }
    }

    fn clear(&self) {
        self.collision_events.lock().unwrap().clear();
        self.contact_force_events.lock().unwrap().clear();
    }
}

impl rapier::EventHandler for CollisionEventCollector {
    fn handle_collision_event(
        &self,
        _bodies: &rapier::RigidBodySet,
        _colliders: &rapier::ColliderSet,
        event: rapier::CollisionEvent,
        _contact_pair: Option<&rapier::ContactPair>,
    ) {
        self.collision_events.lock().unwrap().push(event);
    }

    fn handle_contact_force_event(
        &self,
        _dt: f32,
        _bodies: &rapier::RigidBodySet,
        _colliders: &rapier::ColliderSet,
        _contact_pair: &rapier::ContactPair,
        _total_force_magnitude: f32,
    ) {
        // Could collect force events here if needed
    }
}

/// A full Rapier2D physics space (world)
struct RapierSpace {
    gravity: rapier::Vector<f32>,
    integration_parameters: rapier::IntegrationParameters,
    physics_pipeline: rapier::PhysicsPipeline,
    island_manager: rapier::IslandManager,
    broad_phase: rapier::DefaultBroadPhase,
    narrow_phase: rapier::NarrowPhase,
    rigid_body_set: rapier::RigidBodySet,
    collider_set: rapier::ColliderSet,
    impulse_joint_set: rapier::ImpulseJointSet,
    multibody_joint_set: rapier::MultibodyJointSet,
    ccd_solver: rapier::CCDSolver,
    query_pipeline: rapier::QueryPipeline,
    event_collector: CollisionEventCollector,
    active: bool,
}

impl RapierSpace {
    fn new() -> Self {
        Self {
            gravity: rapier::Vector::new(0.0, 980.0), // Godot uses pixels, downward
            integration_parameters: rapier::IntegrationParameters {
                dt: 1.0 / 60.0,
                ..Default::default()
            },
            physics_pipeline: rapier::PhysicsPipeline::new(),
            island_manager: rapier::IslandManager::new(),
            broad_phase: rapier::DefaultBroadPhase::new(),
            narrow_phase: rapier::NarrowPhase::new(),
            rigid_body_set: rapier::RigidBodySet::new(),
            collider_set: rapier::ColliderSet::new(),
            impulse_joint_set: rapier::ImpulseJointSet::new(),
            multibody_joint_set: rapier::MultibodyJointSet::new(),
            ccd_solver: rapier::CCDSolver::new(),
            query_pipeline: rapier::QueryPipeline::new(),
            event_collector: CollisionEventCollector::new(),
            active: false,
        }
    }

    fn step(&mut self, dt: f32) {
        self.integration_parameters.dt = dt;
        self.event_collector.clear();
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
            None,
            &(),
            &self.event_collector,
        );
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum ShapeType {
    WorldBoundary,
    SeparationRay,
    Segment,
    Circle,
    Rectangle,
    Capsule,
    ConvexPolygon,
    ConcavePolygon,
}

struct ShapeData {
    shape_type: ShapeType,
    data: Variant,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum BodyMode {
    Static,
    Kinematic,
    Rigid,
    RigidLinear,
}

struct BodyData {
    space_rid: Option<Rid>,
    rb_handle: Option<rapier::RigidBodyHandle>,
    shapes: Vec<BodyShapeEntry>,
    mode: BodyMode,
    collision_layer: u32,
    collision_mask: u32,
    collision_priority: f32,
    object_instance_id: u64,
    canvas_instance_id: u64,
    constant_force: Vector2,
    constant_torque: f32,
    max_contacts_reported: i32,
    contacts_depth_threshold: f32,
    omit_force_integration: bool,
    pickable: bool,
    state_sync_callback: Option<Callable>,
    force_integration_callback: Option<(Callable, Variant)>,
}

struct BodyShapeEntry {
    shape_rid: Rid,
    transform: Transform2D,
    disabled: bool,
    one_way: bool,
    one_way_margin: f32,
    collider_handle: Option<rapier::ColliderHandle>,
}

struct AreaData {
    space_rid: Option<Rid>,
    shapes: Vec<AreaShapeEntry>,
    transform: Transform2D,
    collision_layer: u32,
    collision_mask: u32,
    monitorable: bool,
    pickable: bool,
    object_instance_id: u64,
    canvas_instance_id: u64,
    monitor_callback: Option<Callable>,
    area_monitor_callback: Option<Callable>,
}

struct AreaShapeEntry {
    shape_rid: Rid,
    transform: Transform2D,
    disabled: bool,
}

struct JointData {
    // Stub for now
}

// ── Direct body state ────────────────────────────────────────────────────────

/// Snapshot-based direct body state. Populated when body_get_direct_state() is called.
/// Write-back operations (apply_impulse, set_velocity, etc.) are cached and applied
/// to the server's body on the next step. For now, writes are no-ops since Godot
/// typically uses this read-only during _integrate_forces.
#[derive(GodotClass)]
#[class(base=PhysicsDirectBodyState2DExtension, tool)]
pub struct EvolveDirectBodyState {
    base: Base<PhysicsDirectBodyState2DExtension>,
    body_rid: Rid,
    space_rid: Rid,
    cached_transform: Transform2D,
    cached_linear_velocity: Vector2,
    cached_angular_velocity: f32,
    cached_inverse_mass: f32,
    cached_inverse_inertia: f32,
    cached_total_gravity: Vector2,
    cached_constant_force: Vector2,
    cached_constant_torque: f32,
    sleeping: bool,
    step: f32,
}

#[godot_api]
impl IPhysicsDirectBodyState2DExtension for EvolveDirectBodyState {
    fn init(base: Base<PhysicsDirectBodyState2DExtension>) -> Self {
        Self {
            base,
            body_rid: Rid::Invalid,
            space_rid: Rid::Invalid,
            cached_transform: Transform2D::IDENTITY,
            cached_linear_velocity: Vector2::ZERO,
            cached_angular_velocity: 0.0,
            cached_inverse_mass: 0.0,
            cached_inverse_inertia: 0.0,
            cached_total_gravity: Vector2::ZERO,
            cached_constant_force: Vector2::ZERO,
            cached_constant_torque: 0.0,
            sleeping: false,
            step: 1.0 / 60.0,
        }
    }

    fn get_total_gravity(&self) -> Vector2 {
        self.cached_total_gravity
    }
    fn get_total_linear_damp(&self) -> f32 {
        0.0
    }
    fn get_total_angular_damp(&self) -> f32 {
        0.0
    }
    fn get_center_of_mass(&self) -> Vector2 {
        self.cached_transform.origin
    }
    fn get_center_of_mass_local(&self) -> Vector2 {
        Vector2::ZERO
    }
    fn get_inverse_mass(&self) -> f32 {
        self.cached_inverse_mass
    }
    fn get_inverse_inertia(&self) -> f32 {
        self.cached_inverse_inertia
    }
    fn set_linear_velocity(&mut self, velocity: Vector2) {
        self.cached_linear_velocity = velocity;
        WRITE_BACK_QUEUE
            .lock()
            .unwrap()
            .push(BodyWriteBack::LinearVelocity {
                body: self.body_rid,
                velocity,
            });
    }
    fn get_linear_velocity(&self) -> Vector2 {
        self.cached_linear_velocity
    }
    fn set_angular_velocity(&mut self, velocity: f32) {
        self.cached_angular_velocity = velocity;
        WRITE_BACK_QUEUE
            .lock()
            .unwrap()
            .push(BodyWriteBack::AngularVelocity {
                body: self.body_rid,
                velocity,
            });
    }
    fn get_angular_velocity(&self) -> f32 {
        self.cached_angular_velocity
    }
    fn set_transform(&mut self, transform: Transform2D) {
        self.cached_transform = transform;
        WRITE_BACK_QUEUE
            .lock()
            .unwrap()
            .push(BodyWriteBack::Transform {
                body: self.body_rid,
                transform,
            });
    }
    fn get_transform(&self) -> Transform2D {
        self.cached_transform
    }
    fn get_velocity_at_local_position(&self, local_position: Vector2) -> Vector2 {
        // v = linear_vel + angular_vel × r (2D cross product)
        let perp = Vector2::new(-local_position.y, local_position.x) * self.cached_angular_velocity;
        self.cached_linear_velocity + perp
    }
    fn apply_central_impulse(&mut self, impulse: Vector2) {
        WRITE_BACK_QUEUE
            .lock()
            .unwrap()
            .push(BodyWriteBack::CentralImpulse {
                body: self.body_rid,
                impulse,
            });
    }
    fn apply_impulse(&mut self, impulse: Vector2, position: Vector2) {
        WRITE_BACK_QUEUE
            .lock()
            .unwrap()
            .push(BodyWriteBack::Impulse {
                body: self.body_rid,
                impulse,
                position,
            });
    }
    fn apply_torque_impulse(&mut self, impulse: f32) {
        WRITE_BACK_QUEUE
            .lock()
            .unwrap()
            .push(BodyWriteBack::TorqueImpulse {
                body: self.body_rid,
                impulse,
            });
    }
    fn apply_central_force(&mut self, force: Vector2) {
        WRITE_BACK_QUEUE
            .lock()
            .unwrap()
            .push(BodyWriteBack::CentralForce {
                body: self.body_rid,
                force,
            });
    }
    fn apply_force(&mut self, force: Vector2, position: Vector2) {
        WRITE_BACK_QUEUE.lock().unwrap().push(BodyWriteBack::Force {
            body: self.body_rid,
            force,
            position,
        });
    }
    fn apply_torque(&mut self, torque: f32) {
        WRITE_BACK_QUEUE
            .lock()
            .unwrap()
            .push(BodyWriteBack::Torque {
                body: self.body_rid,
                torque,
            });
    }
    fn add_constant_central_force(&mut self, force: Vector2) {
        self.cached_constant_force += force;
    }
    fn add_constant_force(&mut self, force: Vector2, _position: Vector2) {
        self.cached_constant_force += force;
    }
    fn add_constant_torque(&mut self, torque: f32) {
        self.cached_constant_torque += torque;
    }
    fn set_constant_force(&mut self, force: Vector2) {
        self.cached_constant_force = force;
    }
    fn get_constant_force(&self) -> Vector2 {
        self.cached_constant_force
    }
    fn set_constant_torque(&mut self, torque: f32) {
        self.cached_constant_torque = torque;
    }
    fn get_constant_torque(&self) -> f32 {
        self.cached_constant_torque
    }
    fn set_sleep_state(&mut self, enabled: bool) {
        self.sleeping = enabled;
    }
    fn is_sleeping(&self) -> bool {
        self.sleeping
    }
    fn get_contact_count(&self) -> i32 {
        0
    }
    fn get_contact_local_position(&self, _contact_idx: i32) -> Vector2 {
        Vector2::ZERO
    }
    fn get_contact_local_normal(&self, _contact_idx: i32) -> Vector2 {
        Vector2::ZERO
    }
    fn get_contact_local_shape(&self, _contact_idx: i32) -> i32 {
        0
    }
    fn get_contact_local_velocity_at_position(&self, _contact_idx: i32) -> Vector2 {
        Vector2::ZERO
    }
    fn get_contact_collider(&self, _contact_idx: i32) -> Rid {
        Rid::Invalid
    }
    fn get_contact_collider_position(&self, _contact_idx: i32) -> Vector2 {
        Vector2::ZERO
    }
    fn get_contact_collider_id(&self, _contact_idx: i32) -> u64 {
        0
    }
    fn get_contact_collider_object(&self, _contact_idx: i32) -> Option<Gd<Object>> {
        None
    }
    fn get_contact_collider_shape(&self, _contact_idx: i32) -> i32 {
        0
    }
    fn get_contact_collider_velocity_at_position(&self, _contact_idx: i32) -> Vector2 {
        Vector2::ZERO
    }
    fn get_contact_impulse(&self, _contact_idx: i32) -> Vector2 {
        Vector2::ZERO
    }
    fn get_step(&self) -> f32 {
        self.step
    }
    fn integrate_forces(&mut self) { /* Rapier handles integration */
    }
    fn get_space_state(&mut self) -> Option<Gd<PhysicsDirectSpaceState2D>> {
        None
    }
    fn set_collision_layer(&mut self, _layer: u32) {}
    fn get_collision_layer(&self) -> u32 {
        0
    }
    fn set_collision_mask(&mut self, _mask: u32) {}
    fn get_collision_mask(&self) -> u32 {
        0
    }
}

// ── Direct space state ───────────────────────────────────────────────────────

#[derive(GodotClass)]
#[class(base=PhysicsDirectSpaceState2DExtension, tool)]
pub struct EvolveDirectSpaceState {
    base: Base<PhysicsDirectSpaceState2DExtension>,
    space_rid: Rid,
}

#[godot_api]
impl IPhysicsDirectSpaceState2DExtension for EvolveDirectSpaceState {
    fn init(base: Base<PhysicsDirectSpaceState2DExtension>) -> Self {
        Self {
            base,
            space_rid: Rid::Invalid,
        }
    }

    unsafe fn intersect_ray_rawptr(
        &mut self,
        _from: Vector2,
        _to: Vector2,
        _collision_mask: u32,
        _collide_with_bodies: bool,
        _collide_with_areas: bool,
        _hit_from_inside: bool,
        _result: RawPtr<*mut godot::classes::native::PhysicsServer2DExtensionRayResult>,
    ) -> bool {
        false
    }

    unsafe fn intersect_point_rawptr(
        &mut self,
        _position: Vector2,
        _canvas_instance_id: u64,
        _collision_mask: u32,
        _collide_with_bodies: bool,
        _collide_with_areas: bool,
        _results: RawPtr<*mut godot::classes::native::PhysicsServer2DExtensionShapeResult>,
        _max_results: i32,
    ) -> i32 {
        0
    }

    unsafe fn intersect_shape_rawptr(
        &mut self,
        _shape_rid: Rid,
        _transform: Transform2D,
        _motion: Vector2,
        _margin: f32,
        _collision_mask: u32,
        _collide_with_bodies: bool,
        _collide_with_areas: bool,
        _results: RawPtr<*mut godot::classes::native::PhysicsServer2DExtensionShapeResult>,
        _max_results: i32,
    ) -> i32 {
        0
    }

    unsafe fn cast_motion_rawptr(
        &mut self,
        _shape_rid: Rid,
        _transform: Transform2D,
        _motion: Vector2,
        _margin: f32,
        _collision_mask: u32,
        _collide_with_bodies: bool,
        _collide_with_areas: bool,
        _closest_safe: RawPtr<*mut f64>,
        _closest_unsafe: RawPtr<*mut f64>,
    ) -> bool {
        false
    }

    unsafe fn collide_shape_rawptr(
        &mut self,
        _shape_rid: Rid,
        _transform: Transform2D,
        _motion: Vector2,
        _margin: f32,
        _collision_mask: u32,
        _collide_with_bodies: bool,
        _collide_with_areas: bool,
        _results: RawPtr<*mut c_void>,
        _max_results: i32,
        _result_count: RawPtr<*mut i32>,
    ) -> bool {
        false
    }

    unsafe fn rest_info_rawptr(
        &mut self,
        _shape_rid: Rid,
        _transform: Transform2D,
        _motion: Vector2,
        _margin: f32,
        _collision_mask: u32,
        _collide_with_bodies: bool,
        _collide_with_areas: bool,
        _result: RawPtr<*mut godot::classes::native::PhysicsServer2DExtensionShapeRestInfo>,
    ) -> bool {
        false
    }
}

// ── Main server ──────────────────────────────────────────────────────────────

#[derive(GodotClass)]
#[class(base=PhysicsServer2DExtension, tool)]
pub struct EvolvePhysicsServer {
    base: Base<PhysicsServer2DExtension>,
    spaces: HashMap<Rid, RapierSpace>,
    bodies: HashMap<Rid, BodyData>,
    shapes: HashMap<Rid, ShapeData>,
    areas: HashMap<Rid, AreaData>,
    joints: HashMap<Rid, JointData>,
    next_rid_id: u64,
    active: bool,
}

impl EvolvePhysicsServer {
    fn alloc_rid(&mut self) -> Rid {
        self.next_rid_id += 1;
        Rid::new(self.next_rid_id)
    }

    /// Build a Rapier collider from a shape RID's stored data.
    fn build_collider_for_shape(&self, shape_rid: Rid) -> Option<rapier::Collider> {
        let shape = self.shapes.get(&shape_rid)?;
        match shape.shape_type {
            ShapeType::Circle => {
                let radius = shape.data.try_to::<f32>().unwrap_or(10.0);
                Some(rapier::ColliderBuilder::ball(radius).build())
            }
            ShapeType::Rectangle => {
                // Rectangle data is a Vector2 of half-extents
                let half = shape
                    .data
                    .try_to::<Vector2>()
                    .unwrap_or(Vector2::new(10.0, 10.0));
                Some(rapier::ColliderBuilder::cuboid(half.x, half.y).build())
            }
            ShapeType::Capsule => {
                // Godot capsule data: [height, radius] array
                let arr = shape.data.try_to::<Array<Variant>>().ok();
                let (height, radius) = if let Some(ref a) = arr {
                    let h: f32 = if a.len() > 0 {
                        a.at(0).try_to::<f32>().unwrap_or(20.0)
                    } else {
                        20.0
                    };
                    let r: f32 = if a.len() > 1 {
                        a.at(1).try_to::<f32>().unwrap_or(10.0)
                    } else {
                        10.0
                    };
                    (h, r)
                } else {
                    (20.0, 10.0)
                };
                Some(rapier::ColliderBuilder::capsule_y(height / 2.0, radius).build())
            }
            ShapeType::Segment => {
                // Segment data is a Rect2: position = point A, size = point B
                let rect = shape.data.try_to::<godot::builtin::Rect2>().ok();
                if let Some(r) = rect {
                    let a = rapier::Point::new(r.position.x, r.position.y);
                    let b = rapier::Point::new(r.position.x + r.size.x, r.position.y + r.size.y);
                    Some(rapier::ColliderBuilder::segment(a, b).build())
                } else {
                    None
                }
            }
            ShapeType::ConvexPolygon => {
                // ConvexPolygon data is a PackedVector2Array of points
                let points = shape.data.try_to::<PackedVector2Array>().ok();
                if let Some(pts) = points {
                    let rapier_pts: Vec<rapier::Point<f32>> = pts
                        .as_slice()
                        .iter()
                        .map(|p| rapier::Point::new(p.x, p.y))
                        .collect();
                    if rapier_pts.len() >= 3 {
                        rapier::ColliderBuilder::convex_hull(&rapier_pts).map(|b| b.build())
                    } else {
                        None
                    }
                } else {
                    None
                }
            }
            ShapeType::ConcavePolygon => {
                // ConcavePolygon data is a PackedVector2Array of segment pairs (p0,p1,p2,p3,...)
                let points = shape.data.try_to::<PackedVector2Array>().ok();
                if let Some(pts) = points {
                    let slice = pts.as_slice();
                    if slice.len() >= 4 && slice.len() % 2 == 0 {
                        let vertices: Vec<rapier::Point<f32>> =
                            slice.iter().map(|p| rapier::Point::new(p.x, p.y)).collect();
                        let indices: Vec<[u32; 2]> = (0..vertices.len() / 2)
                            .map(|i| [i as u32 * 2, i as u32 * 2 + 1])
                            .collect();
                        Some(rapier::ColliderBuilder::polyline(vertices, Some(indices)).build())
                    } else {
                        None
                    }
                } else {
                    None
                }
            }
            _ => None, // WorldBoundary, SeparationRay — no Rapier equivalent
        }
    }

    /// Sync all shape colliders for a body that's in a space.
    fn sync_body_colliders(&mut self, body_rid: Rid) {
        // Gather what we need without holding borrows
        let info = {
            let bd = match self.bodies.get(&body_rid) {
                Some(bd) => bd,
                None => return,
            };
            let (space_rid, rb_handle) = match (bd.space_rid, bd.rb_handle) {
                (Some(s), Some(h)) => (s, h),
                _ => return,
            };
            let shape_info: Vec<(usize, Rid, bool)> = bd
                .shapes
                .iter()
                .enumerate()
                .map(|(i, s)| (i, s.shape_rid, s.disabled))
                .filter(|(_, _, disabled)| !disabled)
                .filter(|(i, _, _)| bd.shapes[*i].collider_handle.is_none())
                .collect();
            (space_rid, rb_handle, shape_info)
        };

        let (space_rid, rb_handle, shape_info) = info;

        for (idx, shape_rid, _) in shape_info {
            if let Some(collider) = self.build_collider_for_shape(shape_rid) {
                if let Some(space_data) = self.spaces.get_mut(&space_rid) {
                    let ch = space_data.collider_set.insert_with_parent(
                        collider,
                        rb_handle,
                        &mut space_data.rigid_body_set,
                    );
                    if let Some(bd) = self.bodies.get_mut(&body_rid) {
                        if let Some(entry) = bd.shapes.get_mut(idx) {
                            entry.collider_handle = Some(ch);
                        }
                    }
                }
            }
        }
    }

    fn shape_type_to_godot(&self, st: ShapeType) -> physics_server_2d::ShapeType {
        match st {
            ShapeType::WorldBoundary => physics_server_2d::ShapeType::WORLD_BOUNDARY,
            ShapeType::SeparationRay => physics_server_2d::ShapeType::SEPARATION_RAY,
            ShapeType::Segment => physics_server_2d::ShapeType::SEGMENT,
            ShapeType::Circle => physics_server_2d::ShapeType::CIRCLE,
            ShapeType::Rectangle => physics_server_2d::ShapeType::RECTANGLE,
            ShapeType::Capsule => physics_server_2d::ShapeType::CAPSULE,
            ShapeType::ConvexPolygon => physics_server_2d::ShapeType::CONVEX_POLYGON,
            ShapeType::ConcavePolygon => physics_server_2d::ShapeType::CONCAVE_POLYGON,
        }
    }
}

#[godot_api]
impl IPhysicsServer2DExtension for EvolvePhysicsServer {
    fn init(base: Base<PhysicsServer2DExtension>) -> Self {
        godot_print!("[EvolvePhysics] Rapier2D physics server initializing");
        Self {
            base,
            spaces: HashMap::new(),
            bodies: HashMap::new(),
            shapes: HashMap::new(),
            areas: HashMap::new(),
            joints: HashMap::new(),
            next_rid_id: 0,
            active: false,
        }
    }

    // ── Shape creation ───────────────────────────────────────────────────

    fn world_boundary_shape_create(&mut self) -> Rid {
        let rid = self.alloc_rid();
        self.shapes.insert(
            rid,
            ShapeData {
                shape_type: ShapeType::WorldBoundary,
                data: Variant::nil(),
            },
        );
        rid
    }

    fn separation_ray_shape_create(&mut self) -> Rid {
        let rid = self.alloc_rid();
        self.shapes.insert(
            rid,
            ShapeData {
                shape_type: ShapeType::SeparationRay,
                data: Variant::nil(),
            },
        );
        rid
    }

    fn segment_shape_create(&mut self) -> Rid {
        let rid = self.alloc_rid();
        self.shapes.insert(
            rid,
            ShapeData {
                shape_type: ShapeType::Segment,
                data: Variant::nil(),
            },
        );
        rid
    }

    fn circle_shape_create(&mut self) -> Rid {
        let rid = self.alloc_rid();
        self.shapes.insert(
            rid,
            ShapeData {
                shape_type: ShapeType::Circle,
                data: Variant::nil(),
            },
        );
        rid
    }

    fn rectangle_shape_create(&mut self) -> Rid {
        let rid = self.alloc_rid();
        self.shapes.insert(
            rid,
            ShapeData {
                shape_type: ShapeType::Rectangle,
                data: Variant::nil(),
            },
        );
        rid
    }

    fn capsule_shape_create(&mut self) -> Rid {
        let rid = self.alloc_rid();
        self.shapes.insert(
            rid,
            ShapeData {
                shape_type: ShapeType::Capsule,
                data: Variant::nil(),
            },
        );
        rid
    }

    fn convex_polygon_shape_create(&mut self) -> Rid {
        let rid = self.alloc_rid();
        self.shapes.insert(
            rid,
            ShapeData {
                shape_type: ShapeType::ConvexPolygon,
                data: Variant::nil(),
            },
        );
        rid
    }

    fn concave_polygon_shape_create(&mut self) -> Rid {
        let rid = self.alloc_rid();
        self.shapes.insert(
            rid,
            ShapeData {
                shape_type: ShapeType::ConcavePolygon,
                data: Variant::nil(),
            },
        );
        rid
    }

    fn shape_set_data(&mut self, shape: Rid, data: Variant) {
        if let Some(s) = self.shapes.get_mut(&shape) {
            s.data = data;
        }
    }

    fn shape_set_custom_solver_bias(&mut self, _shape: Rid, _bias: f32) {
        // Stub — Rapier doesn't have this concept
    }

    fn shape_get_type(&self, shape: Rid) -> physics_server_2d::ShapeType {
        self.shapes
            .get(&shape)
            .map(|s| self.shape_type_to_godot(s.shape_type))
            .unwrap_or(physics_server_2d::ShapeType::CUSTOM)
    }

    fn shape_get_data(&self, shape: Rid) -> Variant {
        self.shapes
            .get(&shape)
            .map(|s| s.data.clone())
            .unwrap_or(Variant::nil())
    }

    fn shape_get_custom_solver_bias(&self, _shape: Rid) -> f32 {
        0.0
    }

    unsafe fn shape_collide_rawptr(
        &mut self,
        _shape_a: Rid,
        _xform_a: Transform2D,
        _motion_a: Vector2,
        _shape_b: Rid,
        _xform_b: Transform2D,
        _motion_b: Vector2,
        _results: RawPtr<*mut c_void>,
        _result_max: i32,
        _result_count: RawPtr<*mut i32>,
    ) -> bool {
        false // Stub
    }

    // ── Space ────────────────────────────────────────────────────────────

    fn space_create(&mut self) -> Rid {
        let rid = self.alloc_rid();
        self.spaces.insert(rid, RapierSpace::new());
        rid
    }

    fn space_set_active(&mut self, space: Rid, active: bool) {
        if let Some(s) = self.spaces.get_mut(&space) {
            s.active = active;
        }
    }

    fn space_is_active(&self, space: Rid) -> bool {
        self.spaces.get(&space).map(|s| s.active).unwrap_or(false)
    }

    fn space_set_param(
        &mut self,
        _space: Rid,
        _param: physics_server_2d::SpaceParameter,
        _value: f32,
    ) {
        // Stub
    }

    fn space_get_param(&self, _space: Rid, _param: physics_server_2d::SpaceParameter) -> f32 {
        0.0
    }

    fn space_get_direct_state(&mut self, space: Rid) -> Option<Gd<PhysicsDirectSpaceState2D>> {
        if !self.spaces.contains_key(&space) {
            return None;
        }
        let mut state = EvolveDirectSpaceState::new_alloc();
        state.bind_mut().space_rid = space;
        Some(state.upcast())
    }

    fn space_set_debug_contacts(&mut self, _space: Rid, _max_contacts: i32) {}

    fn space_get_contacts(&self, _space: Rid) -> PackedVector2Array {
        PackedVector2Array::new()
    }

    fn space_get_contact_count(&self, _space: Rid) -> i32 {
        0
    }

    // ── Area ─────────────────────────────────────────────────────────────

    fn area_create(&mut self) -> Rid {
        let rid = self.alloc_rid();
        self.areas.insert(
            rid,
            AreaData {
                space_rid: None,
                shapes: Vec::new(),
                transform: Transform2D::IDENTITY,
                collision_layer: 1,
                collision_mask: 1,
                monitorable: false,
                pickable: false,
                object_instance_id: 0,
                canvas_instance_id: 0,
                monitor_callback: None,
                area_monitor_callback: None,
            },
        );
        rid
    }

    fn area_set_space(&mut self, area: Rid, space: Rid) {
        if let Some(a) = self.areas.get_mut(&area) {
            a.space_rid = if space.is_valid() { Some(space) } else { None };
        }
    }

    fn area_get_space(&self, area: Rid) -> Rid {
        self.areas
            .get(&area)
            .and_then(|a| a.space_rid)
            .unwrap_or(Rid::Invalid)
    }

    fn area_add_shape(&mut self, area: Rid, shape: Rid, transform: Transform2D, disabled: bool) {
        if let Some(a) = self.areas.get_mut(&area) {
            a.shapes.push(AreaShapeEntry {
                shape_rid: shape,
                transform,
                disabled,
            });
        }
    }

    fn area_set_shape(&mut self, area: Rid, shape_idx: i32, shape: Rid) {
        if let Some(a) = self.areas.get_mut(&area) {
            if let Some(entry) = a.shapes.get_mut(shape_idx as usize) {
                entry.shape_rid = shape;
            }
        }
    }

    fn area_set_shape_transform(&mut self, area: Rid, shape_idx: i32, transform: Transform2D) {
        if let Some(a) = self.areas.get_mut(&area) {
            if let Some(entry) = a.shapes.get_mut(shape_idx as usize) {
                entry.transform = transform;
            }
        }
    }

    fn area_set_shape_disabled(&mut self, area: Rid, shape_idx: i32, disabled: bool) {
        if let Some(a) = self.areas.get_mut(&area) {
            if let Some(entry) = a.shapes.get_mut(shape_idx as usize) {
                entry.disabled = disabled;
            }
        }
    }

    fn area_get_shape_count(&self, area: Rid) -> i32 {
        self.areas
            .get(&area)
            .map(|a| a.shapes.len() as i32)
            .unwrap_or(0)
    }

    fn area_get_shape(&self, area: Rid, shape_idx: i32) -> Rid {
        self.areas
            .get(&area)
            .and_then(|a| a.shapes.get(shape_idx as usize))
            .map(|e| e.shape_rid)
            .unwrap_or(Rid::Invalid)
    }

    fn area_get_shape_transform(&self, area: Rid, shape_idx: i32) -> Transform2D {
        self.areas
            .get(&area)
            .and_then(|a| a.shapes.get(shape_idx as usize))
            .map(|e| e.transform)
            .unwrap_or(Transform2D::IDENTITY)
    }

    fn area_remove_shape(&mut self, area: Rid, shape_idx: i32) {
        if let Some(a) = self.areas.get_mut(&area) {
            let idx = shape_idx as usize;
            if idx < a.shapes.len() {
                a.shapes.remove(idx);
            }
        }
    }

    fn area_clear_shapes(&mut self, area: Rid) {
        if let Some(a) = self.areas.get_mut(&area) {
            a.shapes.clear();
        }
    }

    fn area_attach_object_instance_id(&mut self, area: Rid, id: u64) {
        if let Some(a) = self.areas.get_mut(&area) {
            a.object_instance_id = id;
        }
    }

    fn area_get_object_instance_id(&self, area: Rid) -> u64 {
        self.areas
            .get(&area)
            .map(|a| a.object_instance_id)
            .unwrap_or(0)
    }

    fn area_attach_canvas_instance_id(&mut self, area: Rid, id: u64) {
        if let Some(a) = self.areas.get_mut(&area) {
            a.canvas_instance_id = id;
        }
    }

    fn area_get_canvas_instance_id(&self, area: Rid) -> u64 {
        self.areas
            .get(&area)
            .map(|a| a.canvas_instance_id)
            .unwrap_or(0)
    }

    fn area_set_param(
        &mut self,
        _area: Rid,
        _param: physics_server_2d::AreaParameter,
        _value: Variant,
    ) {
    }

    fn area_set_transform(&mut self, area: Rid, transform: Transform2D) {
        if let Some(a) = self.areas.get_mut(&area) {
            a.transform = transform;
        }
    }

    fn area_get_param(&self, _area: Rid, _param: physics_server_2d::AreaParameter) -> Variant {
        Variant::nil()
    }

    fn area_get_transform(&self, area: Rid) -> Transform2D {
        self.areas
            .get(&area)
            .map(|a| a.transform)
            .unwrap_or(Transform2D::IDENTITY)
    }

    fn area_set_collision_layer(&mut self, area: Rid, layer: u32) {
        if let Some(a) = self.areas.get_mut(&area) {
            a.collision_layer = layer;
        }
    }

    fn area_get_collision_layer(&self, area: Rid) -> u32 {
        self.areas
            .get(&area)
            .map(|a| a.collision_layer)
            .unwrap_or(0)
    }

    fn area_set_collision_mask(&mut self, area: Rid, mask: u32) {
        if let Some(a) = self.areas.get_mut(&area) {
            a.collision_mask = mask;
        }
    }

    fn area_get_collision_mask(&self, area: Rid) -> u32 {
        self.areas.get(&area).map(|a| a.collision_mask).unwrap_or(0)
    }

    fn area_set_monitorable(&mut self, area: Rid, monitorable: bool) {
        if let Some(a) = self.areas.get_mut(&area) {
            a.monitorable = monitorable;
        }
    }

    fn area_set_pickable(&mut self, area: Rid, pickable: bool) {
        if let Some(a) = self.areas.get_mut(&area) {
            a.pickable = pickable;
        }
    }

    fn area_set_monitor_callback(&mut self, area: Rid, callback: Callable) {
        if let Some(a) = self.areas.get_mut(&area) {
            a.monitor_callback = if callback.is_valid() {
                Some(callback)
            } else {
                None
            };
        }
    }

    fn area_set_area_monitor_callback(&mut self, area: Rid, callback: Callable) {
        if let Some(a) = self.areas.get_mut(&area) {
            a.area_monitor_callback = if callback.is_valid() {
                Some(callback)
            } else {
                None
            };
        }
    }

    // ── Body ─────────────────────────────────────────────────────────────

    fn body_create(&mut self) -> Rid {
        let rid = self.alloc_rid();
        self.bodies.insert(
            rid,
            BodyData {
                space_rid: None,
                rb_handle: None,
                shapes: Vec::new(),
                mode: BodyMode::Rigid,
                collision_layer: 1,
                collision_mask: 1,
                collision_priority: 1.0,
                object_instance_id: 0,
                canvas_instance_id: 0,
                constant_force: Vector2::ZERO,
                constant_torque: 0.0,
                max_contacts_reported: 0,
                contacts_depth_threshold: 0.0,
                omit_force_integration: false,
                pickable: false,
                state_sync_callback: None,
                force_integration_callback: None,
            },
        );
        rid
    }

    fn body_set_space(&mut self, body: Rid, space: Rid) {
        // Remove from old space if needed
        if let Some(bd) = self.bodies.get(&body) {
            if let (Some(old_space_rid), Some(rb_handle)) = (bd.space_rid, bd.rb_handle) {
                if let Some(space_data) = self.spaces.get_mut(&old_space_rid) {
                    // Remove colliders first
                    let colliders: Vec<_> = space_data
                        .rigid_body_set
                        .get(rb_handle)
                        .map(|rb| rb.colliders().to_vec())
                        .unwrap_or_default();
                    for ch in colliders {
                        space_data.collider_set.remove(
                            ch,
                            &mut space_data.island_manager,
                            &mut space_data.rigid_body_set,
                            true,
                        );
                    }
                    space_data.rigid_body_set.remove(
                        rb_handle,
                        &mut space_data.island_manager,
                        &mut space_data.collider_set,
                        &mut space_data.impulse_joint_set,
                        &mut space_data.multibody_joint_set,
                        true,
                    );
                }
            }
        }

        if let Some(bd) = self.bodies.get_mut(&body) {
            bd.rb_handle = None;
            for shape_entry in &mut bd.shapes {
                shape_entry.collider_handle = None;
            }

            if space.is_valid() && self.spaces.contains_key(&space) {
                bd.space_rid = Some(space);
                // Create rigid body in space
                let rb = match bd.mode {
                    BodyMode::Static => rapier::RigidBodyBuilder::fixed().build(),
                    BodyMode::Kinematic => {
                        rapier::RigidBodyBuilder::kinematic_position_based().build()
                    }
                    BodyMode::Rigid | BodyMode::RigidLinear => {
                        rapier::RigidBodyBuilder::dynamic().can_sleep(false).build()
                    }
                };
                let space_data = self.spaces.get_mut(&space).unwrap();
                let rb_handle = space_data.rigid_body_set.insert(rb);
                bd.rb_handle = Some(rb_handle);
            } else {
                bd.space_rid = None;
            }
        }

        // Wire shapes into Rapier colliders (separate pass to avoid borrow issues)
        self.sync_body_colliders(body);
    }

    fn body_get_space(&self, body: Rid) -> Rid {
        self.bodies
            .get(&body)
            .and_then(|b| b.space_rid)
            .unwrap_or(Rid::Invalid)
    }

    fn body_set_mode(&mut self, body: Rid, mode: physics_server_2d::BodyMode) {
        if let Some(bd) = self.bodies.get_mut(&body) {
            bd.mode = match mode {
                physics_server_2d::BodyMode::STATIC => BodyMode::Static,
                physics_server_2d::BodyMode::KINEMATIC => BodyMode::Kinematic,
                physics_server_2d::BodyMode::RIGID => BodyMode::Rigid,
                physics_server_2d::BodyMode::RIGID_LINEAR => BodyMode::RigidLinear,
                _ => BodyMode::Rigid,
            };
            // Update rapier body type if in space
            if let (Some(space_rid), Some(rb_handle)) = (bd.space_rid, bd.rb_handle) {
                if let Some(space) = self.spaces.get_mut(&space_rid) {
                    if let Some(rb) = space.rigid_body_set.get_mut(rb_handle) {
                        match bd.mode {
                            BodyMode::Static => {
                                rb.set_body_type(rapier::RigidBodyType::Fixed, true)
                            }
                            BodyMode::Kinematic => rb
                                .set_body_type(rapier::RigidBodyType::KinematicPositionBased, true),
                            BodyMode::Rigid | BodyMode::RigidLinear => {
                                rb.set_body_type(rapier::RigidBodyType::Dynamic, true)
                            }
                        }
                    }
                }
            }
        }
    }

    fn body_get_mode(&self, body: Rid) -> physics_server_2d::BodyMode {
        self.bodies
            .get(&body)
            .map(|b| match b.mode {
                BodyMode::Static => physics_server_2d::BodyMode::STATIC,
                BodyMode::Kinematic => physics_server_2d::BodyMode::KINEMATIC,
                BodyMode::Rigid => physics_server_2d::BodyMode::RIGID,
                BodyMode::RigidLinear => physics_server_2d::BodyMode::RIGID_LINEAR,
            })
            .unwrap_or(physics_server_2d::BodyMode::STATIC)
    }

    fn body_add_shape(&mut self, body: Rid, shape: Rid, transform: Transform2D, disabled: bool) {
        if let Some(bd) = self.bodies.get_mut(&body) {
            bd.shapes.push(BodyShapeEntry {
                shape_rid: shape,
                transform,
                disabled,
                one_way: false,
                one_way_margin: 0.0,
                collider_handle: None,
            });
            // TODO: actually create collider in rapier space if body is in a space
        }
    }

    fn body_set_shape(&mut self, body: Rid, shape_idx: i32, shape: Rid) {
        if let Some(bd) = self.bodies.get_mut(&body) {
            if let Some(entry) = bd.shapes.get_mut(shape_idx as usize) {
                entry.shape_rid = shape;
            }
        }
    }

    fn body_set_shape_transform(&mut self, body: Rid, shape_idx: i32, transform: Transform2D) {
        if let Some(bd) = self.bodies.get_mut(&body) {
            if let Some(entry) = bd.shapes.get_mut(shape_idx as usize) {
                entry.transform = transform;
            }
        }
    }

    fn body_get_shape_count(&self, body: Rid) -> i32 {
        self.bodies
            .get(&body)
            .map(|b| b.shapes.len() as i32)
            .unwrap_or(0)
    }

    fn body_get_shape(&self, body: Rid, shape_idx: i32) -> Rid {
        self.bodies
            .get(&body)
            .and_then(|b| b.shapes.get(shape_idx as usize))
            .map(|e| e.shape_rid)
            .unwrap_or(Rid::Invalid)
    }

    fn body_get_shape_transform(&self, body: Rid, shape_idx: i32) -> Transform2D {
        self.bodies
            .get(&body)
            .and_then(|b| b.shapes.get(shape_idx as usize))
            .map(|e| e.transform)
            .unwrap_or(Transform2D::IDENTITY)
    }

    fn body_set_shape_disabled(&mut self, body: Rid, shape_idx: i32, disabled: bool) {
        if let Some(bd) = self.bodies.get_mut(&body) {
            if let Some(entry) = bd.shapes.get_mut(shape_idx as usize) {
                entry.disabled = disabled;
            }
        }
    }

    fn body_set_shape_as_one_way_collision(
        &mut self,
        body: Rid,
        shape_idx: i32,
        enable: bool,
        margin: f32,
    ) {
        if let Some(bd) = self.bodies.get_mut(&body) {
            if let Some(entry) = bd.shapes.get_mut(shape_idx as usize) {
                entry.one_way = enable;
                entry.one_way_margin = margin;
            }
        }
    }

    fn body_remove_shape(&mut self, body: Rid, shape_idx: i32) {
        if let Some(bd) = self.bodies.get_mut(&body) {
            let idx = shape_idx as usize;
            if idx < bd.shapes.len() {
                bd.shapes.remove(idx);
            }
        }
    }

    fn body_clear_shapes(&mut self, body: Rid) {
        if let Some(bd) = self.bodies.get_mut(&body) {
            bd.shapes.clear();
        }
    }

    fn body_attach_object_instance_id(&mut self, body: Rid, id: u64) {
        if let Some(bd) = self.bodies.get_mut(&body) {
            bd.object_instance_id = id;
        }
    }

    fn body_get_object_instance_id(&self, body: Rid) -> u64 {
        self.bodies
            .get(&body)
            .map(|b| b.object_instance_id)
            .unwrap_or(0)
    }

    fn body_attach_canvas_instance_id(&mut self, body: Rid, id: u64) {
        if let Some(bd) = self.bodies.get_mut(&body) {
            bd.canvas_instance_id = id;
        }
    }

    fn body_get_canvas_instance_id(&self, body: Rid) -> u64 {
        self.bodies
            .get(&body)
            .map(|b| b.canvas_instance_id)
            .unwrap_or(0)
    }

    fn body_set_continuous_collision_detection_mode(
        &mut self,
        body: Rid,
        mode: physics_server_2d::CcdMode,
    ) {
        if let Some(bd) = self.bodies.get(&body) {
            if let (Some(space_rid), Some(rb_handle)) = (bd.space_rid, bd.rb_handle) {
                if let Some(space) = self.spaces.get_mut(&space_rid) {
                    if let Some(rb) = space.rigid_body_set.get_mut(rb_handle) {
                        rb.enable_ccd(mode != physics_server_2d::CcdMode::DISABLED);
                    }
                }
            }
        }
    }

    fn body_get_continuous_collision_detection_mode(
        &self,
        _body: Rid,
    ) -> physics_server_2d::CcdMode {
        physics_server_2d::CcdMode::DISABLED
    }

    fn body_set_collision_layer(&mut self, body: Rid, layer: u32) {
        if let Some(bd) = self.bodies.get_mut(&body) {
            bd.collision_layer = layer;
        }
    }

    fn body_get_collision_layer(&self, body: Rid) -> u32 {
        self.bodies
            .get(&body)
            .map(|b| b.collision_layer)
            .unwrap_or(0)
    }

    fn body_set_collision_mask(&mut self, body: Rid, mask: u32) {
        if let Some(bd) = self.bodies.get_mut(&body) {
            bd.collision_mask = mask;
        }
    }

    fn body_get_collision_mask(&self, body: Rid) -> u32 {
        self.bodies
            .get(&body)
            .map(|b| b.collision_mask)
            .unwrap_or(0)
    }

    fn body_set_collision_priority(&mut self, body: Rid, priority: f32) {
        if let Some(bd) = self.bodies.get_mut(&body) {
            bd.collision_priority = priority;
        }
    }

    fn body_get_collision_priority(&self, body: Rid) -> f32 {
        self.bodies
            .get(&body)
            .map(|b| b.collision_priority)
            .unwrap_or(1.0)
    }

    fn body_set_param(
        &mut self,
        body: Rid,
        param: physics_server_2d::BodyParameter,
        value: Variant,
    ) {
        if let Some(bd) = self.bodies.get(&body) {
            if let (Some(space_rid), Some(rb_handle)) = (bd.space_rid, bd.rb_handle) {
                if let Some(space) = self.spaces.get_mut(&space_rid) {
                    if let Some(rb) = space.rigid_body_set.get_mut(rb_handle) {
                        match param {
                            physics_server_2d::BodyParameter::MASS => {
                                let mass: f32 = value.to::<f32>();
                                if mass > 0.0 {
                                    rb.set_additional_mass(mass, true);
                                }
                            }
                            physics_server_2d::BodyParameter::LINEAR_DAMP => {
                                rb.set_linear_damping(value.to::<f32>());
                            }
                            physics_server_2d::BodyParameter::ANGULAR_DAMP => {
                                rb.set_angular_damping(value.to::<f32>());
                            }
                            physics_server_2d::BodyParameter::GRAVITY_SCALE => {
                                rb.set_gravity_scale(value.to::<f32>(), true);
                            }
                            _ => {} // Bounce, friction handled on colliders
                        }
                    }
                }
            }
        }
    }

    fn body_get_param(&self, body: Rid, param: physics_server_2d::BodyParameter) -> Variant {
        if let Some(bd) = self.bodies.get(&body) {
            if let (Some(space_rid), Some(rb_handle)) = (bd.space_rid, bd.rb_handle) {
                if let Some(space) = self.spaces.get(&space_rid) {
                    if let Some(rb) = space.rigid_body_set.get(rb_handle) {
                        return match param {
                            physics_server_2d::BodyParameter::MASS => Variant::from(rb.mass()),
                            physics_server_2d::BodyParameter::LINEAR_DAMP => {
                                Variant::from(rb.linear_damping())
                            }
                            physics_server_2d::BodyParameter::ANGULAR_DAMP => {
                                Variant::from(rb.angular_damping())
                            }
                            physics_server_2d::BodyParameter::GRAVITY_SCALE => {
                                Variant::from(rb.gravity_scale())
                            }
                            _ => Variant::from(0.0f32),
                        };
                    }
                }
            }
        }
        Variant::from(0.0f32)
    }

    fn body_reset_mass_properties(&mut self, _body: Rid) {
        // Stub
    }

    fn body_set_state(&mut self, body: Rid, state: physics_server_2d::BodyState, value: Variant) {
        if let Some(bd) = self.bodies.get(&body) {
            if let (Some(space_rid), Some(rb_handle)) = (bd.space_rid, bd.rb_handle) {
                if let Some(space) = self.spaces.get_mut(&space_rid) {
                    if let Some(rb) = space.rigid_body_set.get_mut(rb_handle) {
                        match state {
                            physics_server_2d::BodyState::TRANSFORM => {
                                let t: Transform2D = value.to::<Transform2D>();
                                let origin = t.origin;
                                rb.set_translation(rapier::Vector::new(origin.x, origin.y), true);
                                rb.set_rotation(rapier::Rotation::new(t.rotation()), true);
                            }
                            physics_server_2d::BodyState::LINEAR_VELOCITY => {
                                let v: Vector2 = value.to::<Vector2>();
                                rb.set_linvel(rapier::Vector::new(v.x, v.y), true);
                            }
                            physics_server_2d::BodyState::ANGULAR_VELOCITY => {
                                rb.set_angvel(value.to::<f32>(), true);
                            }
                            _ => {}
                        }
                    }
                }
            }
        }
    }

    fn body_get_state(&self, body: Rid, state: physics_server_2d::BodyState) -> Variant {
        if let Some(bd) = self.bodies.get(&body) {
            if let (Some(space_rid), Some(rb_handle)) = (bd.space_rid, bd.rb_handle) {
                if let Some(space) = self.spaces.get(&space_rid) {
                    if let Some(rb) = space.rigid_body_set.get(rb_handle) {
                        return match state {
                            physics_server_2d::BodyState::TRANSFORM => {
                                let pos = rb.translation();
                                let rot = rb.rotation().angle();
                                let t =
                                    Transform2D::from_angle_origin(rot, Vector2::new(pos.x, pos.y));
                                Variant::from(t)
                            }
                            physics_server_2d::BodyState::LINEAR_VELOCITY => {
                                let v = rb.linvel();
                                Variant::from(Vector2::new(v.x, v.y))
                            }
                            physics_server_2d::BodyState::ANGULAR_VELOCITY => {
                                Variant::from(rb.angvel())
                            }
                            _ => Variant::nil(),
                        };
                    }
                }
            }
        }
        Variant::nil()
    }

    fn body_apply_central_impulse(&mut self, body: Rid, impulse: Vector2) {
        if let Some(bd) = self.bodies.get(&body) {
            if let (Some(space_rid), Some(rb_handle)) = (bd.space_rid, bd.rb_handle) {
                if let Some(space) = self.spaces.get_mut(&space_rid) {
                    if let Some(rb) = space.rigid_body_set.get_mut(rb_handle) {
                        rb.apply_impulse(rapier::Vector::new(impulse.x, impulse.y), true);
                    }
                }
            }
        }
    }

    fn body_apply_torque_impulse(&mut self, body: Rid, impulse: f32) {
        if let Some(bd) = self.bodies.get(&body) {
            if let (Some(space_rid), Some(rb_handle)) = (bd.space_rid, bd.rb_handle) {
                if let Some(space) = self.spaces.get_mut(&space_rid) {
                    if let Some(rb) = space.rigid_body_set.get_mut(rb_handle) {
                        rb.apply_torque_impulse(impulse, true);
                    }
                }
            }
        }
    }

    fn body_apply_impulse(&mut self, body: Rid, impulse: Vector2, position: Vector2) {
        if let Some(bd) = self.bodies.get(&body) {
            if let (Some(space_rid), Some(rb_handle)) = (bd.space_rid, bd.rb_handle) {
                if let Some(space) = self.spaces.get_mut(&space_rid) {
                    if let Some(rb) = space.rigid_body_set.get_mut(rb_handle) {
                        rb.apply_impulse_at_point(
                            rapier::Vector::new(impulse.x, impulse.y),
                            rapier::Point::new(position.x, position.y),
                            true,
                        );
                    }
                }
            }
        }
    }

    fn body_apply_central_force(&mut self, body: Rid, force: Vector2) {
        if let Some(bd) = self.bodies.get(&body) {
            if let (Some(space_rid), Some(rb_handle)) = (bd.space_rid, bd.rb_handle) {
                if let Some(space) = self.spaces.get_mut(&space_rid) {
                    if let Some(rb) = space.rigid_body_set.get_mut(rb_handle) {
                        rb.add_force(rapier::Vector::new(force.x, force.y), true);
                    }
                }
            }
        }
    }

    fn body_apply_force(&mut self, body: Rid, force: Vector2, position: Vector2) {
        if let Some(bd) = self.bodies.get(&body) {
            if let (Some(space_rid), Some(rb_handle)) = (bd.space_rid, bd.rb_handle) {
                if let Some(space) = self.spaces.get_mut(&space_rid) {
                    if let Some(rb) = space.rigid_body_set.get_mut(rb_handle) {
                        rb.add_force_at_point(
                            rapier::Vector::new(force.x, force.y),
                            rapier::Point::new(position.x, position.y),
                            true,
                        );
                    }
                }
            }
        }
    }

    fn body_apply_torque(&mut self, body: Rid, torque: f32) {
        if let Some(bd) = self.bodies.get(&body) {
            if let (Some(space_rid), Some(rb_handle)) = (bd.space_rid, bd.rb_handle) {
                if let Some(space) = self.spaces.get_mut(&space_rid) {
                    if let Some(rb) = space.rigid_body_set.get_mut(rb_handle) {
                        rb.add_torque(torque, true);
                    }
                }
            }
        }
    }

    fn body_add_constant_central_force(&mut self, body: Rid, force: Vector2) {
        if let Some(bd) = self.bodies.get_mut(&body) {
            bd.constant_force += force;
        }
    }

    fn body_add_constant_force(&mut self, body: Rid, force: Vector2, _position: Vector2) {
        if let Some(bd) = self.bodies.get_mut(&body) {
            bd.constant_force += force;
        }
    }

    fn body_add_constant_torque(&mut self, body: Rid, torque: f32) {
        if let Some(bd) = self.bodies.get_mut(&body) {
            bd.constant_torque += torque;
        }
    }

    fn body_set_constant_force(&mut self, body: Rid, force: Vector2) {
        if let Some(bd) = self.bodies.get_mut(&body) {
            bd.constant_force = force;
        }
    }

    fn body_get_constant_force(&self, body: Rid) -> Vector2 {
        self.bodies
            .get(&body)
            .map(|b| b.constant_force)
            .unwrap_or(Vector2::ZERO)
    }

    fn body_set_constant_torque(&mut self, body: Rid, torque: f32) {
        if let Some(bd) = self.bodies.get_mut(&body) {
            bd.constant_torque = torque;
        }
    }

    fn body_get_constant_torque(&self, body: Rid) -> f32 {
        self.bodies
            .get(&body)
            .map(|b| b.constant_torque)
            .unwrap_or(0.0)
    }

    fn body_set_axis_velocity(&mut self, body: Rid, axis_velocity: Vector2) {
        if let Some(bd) = self.bodies.get(&body) {
            if let (Some(space_rid), Some(rb_handle)) = (bd.space_rid, bd.rb_handle) {
                if let Some(space) = self.spaces.get_mut(&space_rid) {
                    if let Some(rb) = space.rigid_body_set.get_mut(rb_handle) {
                        let mut v = *rb.linvel();
                        let axis = rapier::Vector::new(axis_velocity.x, axis_velocity.y);
                        let axis_norm = axis.normalize();
                        v -= axis_norm * v.dot(&axis_norm);
                        v += axis;
                        rb.set_linvel(v, true);
                    }
                }
            }
        }
    }

    fn body_add_collision_exception(&mut self, _body: Rid, _excepted_body: Rid) {}
    fn body_remove_collision_exception(&mut self, _body: Rid, _excepted_body: Rid) {}
    fn body_get_collision_exceptions(&self, _body: Rid) -> Array<Rid> {
        Array::new()
    }

    fn body_set_max_contacts_reported(&mut self, body: Rid, amount: i32) {
        if let Some(bd) = self.bodies.get_mut(&body) {
            bd.max_contacts_reported = amount;
        }
    }

    fn body_get_max_contacts_reported(&self, body: Rid) -> i32 {
        self.bodies
            .get(&body)
            .map(|b| b.max_contacts_reported)
            .unwrap_or(0)
    }

    fn body_set_contacts_reported_depth_threshold(&mut self, body: Rid, threshold: f32) {
        if let Some(bd) = self.bodies.get_mut(&body) {
            bd.contacts_depth_threshold = threshold;
        }
    }

    fn body_get_contacts_reported_depth_threshold(&self, body: Rid) -> f32 {
        self.bodies
            .get(&body)
            .map(|b| b.contacts_depth_threshold)
            .unwrap_or(0.0)
    }

    fn body_set_omit_force_integration(&mut self, body: Rid, enable: bool) {
        if let Some(bd) = self.bodies.get_mut(&body) {
            bd.omit_force_integration = enable;
        }
    }

    fn body_is_omitting_force_integration(&self, body: Rid) -> bool {
        self.bodies
            .get(&body)
            .map(|b| b.omit_force_integration)
            .unwrap_or(false)
    }

    fn body_set_state_sync_callback(&mut self, body: Rid, callable: Callable) {
        if let Some(bd) = self.bodies.get_mut(&body) {
            bd.state_sync_callback = if callable.is_valid() {
                Some(callable)
            } else {
                None
            };
        }
    }

    fn body_set_force_integration_callback(
        &mut self,
        body: Rid,
        callable: Callable,
        userdata: Variant,
    ) {
        if let Some(bd) = self.bodies.get_mut(&body) {
            bd.force_integration_callback = if callable.is_valid() {
                Some((callable, userdata))
            } else {
                None
            };
        }
    }

    unsafe fn body_collide_shape_rawptr(
        &mut self,
        _body: Rid,
        _body_shape: i32,
        _shape: Rid,
        _shape_xform: Transform2D,
        _motion: Vector2,
        _results: RawPtr<*mut c_void>,
        _result_max: i32,
        _result_count: RawPtr<*mut i32>,
    ) -> bool {
        false
    }

    fn body_set_pickable(&mut self, body: Rid, pickable: bool) {
        if let Some(bd) = self.bodies.get_mut(&body) {
            bd.pickable = pickable;
        }
    }

    fn body_get_direct_state(&mut self, body: Rid) -> Option<Gd<PhysicsDirectBodyState2D>> {
        let bd = self.bodies.get(&body)?;
        let (space_rid, rb_handle) = (bd.space_rid?, bd.rb_handle?);
        let space = self.spaces.get(&space_rid)?;
        let rb = space.rigid_body_set.get(rb_handle)?;

        // Snapshot current state into the direct state object
        let pos = rb.translation();
        let rot = rb.rotation().angle();
        let linvel = rb.linvel();
        let angvel = rb.angvel();
        let mass = rb.mass();
        let inv_inertia = 1.0
            / rb.mass_properties()
                .local_mprops
                .inv_principal_inertia_sqrt
                .powi(-2)
                .max(f32::EPSILON);

        let transform = Transform2D::from_angle_origin(rot, Vector2::new(pos.x, pos.y));

        let inv_mass = if mass > 0.0 { 1.0 / mass } else { 0.0 };
        let inv_inertia_val = rb
            .mass_properties()
            .local_mprops
            .inv_principal_inertia_sqrt
            .powi(2);
        let gravity = Vector2::new(space.gravity.x, space.gravity.y);
        let dt = space.integration_parameters.dt;
        let linvel_v = Vector2::new(linvel.x, linvel.y);

        let state: Gd<EvolveDirectBodyState> = Gd::from_init_fn(|base| EvolveDirectBodyState {
            base,
            body_rid: body,
            space_rid,
            cached_transform: transform,
            cached_linear_velocity: linvel_v,
            cached_angular_velocity: angvel,
            cached_inverse_mass: inv_mass,
            cached_inverse_inertia: inv_inertia_val,
            cached_total_gravity: gravity,
            cached_constant_force: Vector2::ZERO,
            cached_constant_torque: 0.0,
            sleeping: false,
            step: dt,
        });
        Some(state.upcast())
    }

    unsafe fn body_test_motion_rawptr(
        &self,
        body: Rid,
        from: Transform2D,
        motion: Vector2,
        margin: f32,
        _collide_separation_ray: bool,
        _recovery_as_collision: bool,
        result: RawPtr<*mut PhysicsServer2DExtensionMotionResult>,
    ) -> bool {
        let bd = match self.bodies.get(&body) {
            Some(b) => b,
            None => return false,
        };
        let space_rid = match bd.space_rid {
            Some(r) => r,
            None => return false,
        };
        let space = match self.spaces.get(&space_rid) {
            Some(s) => s,
            None => return false,
        };

        // Get the body's first shape
        let shape_entry = match bd.shapes.first() {
            Some(s) => s,
            None => return false,
        };
        let shape_data = match self.shapes.get(&shape_entry.shape_rid) {
            Some(s) => s,
            None => return false,
        };

        // Build a rapier shape for the cast
        let rapier_shape: rapier::SharedShape = match shape_data.shape_type {
            ShapeType::Circle => {
                let radius = shape_data.data.try_to::<f32>().unwrap_or(10.0);
                rapier::SharedShape::ball(radius)
            }
            ShapeType::Rectangle => {
                let half = shape_data
                    .data
                    .try_to::<Vector2>()
                    .unwrap_or(Vector2::new(10.0, 10.0));
                rapier::SharedShape::cuboid(half.x, half.y)
            }
            ShapeType::Capsule => {
                let arr = shape_data.data.try_to::<Array<Variant>>().ok();
                let (height, radius) = if let Some(ref a) = arr {
                    let h: f32 = if a.len() > 0 {
                        a.at(0).try_to::<f32>().unwrap_or(20.0)
                    } else {
                        20.0
                    };
                    let r: f32 = if a.len() > 1 {
                        a.at(1).try_to::<f32>().unwrap_or(10.0)
                    } else {
                        10.0
                    };
                    (h, r)
                } else {
                    (20.0, 10.0)
                };
                rapier::SharedShape::capsule_y(height / 2.0, radius)
            }
            ShapeType::Segment => {
                let rect = shape_data.data.try_to::<godot::builtin::Rect2>().ok();
                if let Some(r) = rect {
                    let a = rapier::Point::new(r.position.x, r.position.y);
                    let b = rapier::Point::new(r.position.x + r.size.x, r.position.y + r.size.y);
                    rapier::SharedShape::segment(a, b)
                } else {
                    return false;
                }
            }
            ShapeType::ConvexPolygon => {
                let points = shape_data.data.try_to::<PackedVector2Array>().ok();
                if let Some(pts) = points {
                    let rapier_pts: Vec<rapier::Point<f32>> = pts
                        .as_slice()
                        .iter()
                        .map(|p| rapier::Point::new(p.x, p.y))
                        .collect();
                    if let Some(shape) = rapier::SharedShape::convex_hull(&rapier_pts) {
                        shape
                    } else {
                        return false;
                    }
                } else {
                    return false;
                }
            }
            _ => return false,
        };

        // Compute shape position from the `from` transform + shape local transform
        let shape_xf = shape_entry.transform;
        let origin_x = from.origin.x + shape_xf.origin.x;
        let origin_y = from.origin.y + shape_xf.origin.y;
        let rotation = from.rotation();
        let shape_iso =
            rapier::Isometry::new(rapier::Vector::new(origin_x, origin_y), rotation as f32);

        let motion_len = (motion.x * motion.x + motion.y * motion.y).sqrt();
        if motion_len < 1.0e-6 {
            // No motion — check if already overlapping (depenetration)
            return false;
        }
        let dir = rapier::Vector::new(motion.x / motion_len, motion.y / motion_len);

        // Exclude the body's own colliders
        let exclude_rb = bd.rb_handle;
        let filter = if let Some(rb_h) = exclude_rb {
            rapier::QueryFilter::default().exclude_rigid_body(rb_h)
        } else {
            rapier::QueryFilter::default()
        };

        let hit = space.query_pipeline.cast_shape(
            &space.rigid_body_set,
            &space.collider_set,
            &shape_iso,
            &dir,
            rapier_shape.as_ref(),
            rapier2d::parry::query::ShapeCastOptions {
                max_time_of_impact: motion_len,
                target_distance: margin,
                stop_at_penetration: true,
                compute_impact_geometry_on_penetration: true,
            },
            filter,
        );

        if let Some((collider_handle, hit_data)) = hit {
            let result_ptr: *mut PhysicsServer2DExtensionMotionResult = result.ptr();
            if !result_ptr.is_null() {
                let toi_frac = hit_data.time_of_impact / motion_len;
                let travel = motion * toi_frac;
                let remainder = motion * (1.0 - toi_frac);

                // Get collision details
                let normal1 = hit_data.normal1;
                let witness1 = hit_data.witness1;

                // Look up the collider's body to get object instance id
                let collider = space.collider_set.get(collider_handle);
                let collider_body_handle = collider.and_then(|c| c.parent());
                let collider_body_rid = collider_body_handle.and_then(|rbh| {
                    self.bodies
                        .iter()
                        .find(|(_, bd)| {
                            bd.rb_handle == Some(rbh) && bd.space_rid == Some(space_rid)
                        })
                        .map(|(r, _)| *r)
                });
                let collider_instance_id = collider_body_rid
                    .and_then(|r| self.bodies.get(&r))
                    .map(|bd| bd.object_instance_id)
                    .unwrap_or(0);

                let r = &mut *result_ptr;
                r.travel = travel;
                r.remainder = remainder;
                r.collision_point = Vector2::new(witness1.x, witness1.y);
                r.collision_normal = Vector2::new(normal1.x, normal1.y);
                r.collision_depth = margin;
                r.collision_safe_fraction = toi_frac;
                r.collision_unsafe_fraction = toi_frac;
                r.collider_velocity = Vector2::ZERO;
                r.collider_id = godot::classes::native::ObjectId {
                    id: collider_instance_id,
                };
                r.collider = collider_body_rid.unwrap_or(Rid::Invalid);
                r.collider_shape = 0;
                r.collision_local_shape = 0;
            }
            return true;
        }
        false
    }

    // ── Joint (stubs) ────────────────────────────────────────────────────

    fn joint_create(&mut self) -> Rid {
        let rid = self.alloc_rid();
        self.joints.insert(rid, JointData {});
        rid
    }

    fn joint_clear(&mut self, _joint: Rid) {}

    fn joint_set_param(&mut self, _joint: Rid, _param: physics_server_2d::JointParam, _value: f32) {
    }
    fn joint_get_param(&self, _joint: Rid, _param: physics_server_2d::JointParam) -> f32 {
        0.0
    }
    fn joint_disable_collisions_between_bodies(&mut self, _joint: Rid, _disable: bool) {}
    fn joint_is_disabled_collisions_between_bodies(&self, _joint: Rid) -> bool {
        false
    }

    fn joint_make_pin(&mut self, _joint: Rid, _anchor: Vector2, _body_a: Rid, _body_b: Rid) {}
    fn joint_make_groove(
        &mut self,
        _joint: Rid,
        _a_groove1: Vector2,
        _a_groove2: Vector2,
        _b_anchor: Vector2,
        _body_a: Rid,
        _body_b: Rid,
    ) {
    }
    fn joint_make_damped_spring(
        &mut self,
        _joint: Rid,
        _anchor_a: Vector2,
        _anchor_b: Vector2,
        _body_a: Rid,
        _body_b: Rid,
    ) {
    }

    fn pin_joint_set_flag(
        &mut self,
        _joint: Rid,
        _flag: physics_server_2d::PinJointFlag,
        _enabled: bool,
    ) {
    }
    fn pin_joint_get_flag(&self, _joint: Rid, _flag: physics_server_2d::PinJointFlag) -> bool {
        false
    }
    fn pin_joint_set_param(
        &mut self,
        _joint: Rid,
        _param: physics_server_2d::PinJointParam,
        _value: f32,
    ) {
    }
    fn pin_joint_get_param(&self, _joint: Rid, _param: physics_server_2d::PinJointParam) -> f32 {
        0.0
    }
    fn damped_spring_joint_set_param(
        &mut self,
        _joint: Rid,
        _param: physics_server_2d::DampedSpringParam,
        _value: f32,
    ) {
    }
    fn damped_spring_joint_get_param(
        &self,
        _joint: Rid,
        _param: physics_server_2d::DampedSpringParam,
    ) -> f32 {
        0.0
    }
    fn joint_get_type(&self, _joint: Rid) -> physics_server_2d::JointType {
        physics_server_2d::JointType::PIN
    }

    // ── Global lifecycle ─────────────────────────────────────────────────

    fn free_rid(&mut self, rid: Rid) {
        // Try removing from each collection
        if self.shapes.remove(&rid).is_some() {
            return;
        }
        if self.areas.remove(&rid).is_some() {
            return;
        }
        if self.joints.remove(&rid).is_some() {
            return;
        }
        if let Some(bd) = self.bodies.remove(&rid) {
            // Clean up rapier body
            if let (Some(space_rid), Some(rb_handle)) = (bd.space_rid, bd.rb_handle) {
                if let Some(space) = self.spaces.get_mut(&space_rid) {
                    let colliders: Vec<_> = space
                        .rigid_body_set
                        .get(rb_handle)
                        .map(|rb| rb.colliders().to_vec())
                        .unwrap_or_default();
                    for ch in colliders {
                        space.collider_set.remove(
                            ch,
                            &mut space.island_manager,
                            &mut space.rigid_body_set,
                            true,
                        );
                    }
                    space.rigid_body_set.remove(
                        rb_handle,
                        &mut space.island_manager,
                        &mut space.collider_set,
                        &mut space.impulse_joint_set,
                        &mut space.multibody_joint_set,
                        true,
                    );
                }
            }
            return;
        }
        self.spaces.remove(&rid);
    }

    fn set_active(&mut self, active: bool) {
        self.active = active;
    }

    fn init_ext(&mut self) {
        godot_print!("[EvolvePhysics] Rapier2D physics server ready");
    }

    fn step(&mut self, step: f32) {
        if !self.active {
            return;
        }

        // Drain write-back queue from DirectBodyState
        {
            let mut queue = WRITE_BACK_QUEUE.lock().unwrap();
            for wb in queue.drain(..) {
                match wb {
                    BodyWriteBack::CentralImpulse { body, impulse } => {
                        self.body_apply_central_impulse(body, impulse);
                    }
                    BodyWriteBack::Impulse {
                        body,
                        impulse,
                        position,
                    } => {
                        self.body_apply_impulse(body, impulse, position);
                    }
                    BodyWriteBack::TorqueImpulse { body, impulse } => {
                        self.body_apply_torque_impulse(body, impulse);
                    }
                    BodyWriteBack::CentralForce { body, force } => {
                        self.body_apply_central_force(body, force);
                    }
                    BodyWriteBack::Force {
                        body,
                        force,
                        position,
                    } => {
                        self.body_apply_force(body, force, position);
                    }
                    BodyWriteBack::Torque { body, torque } => {
                        self.body_apply_torque(body, torque);
                    }
                    BodyWriteBack::LinearVelocity { body, velocity } => {
                        self.body_set_state(
                            body,
                            physics_server_2d::BodyState::LINEAR_VELOCITY,
                            Variant::from(velocity),
                        );
                    }
                    BodyWriteBack::AngularVelocity { body, velocity } => {
                        self.body_set_state(
                            body,
                            physics_server_2d::BodyState::ANGULAR_VELOCITY,
                            Variant::from(velocity),
                        );
                    }
                    BodyWriteBack::Transform { body, transform } => {
                        self.body_set_state(
                            body,
                            physics_server_2d::BodyState::TRANSFORM,
                            Variant::from(transform),
                        );
                    }
                }
            }
        }

        // Apply constant forces to all bodies
        let body_entries: Vec<(Option<Rid>, Option<rapier::RigidBodyHandle>, Vector2, f32)> = self
            .bodies
            .values()
            .map(|bd| {
                (
                    bd.space_rid,
                    bd.rb_handle,
                    bd.constant_force,
                    bd.constant_torque,
                )
            })
            .collect();

        for (space_rid, rb_handle, force, torque) in body_entries {
            if let (Some(space_rid), Some(rb_handle)) = (space_rid, rb_handle) {
                if force != Vector2::ZERO || torque != 0.0 {
                    if let Some(space) = self.spaces.get_mut(&space_rid) {
                        if let Some(rb) = space.rigid_body_set.get_mut(rb_handle) {
                            if force != Vector2::ZERO {
                                rb.add_force(rapier::Vector::new(force.x, force.y), true);
                            }
                            if torque != 0.0 {
                                rb.add_torque(torque, true);
                            }
                        }
                    }
                }
            }
        }

        // Step all active spaces
        for space in self.spaces.values_mut() {
            if space.active {
                space.step(step);
                space.query_pipeline.update(&space.collider_set);
            }
        }
    }

    fn sync(&mut self) {
        // Call state sync callbacks for bodies that moved
    }

    fn flush_queries(&mut self) {}
    fn end_sync(&mut self) {}

    fn finish(&mut self) {
        godot_print!("[EvolvePhysics] Rapier2D physics server shutting down");
        self.spaces.clear();
        self.bodies.clear();
        self.shapes.clear();
        self.areas.clear();
        self.joints.clear();
    }

    fn is_flushing_queries(&self) -> bool {
        false
    }

    fn get_process_info(&mut self, _process_info: physics_server_2d::ProcessInfo) -> i32 {
        0
    }
}
