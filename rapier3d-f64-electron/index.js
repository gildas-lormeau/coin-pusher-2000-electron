const nativeRapier = require("./rapier3d-f64-electron");

class Vector3 {
    constructor(x = 0, y = 0, z = 0) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
}

class Quaternion {
    constructor(x = 0, y = 0, z = 0, w = 1) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }
}

function toEuler(quaternion) {
    const x = quaternion.x;
    const y = quaternion.y;
    const z = quaternion.z;
    const w = quaternion.w;
    const sinr_cosp = 2 * (w * x + y * z);
    const cosr_cosp = 1 - 2 * (x * x + y * y);
    const roll = Math.atan2(sinr_cosp, cosr_cosp);
    const sinp = 2 * (w * y - z * x);
    let pitch = 0;
    if (Math.abs(sinp) >= 1) {
        pitch = Math.sign(sinp) * (Math.PI / 2);
    }
    else {
        pitch = Math.asin(sinp);
    }
    const siny_cosp = 2 * (w * z + x * y);
    const cosy_cosp = 1 - 2 * (y * y + z * z);
    const yaw = Math.atan2(siny_cosp, cosy_cosp);
    return {
        x: roll,
        y: pitch,
        z: yaw
    };
}


class World {

    colliders = new Map();
    impulseJoints = new Map();

    #rigidBodies = new Map();
    #timestep = 1 / 60;

    constructor(gravity) {
        const rigidBodies = this.#rigidBodies;

        nativeRapier.initWorld(gravity.x, gravity.y, gravity.z);
        this.integrationParameters = {
            /**
             * @param {number} value
             */
            set numSolverIterations(value) {
                nativeRapier.setIntegrationParametersNumSolverIterations(value);
            },
            /**
             * @param {number} value
             */
            set numAdditionalFrictionIterations(value) {
                nativeRapier.setIntegrationParametersNumAdditionalFrictionIterations(value);
            },
            /**
             * @param {number} value
             */
            set numInternalPgsIterations(value) {
                nativeRapier.setIntegrationParametersNumInternalPgsIterations(value);
            }
        };
        this.bodies = {
            get(handle) {
                return rigidBodies.get(handle);
            }
        };
    }

    /**
     * @param {number} value
     */
    set timestep(value) {
        this.#timestep = value;
        nativeRapier.setTimestep(value);
    }

    createRigidBody(rigidBodyDesc) {
        let handle;
        switch (rigidBodyDesc.bodyType) {
            case "dynamic":
                handle = nativeRapier.createDynamicBody(rigidBodyDesc);
                break;
            case "kinematic":
                handle = nativeRapier.createKinematicBody(rigidBodyDesc);
                break;
            case "fixed":
            default:
                handle = nativeRapier.createFixedBody(rigidBodyDesc);
                break;
        }
        const body = new RigidBody(handle);
        this.#rigidBodies.set(handle, body);
        return body;
    }

    createCollider(colliderDesc, body) {
        let handle;
        const translation = colliderDesc.translation;
        let rotation = colliderDesc.rotation;
        if (rotation) {
            rotation = toEuler(rotation);
        }
        switch (colliderDesc.shape.type) {
            case "cuboid":
                handle = nativeRapier.addBoxCollider(
                    body.handle,
                    colliderDesc.shape.hx,
                    colliderDesc.shape.hy,
                    colliderDesc.shape.hz,
                    colliderDesc.sensor,
                    translation?.x,
                    translation?.y,
                    translation?.z,
                    rotation?.x,
                    rotation?.y,
                    rotation?.z
                );
                break;
            case "cylinder":
                handle = nativeRapier.addCylinderCollider(
                    body.handle,
                    colliderDesc.shape.halfHeight,
                    colliderDesc.shape.radius,
                    colliderDesc.sensor,
                    translation?.x,
                    translation?.y,
                    translation?.z,
                    rotation?.x,
                    rotation?.y,
                    rotation?.z
                );
                break;
            case "trimesh":
                const verticesTyped = new Float64Array(colliderDesc.shape.vertices);
                const indicesTyped = new Float64Array(colliderDesc.shape.indices);
                handle = nativeRapier.addTrimeshCollider(
                    body.handle,
                    verticesTyped,
                    indicesTyped,
                    colliderDesc.sensor,
                    colliderDesc.shape.flags,
                    translation?.x,
                    translation?.y,
                    translation?.z,
                    rotation?.x,
                    rotation?.y,
                    rotation?.z
                );
                break;
            case "convexHull":
                const convexVerticesTyped = new Float64Array(colliderDesc.shape.vertices);
                handle = nativeRapier.addConvexHullCollider(
                    body.handle,
                    convexVerticesTyped,
                    colliderDesc.sensor,
                    translation?.x,
                    translation?.y,
                    translation?.z,
                    rotation?.x,
                    rotation?.y,
                    rotation?.z
                );
                break;
            default:
                throw new Error(`Unsupported collider shape: ${colliderDesc.shape.type}.`);
        }
        const collider = new Collider(handle, colliderDesc, this);
        body.colliders.push(collider);
        this.colliders.set(handle, collider);
        return collider;
    }

    createImpulseJoint(jointData, body1, body2, wakeUp = true) {
        let handle;
        switch (jointData.type) {
            case "revolute":
                handle = nativeRapier.createRevoluteJoint(
                    body1.handle,
                    body2.handle,
                    jointData.anchor1.x,
                    jointData.anchor1.y,
                    jointData.anchor1.z,
                    jointData.anchor2.x,
                    jointData.anchor2.y,
                    jointData.anchor2.z,
                    jointData.axis.x,
                    jointData.axis.y,
                    jointData.axis.z,
                    wakeUp
                );
                break;
            case "fixed":
                handle = nativeRapier.createFixedJoint(
                    body1.handle,
                    body2.handle,
                    jointData.anchor1.x,
                    jointData.anchor1.y,
                    jointData.anchor1.z,
                    jointData.anchor2.x,
                    jointData.anchor2.y,
                    jointData.anchor2.z,
                    jointData.frame1.x,
                    jointData.frame1.y,
                    jointData.frame1.z,
                    jointData.frame1.w,
                    jointData.frame2.x,
                    jointData.frame2.y,
                    jointData.frame2.z,
                    jointData.frame2.w,
                    wakeUp
                );
                break;
            default:
                throw new Error(`Unsupported joint type: ${jointData.type}`);
        }
        const joint = new ImpulseJoint(handle, jointData, body1, body2);
        this.impulseJoints.set(handle, joint);
        return joint;
    }

    step(timestep = this.#timestep) {
        RigidBody.step();
        nativeRapier.stepSimulation(timestep);
    }

    forEachCollider(callback) {
        for (const collider of this.colliders.values()) {
            callback(collider);
        }
    }

    intersectionPairsWith(collider, callback) {
        const collidersHandles = nativeRapier.intersectionPairsWith(collider.handle);
        for (const colliderHandle of collidersHandles) {
            const otherCollider = this.colliders.get(colliderHandle);
            if (otherCollider && otherCollider.handle !== collider.handle) {
                callback(otherCollider);
            }
        }
    }

    takeSnapshot() {
        return new Uint8Array(nativeRapier.takeSnapshot());
    }

    static restoreSnapshot(snapshot) {
        const world = new World(new Vector3());
        nativeRapier.restoreSnapshot(snapshot);
        const bodies = nativeRapier.getWorldBodies();
        for (const handle of bodies) {
            const body = new RigidBody(handle);
            world.#rigidBodies.set(handle, body);
        }
        const colliders = nativeRapier.getWorldColliders();
        for (const handle of colliders) {
            let colliderDesc;
            const shapeType = nativeRapier.getColliderShapeType(handle);
            if (shapeType === 1) {
                const extents = nativeRapier.getColliderHalfExtents(handle);
                colliderDesc = ColliderDesc.cuboid(extents[0], extents[1], extents[2]);
            } else if (shapeType === 10) {
                const radius = nativeRapier.getColliderRadius(handle);
                const halfHeight = nativeRapier.getColliderHalfHeight(handle);
                colliderDesc = ColliderDesc.cylinder(halfHeight, radius);
            } else if (shapeType === 6) {
                const vertices = nativeRapier.getColliderVertices(handle);
                const indices = nativeRapier.getColliderIndices(handle);
                const flags = nativeRapier.getColliderFlags(handle);
                colliderDesc = ColliderDesc.trimesh(vertices, indices, flags);
            } else if (shapeType === 9) {
                const vertices = nativeRapier.getColliderVertices(handle);
                colliderDesc = ColliderDesc.convexHull(vertices, nativeRapier.getColliderIndices(handle));
            }
            const collider = new Collider(handle, colliderDesc, world);
            world.colliders.set(handle, collider);
        }
        for (const body of world.#rigidBodies.values()) {
            for (let colliderIndex = 0; colliderIndex < nativeRapier.getBodyNumColliders(body.handle); colliderIndex++) {
                const handle = nativeRapier.getBodyCollider(body.handle, colliderIndex);
                const collider = world.colliders.get(handle);
                body.colliders.push(collider);
            }
        }
        const joints = nativeRapier.getWorldImpulseJoints();
        for (const handle of joints) {
            let jointData;
            const data = nativeRapier.getJointData(handle);
            const body1 = world.#rigidBodies.get(data[1]);
            const body2 = world.#rigidBodies.get(data[2]);
            const anchor1 = new Vector3(data[3], data[4], data[5]);
            const anchor2 = new Vector3(data[6], data[7], data[8]);
            if (data[0] === 0) {
                const axis = new Vector3(data[9], data[10], data[11]);
                jointData = JointData.revolute(anchor1, anchor2, axis);
            } else {
                const frame1 = new Quaternion(data[9], data[10], data[11], data[12]);
                const frame2 = new Quaternion(data[13], data[14], data[15], data[16]);
                jointData = JointData.fixed(anchor1, frame1, anchor2, frame2);
            }
            const joint = new ImpulseJoint(handle, jointData, body1, body2);
            world.impulseJoints.set(handle, joint);
        }
        return world;
    }

    connectBodiesWithRevoluteJoint({ body1, body2, anchor1, anchor2, axis }) {
        const jointData = JointData.revolute(anchor1, anchor2, axis);
        return this.createImpulseJoint(jointData, body1, body2);
    }

    connectBodiesWithFixedJoint({ body1, body2, anchor1, anchor2, frame1, frame2 }) {
        const jointData = JointData.fixed(anchor1, frame1, anchor2, frame2);
        return this.createImpulseJoint(jointData, body1, body2);
    }
}

class RigidBody {
    static #translations = [];
    static #translationsInvalidated = true;
    static #rotations = [];
    static #rotationsInvalidated = true;

    static step() {
        this.#translationsInvalidated = true;
        this.#rotationsInvalidated = true;
    }

    colliders = [];

    constructor(handle) {
        this.handle = handle;
    }

    translation() {
        if (RigidBody.#translationsInvalidated) {
            RigidBody.#translationsInvalidated = false;
            const positions = nativeRapier.getBodyTranslations();
            for (let i = 0; i < positions.length; i += 4) {
                const handle = positions[i];
                const x = positions[i + 1];
                const y = positions[i + 2];
                const z = positions[i + 3];
                let translation = RigidBody.#translations[handle];
                if (!translation) {
                    translation = new Vector3();
                    RigidBody.#translations[handle] = translation;
                }
                Object.assign(translation, { x, y, z });
            }
        }
        return RigidBody.#translations[this.handle];
    }

    rotation() {
        if (RigidBody.#rotationsInvalidated) {
            RigidBody.#rotationsInvalidated = false;
            const rotations = nativeRapier.getBodyRotations();
            for (let i = 0; i < rotations.length; i += 5) {
                const handle = rotations[i];
                const x = rotations[i + 1];
                const y = rotations[i + 2];
                const z = rotations[i + 3];
                const w = rotations[i + 4];
                let rotation = RigidBody.#rotations[handle];
                if (!rotation) {
                    rotation = new Quaternion();
                    RigidBody.#rotations[handle] = rotation;
                }
                Object.assign(rotation, { x, y, z, w });
            }
        }
        return RigidBody.#rotations[this.handle];
    }

    setTranslation(vector, wakeUp = true) {
        nativeRapier.setBodyTranslation(this.handle, vector.x, vector.y, vector.z, wakeUp);
        return this;
    }

    setRotation(quaternion, wakeUp = true) {
        nativeRapier.setBodyRotation(this.handle, quaternion.x, quaternion.y, quaternion.z, quaternion.w, wakeUp);
        return this;
    }

    linvel() {
        const vel = nativeRapier.getBodyVelocity(this.handle);
        return new Vector3(vel[0], vel[1], vel[2]);
    }

    setLinvel(vector3, wakeUp = true) {
        nativeRapier.setBodyVelocity(this.handle, vector3.x, vector3.y, vector3.z, wakeUp);
    }

    angvel() {
        const vel = nativeRapier.getBodyAngularVelocity(this.handle);
        return new Vector3(vel[0], vel[1], vel[2]);
    }

    setAngvel(vector3, wakeUp = true) {
        nativeRapier.setBodyAngularVelocity(this.handle, vector3.x, vector3.y, vector3.z, wakeUp);
    }

    setNextKinematicTranslation(vector3) {
        nativeRapier.setBodyNextKinematicTranslation(this.handle, vector3.x, vector3.y, vector3.z);
    }

    setNextKinematicRotation(quaternion) {
        nativeRapier.setBodyNextKinematicRotation(this.handle, quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    }

    applyImpulse(vector3, wakeUp = true) {
        nativeRapier.applyImpulse(this.handle, vector3.x, vector3.y, vector3.z, wakeUp);
    }

    setEnabled(enabled) {
        nativeRapier.setBodyEnabled(this.handle, enabled);
        return this;
    }

    isEnabled() {
        return nativeRapier.isBodyEnabled(this.handle);
    }

    setEnabledRotations(enableX, enableY, enableZ, wakeUp = true) {
        nativeRapier.setBodyEnabledRotations(this.handle, enableX, enableY, enableZ, wakeUp);
        return this;
    }

    setEnabledTranslations(enableX, enableY, enableZ, wakeUp = true) {
        nativeRapier.setBodyEnabledTranslations(this.handle, enableX, enableY, enableZ, wakeUp);
        return this;
    }

    setSoftCcdPrediction(prediction) {
        nativeRapier.setBodySoftCcdPrediction(this.handle, prediction);
        return this;
    }

    setAdditionalSolverIterations(iterations) {
        nativeRapier.setBodyAdditionalSolverIterations(this.handle, iterations);
        return this;
    }

    enableCcd(enabled) {
        nativeRapier.setBodyCcdEnabled(this.handle, enabled);
        return this;
    }

    setAngularDamping(damping) {
        nativeRapier.setBodyAngularDamping(this.handle, damping);
        return this;
    }

    setLinearDamping(damping) {
        nativeRapier.setBodyLinearDamping(this.handle, damping);
        return this;
    }

    sleep() {
        nativeRapier.bodySleep(this.handle);
        return this;
    }

    isSleeping() {
        return nativeRapier.isBodySleeping(this.handle);
    }

    numColliders() {
        return this.colliders.length;
    }

    mass() {
        return nativeRapier.getBodyMass(this.handle);
    }

    collider(index) {
        return this.colliders[index];
    }
}

class Collider {
    constructor(handle, colliderDesc, world) {
        this.handle = handle;
        this.colliderDesc = colliderDesc;
        this.world = world;
        this.userData = {};
    }

    setFriction(friction) {
        nativeRapier.setColliderFriction(this.handle, friction);
        return this;
    }

    setRestitution(restitution) {
        nativeRapier.setColliderRestitution(this.handle, restitution);
        return this;
    }

    setDensity(density) {
        nativeRapier.setColliderDensity(this.handle, density);
        return this;
    }

    setCollisionGroups(groups) {
        nativeRapier.setColliderCollisionGroups(this.handle, groups);
        return this;
    }

    setContactSkin(skin) {
        nativeRapier.setColliderContactSkin(this.handle, skin);
        return this;
    }

    isEnabled() {
        return nativeRapier.isColliderEnabled(this.handle);
    }

    setEnabled(enabled) {
        nativeRapier.setColliderEnabled(this.handle, enabled);
        return this;
    }

    parent() {
        const handle = nativeRapier.getColliderParent(this.handle);
        return this.world.bodies.get(handle);
    }

    shapeType() {
        return this.colliderDesc.shape.shapeType;
    }

    translation() {
        const pos = nativeRapier.getColliderTranslation(this.handle);
        return new Vector3(pos[0], pos[1], pos[2]);
    }

    rotation() {
        const rot = nativeRapier.getColliderRotation(this.handle);
        return new Quaternion(rot[0], rot[1], rot[2], rot[3]);
    }

    vertices() {
        const vertices = nativeRapier.getColliderVertices(this.handle);
        return new Float64Array(vertices);
    }

    indices() {
        const indices = nativeRapier.getColliderIndices(this.handle);
        return new Uint32Array(indices);
    }

    radius() {
        return this.colliderDesc.shape.radius;
    }

    halfExtents() {
        return new Vector3(
            this.colliderDesc.shape.hx,
            this.colliderDesc.shape.hy,
            this.colliderDesc.shape.hz
        );
    }

    halfHeight() {
        return this.colliderDesc.shape.halfHeight;
    }
}

class ImpulseJoint {

    #jointData;
    #body1;
    #body2;

    constructor(handle, jointData, body1, body2) {
        this.handle = handle;
        this.#jointData = jointData;
        this.#body1 = body1;
        this.#body2 = body2;
    }

    setLimits(min, max) {
        nativeRapier.setRevoluteJointLimits(this.handle, min, max);
        return this;
    }

    configureMotor(targetPos, targetVel, damping, maxForce) {
        nativeRapier.configureRevoluteJointMotor(
            this.handle,
            targetPos,
            targetVel,
            damping,
            maxForce
        );
        return this;
    }

    anchor1() {
        return this.#jointData.anchor1;
    }

    anchor2() {
        return this.#jointData.anchor2;
    }

    body1() {
        return this.#body1;
    }

    body2() {
        return this.#body2;
    }
}

class RigidBodyDesc {
    constructor() {
        this.translation = new Vector3();
        this.rotation = new Quaternion();
    }

    static fixed() {
        const desc = new RigidBodyDesc();
        desc.bodyType = "fixed";
        return desc;
    }

    static dynamic() {
        const desc = new RigidBodyDesc();
        desc.bodyType = "dynamic";
        return desc;
    }

    static kinematicPositionBased() {
        const desc = new RigidBodyDesc();
        desc.bodyType = "kinematic";
        return desc;
    }

    setTranslation(x, y, z) {
        this.translation = new Vector3(x, y, z);
    }

    setRotation(x, y, z, w) {
        this.rotation = new Quaternion(x, y, z, w);
        return this;
    }
}

class ColliderDesc {
    constructor() {
        this.shape = undefined;
        this.translation = new Vector3();
        this.rotation = new Quaternion();
        this.sensor = false;
        this.friction = undefined;
        this.restitution = undefined;
        this.density = undefined;
        this.translation = undefined;
        this.rotation = undefined;
    }

    static cuboid(hx, hy, hz) {
        const desc = new ColliderDesc();
        desc.shape = { type: "cuboid", hx, hy, hz, shapeType: 1 };
        return desc;
    }

    static cylinder(halfHeight, radius) {
        const desc = new ColliderDesc();
        desc.shape = { type: "cylinder", halfHeight, radius, shapeType: 10 };
        return desc;
    }

    static trimesh(vertices, indices, flags) {
        const desc = new ColliderDesc();
        desc.shape = { type: "trimesh", vertices, indices, flags, shapeType: 6 };
        return desc;
    }

    static convexHull(vertices, indices) {
        const desc = new ColliderDesc();
        desc.shape = { type: "convexHull", vertices, indices, shapeType: 9 };
        return desc;
    }

    setTranslation(x, y, z) {
        this.translation = new Vector3(x, y, z);
        return this;
    }

    setRotation(quaternion) {
        this.rotation = quaternion;
        return this;
    }

    setSensor(sensor) {
        this.sensor = sensor;
        return this;
    }
}

class JointData {
    static revolute(anchor1, anchor2, axis) {
        return { type: "revolute", anchor1, anchor2, axis };
    }

    static fixed(anchor1, frame1, anchor2, frame2) {
        return { type: "fixed", anchor1, anchor2, frame1, frame2 };
    }
}

const TriMeshFlags = {
    ORIENTED: 8,
    FIX_INTERNAL_EDGES: 144
};

export {
    World,
    RigidBodyDesc,
    ColliderDesc,
    JointData,
    TriMeshFlags
};