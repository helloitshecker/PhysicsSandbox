#include "Physics.hpp"
#include <cmath>
#include <print>

static bool overlap(const RigidBody& a, const RigidBody& b) {
    const Vec2F amax = a.max();
    const Vec2F amin = a.min();
    const Vec2F bmax = b.max();
    const Vec2F bmin = b.min();

    return !(
        amax.x < bmin.x ||
        amin.x > bmax.x ||
        amax.y < bmin.y ||
        amin.y > bmax.y
        );
}

static bool collide(const RigidBody& a, const RigidBody& b) {
    Vec2F d = a.position - b.position;
    double r = a.radius + b.radius;
    return (d.x * d.x + d.y * d.y) <= (r * r);
}

void PhysicsWorld::addBody(const RigidBody& body) {
    bodies.push_back(body);

//#ifndef NDEBUG
//    std::println("DEBUG: Added rigidbody | total = {}", bodies.size());
//#endif
}

void PhysicsWorld::tick(double dt) {
    for (auto& body : bodies) {
        reactToGravity(body, dt);
    }

    for (auto& body : bodies) {
        updatePosition(body, dt);
    }

    auto broad = broadphase();
    auto collisions = narrowphase(broad);

    constexpr int SOLVER_ITERS = 5;

    for (int i = 0; i < SOLVER_ITERS; i++) {
        for (auto& pair : collisions) {
            reactToCollision(pair);
        }
    }

}

std::vector<RigidBodyPair> PhysicsWorld::broadphase() const {
    std::vector<RigidBodyPair> possibilities;

    for (size_t i = 0; i < bodies.size(); i++) {
        for (size_t j = i + 1; j < bodies.size(); j++) {
            if (overlap(bodies[i], bodies[j])) {
                possibilities.push_back({
                    (RigidBody*) & bodies[i],
                    (RigidBody*) & bodies[j]
                });
            }
        }
    }

    return possibilities;
}

std::vector<RigidBodyPair>
PhysicsWorld::narrowphase(const std::vector<RigidBodyPair>& pairs) const {
    std::vector<RigidBodyPair> collisions;

    for (const auto& pair : pairs) {
        if (collide(*pair.a, *pair.b)) {
            collisions.push_back(pair);
        }
    }

    return collisions;
}

void PhysicsWorld::reactToCollision(RigidBodyPair& pair) {
    RigidBody& a = *pair.a;
    RigidBody& b = *pair.b;

    if (!a.dynamic && !b.dynamic)
        return;

    Vec2F delta = b.position - a.position;
    double dist = delta.length();
    
    if (dist == 0.0)
        return;

    Vec2F normal = delta / dist;
    double penetration = (a.radius + b.radius) - dist;

    if (penetration > 0.0) {
        if (a.dynamic && b.dynamic) {
            a.position -= normal * (penetration * 0.5);
            b.position += normal * (penetration * 0.5);
        }
        else if (a.dynamic) {
            a.position -= normal * penetration;
        }
        else if (b.dynamic) {
            b.position += normal * penetration;
        }
    }

    Vec2F rv = b.velocity - a.velocity;
    double velAlongNormal = rv.dot(normal);

    if (velAlongNormal > 0.0)
        return;

    constexpr double restitution = 0.5;

    double invMassA = a.dynamic ? 1.0 / a.mass : 0.0;
    double invMassB = b.dynamic ? 1.0 / b.mass : 0.0;

    double j = -(1.0 + restitution) * velAlongNormal;
    j /= (invMassA + invMassB);

    Vec2F impulse = { j * normal.x, j * normal.y };

    if (a.dynamic)
        a.velocity -= impulse * invMassA;
    if (b.dynamic)
        b.velocity += impulse * invMassB;
}

void PhysicsWorld::reactToGravity(RigidBody& body, double dt) {
    if (!body.dynamic || !body.gravity)
        return;

    body.velocity.y += g * dt;
}

void PhysicsWorld::updatePosition(RigidBody& body, double dt) {
    if (!body.dynamic)
        return;

    body.position += body.velocity * dt;
}
