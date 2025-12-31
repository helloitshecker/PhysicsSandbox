#pragma once

#include <raylib.h>
#include <print>
#include <vector>

constexpr double g = 988.0;

struct Vec2F {
    
    double x;
    double y;

    Vec2F operator+(const Vec2F& other) const {
        return { x + other.x, y + other.y };
    }

    Vec2F operator-(const Vec2F& other) const {
        return { x - other.x, y - other.y };
    }

    Vec2F operator*(double s) const {
        return { x * s, y * s };
    }

    Vec2F operator/(double s) const {
        return { x / s, y / s };
    }

    Vec2F& operator+=(const Vec2F& other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vec2F& operator-=(const Vec2F& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    Vec2F& operator*=(double s) {
        x *= s;
        y *= s;
        return *this;
    }

    Vec2F& operator/=(double s) {
        x /= s;
        y /= s;
        return *this;
    }

    double dot(const Vec2F& other) const {
        return x * other.x + y * other.y;
    }

    double length() const {
        return std::sqrt(x * x + y * y);
    }

    Vec2F normalized() const {
        double len = length();
        return (len > 0.0) ? (*this / len) : Vec2F{ 0.0, 0.0 };
    }

    void negate() {
        x = -x;
        y = -y;
    }

};

struct Vec2I {
	int x;
	int y;
};

// First making for circle
enum class Shape {
	CIRCLE,
	SQUARE,
	RECTANGLE,
};

struct ScreenState {
	Vec2I dimensions;
};

struct RigidBody {
	Vec2F position;
	Vec2F velocity;
	
	double radius;
	double mass;
	
	bool gravity;
	bool dynamic;

	Vec2F min() const {
		return {
			position.x - radius,
			position.y - radius
		};
	}

	Vec2F max() const {
		return {
			position.x + radius,
			position.y + radius
		};
	}

	Vec2F momentum() const {
		// p = m * v
		return { velocity.x * mass , velocity.y * mass };
	}
};

struct RigidBodyPair {
	RigidBody* a;
	RigidBody* b;
};

class PhysicsWorld {
public:

	std::vector<RigidBody> bodies;

	void addBody(const RigidBody& body);
	void tick(const double dt);
	void updatePosition(RigidBody& body, const double dt);

	std::vector<RigidBodyPair> broadphase() const;
	std::vector<RigidBodyPair> narrowphase(const std::vector<RigidBodyPair>& pairs) const;
	void reactToCollision(RigidBodyPair& pair);
	void reactToGravity(RigidBody& body, const double dt);
};