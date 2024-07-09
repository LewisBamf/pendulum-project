#include <cmath>
#include <SFML/Graphics.hpp>
#include <vector>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const float g = 980.0f; // Acceleration due to gravity

class Vector2 {
public:
    float x;
    float y;

    Vector2() : x(0), y(0) {}
    Vector2(float x, float y) : x(x), y(y) {}

    Vector2 operator+(const Vector2& other) const {
        return Vector2(x + other.x, y + other.y);
    }

    Vector2 operator-(const Vector2& other) const {
        return Vector2(x - other.x, y - other.y);
    }

    Vector2 operator*(float scalar) const {
        return Vector2(x * scalar, y * scalar);
    }

    Vector2 operator/(float scalar) const {
        return Vector2(x / scalar, y / scalar);
    }

    float length() const {
        return std::sqrt(x * x + y * y);
    }

    Vector2 normalize() const {
        float len = length();
        if (len > 0) {
            return Vector2(x / len, y / len);
        }
        return *this;
    }

    float dot(const Vector2& other) const {
        return x * other.x + y * other.y;
    }
};

// Pendulum class
class Pendulum {
public:
    float length;
    float mass;
    float angle;
    float angularVelocity;
    float angularAcceleration;
    sf::CircleShape bob;
    sf::RectangleShape line;
    sf::Vector2f pivot;

    Pendulum(float length, float mass, float angle, float initialAngularVelocity = 0)
        : length(length), mass(mass), angle(angle), angularVelocity(initialAngularVelocity), angularAcceleration(0) {
        bob.setRadius(10);
        bob.setFillColor(sf::Color::Red);
        bob.setOrigin(10, 10);
        line.setFillColor(sf::Color::Black);
    }

    void updatePhysics(float dt) {
        // Implementation depends on the coupled dynamics for double pendulum
    }

    void draw(sf::RenderWindow& window) {
        window.draw(line);
        window.draw(bob);
    }
};

class DoublePendulum {
private:
    Pendulum p1;
    Pendulum p2;

public:
    DoublePendulum(float length1, float mass1, float angle1, float initialAngularVelocity1,
        float length2, float mass2, float angle2, float initialAngularVelocity2)
        : p1(length1, mass1, angle1, initialAngularVelocity1), p2(length2, mass2, angle2, initialAngularVelocity2) {
        p1.pivot = sf::Vector2f(500, 300);
    }

    void updatePhysics(float dt) {
        float m1 = p1.mass;
        float m2 = p2.mass;
        float l1 = p1.length;
        float l2 = p2.length;
        float a1 = p1.angle;
        float a2 = p2.angle;
        float a1_v = p1.angularVelocity;
        float a2_v = p2.angularVelocity;

        // Calculate angular accelerations using the equations of motion for a double pendulum
        float num1 = -g * (2 * m1 + m2) * std::sin(a1);
        float num2 = -m2 * g * std::sin(a1 - 2 * a2);
        float num3 = -2 * std::sin(a1 - a2) * m2;
        float num4 = a2_v * a2_v * l2 + a1_v * a1_v * l1 * std::cos(a1 - a2);
        float den = l1 * (2 * m1 + m2 - m2 * std::cos(2 * a1 - 2 * a2));
        float a1_a = (num1 + num2 + num3 * num4) / den;

        num1 = 2 * std::sin(a1 - a2);
        num2 = a1_v * a1_v * l1 * (m1 + m2);
        num3 = g * (m1 + m2) * std::cos(a1);
        num4 = a2_v * a2_v * l2 * m2 * std::cos(a1 - a2);
        den = l2 * (2 * m1 + m2 - m2 * std::cos(2 * a1 - 2 * a2));
        float a2_a = (num1 * (num2 + num3 + num4)) / den;

        // Update angular velocities and angles
        p1.angularVelocity += a1_a * dt;
        p1.angle += p1.angularVelocity * dt;

        p2.angularVelocity += a2_a * dt;
        p2.angle += p2.angularVelocity * dt;

        // Damping (optional)
        p1.angularVelocity *= 0.9999f;
        p2.angularVelocity *= 0.9999f;

        // Update positions
        float x1 = p1.length * std::sin(p1.angle);
        float y1 = p1.length * std::cos(p1.angle);
        p1.bob.setPosition(p1.pivot.x + x1, p1.pivot.y + y1);

        float x2 = x1 + p2.length * std::sin(p2.angle);
        float y2 = y1 + p2.length * std::cos(p2.angle);
        p2.bob.setPosition(p1.pivot.x + x2, p1.pivot.y + y2);

        // Update lines
        sf::Vector2f bobPosition1 = p1.bob.getPosition();
        sf::Vector2f lineVec1 = bobPosition1 - p1.pivot;
        float lineLength1 = std::sqrt(lineVec1.x * lineVec1.x + lineVec1.y * lineVec1.y);
        float lineAngle1 = std::atan2(lineVec1.y, lineVec1.x) * 180 / M_PI;
        p1.line.setSize(sf::Vector2f(lineLength1, 5)); // Set the thickness of the line here (5 pixels)
        p1.line.setOrigin(0, 2.5); // Set origin to middle of the thickness
        p1.line.setPosition(p1.pivot);
        p1.line.setRotation(lineAngle1);

        sf::Vector2f bobPosition2 = p2.bob.getPosition();
        sf::Vector2f lineVec2 = bobPosition2 - bobPosition1;
        float lineLength2 = std::sqrt(lineVec2.x * lineVec2.x + lineVec2.y * lineVec2.y);
        float lineAngle2 = std::atan2(lineVec2.y, lineVec2.x) * 180 / M_PI;
        p2.line.setSize(sf::Vector2f(lineLength2, 5)); // Set the thickness of the line here (5 pixels)
        p2.line.setOrigin(0, 2.5); // Set origin to middle of the thickness
        p2.line.setPosition(bobPosition1);
        p2.line.setRotation(lineAngle2);
    }

    void draw(sf::RenderWindow& window) {
        p1.draw(window);
        p2.draw(window);
    }
};

int main() {
    sf::RenderWindow window(sf::VideoMode(1000, 600), "Double Pendulum Simulation");

    const float length1 = 140;
    const float mass1 = 10;
    const float angle1 = M_PI / 2;
    const float initialAngularVelocity1 = 4.0f; // Set initial angular velocity for the first pendulum

    const float length2 = 100;
    const float mass2 = 10;
    const float angle2 = M_PI / 6;
    const float initialAngularVelocity2 = 4.0f; // Set initial angular velocity for the second pendulum

    DoublePendulum doublePendulum(length1, mass1, angle1, initialAngularVelocity1, length2, mass2, angle2, initialAngularVelocity2);

    sf::Clock displayClock;
    sf::Clock physicsClock;
    float physicsDt = 1.0f / 240.0f;
    float physicsAccumulator = 0.0f;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        float frameTime = displayClock.restart().asSeconds();
        physicsAccumulator += frameTime;

        // Physics update loop
        while (physicsAccumulator >= physicsDt) {
            doublePendulum.updatePhysics(physicsDt);
            physicsAccumulator -= physicsDt;
        }

        // Clear window
        window.clear(sf::Color::White);

        // Draw double pendulum
        doublePendulum.draw(window);

        // Display frame
        window.display();

        // Ensure consistent frame rate
        sf::sleep(sf::seconds(1.0f / 60.0f) - displayClock.getElapsedTime());
    }

    return 0;
}