// SimplecticIntegrator.cpp
//

#include <iostream>

struct Vector
{
private:
	float v[3];

public:
	Vector(float x, float y, float z)
	{
		v[0] = x;
		v[1] = y;
		v[2] = z;
	}

	Vector()
	{
		v[0] = 0;
		v[1] = 0;
		v[2] = 0;
	}

	Vector& operator += (const Vector& rhs)
	{
		v[0] += rhs.v[0];
		v[1] += rhs.v[1];
		v[2] += rhs.v[2];
		return *this;
	}

	Vector operator * (float rhs) const
	{
		return Vector(v[0] * rhs, v[1] * rhs, v[2] * rhs);
	}

	float Squared() const
	{
		return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
	}

	float Magnitude() const
	{
		return sqrtf(Squared());
	}

	float Y() const
	{
		return v[1];
	}
};

Vector operator * (float value, const Vector& rhs)
{
	return rhs * value;
}

Vector operator + (const Vector& lhs, const Vector& rhs)
{
	return Vector(lhs) += rhs;
}


class Rigidbody
{
	float rmass; // reciprocal of the mass
	Vector position;
	Vector velocity;
	Vector velocityChanges;
	Vector accelerations;

public:
	Rigidbody(float mass, const Vector &position, const Vector& u)
	{
		this->rmass = 1.f / mass;
		this->position = position;
		this->velocity = u;
	}

	void AddImpulse(const Vector& force)
	{
		velocityChanges += force * rmass;
	}

	void AddForce(const Vector& force)
	{
		accelerations += force * rmass;
	}

	float Mass() const
	{
		return 1.f / rmass;
	}

	float Height() const
	{
		return position.Y();
	}

	Vector Velocity() const
	{
		return velocity;
	}

	void Update(float dt)
	{
		velocity += velocityChanges;
		velocity += accelerations * dt;
		position += velocity * dt;
		velocityChanges = Vector();
		accelerations = Vector();
	}
};

class PhysicsTest
{
	Rigidbody* ball = NULL;
	Vector gravity;
	float dt;

public:
	PhysicsTest(float dt, const Vector& gravity) : dt(dt), gravity(gravity)
	{
	}

	float MechanicalEnergy(Vector velocity, float height, float mass)
	{
		float Pe = mass * gravity.Magnitude() * height;
		float Ke = 0.5f * mass * velocity.Squared();
		return Pe + Ke;
	}

	void CreateBall(float radius, float mass, Vector u)
	{
		ball = new Rigidbody(mass, Vector(0, radius, 0), u);
	}

	void HitBall(Vector v)
	{
		ball->AddImpulse(v);
	}

	float SimulateToFindPeak(float& Me)
	{
		float h = 0.0f;
		float peak = 0.0f;
		float baseline = ball->Height();
		Vector v = ball->Velocity();
		do
		{
			ball->AddForce(gravity * ball->Mass());
			ball->Update(dt);
			h = ball->Height() - baseline;
			v = ball->Velocity();
			peak = std::max(h, peak);
		} while (v.Y() > 0.0f);
		Me = MechanicalEnergy(v, peak, ball->Mass());
		return peak;
	}

	~PhysicsTest()
	{
		delete ball;
	}
};


void Experiment(bool correction, bool impulse)
{
	float tennisBallRadius = 0.068f; // 6.8cm
	float tennisBallMass = 0.057f; // 57g
	float netHeight = 0.941f; // 94.1cm
	int fps = 50;
	float dt = 1.f / fps;
	Vector gravity = Vector(0, -1, 0) * 9.81f;
	PhysicsTest test(dt, gravity);
	Vector u(0, sqrtf(-2.0f * gravity.Y() * netHeight), 0);
	float Me = test.MechanicalEnergy(u, 0.0f, tennisBallMass);
	std::cout.precision(2);

	Vector Eu;
	if (correction)
	{
		// calculate the required adjustment to the velocity to get it
		// to what it would have needed to be, half a time step ago,
		// so that it is correct velocity when applied by updateForces()
		Eu = -0.5f * gravity * dt;
	}

	if (impulse)
	{
		test.CreateBall(tennisBallRadius, tennisBallMass, Vector());
		test.HitBall((u + Eu) * tennisBallMass);
		std::cout << "Impulse energy is " << test.MechanicalEnergy(u, 0.0f, tennisBallMass) << " J" << std::endl;
	}
	else
	{
		std::cout << "Initial energy is " << test.MechanicalEnergy(u, 0.0f, tennisBallMass) << " J" << std::endl;
		test.CreateBall(tennisBallRadius, tennisBallMass, u + Eu);
	}

	float peak = test.SimulateToFindPeak(Me);
	std::cout << "Ball mechanical energy " << Me << " J at peak height of " << peak << " m" << std::endl;
	std::cout << std::endl;
}

int main()
{
	bool correction = true;
	bool impulse = true;
	std::cout << "Test with initial velocity..." << std::endl;
	Experiment(!correction, !impulse);
	std::cout << "Test with impulse..." << std::endl;
	Experiment(!correction, impulse);
	std::cout << "Apply correction to initial velocity..." << std::endl;
	Experiment(correction, !impulse);
	std::cout << "Apply correction to impulse..." << std::endl;
	Experiment(correction, impulse);
	return 0;
}
