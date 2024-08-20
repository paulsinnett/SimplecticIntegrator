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
	Vector accelerations;
	Vector velocityChanges;

public:
	Rigidbody(float mass, const Vector &position, const Vector& u)
	{
		this->rmass = 1.f / mass;
		this->position = position;
		this->velocityChanges = u;
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

	void EulerCromerUpdate(float dt)
	{
		// update accelerations
		velocity += accelerations * dt;
		accelerations = Vector();

		// update impulses
		velocity += velocityChanges;
		velocityChanges = Vector();

		// update position
		position += velocity * dt;
	}

	void LeapfrogUpdate(float dt)
	{
		velocity += 0.5f * accelerations * dt + velocityChanges;
		velocityChanges = Vector();

		position += velocity * dt;

		velocity += 0.5f * accelerations * dt;
		accelerations = Vector();
	}

	void FakeCollision()
	{
		if (Height() < 0.0f)
		{
			position = Vector();
			velocity = Vector();
		}
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

	float SimulateToFindPeak(bool leapfrog, float& Me)
	{
		float h = 0.0f;
		float peak = 0.0f;
		float baseline = ball->Height();
		Vector v = ball->Velocity();
		do
		{
			ball->AddForce(gravity * ball->Mass());
			if (leapfrog)
			{
				ball->LeapfrogUpdate(dt);
			}
			else
			{
				ball->EulerCromerUpdate(dt);
			}
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


void Experiment(bool leapfrog, bool impulse)
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

	if (impulse)
	{
		test.CreateBall(tennisBallRadius, tennisBallMass, Vector());
		test.HitBall(u * tennisBallMass);
		std::cout << "Impulse energy is " << Me << " J" << std::endl;
	}
	else
	{
		std::cout << "Initial energy is " << Me << " J" << std::endl;
		test.CreateBall(tennisBallRadius, tennisBallMass, u);
	}

	float peak = test.SimulateToFindPeak(leapfrog, Me);
	std::cout << "Ball mechanical energy " << Me << " J at peak height of " << peak << " m" << std::endl;
	std::cout << std::endl;
}

int main()
{
	bool impulse = true;
	bool leapfrog = true;
	std::cout << "Test Euler-Cromer with initial velocity..." << std::endl;
	Experiment(!leapfrog, !impulse);
	std::cout << "Test Euler-Cromer with impulse..." << std::endl;
	Experiment(!leapfrog, impulse);
	std::cout << "Test Leapfrog with initial velocity..." << std::endl;
	Experiment(leapfrog, !impulse);
	std::cout << "Test Leapfrog with impulse..." << std::endl;
	Experiment(leapfrog, impulse);
	return 0;
}
