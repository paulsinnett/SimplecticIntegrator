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

	Vector operator -= (const Vector& rhs)
	{
		v[0] -= rhs.v[0];
		v[1] -= rhs.v[1];
		v[2] -= rhs.v[2];
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

Vector operator - (const Vector& lhs, const Vector& rhs)
{
	return Vector(lhs) -= rhs;
}

enum UpdateMethod
{
	Euler,
	EulerCromer,
	Leapfrog
};

class Rigidbody
{
	float rmass; // reciprocal of the mass
	Vector position;
	Vector internalVelocity;
	Vector velocity;
	Vector accelerations;
	Vector velocityChanges;
	UpdateMethod method;
	bool asleep = true;
	bool wake = true;

public:
	Rigidbody(float mass, const Vector &position, const Vector& u, UpdateMethod method): method(method)
	{
		this->rmass = 1.f / mass;
		this->position = position;
		this->internalVelocity = u;
		this->velocity = u;
	}

	void WakeUp()
	{
		wake = true;
	}

	void AddImpulse(const Vector& force)
	{
		velocityChanges += force * rmass;
	}

	void AddForce(const Vector& force)
	{
		accelerations += force * rmass;
	}

	Vector VectorTo(const Rigidbody& other) const
	{
		return other.position - position;
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
		switch (method)
		{
		case Euler:
			EulerUpdate(dt);
			break;
		case EulerCromer:
			EulerCromerUpdate(dt);
			break;
		case Leapfrog:
			LeapfrogUpdate(dt);
			break;
		}
		FakeCollision();
	}

	void EulerUpdate(float dt)
	{
		if (asleep)
		{
			if (wake)
			{
				asleep = false;
				wake = false;
			}
			else
			{
				return;
			}
		}

		// update impulses
		internalVelocity += velocityChanges;

		// update position
		position += internalVelocity * dt;

		// update accelerations
		internalVelocity += accelerations * dt;

		// update velocity (for reporting the current velocity at this time step)
		velocity = internalVelocity - 0.5f * accelerations * dt;

		// clear stored impulses and accelerations
		accelerations = Vector();
		velocityChanges = Vector();
	}

	void EulerCromerUpdate(float dt)
	{
		if (asleep)
		{
			if (wake)
			{
				asleep = false;
				wake = false;
			}
			else
			{
				return;
			}
		}


		// update impulses
		internalVelocity += velocityChanges;

		// update accelerations
		internalVelocity += accelerations * dt;

		// update velocity (for reporting the current velocity at this time step)
		velocity = internalVelocity + 0.5f * accelerations * dt;

		// update position
		position += internalVelocity * dt;

		// clear stored impulses and accelerations
		velocityChanges = Vector();
		accelerations = Vector();
	}

	void LeapfrogUpdate(float dt)
	{
		float vstep = 0.5f;
		if (asleep)
		{
			if (wake)
			{
				asleep = false;
				wake = false;
				vstep = 0.f;
			}
			else
			{
				return;
			}
		}

		// update impulses
		internalVelocity += velocityChanges;

		// update accelerations
		internalVelocity += vstep * accelerations * dt;
		internalVelocity += 0.5f * accelerations * dt;
		
		// update velocity (for reporting the current velocity at this time step)
		velocity = internalVelocity + 0.5f * accelerations * dt;

		// update position
		position += internalVelocity * dt;

		// clear stored impulses and accelerations
		velocityChanges = Vector();
		accelerations = Vector();
	}

	void FakeCollision()
	{
		if (Height() < 0.0f)
		{
			position = Vector();
			internalVelocity = Vector();
			velocity = Vector();
			asleep = true;
		}
	}
};

class PhysicsTest
{
	Rigidbody* ball = NULL;
	Rigidbody* Earth = NULL;
	Rigidbody* Sun = NULL;
	Vector gravity;
	float dt;
	UpdateMethod method;

public:
	PhysicsTest(float dt, const Vector& gravity, UpdateMethod method) : dt(dt), gravity(gravity), method(method)
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
		ball = new Rigidbody(mass, Vector(0, radius, 0), u, method);
	}

	void CreateBodies()
	{
		// mass in solar masses
		// velocities in AU / year
		Earth = new Rigidbody(3.003353e-06f, Vector(0.9832924f, 0, 0), Vector(0, 0, 6.38966f), method);
		// The sun gets an initial velocity to counter the Earth's initial velocity
		Sun = new Rigidbody(1.f, Vector(0, 0, 0), Vector(0, 0, -1.91904e-5f), method);
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
		Vector v;
		do
		{
			ball->AddForce(gravity * ball->Mass());
			ball->Update(dt);
			h = ball->Height() - baseline;
			v = ball->Velocity();
			peak = std::max(h, peak);
			Me = MechanicalEnergy(v, peak, ball->Mass());
		} while (v.Y() > 0.0f);
		Me = MechanicalEnergy(v, peak, ball->Mass());
		return peak;
	}

	float MeasureAphelionDrift(float orbits)
	{
		float aphelion = Earth->VectorTo(*Sun).Magnitude();
		do
		{
			// G in AU^3 / (SolarMasses * year^2)
			float G = 39.478716f;
			float F = G * Sun->Mass() * Earth->Mass() / Earth->VectorTo(*Sun).Squared();
			Earth->AddForce(F * Earth->VectorTo(*Sun));
			Sun->AddForce(F * Sun->VectorTo(*Earth));
			Earth->Update(dt);
			Sun->Update(dt);
			orbits -= dt;
			aphelion = std::max(aphelion, Earth->VectorTo(*Sun).Magnitude());
		}
		while (orbits > 0);

		return aphelion;
	}

	~PhysicsTest()
	{
		delete ball;
		delete Earth;
		delete Sun;
	}
};

const char* MethodName(UpdateMethod method)
{
	switch (method)
	{
	case Euler:
		return "Euler";
	case EulerCromer:
		return "Euler-Cromer";
	case Leapfrog:
		return "Leapfrog";
	}
	return "";
}

void InitialVelocityJumpHeightExperiment(UpdateMethod method)
{
	std::cout << "Test initial velocity jump with " << MethodName(method) << " method..." << std::endl;
	float tennisBallRadius = 0.068f; // 6.8cm
	float tennisBallMass = 0.057f; // 57g
	float netHeight = 0.941f; // 94.1cm
	int fps = 50;
	float dt = 1.f / fps;
	Vector gravity = Vector(0, -1, 0) * 9.81f;
	PhysicsTest test(dt, gravity, method);
	Vector u(0, sqrtf(-2.0f * gravity.Y() * netHeight), 0);
	float Me = test.MechanicalEnergy(u, 0.0f, tennisBallMass);
	std::cout.precision(2);

	std::cout << "Initial energy is " << Me << " J" << std::endl;
	test.CreateBall(tennisBallRadius, tennisBallMass, u);

	float peak = test.SimulateToFindPeak(Me);
	std::cout << "Ball mechanical energy " << Me << " J at peak height of " << peak << " m" << std::endl;
	std::cout << std::endl;
}

void ImpulseJumpHeightExperiment(UpdateMethod method)
{
	std::cout << "Test impulse jump with " << MethodName(method) << " method..." << std::endl;
	float tennisBallRadius = 0.068f; // 6.8cm
	float tennisBallMass = 0.057f; // 57g
	float netHeight = 0.941f; // 94.1cm
	int fps = 50;
	float dt = 1.f / fps;
	Vector gravity = Vector(0, -1, 0) * 9.81f;
	PhysicsTest test(dt, gravity, method);
	Vector u(0, sqrtf(-2.0f * gravity.Y() * netHeight), 0);
	float Me = test.MechanicalEnergy(u, 0.0f, tennisBallMass);
	std::cout.precision(2);

	test.CreateBall(tennisBallRadius, tennisBallMass, Vector());
	test.HitBall(u * tennisBallMass);
	std::cout << "Impulse energy is " << Me << " J" << std::endl;

	float peak = test.SimulateToFindPeak(Me);
	std::cout << "Ball mechanical energy " << Me << " J at peak height of " << peak << " m" << std::endl;
	std::cout << std::endl;
}

void OrbitExperiment(UpdateMethod method)
{
	std::cout << "Test orbit with " << MethodName(method) << " method..." << std::endl;
	std::cout.precision(2);
	int fpy = 50;
	float dt = 1.f / fpy;
	PhysicsTest test(dt, Vector(), method);
	test.CreateBodies();
	int orbits = 100;
	float aphelionError = std::fabsf(test.MeasureAphelionDrift((float)orbits) - 1.0167f);
	std::cout << "Aphelion error is " << aphelionError << " AU after " << orbits << " orbits" << std::endl;
	std::cout << std::endl;
}

int main()
{
	UpdateMethod methods[] = { Euler, EulerCromer, Leapfrog };
	for (int i = 0; i < 3; i++)
	{
		InitialVelocityJumpHeightExperiment(methods[i]);
		ImpulseJumpHeightExperiment(methods[i]);
		OrbitExperiment(methods[i]);
	}

	return 0;
}
