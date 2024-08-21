# Symplectic Integrator

This project contains (a stripped down) symplectic Euler-Cromer integrator used in PhysX. The core feature of this integrator is that it updates velocity values before updating the position value.

```cs
    velocity += acceleration * dt;
    position += velocity * dt;
```

It is this swapping of the order of these two instructions that turns the integrator from a first point approximation (FPA) to a last point approximation (LPA) method.[<sup>1</sup>](#1) It is also what turns it into a symplectic method.

The project also contains the standard Euler (FPA) method and a Leapfrog method.

It runs some experiments with each different integrator to test the conservation of energy in some common scenarios.

Measuring the mechanical energy at any time requires adjusting the velocity value stored in the integrator. All of these methods are implicitly leapfrog methods in that the velocity is offset in time relative to the position. The Euler method gives the velocity a half step in front, whereas the Euler-Cromer and Leapfrog methods give the velocity a half step behind.

The example code handles this by maintaining two variables `internalVelocity` and `velocity`. `internalVelocity` is the offset value. `velocity` is adjusted to give the velocity value as it would appear at the current time step.

This is necessary only for examining the current velocity, but it could be useful within simulations to provide a more accurate stamp for the current velocity than the value stored internally in the integrator.

The key difference with the Leapfrog method compared to the Euler-Cromer is how they handle the starting conditions. The Leapfrog method adds an extra step such that when a Rigidbody wakes, the integrator is started with the correct velocity. After the first update, the Euler-Cromer and Leapfrog implementations are the same.

## Results

```
Test initial velocity jump with Euler method...
Initial energy is 0.53 J
Ball mechanical energy 0.55 J at peak height of 0.98 m

Test impulse jump with Euler method...
Impulse energy is 0.53 J
Ball mechanical energy 0.55 J at peak height of 0.98 m
```

With the standard Euler method, you can see that the integrator gains a little bit of extra energy 0.02 J resulting in the ball overshooting the target by 4cm.

```
Test orbit with Euler method...
Aphelion error is 12 AU after 100 orbits
```

And in the case of the orbit simulation the aphelion rapidly increases indicating that it is gaining momentum over time.

The aphelion error is the aphelion of the simulated orbit compared to Earth's aphelion value of 1.0167 AU.

```
Test initial velocity jump with Euler-Cromer method...
Initial energy is 0.53 J
Ball mechanical energy 0.5 J at peak height of 0.9 m

Test impulse jump with Euler-Cromer method...
Impulse energy is 0.53 J
Ball mechanical energy 0.5 J at peak height of 0.9 m
```

Alternatively, the Euler-Cromer integrator loses a little energy 0.03 J resulting in the ball falling short by 4cm. These values match my measurements from the same program running with the PhysX library.

```
Test orbit with Euler-Cromer method...
Aphelion error is 0.036 AU after 100 orbits
```

However, the orbit simulation is stable and doesn't increase beyond 3% of the original orbit. (I tested this up to 10,000 orbits.)

```
Test initial velocity jump with Leapfrog method...
Initial energy is 0.53 J
Ball mechanical energy 0.53 J at peak height of 0.94 m

Test impulse jump with Leapfrog method...
Impulse energy is 0.53 J
Ball mechanical energy 0.53 J at peak height of 0.94 m
```

The leapfrog method maintains the correct energy throughout the jump and hits the target height with only a slight error due to the peak of the jump not landing on an exact physics frame update.

```
Test orbit with Leapfrog method...
Aphelion error is 0.0045 AU after 100 orbits
```

The orbit simulation is also stable, and the bound of the error is smaller than the Euler-Cromer method.

## References
<sup><a id="1">1</a></sup>
Alan Cromer; Stable solutions using the Euler approximation. Am. J. Phys. 1 May 1981; 49 (5): 455–459. https://doi.org/10.1119/1.12478
