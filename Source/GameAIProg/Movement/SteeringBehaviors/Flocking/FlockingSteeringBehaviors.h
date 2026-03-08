#pragma once
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
class Flock;

// COHESION - FLOCKING
//*******************
class Cohesion final : public Seek
{
public:
	Cohesion(Flock *const pFlock) : pFlock(pFlock) {};

	// Cohesion Behavior
	SteeringOutput CalculateSteering(float deltaT, ASteeringAgent &pAgent) override;

private:
	Flock *pFlock = nullptr;
};

// SEPARATION - FLOCKING
//*********************
class Separation final : public ISteeringBehavior
{
public:
	Separation(Flock *const pFlock) : pFlock(pFlock) {};

	SteeringOutput CalculateSteering(float deltaT, ASteeringAgent &pAgent) override;

private:
	Flock *pFlock = nullptr;
};

// VELOCITY MATCH - FLOCKING
//************************
class VelocityMatch final : public ISteeringBehavior
{
public:
	VelocityMatch(Flock *const pFlock) : pFlock(pFlock) {};

	SteeringOutput CalculateSteering(float deltaT, ASteeringAgent &pAgent) override;

private:
	Flock *pFlock = nullptr;
};

// CONDITIONAL EVADE - FLOCKING (evade only within radius)
//********************************************************
class ConditionalEvade final : public Evade
{
public:
	ConditionalEvade() = default;

	SteeringOutput CalculateSteering(float deltaT, ASteeringAgent &pAgent) override;

	void SetEvadeRadius(float radius) { EvadeRadius = radius; }

private:
	float EvadeRadius = 400.f;
};
