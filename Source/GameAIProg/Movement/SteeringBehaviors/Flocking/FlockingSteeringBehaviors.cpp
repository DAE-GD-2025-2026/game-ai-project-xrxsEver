#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"

//*******************
// COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, ASteeringAgent &pAgent)
{
	SteeringOutput Steering{};

	if (pFlock->GetNrOfNeighbors() > 0)
	{
		FVector2D CenterOfMass = pFlock->GetAverageNeighborPos();
		Steering.LinearVelocity = (CenterOfMass - pAgent.GetPosition()).GetSafeNormal() * pAgent.GetMaxLinearSpeed();
	}

	return Steering;
}

//*********************
// SEPARATION (FLOCKING)
SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent &pAgent)
{
	SteeringOutput Steering{};

	const int NrOfNeighbors = pFlock->GetNrOfNeighbors();
	if (NrOfNeighbors > 0)
	{
		FVector2D SeparationForce = FVector2D::ZeroVector;
		const TArray<ASteeringAgent *> &Neighbors = pFlock->GetNeighbors();

		for (int i = 0; i < NrOfNeighbors; ++i)
		{
			FVector2D ToAgent = pAgent.GetPosition() - Neighbors[i]->GetPosition();
			float Distance = ToAgent.Size();
			if (Distance > 0.f)
			{
				SeparationForce += ToAgent / (Distance * Distance);
			}
		}

		Steering.LinearVelocity = SeparationForce.GetSafeNormal() * pAgent.GetMaxLinearSpeed();
	}

	return Steering;
}

//*************************
// VELOCITY MATCH (FLOCKING)
SteeringOutput VelocityMatch::CalculateSteering(float deltaT, ASteeringAgent &pAgent)
{
	SteeringOutput Steering{};

	if (pFlock->GetNrOfNeighbors() > 0)
	{
		Steering.LinearVelocity = pFlock->GetAverageNeighborVelocity();
		Steering.LinearVelocity.Normalize();
		Steering.LinearVelocity *= pAgent.GetMaxLinearSpeed();
	}

	return Steering;
}

//*****************************
// CONDITIONAL EVADE (FLOCKING)
SteeringOutput ConditionalEvade::CalculateSteering(float deltaT, ASteeringAgent &pAgent)
{
	FVector2D ToTarget = Target.Position - pAgent.GetPosition();
	float DistanceSq = ToTarget.SizeSquared();

	if (DistanceSq > EvadeRadius * EvadeRadius)
	{
		SteeringOutput Steering{};
		Steering.IsValid = false;
		return Steering;
	}

	return Evade::CalculateSteering(deltaT, pAgent);
}
