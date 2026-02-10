#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//SEEK
//*******
// TODO: Do the Week01 assignment :^)

SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	return Steering;
};

SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	FVector2D Direction = Agent.GetPosition() - Target.Position;
	Steering.LinearVelocity = Direction.GetSafeNormal() * Agent.GetMaxLinearSpeed();
	return Steering;
};

SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	const FVector2D ToTarget = Target.Position - Agent.GetPosition();
	const float Distance = ToTarget.Size();

	// Settings 
	constexpr float TargetRadius = 10.f;  
	constexpr float SlowRadius = 600.f;   
	
	if (Distance < TargetRadius)
	{
		Steering.LinearVelocity = FVector2D::ZeroVector;
		return Steering;
	}

	float TargetSpeed = Agent.GetMaxLinearSpeed();
	if (Distance < SlowRadius)
	{
		TargetSpeed *= (Distance / SlowRadius);
	}
	
	Steering.LinearVelocity = ToTarget.GetSafeNormal() * TargetSpeed;
	return Steering;
};
