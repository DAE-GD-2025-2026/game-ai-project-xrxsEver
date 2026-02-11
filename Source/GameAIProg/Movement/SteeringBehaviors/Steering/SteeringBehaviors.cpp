#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "DrawDebugHelpers.h" 
//SEEK
//*******
// TODO: Do the Week01 assignment :^)

SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	FVector2D Direction = Target.Position - Agent.GetPosition();
	Steering.LinearVelocity = Direction.GetSafeNormal() * Agent.GetMaxLinearSpeed();
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
	if (m_OriginalMaxSpeed < 0.f)
	{
		m_OriginalMaxSpeed = Agent.GetMaxLinearSpeed();
	}
	// Parameters
	const float SlowRadius = 800.f;
	const float TargetRadius = 50.f;
	// --- DEBUG DRAWING ---
	if (UWorld* World = Agent.GetWorld())
	{
		DrawDebugSphere(World, FVector(Target.Position, 0.f), SlowRadius, 16, FColor::Green, false, -1.f, 0, 2.f);
		DrawDebugSphere(World, FVector(Target.Position, 0.f), TargetRadius, 16, FColor::Red, false, -1.f, 0, 2.f);
	}
	// --- CALCULATIONS ---
	FVector2D ToTarget = Target.Position - Agent.GetPosition();
	float Distance = ToTarget.Size();
	// Default to the original full speed
	float DesiredSpeed = m_OriginalMaxSpeed;
	// Stop if inside TargetRadius
	if (Distance < TargetRadius)
	{
		DesiredSpeed = 0.f;
		// We can early exit, but we should set the agent speed to 0 first to be consistent
		Agent.SetMaxLinearSpeed(DesiredSpeed); 
		Steering.LinearVelocity = FVector2D::ZeroVector;
		return Steering;
	}
	// "Gradually decrease speed"
	if (Distance < SlowRadius)
	{
		DesiredSpeed = m_OriginalMaxSpeed * (Distance / SlowRadius);
	}
	Agent.SetMaxLinearSpeed(DesiredSpeed);
	Steering.LinearVelocity = ToTarget.GetSafeNormal() * Agent.GetMaxLinearSpeed();
	return Steering;
};


SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering{};
    FVector2D Direction = Target.Position - Agent.GetPosition();
    // Safety check
    if (Direction.IsNearlyZero())
        return Steering;

    // --- VISUAL DEBUGGING ---
    if (UWorld* World = Agent.GetWorld())
    {
        DrawDebugLine(World, FVector(Agent.GetPosition(), 0), FVector(Target.Position, 0), FColor::Yellow, false, -1, 0, 2.0f);
        float CurrentRad = FMath::DegreesToRadians(Agent.GetRotation());
        FVector FacingDir = FVector(FMath::Cos(CurrentRad), FMath::Sin(CurrentRad), 0);
        DrawDebugLine(World, FVector(Agent.GetPosition(), 0), FVector(Agent.GetPosition(), 0) + (FacingDir * 100.f), FColor::Red, false, -1, 0, 2.0f);
    }
    // ------------------------
    // [IMPORTANT] Atan2 returns Radians. We MUST convert to Degrees.
    float TargetAngle = FMath::RadiansToDegrees(FMath::Atan2(Direction.Y, Direction.X));
    float CurrentAngle = Agent.GetRotation();
    float AngleDiff = TargetAngle - CurrentAngle;
    while (AngleDiff > 180.f)  AngleDiff -= 360.f;
    while (AngleDiff < -180.f) AngleDiff += 360.f;
    float MaxSpeed = Agent.GetMaxAngularSpeed();
    if (FMath::Abs(AngleDiff) < 2.0f) // 2 Degree Deadzone
    {
        Steering.AngularVelocity = 0.f;
    }
    else
    {
        Steering.AngularVelocity = (AngleDiff > 0) ? MaxSpeed : -MaxSpeed;
        
        float SlowAngle = 30.f; // Start slowing down at 30 degrees difference
        if (FMath::Abs(AngleDiff) < SlowAngle)
           Steering.AngularVelocity *= (FMath::Abs(AngleDiff) / SlowAngle);
    }

    // Face behavior strictly uses Angular Velocity, Linear must be zero
    Steering.LinearVelocity = FVector2D::ZeroVector;

    return Steering;
};

SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	return Steering;
};

SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	return Steering;
};

SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	return Steering;
};