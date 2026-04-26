#pragma once

#include <functional>
#include <memory>

#include "CoreMinimal.h"
#include "BrainComponent.h"
#include "FSM.h"
#include "FSMComponent.generated.h"

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class GAMEAIPROG_API UFSMComponent : public UBrainComponent
{
	GENERATED_BODY()

public:
	using FTransitionPredicate = std::function<bool(const GameAI::FSM::Context &)>;

	// Sets default values for this component's properties
	UFSMComponent();
	virtual ~UFSMComponent() override;

	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, 
		FActorComponentTickFunction* ThisTickFunction) override;

	virtual void StartLogic() override;
	virtual void StopLogic(const FString& Reason) override;

	virtual bool IsRunning() const override;

	GameAI::FSM::State *AddState(std::unique_ptr<GameAI::FSM::State> &&NewState);
	void AddTransition(GameAI::FSM::State *From, GameAI::FSM::State *To, FTransitionPredicate EvalFunc) const;
	void AddTransition(GameAI::FSM::State *From, GameAI::FSM::State *To, std::function<bool()> EvalFunc) const;
	bool SetInitialState(GameAI::FSM::State *NewInitialState);
	GameAI::FSM::State *GetCurrentState() const;

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

private:
	GameAI::FSM::Context BuildContext(float DeltaTime) const;

	std::unique_ptr<GameAI::FSM::FSM> FSMInstance;
	bool bIsRunning{false};
};
