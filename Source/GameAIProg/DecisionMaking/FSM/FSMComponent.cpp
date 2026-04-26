#include "FSMComponent.h"
#include "FSM.h"
#include "AIController.h"

UFSMComponent::~UFSMComponent() = default;

// Sets default values for this component's properties
UFSMComponent::UFSMComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	FSMInstance = std::make_unique<GameAI::FSM::FSM>();
}

GameAI::FSM::State *UFSMComponent::AddState(std::unique_ptr<GameAI::FSM::State> &&NewState)
{
	if (!FSMInstance)
	{
		FSMInstance = std::make_unique<GameAI::FSM::FSM>();
	}

	return FSMInstance->AddState(MoveTemp(NewState));
}

void UFSMComponent::AddTransition(GameAI::FSM::State* From, GameAI::FSM::State* To, FTransitionPredicate EvalFunc) const
{
	if (!ensure(FSMInstance))
	{
		return;
	}

	const bool bAdded = FSMInstance->AddTransition(From, To, MoveTemp(EvalFunc));
	ensureMsgf(bAdded, TEXT("Failed to add FSM transition."));
}

void UFSMComponent::AddTransition(GameAI::FSM::State* From, GameAI::FSM::State* To, std::function<bool()> EvalFunc) const
{
	AddTransition(From, To,
				  [Predicate = MoveTemp(EvalFunc)](const GameAI::FSM::Context &)
				  {
					  return Predicate && Predicate();
				  });
}

bool UFSMComponent::SetInitialState(GameAI::FSM::State *NewInitialState)
{
	if (!FSMInstance)
	{
		return false;
	}

	return FSMInstance->SetInitialState(NewInitialState);
}

// Called when the game starts
void UFSMComponent::BeginPlay()
{
	Super::BeginPlay();

	if (FSMInstance)
	{
		FSMInstance->SetContext(BuildContext(0.0f));
	}
}

// Called every frame
void UFSMComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!FSMInstance || !bIsRunning)
	{
		return;
	}

	FSMInstance->SetContext(BuildContext(DeltaTime));
	FSMInstance->Tick(DeltaTime);
	bIsRunning = FSMInstance->IsRunning();
}

void UFSMComponent::StartLogic()
{
	Super::StartLogic();

	if (!FSMInstance)
	{
		FSMInstance = std::make_unique<GameAI::FSM::FSM>();
	}

	FSMInstance->SetContext(BuildContext(0.0f));
	FSMInstance->Start();
	bIsRunning = FSMInstance->IsRunning();
}

void UFSMComponent::StopLogic(const FString& Reason)
{
	Super::StopLogic(Reason);

	if (FSMInstance)
	{
		FSMInstance->SetContext(BuildContext(0.0f));
		FSMInstance->Stop();
	}

	bIsRunning = false;
}

bool UFSMComponent::IsRunning() const
{
	return bIsRunning;
}

GameAI::FSM::State *UFSMComponent::GetCurrentState() const
{
	if (!FSMInstance)
	{
		return nullptr;
	}

	return FSMInstance->GetCurrentState();
}

GameAI::FSM::Context UFSMComponent::BuildContext(float DeltaTime) const
{
	GameAI::FSM::Context Context{};
	Context.Owner = GetOwner();
	Context.DeltaTime = DeltaTime;

	AAIController *AIController = Cast<AAIController>(GetOwner());
	Context.Controller = AIController;
	Context.Blackboard = AIController ? AIController->GetBlackboardComponent() : nullptr;

	return Context;
}
