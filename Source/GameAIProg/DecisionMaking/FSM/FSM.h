#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "CoreMinimal.h"

class AAIController;
class UBlackboardComponent;
class UObject;

namespace GameAI::FSM
{
    struct Context
    {
        UObject *Owner{nullptr};
        AAIController *Controller{nullptr};
        UBlackboardComponent *Blackboard{nullptr};
        float DeltaTime{0.0f};
    };

    class State
    {
    public:
        explicit State(FName InStateName = NAME_None);
        virtual ~State() = default;

        FName GetStateName() const;

        virtual void OnEnter(const Context &Context);
        virtual void Tick(const Context &Context);
        virtual void OnExit(const Context &Context);

    private:
        FName StateName;
    };

    class Transition final
    {
    public:
        using Predicate = std::function<bool(const Context &)>;

        Transition(State *InFrom, State *InTo, Predicate InPredicate);

        State *GetFromState() const;
        State *GetToState() const;
        bool CanTrigger(const Context &Context) const;

    private:
        State *FromState{nullptr};
        State *ToState{nullptr};
        Predicate EvalPredicate{};
    };

    class FSM final
    {
    public:
        State *AddState(std::unique_ptr<State> &&NewState);
        bool AddTransition(State *From, State *To, Transition::Predicate EvalPredicate);

        bool SetInitialState(State *NewInitialState);

        void SetContext(Context NewContext);

        void Start();
        void Stop();
        void Tick(float DeltaTime);

        bool IsRunning() const;
        State *GetCurrentState() const;

    private:
        bool HasState(const State *CandidateState) const;
        void TransitionTo(State *NewState);

    private:
        std::vector<std::unique_ptr<State>> States{};
        std::vector<Transition> Transitions{};

        Context RuntimeContext{};
        State *InitialState{nullptr};
        State *CurrentState{nullptr};
        bool bIsRunning{false};
    };
}