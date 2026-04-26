#include "FSM.h"

#include <algorithm>

namespace GameAI::FSM
{
    State::State(FName InStateName)
        : StateName(InStateName)
    {
    }

    FName State::GetStateName() const
    {
        return StateName;
    }

    void State::OnEnter(const Context &)
    {
    }

    void State::Tick(const Context &)
    {
    }

    void State::OnExit(const Context &)
    {
    }

    Transition::Transition(State *InFrom, State *InTo, Predicate InPredicate)
        : FromState(InFrom), ToState(InTo), EvalPredicate(MoveTemp(InPredicate))
    {
    }

    State *Transition::GetFromState() const
    {
        return FromState;
    }

    State *Transition::GetToState() const
    {
        return ToState;
    }

    bool Transition::CanTrigger(const Context &Context) const
    {
        return ToState && EvalPredicate && EvalPredicate(Context);
    }

    State *FSM::AddState(std::unique_ptr<State> &&NewState)
    {
        if (!NewState)
        {
            return nullptr;
        }

        State *AddedState = NewState.get();
        States.emplace_back(MoveTemp(NewState));

        if (!InitialState)
        {
            InitialState = AddedState;
        }

        return AddedState;
    }

    bool FSM::AddTransition(State *From, State *To, Transition::Predicate EvalPredicate)
    {
        if (!To || !EvalPredicate)
        {
            return false;
        }

        if ((From && !HasState(From)) || !HasState(To))
        {
            return false;
        }

        Transitions.emplace_back(From, To, MoveTemp(EvalPredicate));
        return true;
    }

    bool FSM::SetInitialState(State *NewInitialState)
    {
        if (!HasState(NewInitialState))
        {
            return false;
        }

        InitialState = NewInitialState;
        return true;
    }

    void FSM::SetContext(Context NewContext)
    {
        RuntimeContext = NewContext;
    }

    void FSM::Start()
    {
        if (bIsRunning)
        {
            return;
        }

        if (!InitialState && !States.empty())
        {
            InitialState = States.front().get();
        }

        CurrentState = InitialState;
        if (!CurrentState)
        {
            return;
        }

        bIsRunning = true;
        CurrentState->OnEnter(RuntimeContext);
    }

    void FSM::Stop()
    {
        if (!bIsRunning)
        {
            return;
        }

        if (CurrentState)
        {
            CurrentState->OnExit(RuntimeContext);
        }

        CurrentState = nullptr;
        bIsRunning = false;
    }

    void FSM::Tick(float DeltaTime)
    {
        if (!bIsRunning || !CurrentState)
        {
            return;
        }

        RuntimeContext.DeltaTime = DeltaTime;

        for (const Transition &Transition : Transitions)
        {
            if (Transition.GetFromState() && Transition.GetFromState() != CurrentState)
            {
                continue;
            }

            if (!Transition.CanTrigger(RuntimeContext))
            {
                continue;
            }

            if (State *NextState = Transition.GetToState();
                NextState && NextState != CurrentState)
            {
                TransitionTo(NextState);
            }

            break;
        }

        if (CurrentState)
        {
            CurrentState->Tick(RuntimeContext);
        }
    }

    bool FSM::IsRunning() const
    {
        return bIsRunning;
    }

    State *FSM::GetCurrentState() const
    {
        return CurrentState;
    }

    bool FSM::HasState(const State *CandidateState) const
    {
        if (!CandidateState)
        {
            return false;
        }

        return std::any_of(States.begin(), States.end(),
                           [CandidateState](const std::unique_ptr<State> &ExistingState)
                           {
                               return ExistingState.get() == CandidateState;
                           });
    }

    void FSM::TransitionTo(State *NewState)
    {
        if (!NewState || NewState == CurrentState)
        {
            return;
        }

        if (CurrentState)
        {
            CurrentState->OnExit(RuntimeContext);
        }

        CurrentState = NewState;
        CurrentState->OnEnter(RuntimeContext);
    }
}