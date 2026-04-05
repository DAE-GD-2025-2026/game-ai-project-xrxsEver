#pragma once
#include <stack>
#include "Shared/Graph/Graph.h"

namespace GameAI
{
	enum class Eulerianity
	{
		notEulerian,
		semiEulerian,
		eulerian,
	};

	class EulerianPath final
	{
	public:
		EulerianPath(Graph *const pGraph);

		Eulerianity IsEulerian() const;
		std::vector<Node *> FindPath(Eulerianity &eulerianity) const;

	private:
		void VisitAllNodesDFS(const std::vector<Node *> &pNodes, std::vector<bool> &visited, int startIndex) const;
		bool IsConnected() const;

		Graph *m_pGraph;
	};

	inline EulerianPath::EulerianPath(Graph *const pGraph)
		: m_pGraph(pGraph)
	{
	}

	inline Eulerianity EulerianPath::IsEulerian() const
	{
		// If the graph is not connected no Eulerian Trail
		if (!IsConnected())
			return Eulerianity::notEulerian;

		// Count nodes with odd degree
		std::vector<Node *> Nodes = m_pGraph->GetActiveNodes();
		int oddCount = 0;
		for (auto *node : Nodes)
		{

			int degree = static_cast<int>(m_pGraph->FindConnectionsWith(node->GetId()).size());
			if (degree % 2 != 0)
				oddCount++;

		}

		// A graph with more than 2 nodes with an odd degree is not Eulerian !!!!!!!!!!!!!

		if (oddCount > 2)
			return Eulerianity::notEulerian;

		// A connected graph with exactly 2 nodes with an odd degree is Semi-Eulerian !!!!!!!!!!!!!
		if (oddCount == 2)
			return Eulerianity::semiEulerian;

		// THIS IS WHERE WE KNOW THE GRAPH IS Eulerian
		return Eulerianity::eulerian;
	}

	inline std::vector<Node *> EulerianPath::FindPath(Eulerianity &eulerianity) const
	{
		Graph graphCopy = m_pGraph->Clone();
		std::vector<Node *> Path = {};
		std::vector<Node *> Nodes = graphCopy.GetActiveNodes();
		int currentNodeId{Graphs::InvalidNodeId};

		// Check for Euler path
		eulerianity = IsEulerian();
		// If !EULER return the empty path
		if (eulerianity == Eulerianity::notEulerian)
			return Path;

		// Choose a starting node
		if (eulerianity == Eulerianity::eulerian)
		{
			currentNodeId = Nodes.front()->GetId();
		}
		else // semiEulerian
		{
			// Exactly 2 nodes with odd degree
			for (auto *node : Nodes)
			{
				int degree = static_cast<int>(graphCopy.FindConnectionsWith(node->GetId()).size());
				if (degree % 2 != 0)
				{
					currentNodeId = node->GetId();
					break;
				}
			}
		}

		std::stack<int> nodeStack;

		// Repeat until no more connections AND the stack is empty
		while (!graphCopy.FindConnectionsWith(currentNodeId).empty() || !nodeStack.empty())
		{
			// If the node has neighbors 
			if (!graphCopy.FindConnectionsWith(currentNodeId).empty())
			{
				// Add this node to the stack
				nodeStack.push(currentNodeId);
				// Take any of its neighbors
				auto connections = graphCopy.FindConnectionsWith(currentNodeId);
				int neighborId = (connections[0]->GetFromId() == currentNodeId)
									 ? connections[0]->GetToId()
									 : connections[0]->GetFromId();

				// Remove the edge between selected neighbor and that node from copy
				graphCopy.RemoveConnection(currentNodeId, neighborId);
				graphCopy.RemoveConnection(neighborId, currentNodeId);

				// Set neighbor as current node
				currentNodeId = neighborId;
			}
			else
			{
				// Add current node to path 
				Path.push_back(m_pGraph->GetNode(currentNodeId).get());

				// Pop a node from the stack and set it as current
				currentNodeId = nodeStack.top();
				nodeStack.pop();
			}
		}

		// add the last currentnode to the path
		Path.push_back(m_pGraph->GetNode(currentNodeId).get());

		// The obtained path is in reversed order so reverse it before returning
		std::reverse(Path.begin(), Path.end());
		return Path;
	}

	inline void EulerianPath::VisitAllNodesDFS(const std::vector<Node *> &Nodes, std::vector<bool> &visited, int startIndex) const
	{
		visited[startIndex] = true;

		std::vector<Connection *> connections = m_pGraph->FindConnectionsWith(startIndex);

		// Recursively visit new valid nodes
		for (auto *connection : connections)
		{
			int neighborId = (connection->GetFromId() == startIndex) ? connection->GetToId() : connection->GetFromId();

			if (!visited[neighborId])
			{
				VisitAllNodesDFS(Nodes, visited, neighborId);
			}
		}
	}

	inline bool EulerianPath::IsConnected() const
	{
		std::vector<Node *> Nodes = m_pGraph->GetActiveNodes();
		if (Nodes.size() == 0)
			return false;

		int maxId = 0;
		for (auto *node : Nodes)
		{
			if (node->GetId() > maxId)
				maxId = node->GetId();
		}
		std::vector<bool> visited(maxId + 1, false);

		// Choose a starting node that has at least one connection
		Node *startingpoint = nullptr;
		for (auto *node : Nodes)
		{
			if (!m_pGraph->FindConnectionsWith(node->GetId()).empty())
			{
				startingpoint = node;
				break;
			}
		}

		if (!startingpoint)
			return false;

		// Start a DFS from that node
		VisitAllNodesDFS(Nodes, visited, startingpoint->GetId());

		for (auto *node : Nodes)
		{
			if (!visited[node->GetId()])
				return false;
		}

		return true;
	}
}