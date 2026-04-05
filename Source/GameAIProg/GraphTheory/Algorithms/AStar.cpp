#include "AStar.h"

using namespace GameAI;

AStar::AStar(Graph *const pGraph, HeuristicFunctions::Heuristic hFunction)
	: pGraph(pGraph), HeuristicFunction(hFunction)
{
}

std::vector<Node *> AStar::FindPath(Node *const pStartNode, Node *const pGoalNode)
{
	std::vector<Node *> path{};
	std::vector<NodeRecord> openList{};
	std::vector<NodeRecord> closedList{};
	NodeRecord currentNodeRecord{};

	// 1. Create a startRecord and add it to the openList
	NodeRecord startRecord{};
	startRecord.pNode = pStartNode;
	startRecord.pConnection = nullptr;
	startRecord.costSoFar = 0.f;
	startRecord.estimatedTotalCost = GetHeuristicCost(pStartNode, pGoalNode);
	openList.push_back(startRecord);

	// 2. While loop (part 1)
	while (!openList.empty())
	{
		// A. Get record from the open list with lowest F-score
		currentNodeRecord = *std::min_element(openList.begin(), openList.end());

		// B. Check if that record refers to the end node
		if (currentNodeRecord.pNode == pGoalNode)
		{
			break;
		}

		// C. Get all the connections of the current node and loop over them
		std::vector<Connection *> connections = pGraph->FindConnectionsFrom(currentNodeRecord.pNode->GetId());
		for (auto &pConnection : connections)
		{
			// 1. Get the node this connection is pointing to
			Node *pNextNode = pGraph->GetNode(pConnection->GetToId()).get();

			// 2. Calculate the total G-Cost so far
			float gCost = currentNodeRecord.costSoFar + pConnection->GetWeight();

			// D. Check if the connection leads to a node already on the closedList
			auto closedIt = std::find_if(closedList.begin(), closedList.end(),
										 [pNextNode](NodeRecord const &record)
										 { return record.pNode == pNextNode; });

			if (closedIt != closedList.end())
			{
				// If existing record is cheaper, skip this connection
				if (closedIt->costSoFar <= gCost)
					continue;

				// Else remove it from the closedList (it will be replaced)
				closedList.erase(closedIt);
			}
			else
			{
				// E. Check if the connection leads to a node already on the openList
				auto openIt = std::find_if(openList.begin(), openList.end(),
										   [pNextNode](NodeRecord const &record)
										   { return record.pNode == pNextNode; });

				if (openIt != openList.end())
				{
					// If existing record is cheaper, skip this connection
					if (openIt->costSoFar <= gCost)
						continue;

					// Else remove it from the openList (it will be replaced)
					openList.erase(openIt);
				}
			}

			// F. Create a new NodeRecord and add it to the openList
			NodeRecord newRecord{};
			newRecord.pNode = pNextNode;
			newRecord.pConnection = pConnection;
			newRecord.costSoFar = gCost;
			newRecord.estimatedTotalCost = gCost + GetHeuristicCost(pNextNode, pGoalNode);
			openList.push_back(newRecord);
		}

		// G. Remove the currentNodeRecord from the openList and add it to the closedList
		openList.erase(std::remove(openList.begin(), openList.end(), currentNodeRecord), openList.end());
		closedList.push_back(currentNodeRecord);
	}

	// 3. Backtracking - Reconstruct path from last connection to start node
	while (currentNodeRecord.pNode != pStartNode)
	{
		path.push_back(currentNodeRecord.pNode);

		// Look in the closedList for a record where pNode matches the connection's fromNodeId
		auto it = std::find_if(closedList.begin(), closedList.end(),
							   [&currentNodeRecord](NodeRecord const &record)
							   {
								   return record.pNode->GetId() == currentNodeRecord.pConnection->GetFromId();
							   });

		if (it != closedList.end())
		{
			currentNodeRecord = *it;
		}
		else
		{
			break;
		}
	}

	// Add the start node to the path
	path.push_back(pStartNode);

	// Reverse the path (it was built backwards)
	std::reverse(path.begin(), path.end());

	return path;
}

float AStar::GetHeuristicCost(Node *const pStartNode, Node *const pEndNode) const
{
	FVector2D toDestination = pGraph->GetNode(pEndNode->GetId())->GetPosition() - pGraph->GetNode(pStartNode->GetId())->GetPosition();
	return HeuristicFunction(abs(toDestination.X), abs(toDestination.Y));
}