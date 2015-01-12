#pragma once

#include <vector>
#include <string>


//// Required for the priority queue to compare two nodes
//class CompareNode
//{
//public:
//	// Returns true if n1 is earlier than n2
//	// -> cost of n1 is lower than cost of n2
//	//bool operator()(NodeType* n1, NodeType* n2) 	{ return n1->priority < n2->priority; }
//
//	// Returns true if n2 is earlier than n1
//	// -> cost of n1 is higher than cost of n2
//	bool operator()(NodeType* n1, NodeType* n2) 	{ return n1->cost_accumulator > n2->cost_accumulator; }
//};

/*std::priority_queue<NodeType*, std::vector<NodeType*>, CompareNode> pq;*/


using namespace std;

template< typename T > class PriorityQueue
{
public:
	PriorityQueue()
	{
	}
	~PriorityQueue()
	{
	}

	// Pushes a new element with the given position on the queue
	// When the element is already contained the position is changed 
	void push(T elem, int pos)
	{
		for (auto it = m_elements.begin(); it != m_elements.end(); it++)
		{
			if ((*it).value == elem)
			{
				(*it).position = pos;
				return;
			}
		}

		element e;
		e.value = elem;
		e.position = pos;

		m_elements.push_back(e);
	}

	// Returnes the element with the lowest position
	// Throws an error when the container is empty
	T pop()
	{
		if (!m_elements.size()) throw string("Container is empty");

		int pos_min = m_elements[0].position;
		unsigned int index = 0;

		for (unsigned int i = 1; i < m_elements.size(); i++)
		{
			if (m_elements[i].position < pos_min)
			{
				pos_min = m_elements[i].position;
				index = i;
			}
		}

		element e = m_elements[index];
		
		m_elements.erase(m_elements.begin() + index);

		return e.value;
	}

	// Searches the element in the container. Position is ignored.
	bool find(T to_find)
	{
		auto it = m_elements.begin();

		for (; it != m_elements.end(); it++)
		{
			if ((*it).value == to_find) break;
		}

		return it != m_elements.end();
	}

	// Searches the element in the container. Returns true if the element is found and if it has a higher position
	bool find(T to_find, int pos)
	{
		for (auto it = m_elements.begin(); it != m_elements.end(); it++)
		{
			if ((*it).value == to_find && (*it).position > pos) return true;
		}

		return false;
	}

	// Returns true if the container is empty
	bool empty()
	{
		return m_elements.empty();
	}


private:
	struct element
	{
		T value;
		int position;


	};

	vector<element> m_elements;

};

