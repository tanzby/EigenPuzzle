//
// Created by iceytan on 2019/10/22.
//
#pragma once

#include <queue>
#include <unordered_set>
#include <iostream>
#include <stack>
#include <utility>
#include <unordered_map>
#include <memory>
#include <algorithm>

#include "Puzzle.hpp"

class Greedy: public Puzzle::Solver
{
    struct Node;
    typedef std::shared_ptr<Node> NodePtr;

    struct Node
    {
        float h_cost = 0;
        State state {};
        Action  action;
        NodePtr prev_node_ptr{};
        Node(float h, const State& state, const Action& action):
                h_cost(h), state(state), action(action){}
    };

    struct NodePtrCmp
    {
        bool operator () (const NodePtr& lhs, const NodePtr& rhs)
        {
            return lhs->h_cost > rhs->h_cost;
        }
    };

    State::DistanceType heuristic_type;

public:

    Greedy()
    {
        heuristic_type = State::Manhattan;
    }

    void setHeuristicType(State::DistanceType type)
    {
        assert(type==State::Manhattan || type == State::OneHot || type == State::Euclidean);
        heuristic_type = type;
    }


    std::string getName() override
    {
        static std::string basic_name = "Greedy";

        switch (heuristic_type)
        {
            case State::DistanceType::Manhattan: return basic_name+ " with Manhattan";
            case State::DistanceType::Euclidean: return basic_name+ " with Euclidean";
            case State::DistanceType::OneHot: return basic_name+ " with OneHot";
            default: return basic_name;
        }
    }

    void compute(PuzzlePtr puzzle, std::vector<Action>& action_list, int& node_sum) override
    {
        std::priority_queue<NodePtr, std::vector<NodePtr>, NodePtrCmp> q;

        std::unordered_map<int, bool> closed;
        closed[puzzle->getSourceState().getHash()] = true;

        q.push(std::make_shared<Node>(puzzle->getStateCost(puzzle->getSourceState()),
                                      puzzle->getSourceState(), Action()));

        while(!q.empty())
        {
            node_sum++;
            auto node = q.top();
            q.pop();

            if (puzzle->getStateCost(node->state,State::Binary) < 1.0)
            {
                Node * cur = node.get();
                while(cur->prev_node_ptr!= nullptr)
                {
                    action_list.emplace_back(cur->action);
                    cur = cur->prev_node_ptr.get();
                }
                std::reverse(action_list.begin(),action_list.end());

                break;
            }
            auto actions = puzzle->getActions(node->state);

            for(auto& act: actions)
            {
                int current_hash = node->state.tryActionAndGetHash(act);
                if (closed[current_hash]) continue;
                closed[current_hash]=true;

                auto new_state = node->state.executeAction(act);
                float act_cost = puzzle->getStateCost(new_state,heuristic_type);

                auto new_node = std::make_shared<Node>(act_cost, new_state, act);
                new_node->prev_node_ptr = node;
                q.push(new_node);
            }
        }
    }
};

class AStar: public Puzzle::Solver
{
    struct Node;
    typedef std::shared_ptr<Node> NodePtr;

    struct Node
    {
        float g_cost = 0;
        float h_cost = 0;
        State state {};
        Action  action;
        NodePtr prev_node_ptr{};
        Node(float g, float h, const State& state, const Action& action):
        g_cost(g), h_cost(h), state(state), action(action){}
    };

    struct NodePtrCmp
    {
        bool operator () (const NodePtr& lhs, const NodePtr& rhs)
        {
            return lhs->g_cost + lhs->h_cost > rhs->g_cost + rhs->h_cost;
        }
    };

    State::DistanceType heuristic_type;

public:

    AStar()
    {
        heuristic_type = State::Manhattan;
    }

    void setHeuristicType(State::DistanceType type)
    {
        assert(type==State::Manhattan || type == State::OneHot || type == State::Euclidean);
        heuristic_type = type;
    }

    std::string getName() override
    {
        static std::string basic_name = "A Star";

        switch (heuristic_type)
        {
            case State::DistanceType::Manhattan: return basic_name+ " with Manhattan";
            case State::DistanceType::Euclidean: return basic_name+ " with Euclidean";
            case State::DistanceType::OneHot: return basic_name+ " with OneHot";
            default: return basic_name;
        }
    }

    void compute(PuzzlePtr puzzle, std::vector<Action>& action_list, int& node_sum) override
    {
        std::priority_queue<NodePtr, std::vector<NodePtr>, NodePtrCmp> q;

        std::unordered_map<int, bool> closed;
        closed[puzzle->getSourceState().getHash()] = true;

        q.push(std::make_shared<Node>(0, puzzle->getStateCost(puzzle->getSourceState()),
                puzzle->getSourceState(), Action()));

        while(!q.empty())
        {
            node_sum++;

            auto node = q.top();
            q.pop();

            if (puzzle->getStateCost(node->state,State::Binary) < 1.0)
            {
                Node * cur = node.get();
                while(cur->prev_node_ptr!= nullptr)
                {
                    action_list.emplace_back(cur->action);
                    cur = cur->prev_node_ptr.get();
                }
                std::reverse(action_list.begin(),action_list.end());

                break;
            }
            auto actions = puzzle->getActions(node->state);

            for(auto& act: actions)
            {
                int current_hash = node->state.tryActionAndGetHash(act);
                if (closed[current_hash]) continue;
                closed[current_hash]=true;

                auto new_state = node->state.executeAction(act);
                float act_cost = puzzle->getStateCost(new_state, heuristic_type);

                auto new_node = std::make_shared<Node>(node->g_cost+1, act_cost, new_state, act);
                new_node->prev_node_ptr = node;
                q.push(new_node);
            }
        }
    }
};

class BFS: public Puzzle::Solver
{
    struct Node;

    typedef std::shared_ptr<Node> NodePtr;

    struct Node
    {
        float g_cost = 0;
        State state {};
        Action  action;
        NodePtr prev_node_ptr{};
        Node(float g,  const State& state, const Action& action):
                g_cost(g), state(state), action(action){}
    };

public:

    std::string getName() override
    {
        return "BFS";
    }

    void compute(PuzzlePtr puzzle, std::vector<Action>& action_list, int& node_sum) override
    {
        std::queue<NodePtr> q;

        std::unordered_map<int, bool> closed;
        closed[puzzle->getSourceState().getHash()] = true;

        q.push(std::make_shared<Node>(0, puzzle->getSourceState(), Action()));

        while(!q.empty())
        {
            node_sum++;
            auto node = q.front(); q.pop();

            if (puzzle->isGoal(node->state))
            {
                Node * cur = node.get();
                while(cur->prev_node_ptr!= nullptr)
                {
                    action_list.emplace_back(cur->action);
                    cur = cur->prev_node_ptr.get();
                }
                std::reverse(action_list.begin(),action_list.end());
                break;
            }

            auto actions = puzzle->getActions(node->state);

            for(auto& act: actions)
            {
                int current_hash = node->state.tryActionAndGetHash(act);
                if (closed[current_hash]) continue;
                closed[current_hash]=true;

                auto new_state = node->state.executeAction(act);
                auto new_node = std::make_shared<Node>(node->g_cost+1, new_state, act);
                new_node->prev_node_ptr = node;
                q.push(new_node);
            }
        }
    }
};

class IDS: public Puzzle::Solver
{
    struct Node;
    typedef std::shared_ptr<Node> NodePtr;

    struct Node
    {
        State state;
        Action  action;
        NodePtr prev_node_ptr;
        Node(const NodePtr& parent, const State& state, const Action& action):
                prev_node_ptr(parent), state(state), action(action){}
    };

    int node_count = 0;
    PuzzlePtr  puzzle;
    std::vector<Action>* action_list;
    std::unordered_map<int, bool> closed;

    bool enable_heuristic_;

    bool dfs(const NodePtr& root, int depth)
    {
        node_count++;

        if (enable_heuristic_)
        {
            if ( depth < puzzle->getStateCost(root->state, State::Manhattan) ) return false; // IDA*
        }
        else if ( depth < 0 ) return false;  // IDS


        if ( puzzle->isGoal(root->state) )
        {
            Node * cur = root.get();
            while(cur->prev_node_ptr!= nullptr)
            {
                action_list->emplace_back(cur->action);
                cur = cur->prev_node_ptr.get();
            }
            std::reverse(action_list->begin(),action_list->end());
            return true;
        }

        for(auto& act: puzzle->getActions(root->state))
        {
            int current_hash = root->state.tryActionAndGetHash(act);
            if (closed[current_hash]) continue;

            auto new_state = root->state.executeAction(act);

            NodePtr new_node(new Node(root, new_state, act));

            closed[new_state.getHash()] = true;

            if (dfs(new_node, depth-1)) return true;

            closed[new_state.getHash()] = false;
        }
        return false;
    }

public:

    IDS()
    {
        enable_heuristic_ = true;
    }

    void enableHeuristic(bool enable)
    {
        enable_heuristic_ = enable;
    }

    std::string getName() override
    {
        return "IDS" + std::string(enable_heuristic_? " with Heuristic": " without Heuristic");
    }

    void compute(PuzzlePtr puzzle, std::vector<Action>& action_list, int& node_sum) override
    {

        this->puzzle = puzzle;
        this->action_list = &action_list;

        int max_depth  = 40;

        NodePtr root(new Node(nullptr, puzzle->getSourceState(),Action()));

        for(int depth = 1; depth < max_depth; ++depth)
        {
            node_count = 0;

            closed.clear();
            closed[root->state.getHash()] = true;

            if (dfs(root, depth))
            {
                break;
            }
        }

        node_sum = node_count;
    }
};






