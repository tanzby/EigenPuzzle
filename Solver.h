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

    void compute(PuzzlePtr puzzle, std::vector<Action>& action_list) override
    {
        std::priority_queue<NodePtr, std::vector<NodePtr>, NodePtrCmp> q;

        std::unordered_map<int, bool> has_visit;
        has_visit[puzzle->getSourceState().getHash()] = true;

        q.push(std::make_shared<Node>(puzzle->getStateCost(puzzle->getSourceState()),
                                      puzzle->getSourceState(), Action()));

        while(!q.empty())
        {
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
                if (has_visit[current_hash]) continue;
                has_visit[current_hash]=true;

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

    void compute(PuzzlePtr puzzle, std::vector<Action>& action_list) override
    {
        std::priority_queue<NodePtr, std::vector<NodePtr>, NodePtrCmp> q;

        std::unordered_map<int, bool> has_visit;
        has_visit[puzzle->getSourceState().getHash()] = true;

        q.push(std::make_shared<Node>(0, puzzle->getStateCost(puzzle->getSourceState()),
                puzzle->getSourceState(), Action()));

        while(!q.empty())
        {
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
                if (has_visit[current_hash]) continue;
                has_visit[current_hash]=true;

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

    void compute(PuzzlePtr puzzle, std::vector<Action>& action_list) override
    {
        std::queue<NodePtr> q;

        std::unordered_map<int, bool> has_visit;
        has_visit[puzzle->getSourceState().getHash()] = true;

        q.push(std::make_shared<Node>(0, puzzle->getSourceState(), Action()));

        while(!q.empty())
        {
            auto node = q.front();
            q.pop();

            if (puzzle->getStateCost(node->state,State::Binary) < 1.f)
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
                if (has_visit[current_hash]!=0) continue;
                has_visit[current_hash]=true;

                auto new_state = node->state.executeAction(act);
                float act_cost = puzzle->getStateCost(new_state);

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
        int g_cost = 0;
        State state {};
        Action  action;
        NodePtr prev_node_ptr{};
        Node(int g, const State& state, const Action& action):
                g_cost(g), state(state), action(action){}
    };

    PuzzlePtr puzzle = nullptr;

    std::vector<Action>* action_list_ptr;

    std::unordered_map<int, bool> has_visit;

    bool DFS(const NodePtr& node, int depth)
    {
        if (puzzle->getStateCost(node->state,State::Binary) < 1.0)
        {
            std::vector<Action>& action_list = *action_list_ptr;
            Node * cur = node.get();
            while(cur->prev_node_ptr!= nullptr)
            {
                action_list.emplace_back(cur->action);
                cur = cur->prev_node_ptr.get();
            }
            std::reverse(action_list.begin(),action_list.end());
            return true;
        }

        if ( depth < 1 ) return false;

        auto actions = puzzle->getActions(node->state);

        for(auto& act: actions)
        {
            int current_hash = node->state.tryActionAndGetHash(act);
            if (has_visit[current_hash]!=0) continue;
            has_visit[current_hash]=true;

            auto new_state = node->state.executeAction(act);
            float act_cost = puzzle->getStateCost(new_state);

            NodePtr new_node = std::make_shared<Node>(node->g_cost+1, new_state, act);

            new_node->prev_node_ptr = node;

            if (DFS(new_node, depth-1))
            {
                return true;
            }

            has_visit[current_hash] = false;
        }

        return false;
    }

public:

    std::string getName() override
    {
        return "IDS";
    }

    void compute(PuzzlePtr puzzle, std::vector<Action>& action_list) override
    {
        this->puzzle = puzzle;
        this->action_list_ptr = & action_list;

        int max_depth  = 30;

        NodePtr root = std::make_shared<Node>(0,puzzle->getSourceState(),Action());

        for(int depth = 1; depth < max_depth; ++depth)
        {
            has_visit.clear();
            has_visit[root->state.getHash()] = true;

            if (DFS(root,depth))
            {
                break;
            }
        }
    }
};






