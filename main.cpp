#include "Puzzle.hpp"
#include "Solver.h"

#include <queue>
#include <random>


void basic_test()
{
    Puzzle puzzle;

    State target_state({1,2,3,4,5,6,7,8,0});

    std::vector<State> states{
            State({1,2,3,4,5,6,7,0,8}),
            State({1,3,5,7,0,2,8,4,6}),
            State({5,4,3,6,0,2,1,7,8}),
            State({5,6,4,3,0,2,1,7,8})
    };

    Greedy greedy;
    AStar  aStart;
    IDS    ids;
    BFS    bfs;

    for(int i = 0; i < states.size(); ++i)
    {
        auto& state = states[i];
        puzzle.setState(target_state, state);

        printf("\n--------------------------------------------------  "
               "task %2d  -------------------------------------------------\n\n", i+1);

        greedy.setHeuristicType(State::Manhattan);
        puzzle.Solve(&greedy);
        greedy.setHeuristicType(State::Euclidean);
        puzzle.Solve(&greedy);
        greedy.setHeuristicType(State::OneHot);
        puzzle.Solve(&greedy);

        aStart.setHeuristicType(State::Manhattan);
        puzzle.Solve(&aStart);
        aStart.setHeuristicType(State::Euclidean);
        puzzle.Solve(&aStart);
        aStart.setHeuristicType(State::OneHot);
        puzzle.Solve(&aStart);

        ids.enableHeuristic(false);
        puzzle.Solve(&ids);
        ids.enableHeuristic(true);
        puzzle.Solve(&ids);

        puzzle.Solve(&bfs);

        printf("\n----------------------------------------------------"
               "----------------------------------------------------------\n");
    }
}

void random_test(int random_size = 200)
{
    Puzzle puzzle;

    State target_state({1,2,3,4,5,6,7,8,0});

    std::vector<State> random_test_set;

    std::vector<int> basic_vec {1,2,3,4,5,6,7,8,0};
    std::default_random_engine e(time(nullptr));
    while(random_test_set.size()<random_size)
    {
        std::shuffle(basic_vec.begin(),basic_vec.end(), e);
        State new_state (basic_vec);
        if (target_state.check_if_solvable(new_state)){
            random_test_set.emplace_back(new_state);
        }
    }

    Greedy greedy;
    AStar  aStart;
    IDS    ids;

    std::unordered_map<std::string, double> time_table;
    for(int i = 0; i < random_size; ++i)
    {
        auto& state = random_test_set[i];
        printf("> %3d/%3d \n", i, random_size);
        std::cout << state << std::endl;

        puzzle.setState(target_state, state);
        greedy.setHeuristicType(State::Manhattan);
        time_table[greedy.getName()] += puzzle.Solve(&greedy);
        greedy.setHeuristicType(State::Euclidean);
        time_table[greedy.getName()] += puzzle.Solve(&greedy);
        greedy.setHeuristicType(State::OneHot);
        time_table[greedy.getName()] += puzzle.Solve(&greedy);

        aStart.setHeuristicType(State::Manhattan);
        time_table[aStart.getName()] += puzzle.Solve(&aStart);
        aStart.setHeuristicType(State::Euclidean);
        time_table[aStart.getName()] += puzzle.Solve(&aStart);
        aStart.setHeuristicType(State::OneHot);
        time_table[aStart.getName()] += puzzle.Solve(&aStart);

        // puzzle.Solve(&bfs);
        time_table[ids.getName()] += puzzle.Solve(&ids);
    }

    std::cout  << "summary: \n\n";
    for(auto [name, time]: time_table)
    {
        std::cout << name <<": " << time / random_size << std::endl;
    }
}


int main()
{
    basic_test();

    // random_test();

    return 0;
}

