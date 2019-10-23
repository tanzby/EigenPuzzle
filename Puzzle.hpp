#pragma once

#include <iostream>
#include <cmath>
#include <vector>
#include <ctime>

struct Action
{
    int r, c;
    int d_r, d_c;
    Action():r(-1),c(-1),d_c(0),d_r(0){}
    Action(int r, int c, int d_r, int d_c):r(r),c(c),d_c(d_c),d_r(d_r){}
};


class State
{
    constexpr static int N = 9;

    constexpr static int WIDTH = 3;

    int cells_[WIDTH][WIDTH];

    int hash_;

    void computeHash()
    {
        static int fac[]={1,1,2,6,24,120,720,5040,40320,362880};
        hash_ = 0;
        for (int i = 0, cnt = 0; i < N; ++i)
        {
            cnt=0;
            for (int j = i+1; j < N; ++j)
                if(cells_[j/WIDTH][j%WIDTH]<cells_[i/3][i%3]) ++cnt;
            hash_+=cnt*fac[N-1-i];
        }
    }

public:

    enum DistanceType
    {
        Euclidean = 0,
        Manhattan,
        OneHot,
        Binary,
    };

    State()
    {
        hash_ = -1;
    }

    State(const std::vector<int>& data)
    {
        hash_ = -1;
        assert(data.size() == N);

        for(int i = 0; i < N; ++i)
        {
            cells_[i/WIDTH][i%WIDTH] = data[i];
        }

        computeHash();
    }

    const int& operator [](int i) const {
        return cells_[i/WIDTH][i%WIDTH];
    }
    const int& operator () (int r, int c) const {
        return cells_[r][c];
    }
    int& operator () (int r, int c) {
        return cells_[r][c];
    }

    float getDistance(const State& target_state, DistanceType type= Euclidean) const
    {
        static std::pair<int,int> position_map[9];
        static int lastest_hash = -1;

        assert(hash_!=-1 && target_state.hash_!=-1);

        if (target_state.getHash() == hash_)  return 0;

        if (type == Binary) return target_state.getHash() != hash_;

        if (target_state.getHash()!=lastest_hash)
        {
            for (int r = 0; r < WIDTH; r++)
            {
                for (int c = 0; c < WIDTH; ++c)
                {
                    position_map[target_state(r,c)].first  = r;
                    position_map[target_state(r,c)].second = c;
                }
            }
        }

        float cost = 0;
        for (int r = 0; r < WIDTH; r++)
        {
            for (int c = 0; c < WIDTH; ++c)
            {
                if (target_state(r, c) == 0) continue;
                if (type == OneHot)
                {
                    cost += float(target_state(r, c) != cells_[r][c]);
                    continue;
                }

                auto& tr = position_map[cells_[r][c]].first;
                auto& tc = position_map[cells_[r][c]].second;

                if (type==Euclidean)
                {
                    cost+= std::sqrt((r-tr)*(r-tr)+(c-tc)*(c-tc));
                }
                else if(type == Manhattan)
                {
                    cost+= (std::abs(r-tr)+std::abs(c-tc));
                }

            }
        }
        return cost;
    }

    inline int getHash() const
    {
        assert(hash_!=-1);
        return hash_;
    }

    State executeAction(const Action& action)
    {
        assert(action.r>=0 && action.r < WIDTH);
        assert(action.c>=0 && action.c < WIDTH);

        int move_to_r =  action.r + action.d_r;
        int move_to_c =  action.c + action.d_c;

        if (cells_[move_to_r][move_to_c] != 0 )
        {
            throw std::logic_error("being occupied, can not move to here");
        }

        State new_state = *this;
        std::swap(new_state.cells_[move_to_r][move_to_c],
                  new_state.cells_[action.r][action.c]);

        new_state.computeHash();

        return new_state;
    }

    float tryActionAndGetCost(const State& target, const Action& action, DistanceType distanceType=Euclidean)
    {
        assert(action.r>=0 && action.r < WIDTH);
        assert(action.c>=0 && action.c < WIDTH);

        int move_to_r =  action.r + action.d_r;
        int move_to_c =  action.c + action.d_c;

        if (cells_[move_to_r][move_to_c] != 0 )
        {
            throw std::logic_error("being occupied, can not move to here");
        }

        std::swap(cells_[move_to_r][move_to_c],
                  cells_[action.r][action.c]);

        float cost = getDistance(target, distanceType);

        std::swap(cells_[move_to_r][move_to_c],
                  cells_[action.r][action.c]);

        return cost;

    }

    size_t tryActionAndGetHash(const Action& action)
    {
        assert(action.r>=0 && action.r < WIDTH);
        assert(action.c>=0 && action.c < WIDTH);

        int move_to_r =  action.r + action.d_r;
        int move_to_c =  action.c + action.d_c;

        if (cells_[move_to_r][move_to_c] != 0 )
        {
            throw std::logic_error("being occupied, can not move to here");
        }

        std::swap(cells_[move_to_r][move_to_c],
                  cells_[action.r][action.c]);

        this->computeHash();

        int ret = getHash();

        std::swap(cells_[move_to_r][move_to_c],
                  cells_[action.r][action.c]);

        return ret;
    }

    friend std::istream &operator>>(std::istream &in, State &x)
    {
        memset(x.cells_, 0 , sizeof(x.cells_));
        int temp;
        for(int r = 0; r < WIDTH; ++r)
        {
            for(int c = 0; c < WIDTH; ++c)
            {
                in >> x.cells_[r][c];
            }
        }
        return in;
    }
    friend std::ostream &operator<<(std::ostream &out, State &x)
    {
        for(int r = 0; r < WIDTH; ++r)
        {
            for(int c = 0; c < WIDTH; ++c)
            {
                out << x.cells_[r][c] <<" ";
            }
            out << std::endl;
        }
        return out;
    }

    bool check_if_solvable(const State& target) const
    {
        auto _compute = []( const int cells [WIDTH][WIDTH]) -> int
        {
            int cnt=0;
            for(int i=0;i<N;i++){
                const int& s = cells[i/WIDTH][i%WIDTH];
                if(s==0) continue;
                for(int j=i-1;j>=0;j--){
                    if(cells[j/WIDTH][j%WIDTH]>s) cnt++;
                }
            }
            return cnt;
        };
        // 两个不同的状态序列的逆序数同奇偶
        return (_compute(target.cells_)%2) == (_compute(cells_)%2);

    }
};

class Puzzle;
typedef Puzzle* PuzzlePtr;

class Puzzle
{
    constexpr static int N = 9;
    constexpr static int WIDTH = 3;

    State  source_state_;
    State  target_state_;

public:

    class Solver
    {
    public:
        virtual std::string getName() = 0;
        virtual void compute(PuzzlePtr puzzle, std::vector<Action>& action_list) = 0;
    };

    void setState(const State& target, const State& source)
    {
        target_state_ = target;
        source_state_ = source;

        assert(source.check_if_solvable(target));
    }

    inline const State& getSourceState() const
    {
        return source_state_;
    }

    inline float getStateCost(const State& state, State::DistanceType type = State::Manhattan) const
    {
        // every node has target_state cache.
        return state.getDistance(target_state_, type);
    }

    std::vector<Action> getActions(const State& state)
    {
        static int d[4][2] ={{-1,0},{+1,0},{0,+1},{0,-1}};

        std::vector<Action> actions;

        for (int r  = 0; r < WIDTH; ++r)
        {
            for (int c = 0; c < WIDTH; ++c)
            {
                if (state(r,c) == 0)
                {
                    for (int i = 0; i < 4; ++i)
                    {
                        if (r+d[i][0] < 0 || c+d[i][1] < 0 || r+d[i][0]>=WIDTH || c+d[i][1]>=WIDTH) continue;

                        actions.emplace_back(r+d[i][0],c+d[i][1],-d[i][0],-d[i][1]);
                    }
                }
            }
        }
        return actions;
    }

    void printActions(const State& state, const std::vector<Action>& action_list)
    {
        State temp = state;
        std::cout <<"origin: \n---------\n"<< temp << std::endl;
        int  count = 0;
        for (auto& act: action_list)
        {
            std::cout <<"act " << ++count <<"\n---------\n"
            << (temp = temp.executeAction(act)) << std::endl;
        }
    }

    double Solve(Solver* solver, bool print_result=false)
    {
        assert(solver!= nullptr);

        auto start_t =clock();

        std::vector<Action> action_list;
        solver->compute(this, action_list);

        auto end_t =clock();

        double end_time=(double)(end_t-start_t)/1000;

        printf("Total runtime for %25s : %10.4fms, \twith %3ld steps.\n",solver->getName().c_str(), end_time, action_list.size());

        if (print_result) printActions(getSourceState(), action_list);

        return end_time;
    }
};


