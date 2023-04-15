
// #include "STcpClient_1.h"
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <random>
#include <algorithm>
#include <iterator>
#include <chrono>

using namespace std;
using namespace std::chrono;

random_device                  rand_dev;
mt19937                        generator(rand_dev());

struct VectorHash {
    size_t operator()(const std::vector<int>& v) const {
        std::hash<int> hasher;
        size_t seed = 0;
        for (int i : v) {
            seed ^= hasher(i) + 0x9e3779b9 + (seed<<6) + (seed>>2);
        }
        return seed;
    }
};

vector<int> Next_Node(int, int, int);

class Node {
    public:
        Node* parent;    // parent node
        vector<Node> children; // list of children node
        int depth;
        vector<vector<int> > state; // board state
        int visitCount;
        int winCount;
        bool is_fully_expanded;
        bool isTerminal;
        unordered_set<vector<int>, VectorHash> possible_moves; // [x, y, len, direction]


        Node(){

        };

        Node(Node *parent, vector<vector<int> > state, int depth) {
            this->parent = parent;
            this->children = vector<Node>();
            this->depth = depth;
            this->state = state;
            this->visitCount = 0;
            this->winCount = 0;
            this->is_fully_expanded = false;
            
            generate_possible_moves();
            // is terminal state
            is_fully_expanded = isTerminal = (possible_moves.size() == 0);
            
        }

        /* generate possible moves as a set, O(12^2) */
        void generate_possible_moves() {
            int direction[] = {1, 2, 3};
            const int BOARD_SIZE = 12;
            const int LEN_SIZE = 3;
            vector<int> action(4);
            
            for(int i=0; i<BOARD_SIZE; i++)
                for(int j=0; j<BOARD_SIZE; j++){
                    if(state[i][j] != 0) 
                        continue;
                    action[0] = i;  // base case: len = 1
                    action[1] = j;
                    action[2] = 1;
                    action[3] = 1;
                    possible_moves.insert(action);

                    for(auto& dir: direction){
                        action[3] = dir;
                        vector<int> pos = Next_Node(i, j, dir);
                        int x = pos[0], y = pos[1];
                        for(int len=2; len<=LEN_SIZE; len++){
                            if(!(0<=x and x<BOARD_SIZE) or !(0<=y and y<BOARD_SIZE) or state[x][y] != 0)
                                break;

                            action[2] = len;
                            possible_moves.insert(action);
                            vector<int> pos = Next_Node(x, y, dir);
                            x = pos[0], y = pos[1];
                        }
                    }
                }
        }

        /* calculate current UCT score of this node */
        double get_score(double exploration_factor){
            return  ((double) winCount/visitCount) + exploration_factor * sqrt(log(parent->visitCount)/visitCount);
        }

        /* generate the next state after a move */
        vector<vector<int> > get_next_state(vector<int> action){
            vector<vector<int> > newState = state;
            int i = action[0];
            int j = action[1];
            int len = action[2];
            int dir = action[3];

            for (size_t c = 0; c < len; c++){
                newState[i][j] = 1;
                vector<int> pos = Next_Node(i, j, dir);
                i = pos[0], j = pos[1];
            }

            return newState;
        }

        vector<int> get_random_move(){
                // cout << "BEGIN get_random_move" << endl;  // DEBUG
            int N = min((int)possible_moves.size(), 10);
            uniform_int_distribution<int>  distr(0, N-1);

            auto it = possible_moves.begin();
            advance(it, distr(generator));              // O(10)
            return *it;
                // cout << "END get_random_move" << endl;  // DEBUG
        }

        vector<vector<int> > get_n_random_move(int n){
                // cout << "BEGIN get_n_random_move" << endl;  // DEBUG
            int N = possible_moves.size();
            int STEP = N / n;

            if(STEP == 0){
                        // cout << "END get_n_random_move" << endl;  // DEBUG
                return vector<vector<int> >(possible_moves.begin(), possible_moves.end());
            }

            uniform_int_distribution<int>  distr(1, STEP);
            vector<vector<int> > selected_moves;
            auto it = possible_moves.begin();

            for (size_t i = 0; i < n; i++){             // O(N)
                selected_moves.push_back(*it);
                advance(it, distr(generator));
            }
                // cout << "END get_n_random_move" << endl;  // DEBUG
            return selected_moves;
        }
};

class MCTS {
    public: 
        Node root;
        vector<vector<int> > rootMoves;
        int NUMBER_OF_RUNS; // number of runs, hyperparameter
        int ROUNDS; // number of simulation, hyperparameter
        double exploration_factor;

        MCTS(vector<vector<int> > state, int NUMBER_OF_RUNS, int ROUNDS, double exploration_factor){
            this->rootMoves = vector<vector<int> >();
            this->root = Node(NULL, state, 0);
            this->NUMBER_OF_RUNS = NUMBER_OF_RUNS;
            this->ROUNDS = ROUNDS;
            this->exploration_factor = exploration_factor;
        }
        
        void main(){
                    auto begin = high_resolution_clock::now();
            while(this->NUMBER_OF_RUNS--){
                Node* node = selection(&root);
                expand(node);
                    // cout << "✅ end one round of simulation" << endl;   // DEBUG
            }
                    auto end = high_resolution_clock::now();
                    auto duration = duration_cast<milliseconds>(end - begin);
                    cout << "Elapsed Time: " << duration.count() << endl;
        }

        /* select a node that is NOT fully expanded */
        Node* selection(Node* node){
                    // cout << "BEGIN selection()" << endl;          // DEBUG
            while (node->is_fully_expanded && !node->isTerminal){
                    // cout << node->is_fully_expanded << " " << node->isTerminal << endl;          // DEBUG
                    // cout << "depth:" << node->depth << endl;          // DEBUG
                node = get_prefered_child(node);

                // if(node == NULL)
                    // cerr << "ERROR: node is null" << endl;          // DEBUG
            }
                    // cout << "END selection()" << endl;          // DEBUG
            return node;
        }

        /* in selection step, select the child with the highest UCT score */
        Node* get_prefered_child(Node* node){
                    // cout << "BEGIN get_prefered_child()" << endl;          // DEBUG
                    int i = 0;
            double maxScore = -999.0;
            Node *ptr = NULL;
            for(auto& child: node->children){
                    // cout << "get_score: " << child.winCount << " " << child.visitCount << " " << child.parent->visitCount << endl;    // DEBUG
                double score = child.get_score(exploration_factor);
                        // cout << i++ << "th score: " << score << endl;          // DEBUG
                if(score > maxScore){
                    maxScore = score;
                    ptr = &child;
                }
            }
                    // cout << "END get_prefered_child()" << endl;          // DEBUG
            return ptr;
        }

        void expand(Node* node){
                    // cout << "BEGIN expand()" << endl;          // DEBUG
            if(node->isTerminal){
                    // cout << "END expand()" << endl;          // DEBUG
                rollout(node);
                return;
            }

            // pick one possible move
            vector<int> action = *(node->possible_moves.begin());
            node->possible_moves.erase(action);

            // mark as fully expanded
            if (node->possible_moves.size() == 0)
                node->is_fully_expanded = true;

            // expand a child
            Node child = Node(node, node->get_next_state(action), node->depth + 1);
            if (node->depth == 0){
                rootMoves.push_back(action);
            }

                    // cout << "END expand()" << endl;          // DEBUG
            rollout(&child);
            node->children.push_back(child);
        }

        /* random game simulation */
        void rollout(Node* node){
                    // cout << "BEGIN rollout()" << endl;          // DEBUG
            // if node is a terminal state, just backpropgation
            if(node->isTerminal){
                backpropgation(node, true);
                    // cout << "END rollout()" << endl;          // DEBUG
                return;
            }

            // randomly sample n moves
            vector<vector<int> > selected_moves = node->get_n_random_move(this->ROUNDS);

            for(auto& action: selected_moves){
                bool is_win = false;
                vector<vector<int> > state = node->get_next_state(action);

                while(1){
                    Node child = Node(NULL, state, 0);

                    if(child.isTerminal)
                        break;
                    
                    // random move
                    action = child.get_random_move();

                    // update state
                    state = child.get_next_state(action);
                    is_win = !is_win;
                }

                backpropgation(node, is_win);
            }
                    // cout << "END rollout()" << endl;          // DEBUG

        }

        void backpropgation(Node* node, bool is_win){
                    // cout << "BEGIN backpropgation()" << endl;          // DEBUG
            // increase win count only for nodes of the same "color"
            is_win = !is_win; // FIXME: flip first
            while(node != NULL){
                node->winCount += is_win;
                node->visitCount += 1;
                node = node->parent;
                is_win = !is_win; // flip between 0 and 1
            }
                    // cout << "END backpropgation()" << endl;          // DEBUG
        }

        /* select the move with highest visit counts */
        vector<int> get_best_move(){
            unordered_map<int, vector<int> > visitCountMap; // visitCount -> list of index
            int maxCount = 0;
            int idx = 0;
            for(auto& child: root.children){
                maxCount = max(maxCount, child.visitCount);
                visitCountMap[child.visitCount].push_back(idx);
                idx++;
            }

            // randomly choose one if there are ties
            uniform_int_distribution<int>  distr(0, visitCountMap[maxCount].size()-1);
            int move = visitCountMap[maxCount][distr(generator)];
            
            return rootMoves[move];
        }
};

/*
    input position (x,y) and direction
    output next node position on this direction
*/
vector<int> Next_Node(int pos_x, int pos_y, int direction) {
    vector<int> result(2);
    
    if (pos_y % 2 == 1) {
        if (direction == 1) {
            result[0] = pos_x;
            result[1] = pos_y - 1;
        }
        else if (direction == 2) {
            result[0] = pos_x + 1;
            result[1] = pos_y - 1;
        }
        else if (direction == 3) {
            result[0] = pos_x - 1;
            result[1] = pos_y;
        }
    }
    else {
        if (direction == 1) {
            result[0] = pos_x - 1;
            result[1] = pos_y - 1;
        }
        else if (direction == 2) {
            result[0] = pos_x;
            result[1] = pos_y - 1;
        }
        else if (direction == 3) {
            result[0] = pos_x - 1;
            result[1] = pos_y;
        }
    }
    return result;
}

void print_state(const vector<vector<int> >& state){
    int N = 12;
    int z = false;
    for (size_t j = 0; j < N; j++){
        if(z)  cout << " ";
        z = !z;

        for (size_t i = 0; i < N; i++)
            cout << state[i][j] << " ";

        cout << endl;
    }
}

vector<vector<int> > generate_board(int numZeros){
    const int N = 12;

    vector<vector<int> >state(N, vector<int>(N, 1));
    // uniform_int_distribution<int>  distr(0, N-1);
    // for (size_t i = 0; i < numZeros; i++)
    //     state[distr(generator)][distr(generator)] = 0;

    // rectangle
    for (size_t i = 0; i < 4; i++){
        for (size_t j = 0; j < 6; j++){
            state[i][j] = 0;
        }
    }

    cout << "generated board:" << endl;
    print_state(state);

    return state;
}

vector<vector<int> > generate_next_state(vector<vector<int> > state, vector<int> action){
    vector<vector<int> > newState = state;
    int i = action[0];
    int j = action[1];
    int len = action[2];
    int dir = action[3];

    for (size_t c = 0; c < len; c++){
        newState[i][j] = 1;
        vector<int> pos = Next_Node(i, j, dir);
        i = pos[0], j = pos[1];
    }

    return newState;
}

/* if terminal, no cell is 0, O(N^2) */ 
bool check_isTerminal(vector<vector<int> > state){
    
    for(auto& v: state)
        for(auto& x: v)
            if(x==0){
                return false;
            }

    return true;
}

void test_generate_move(){
    cout<< "Test: test_generate_move" << endl;
    const int N = 12;
    
    vector<vector<int> >state = generate_board(80);

    Node node(NULL, state, 0);
    node.generate_possible_moves();

    for (auto& v: node.possible_moves){
        for(auto&i:v)
            cout<< i << " ";
            cout<< endl;
        int x = v[0];
        int y = v[1];
        int LEN = v[2];
        int dir = v[3];
        
        for(int len=1; len<=LEN; len++){
            // if(!(0<=x and x<N) or !(0<=y and y<N)){
            if(!(0<=x and x<N) or !(0<=y and y<N) or state[x][y] != 0){
                cerr << "Test failed: test_generate_move" << endl;
                cerr << "❌ wrong move" << endl;
                throw exception();
            }

            vector<int> pos = Next_Node(x, y, dir);
            x = pos[0], y = pos[1];
        }
    }
        cout<< "✅ Test passed" <<endl;
}

int test_play(){
    int N = 12;
    vector<vector<int> > state = generate_board(80);

    int player_no = 0;  // determine who's first

    while(1){
        vector<int> action;
        cout << "player" << player_no << " move:" << endl;
        if(player_no == 0){
            // me: MCTS
                    auto begin = high_resolution_clock::now();
            MCTS mcts(state, 1000, 2, 0.1);
                    auto end = high_resolution_clock::now();
                    auto duration = duration_cast<milliseconds>(end - begin);
                    cout << "MCTS init time: " << duration.count() <<endl;

            mcts.main();
                        // cout << "BEGIN get_best_move()" << endl;          // DEBUG
                    begin = high_resolution_clock::now();
            action = mcts.get_best_move();
                    end = high_resolution_clock::now();
                    duration = duration_cast<milliseconds>(end - begin);
                    cout << "get_best_move time: " << duration.count() <<endl;

                        // cout << "END get_best_move()" << endl;          // DEBUG
        }else{
            // opponent: random move
            // Node node(NULL, state, 0);
            // uniform_int_distribution<int>  distr(0, node.possible_moves.size()-1);

            // auto it = node.possible_moves.begin();
            // advance(it, distr(generator));
            // action = *it;

            MCTS mcts(state, 1000, 2, 0.01);
            mcts.main();
                        // cout << "BEGIN get_best_move()" << endl;          // DEBUG
            action = mcts.get_best_move();
                        // cout << "END get_best_move()" << endl;          // DEBUG
        }

                    // cout << "BEGIN generate_next_state()" << endl;          // DEBUG
        state = generate_next_state(state, action);
                    // cout << "END generate_next_state()" << endl;          // DEBUG

        cout << "action: ";
        for(auto& i: action)
            cout<< i << " ";
        cout << endl;
        print_state(state);
        cout<<endl;
        
        player_no = !player_no;

        if(check_isTerminal(state)){
            cout << "winner: " << player_no << endl;
            return player_no;
        }
    }
}

/*
    輪到此程式移動棋子
    mapStat : 棋盤狀態為 12*12矩陣, 0=可移動區域, -1=障礙, 1~2為玩家1~2佔領區域
    gameStat : 棋盤歷史順序
    return Step
    Step : 4 elements, [x, y, l, dir]
            x, y 表示要畫線起始座標
            l = 線條長度(1~3)
            dir = 方向(1~6),對應方向如下圖所示
              1  2
            3  x  4
              5  6
*/

int main()
{
    int id_package;
    int mapStat[12][12];
    int gameStat[12][12];

    // int n = 1000;
    // while(n--)
        // test_generate_move();
    // return 0;

    vector<int> wins(2);
    for (size_t i = 0; i < 100; i++){
        cout<< "[Round " << i << "]" << endl;
        int winner = test_play();
        wins[winner]++;
    }
    cout<< "Stats:" << endl;
    printf("player 0: 1000, 2, 0.1\n");
    printf("player 1: 1000, 2, 0.01\n");
    cout<<"\tplayer 0: " << wins[0] << endl;
    cout<<"\tplayer 1: " << wins[1] << endl;
    

    //  comment out temporarily

    // while (true)
    // {
    //     if (GetBoard(id_package, mapStat, gameStat))
    //         break;

    //     std::vector<int> step = GetStep(mapStat, gameStat);
    //     SendStep(id_package, step);
    // }
    return 0;
}
