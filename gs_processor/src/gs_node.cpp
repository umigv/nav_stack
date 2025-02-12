// #include <rclcpp/rclcpp.hpp>
// #include <vector>
// #include <fstream>
// #include <queue>
// #include <set>
// #include <string>
// #include <sstream>
// #include <utility>

// using namespace std;
// using namespace std::chrono_literals;

// class BFSNode : public rclcpp::Node {
// public:
//     BFSNode() : Node("bfs_node") {
//         run_bfs();
//     }

// private:
//     const vector<pair<int, int>> DIRECTIONS = {{-1,0}, {-1,-1}, {-1,1}, {0,-1}, {0,1}};

//     bool isValidMove(pair<int, int> pos, const vector<vector<int>>& matrix, const set<pair<int, int>>& visited) {
//         int y = pos.first;
//         int x = pos.second;
//         return (y >= 0) && (y < matrix.size()) && 
//             (x >= 0) && (x < matrix[0].size()) && 
//             (matrix[y][x] == 1) && 
//             (visited.find(pos) == visited.end());
//     }

//     void bfs(vector<vector<int>>& matrix, pair<int, int> start) {
//         queue<pair<int, int>> q;
//         set<pair<int, int>> visited;

//         q.push(start);
//         visited.insert(start);

//         while (!q.empty()) {
//             auto current = q.front();
//             q.pop();

//             for (const auto& dir : DIRECTIONS) {
//                 pair<int, int> new_pos(current.first + dir.first, 
//                                     current.second + dir.second);

//                 if (isValidMove(new_pos, matrix, visited)) {
//                     q.push(new_pos);
//                     visited.insert(new_pos);
//                     matrix[new_pos.first][new_pos.second] = 8;
//                 }
//             }

//             if (q.empty()) {
//                 RCLCPP_INFO(this->get_logger(), "No valid moves left.");
//                 break;
//             }
//         }
//     }

//     void run_bfs() {
//         string file_path = "testout1.txt";
//         ifstream file(file_path);
//         if (!file.is_open()) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path.c_str());
//             return;
//         }

//         vector<vector<int>> matrix_ones;
//         string line;
//         while (getline(file, line)) {
//             line.erase(remove(line.begin(), line.end(), '['), line.end());
//             line.erase(remove(line.begin(), line.end(), ']'), line.end());
            
//             stringstream ss(line);
//             vector<int> row;
//             int num;
//             while (ss >> num) {
//                 row.push_back(num);
//             }
//             if (!row.empty()) {
//                 matrix_ones.push_back(row);
//             }
//         }

//         vector<vector<int>> matrix;
//         for (const auto& row : matrix_ones) {
//             vector<int> scaled_row;
//             for (int val : row) {
//                 scaled_row.push_back(val * 127);
//             }
//             matrix.push_back(scaled_row);
//         }

//         int sum = 0;
//         for (const auto& row : matrix) {
//             for (int val : row) {
//                 sum += val;
//             }
//         }
//         RCLCPP_INFO(this->get_logger(), "Matrix sum: %d", sum);
//         RCLCPP_INFO(this->get_logger(), "Matrix dimensions: %lux%lu", 
//             matrix.size(), matrix.empty() ? 0 : matrix[0].size());
        
//         pair<int, int> start(50, 78);
//         if (start.first >= matrix.size() || start.second >= matrix[0].size()) {
//             RCLCPP_ERROR(this->get_logger(), "Start position out of matrix bounds");
//             return;
//         }
//         RCLCPP_INFO(this->get_logger(), "Start value: %d", matrix[start.first][start.second]);

//         bfs(matrix, start);
//     }
// };

int main(int argc, char** argv) {
    // rclcpp::init(argc, argv);
    // auto node = make_shared<BFSNode>();
    // rclcpp::spin(node);
    // rclcpp::shutdown();
    return 0;
}