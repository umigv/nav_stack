#ifndef NAV2_ARVSTAR_PLANNER__RPASTAR_H_
#define NAV2_ARVSTAR_PLANNER__RPASTAR_H_

#include <iostream>
#include <queue>
#include <map>
#include <cmath>
#include <vector>
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_arvstar_planner
{

class rpastar
{
    protected:
        class Node
        {
        private:
            // distance from start node to current node
            float g_score;
            // straight line distance to the target node
            float h_score;
            //g_score + h_score
            std::pair<int, int> state;
            Node *parent;
        public:
            friend class GlobalPlanner;
            Node();
            Node(int row, int col,Node *parent);
            void set_h(std::pair<int,int> &target_state);
            void set_g(float g_in);
            void set_state(int row, int col);
            void set_parent(Node *parent_in);
            Node *get_parent()
            {
                return parent;
            }
            std::pair<int,int> get_state();
            float get_f_score() const;
            bool at_target(std::pair<int,int> &target_state);
            const bool& operator==(const Node &rhs)
            {
                return this->state == rhs.state;
            }
            
        };

        class customGreater {
            public:
            bool operator() (Node &lhs, Node &rhs)
            {
                return lhs.get_f_score() > rhs.get_f_score();
            }
        };

        class Node_hash {
            size_t operator()(const Node& node) const;
        };

        class Compare_coord{
            bool operator()(const Node& lhs, const Node& rhs);
        };

    public:
        rpastar(std::pair<int,int> start_state_in, std::pair<int,int> target_state_in, nav2_costmap_2d::Costmap2D* msg);
        void find_target();
        void search();
        bool goal_found();
        std::vector<std::pair<int,int>> backtracker();
        void processNode(int x, int y, Node* parent);
        //void gpsCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void costMapCallback(nav2_costmap_2d::Costmap2D *msg);
        double calculateEuclideanDistance(const Node& node1, const Node& node2);

    private:
        std::priority_queue<Node, std::vector<Node>, customGreater> U;
        std::pair<int,int> start_state;
        std::pair<int,int> target_state;
        std::vector<std::pair<int,int>> path;
        std::map<std::pair<int,int>, float> open_set;
        std::map<std::pair<int,int>, float> closed_set;
        std::vector<std::vector<int>> cost_map;
        std::vector<std::vector<Node>> graph;
        bool path_found;
        
};


}
#endif