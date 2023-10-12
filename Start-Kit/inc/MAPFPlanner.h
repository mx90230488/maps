#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"
#include <atomic>
struct Constraint_my {
    int location;
    int agentid;
    int time;
    Constraint_my(int _location,int _agentid,int _time) :
        location(_location),agentid(_agentid),time(_time){ }
    bool operator==(const Constraint_my&c)const{
        if(this->location==c.location&&this->agentid==c.agentid&&this->time==c.time) return true;
        return false;
    }
    
};
struct Conflict_my{
    std::pair<int,int>conflict_agent;
    int location1;
    int location2;
    int time;
    Conflict_my(int first_agent,int second_agent,int _location1,int _location2,int _time) :
            location1(_location1),location2(_location2),time(_time){
                conflict_agent=std::make_pair(first_agent,second_agent);
            }
};
struct TreeNode
{
    SharedEnvironment node_env;
    int cost=0;
    std::vector<std::vector<pair<int,int>>>solution;
    std::vector<Constraint_my>constraints;
    std::vector<pair<int,int>> single_agent_plan(int start,int start_direct,int end,int agentid);
    //int getManhattanDistance(int loc1, int loc2);
    list<pair<int,int>>getNeighbors(int location,int direction,int agentid,int time);
    bool validateMove(int loc, int loc2);
    int getManhattanDistance(int loc1, int loc2);
    bool update_solution();
    void update_cost();
    void add_constraint(Constraint_my &constraint);
    bool is_constraint(int agentid,int location,int time,std::vector<Constraint_my>&constraints);


    TreeNode();
    TreeNode(std::vector<Constraint_my>&constraints):constraints(constraints){};
    TreeNode(SharedEnvironment env):node_env(env){};
    TreeNode operator=(const TreeNode &node)const{
        TreeNode t(node.node_env);
        t.constraints=node.constraints;
        t.solution=node.solution;
        t.cost=node.cost;
        //cout<<"node.cost   t.cost: "<<node.cost<<"   "<<t.cost<<endl;
        return t;
    }
    bool operator==(const TreeNode &node)const{
        if(node.constraints==this->constraints&&node.solution==this->solution&&node.cost==this->cost) return true;
        else return false;
    }
    //TreeNode(std::vector<Constraint_my>&constraints);
    ~TreeNode();
    
};
class MAPFPlanner
{
public:
    
    SharedEnvironment* env;
    TreeNode root;
    std::vector<TreeNode> tree;
    std::atomic<bool> lock1;
    bool start=true;
	MAPFPlanner(SharedEnvironment* env): env(env){};
    MAPFPlanner(){
        env = new SharedEnvironment();
        //root=NULL;
        };
	virtual ~MAPFPlanner(){delete env;};


    virtual void initialize(int preprocess_time_limit);

    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> & plan);
    std::vector<Action> thread_plan();
    int getMinCost();
    std::vector<TreeNode> remove_node(TreeNode&p);
    // Start kit dummy implementation
    //std::vector<pair<int,int>>single_agent_plan(int start,int start_direct, int end,int agentid);
    //bool is_constraint(int agentid,int x,int y,int time,std::vector<Constraint_my>&constraints);
    //int getManhattanDistance(int loc1, int loc2);
    //std::list<pair<int,int>> getNeighbors(int location, int direction,int agentid,int time);
    //bool validateMove(int loc,int loc2);
    //void update_solution();
    //void add_constraint(Constraint_my & constraint);
    //void update_cost();
    TreeNode find_best_node();
    bool hasconflict(TreeNode&node);
    bool hasconflict(std::vector<pair<int,int>>&a,std::vector<pair<int,int>>&b);
    bool hasEdgeConflict(TreeNode&node);
    bool hasEdgeConflict(std::vector<pair<int,int>>&a,std::vector<pair<int,int>>&b);
    Conflict_my getFirstConflict(TreeNode&node);
    
};
