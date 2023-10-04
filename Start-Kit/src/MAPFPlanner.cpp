#include <MAPFPlanner.h>
#include <random>
TreeNode::TreeNode() = default;
TreeNode::~TreeNode() = default;

struct AstarNode
{
    int location;
    int direction;
    int f,g,h;
    AstarNode* parent;
    int t = 0;
    bool closed = false;
    AstarNode(int _location,int _direction, int _g, int _h, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),parent(_parent) {}
    AstarNode(int _location,int _direction, int _g, int _h, int _t, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),t(_t),parent(_parent) {}
};


struct cmp
{
    bool operator()(AstarNode* a, AstarNode* b)
    {
        if(a->f == b->f) return a->g <= b->g;
        else return a->f > b->f;
    }
};
struct cmp1
{
    bool operator()(TreeNode&a,TreeNode&b)
    {
        return a.cost>b.cost;
    }
};
void TreeNode::add_constraint(Constraint_my &constraint){
    this->constraints.emplace_back(constraint);
}
bool TreeNode::update_solution(){
    std::vector<vector<pair<int,int>>>s;
    for(int i=0;i<node_env.num_of_agents;i++){
        std::vector<pair<int,int>>path;
        path=single_agent_plan(node_env.curr_states[i].location,
                                node_env.curr_states[i].orientation,
                                node_env.goal_locations[i].front().first,
                                i);
        if(path.size()==0) return false;
        s.emplace_back(path);
        /*
        for(auto p:path){
            cout<<"agentid "<<i<<" "<<p.first<<" "<<p.second<<endl;
        }
        */
        
    }
    this->solution=s;
    return true;
}

void TreeNode::update_cost(){
    int cost_t=0;
    for(auto &route:solution){
        cost_t+=route.size();
    }
    this->cost=cost_t;
}
TreeNode MAPFPlanner::find_best_node(std::vector<TreeNode>&tree){
    int mi=getMinCost(tree);
    //TreeNode ret(*env);
    for(int i=0;i<tree.size();i++){
        if(tree[i].cost==mi){
            //mi=tree[i].cost;
            //ret=tree[i];
            //cout<<"path : "<<i<<"tree[i].cost : "<<tree[i].cost<<endl;
            return tree[i];
        }
        //cout<<"i : "<<i<<endl;
    }
    return TreeNode();
}
int MAPFPlanner::getMinCost(std::vector<TreeNode>&tree){
    int min=INT_MAX;
    for(auto &node:tree){
        if(node.cost<min){
            min=node.cost;
        }
    }
    return min;
}
bool MAPFPlanner::hasconflict(TreeNode&node){
    auto solutions=node.solution;
    //cout<<"hasconflict   solutions[0][0].first  "<<solutions[0][0].first<<endl;;
    for(int i=0;i<solutions.size();i++){
        for(int j=i+1;j<solutions.size();j++){
            if(hasconflict(solutions[i],solutions[j])){
                //cout<<"cnoflict i : "<<i<<" j : "<<j<<endl;
                return true;
            }
        }
    }
    return false;
}
bool MAPFPlanner::hasconflict(vector<pair<int,int>>&a,vector<pair<int,int>>&b){
    int min_idx=std::min(a.size(),b.size());
    for(int i=0;i<min_idx;i++){
        if(a[i].first==b[i].first){
            //cout<<"a[i].first"<<a[i].first<<endl;
            return true;
        }
    }
    return false;
}
bool MAPFPlanner::hasEdgeConflict(TreeNode&node){
    auto solutions=node.solution;
    for(int i=0;i<solutions.size();i++){
        for(int j=i+1;j<solutions.size();j++){
            if(hasEdgeConflict(solutions[i],solutions[j])){
                //cout<<"EdgeConflict i : "<<i<<" j : "<<j<<endl;
                return true;
            }
        }
    }
    return false;
}
bool MAPFPlanner::hasEdgeConflict(vector<pair<int,int>>&a,vector<pair<int,int>>&b){
    int min_idx=std::min(a.size(),b.size())-1;
    for(int i=0;i<min_idx;i++){
        if(a[i].first==b[i+1].first&&a[i+1].first==b[i].first){
            //cout<<"i : "<<i<<" j : "<<j<<endl;
            //cout<<"a[i].first : "<<a[i].first<<" a[i+1].first : "<<a[i+1].first<<endl;
            return true;
        }
    }
    return false;
}
Conflict_my MAPFPlanner::getFirstConflict(TreeNode&node){
    //cout<<"curr_timestep : "<<node.node_env.curr_timestep<<endl;
    auto solutions=node.solution;
    //cout<<solutions[0][0].first<<endl;
    for(int i=0;i<solutions.size();i++){
        for(int j=i+1;j<solutions.size();j++){
            int min_idx=std::min(solutions[i].size(),solutions[j].size());
            for(int k=0;k<min_idx;k++){
                if(solutions[i][k].first==solutions[j][k].first){
                    //cout<<"i :"<<i<<" j "<<j<<" solutions[i][k].first "<<solutions[i][k].first<<" node.node_env.curr_timestep+k "<<node.node_env.curr_timestep+k<<endl;
                    return Conflict_my(i,j,solutions[i][k].first,solutions[j][k].first,node.node_env.curr_timestep+k);
                }
                    
            }
        }
    }
    for(int i=0;i<solutions.size();i++){
        for(int j=i+1;j<solutions.size();j++){
            int min_idx=std::min(solutions[i].size(),solutions[j].size())-1;
            for(int k=0;k<min_idx;k++){
                //cout<<solutions[i][k].first<<" "<<solutions[j][k+1].first<<" "<<solutions[i][k+1].first<<" "<<solutions[j][k].first<<endl;
                if(solutions[i][k].first==solutions[j][k+1].first&&solutions[i][k+1].first==solutions[j][k].first){
                    //cout<<"solutions[i][k+1].first "<<solutions[i][k+1].first<<" solutions[j][k+1] "<<solutions[j][k+1].first<<" time: "<<node.node_env.curr_timestep+k<<endl;
                    return Conflict_my(i,j,solutions[i][k+1].first,solutions[j][k+1].first,node.node_env.curr_timestep+k);
                }
                    
            }
        }
    }
    return Conflict_my(0,0,0,0,0);
}
std::vector<TreeNode> MAPFPlanner::remove_node(std::vector<TreeNode>&tree,TreeNode &p){
    vector<TreeNode>temp;
    auto it=std::find(tree.begin(),tree.end(),p);
    //cout<<"it "<<it->cost<<endl;
    for(auto its=tree.begin();its!=tree.end();its++){
        if(its==it) continue;
        temp.emplace_back(*its);
        //cout<<"temp "<<its->cost<<endl;
    }
    return temp;
    tree=temp;
    //tree.erase(it);
    /*
    for(int i=0;i<tree.size();i++){
        cout<<"for before -- tree[i].cost "<<tree[i].cost<<endl;;
        if(tree[i]==p){
            cout<<"get p tree[i].cost "<<tree[i].cost<<endl;
            //tree.erase(tree.begin()+i);
            r=i;
        }
    }
    */
    //cout<<"r "<<r<<endl;
    //cout<<(tree.begin()+r)->cost<<endl;
    //tree.erase(tree.begin()+r);
    for(int i=0;i<tree.size();i++){
        cout<<"for after -- tree[i].cost "<<tree[i].cost<<endl;;
    }
    cout<<"auto\n";
    for(auto i:tree){
        cout<<i.cost<<endl;
    }
    
}
void MAPFPlanner::initialize(int preprocess_time_limit)
{
    cout << "planner initialize done" << endl;
}


// plan using simple A* that ignores the time dimension
void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{
    //cout<<"plan env_time: "<<env->curr_timestep<<endl;
    actions = std::vector<Action>(env->curr_states.size(), Action::W);
    //priority_queue<TreeNode,vector<TreeNode>,cmp1>tree;
    vector<TreeNode>tree;
    TreeNode r(*env);
    r.update_solution();
    r.update_cost();
    //cout<<"r.cost : "<<r.cost<<endl;
    tree.push_back(r);
    std::vector<std::vector<pair<int,int>>>ret;
    while(!tree.empty()){
        //cout<<"tree.size() "<<tree.size()<<endl;
        //cout<<"time: "<<env->curr_timestep<<endl;
        //std::vector<TreeNode>te;
        //cout<<"before q"<<endl;
        TreeNode p=find_best_node(tree);
        p.node_env=*env;
        tree=remove_node(tree,p);
        
        //p=find_best_node(tree);
        //p=t;
        //cout<<"p.cost: "<<p.cost<<endl;
        //cout<<"tree.top().cost"<<tree.top().cost<<endl;
        //remove_node(tree,p);
        //cout<<tree[0].solution[3][8].first<<endl;
        if(!hasconflict(p)&&!hasEdgeConflict(p)){
            //cout<<"no conflict----\n";
            //cout<<"p.solution "<<p.solution[0].size()<<endl;
            ret=p.solution;
            break;
        }
        else if(hasconflict(p)){
            
            //cout<<"conflict\n";
        
            
            Conflict_my conflict =getFirstConflict(p);
            //tree.pop_back();
            
            for(int i=0;i<2;i++){
                TreeNode A=p;
                auto new_constraint=Constraint_my(conflict.location1,conflict.conflict_agent.first,conflict.time);
                if(i==1)
                    new_constraint=Constraint_my(conflict.location2,conflict.conflict_agent.second,conflict.time);
                A.add_constraint(new_constraint);
                
                //cout<<"A.constraints.size()  "<<A.constraints.size()<<endl;
                
                if(A.update_solution()==false) continue;
                for(auto a:A.constraints){
                    //cout<<"a.agentid "<<a.agentid<<" a.location "<<a.location<<" a.time "<<a.time<<endl;
                }
                /*
                cout<<"solution--------------\n";
                for(int k=0;k<A.solution.size();k++){
                    cout<<"agent "<<k<<endl;
                    for(int i=0;i<A.solution[k].size();i++){
                        cout<<"location "<<A.solution[k][i].first<<" direction "<<A.solution[k][i].second<<endl;
                    }
                }
                */
                A.update_cost();
                if(A.cost<INT_MAX) tree.emplace_back(A);
            }
            
        }else if(hasEdgeConflict(p)){
            //cout<<"EdgeConflict----\n";
            auto conflict=getFirstConflict(p);
            //cout<<conflict.location1<<" "<<conflict.location2<<" "<<conflict.time<<" "<<conflict.conflict_agent.first<<" "<<conflict.conflict_agent.second<<endl;
            for(int i=0;i<2;i++){
                TreeNode A=p;
                //cout<<"A.cost : "<<A.cost<<"p.cost : "<<p.cost<<endl;
                auto new_constraint=Constraint_my(conflict.location2,conflict.conflict_agent.first,conflict.time);
                auto constraint2=Constraint_my(conflict.location1,conflict.conflict_agent.first,conflict.time+1);
                if(i==1){
                    new_constraint=Constraint_my(conflict.location1,conflict.conflict_agent.second,conflict.time);
                    constraint2=Constraint_my(conflict.location2,conflict.conflict_agent.second,conflict.time+1);
                }
                //cout<<"before: A.constraints.size():  "<<A.constraints.size()<<endl;
                A.add_constraint(new_constraint);
                A.add_constraint(constraint2);
                //cout<<"A.constraints.size()  "<<A.constraints.size()<<endl;
                //cout<<"after: A.constraints.size():  "<<A.constraints.size()<<endl;
                if(A.update_solution()==false) continue;
                
                for(auto a:A.constraints){
                    //cout<<"a.agentid "<<a.agentid<<" a.location "<<a.location<<" a.time "<<a.time<<endl;
                }
                A.update_cost();
                
                if(A.cost<INT_MAX) tree.emplace_back(A);
            }
            //remove_node(tree,p);
        }

    }
    //cout<<"ret[0][1].first"<<ret[0][1].first<<endl;
    for(int i=0;i<ret.size();i++){
        if(ret[i][1].first!=env->curr_states[i].location){
            actions[i]=Action::FW;
            //cout<<"agent "<<i<<" fw "<<endl;
        }
        else if(ret[i][1].second!=env->curr_states[i].orientation){
            int incur=ret[i][1].second-env->curr_states[i].orientation;
            if(incur==1||incur==-3){
                actions[i]=Action::CR;
                //cout<<"agent "<<i<<" cr "<<endl;
            }
            else if(incur==-1||incur==3){
                actions[i]=Action::CCR;
                //cout<<"agent "<<i<<" ccr "<<endl;
            }
        }
        //cout<<"agent "<<i<<" wait "<<endl;
    }
    //cout<<"action after\n";
    /*
    for (int i = 0; i < env->num_of_agents; i++) 
    {
        vector<pair<int,int>> path;
        if (env->goal_locations[i].empty()) 
        {
            path.push_back({env->curr_states[i].location, env->curr_states[i].orientation});
        } 
        else 
        {
            //cout<<"env->curr_states[i].location "<<i<<" "<<env->curr_states[i].location<<endl;
            //cout<<"env->curr_states[i].timestep "<<i<<" "<<env->curr_states[i].timestep<<endl;
            path = single_agent_plan(env->curr_states[i].location,
                                    env->curr_states[i].orientation,
                                    env->goal_locations[i].front().first,
                                    i);
        }
        if (path.front().first != env->curr_states[i].location)
        {
            actions[i] = Action::FW; //forward action
        } 
        else if (path.front().second!= env->curr_states[i].orientation)
        {
            int incr = path.front().second - env->curr_states[i].orientation;
            if (incr == 1 || incr == -3)
            {
                actions[i] = Action::CR; //C--counter clockwise rotate
            } 
            else if (incr == -1 || incr == 3)
            {
                actions[i] = Action::CCR; //CCR--clockwise rotate
            } 
        }

    }
    */
    //root.solution.clear();
  return;
}
bool TreeNode::is_constraint(int agentid,int location,int time,std::vector<Constraint_my>&constraints){
    //cout<<"location : "<<location<<" agentid : "<<agentid<<" time : ---"<<time<<endl;
    for(Constraint_my c:constraints){
        //cout<<"c.location : "<<c.location<<" c.agentid : "<<c.agentid<<" c.time : "<<c.time<<endl;
        if(agentid==c.agentid){
            if(time==c.time&&location==c.location){
                //cout<<"agentid: "<<agentid<<" location: "<<location <<endl;
                return true;
            }
        }
    }
    return false;
}

vector<pair<int,int>> TreeNode::single_agent_plan(int start,int start_direct,int end,int agentid)
{
    //cout<<"start "<<start<<" start_direct "<<start_direct<<" end "<<end<<" agentid "<<agentid<<endl;
    //cout<<"single_agent_plan  time: "<<node_env.curr_timestep<<endl;
    int plan_time=node_env.curr_timestep;
    vector<pair<int,int>> path;
    
    priority_queue<AstarNode*,vector<AstarNode*>,cmp> open_list;
    unordered_map<int,AstarNode*> all_nodes;
    unordered_set<int> close_list;
    AstarNode* s = new AstarNode(start, start_direct, 0, getManhattanDistance(start,end), nullptr);
    s->t=plan_time;
    open_list.push(s);
    all_nodes[start*4 + start_direct] = s;
    //int find=0;

    while (!open_list.empty())
    {
        int len=open_list.size();
        //cout<<"len : "<<len<<endl;
        
            AstarNode* curr = open_list.top();
            open_list.pop();
            close_list.emplace(curr->location*4 + curr->direction);
            if (curr->location == end)
            {
                while(curr->parent!=NULL) 
                {
                    path.insert(path.begin(),make_pair(curr->location,curr->direction));
                    //path.emplace_front(make_pair(curr->location, curr->direction));
                    //cout<<"curr->location  "<<curr->location<<"  curr->direction  "<<curr->direction<<endl;
                    //find=1;
                    curr = curr->parent;
                }
                path.insert(path.begin(),make_pair(curr->location,curr->direction));
                break;
            }
            list<pair<int,int>> neighbors = getNeighbors(curr->location, curr->direction,agentid,plan_time+1);
            //cout<<"curr->location  "<<curr->location<<"  curr->direction  "<<curr->direction<<" agentid "<<agentid<<" curr->t "<<curr->t<<endl;
            for (const pair<int,int>& neighbor: neighbors)
            {
                //cout<<"neighbor.first "<<neighbor.first<<" curr->t+1 "<<curr->t+1<<endl;
                if(is_constraint(agentid,neighbor.first,curr->t+1,constraints)){
                    //cout<<"curr->location  "<<curr->location<<"  curr->direction  "<<curr->direction<<" agentid "<<agentid<<" curr->t "<<curr->t<<endl;
                    //cout<<"continue agentid : "<<agentid<<" neighbor.first : "<<neighbor.first<<" cur->t+1 : "<<curr->t+1<<endl;
                    continue;
                }
                if (close_list.find(neighbor.first*4 + neighbor.second) != close_list.end())
                    continue;
                if (all_nodes.find(neighbor.first*4 + neighbor.second) != all_nodes.end())
                {
                    AstarNode* old = all_nodes[neighbor.first*4 + neighbor.second];
                    if (curr->g + 1 < old->g)
                    {
                        old->g = curr->g+1;
                        old->f = old->h+old->g;
                        old->parent = curr;
                        old->t=curr->t+1;
                    }
                }
                else
                {
                    AstarNode* next_node = new AstarNode(neighbor.first, neighbor.second,
                        curr->g+1,getManhattanDistance(neighbor.first,end), curr);
                    next_node->t=curr->t+1;
                    open_list.push(next_node);
                    all_nodes[neighbor.first*4+neighbor.second] = next_node;
                }
            }
        
        //if(find) break;
        //plan_time++;
    }
    for (auto n: all_nodes)
    {
        delete n.second;
    }
    all_nodes.clear();
    
    return path;
}


int TreeNode::getManhattanDistance(int loc1, int loc2)
{
    int loc1_x = loc1/node_env.cols;
    int loc1_y = loc1%node_env.cols;
    int loc2_x = loc2/node_env.cols;
    int loc2_y = loc2%node_env.cols;
    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}


bool TreeNode::validateMove(int loc, int loc2)
{
    int loc_x = loc/node_env.cols;
    int loc_y = loc%node_env.cols;

    if (loc_x >= node_env.rows || loc_y >= node_env.cols || node_env.map[loc] == 1)
        return false;

    int loc2_x = loc2/node_env.cols;
    int loc2_y = loc2%node_env.cols;
    if (abs(loc_x-loc2_x) + abs(loc_y-loc2_y) > 1)
        return false;
    
    return true;

}


list<pair<int,int>> TreeNode::getNeighbors(int location,int direction,int agentid,int time)
{
    list<pair<int,int>> neighbors;
    //forward
    int candidates[4] = { location + 1,location + node_env.cols, location - 1, location - node_env.cols};
    int forward = candidates[direction];
    int new_direction = direction;
    //cout<<"getNeighbors  agentid: "<<agentid<<" forward : "<<forward<<" time : "<<time<<endl;
    if (forward>=0 && forward < node_env.map.size() && validateMove(forward,location))
        neighbors.emplace_back(make_pair(forward,new_direction));
    //turn left
    new_direction = direction-1;
    if (new_direction == -1)
        new_direction = 3;
    neighbors.emplace_back(make_pair(location,new_direction));
    //turn right
    new_direction = direction+1;
    if (new_direction == 4)
        new_direction = 0;
    neighbors.emplace_back(make_pair(location,new_direction));
    neighbors.emplace_back(make_pair(location,direction)); //wait
    return neighbors;
}
