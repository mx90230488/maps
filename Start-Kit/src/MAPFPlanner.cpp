#include <MAPFPlanner.h>
#include <random>
#include<filesystem>
#include <thread>
#include<chrono>
#include<future>
#include<queue>
#include<unordered_map>
#include<unordered_set>
TreeNode::TreeNode() = default;
TreeNode::~TreeNode() = default;


struct AstarNode
{
    int location;
    int direction;
    int f,g,h;
    int focal;
    AstarNode* parent;
    int t = 0;
    heap_handle_t handle;
    bool closed = false;
    AstarNode(int _location,int _direction, int _g, int _h, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),parent(_parent) {}
    AstarNode(int _location,int _direction, int _g, int _h, int _t, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),t(_t),parent(_parent) {}
    bool operator<(const AstarNode&other) const{
        if(f!=other.f) return f>other.f;
        else return g<other.g;
    }
};



struct cmp
{
    bool operator()(AstarNode* a, AstarNode* b)
    {
        if(a->f == b->f) return a->g <= b->g;
        else return a->f > b->f;
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
/*
TreeNode MAPFPlanner::find_best_node(){
    int mi=getMinCost();
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
*/
/*
int MAPFPlanner::getMinCost(){
    int min=INT_MAX;
    for(auto &node:tree){
        if(node.cost<min){
            min=node.cost;
        }
    }
    return min;
}
*/
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
                    //cout<<"i :"<<i<<" j "<<j<<" solutions[i][k].first "<<solutions[i][k].first<<" k "<<k<<endl;
                    return Conflict_my(i,j,solutions[i][k].first,solutions[j][k].first,k);
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
                    return Conflict_my(i,j,solutions[i][k+1].first,solutions[j][k+1].first,k);
                }
                    
            }
        }
    }
    return Conflict_my(0,0,0,0,0);
}
int MAPFPlanner::focal_heuristic(vector<vector<pair<int,int>>>&solutions){
    int numconflict=0;
    int max_t=0;
    for(auto &sol:solutions){
        max_t=std::max<int>(max_t,sol.size());
    }
    for(int t=0;t<max_t;t++){
        //vertex conflict
        for(int i=0;i<solutions.size();i++){
            if(t>solutions[i].size()) continue;
            for(int j=i+1;j<solutions.size();j++){
                if(t>solutions[j].size()) continue;
                if(solutions[i][t].first==solutions[j][t].first){
                    numconflict++;
                    //cout<<"i "<<i<<" j "<<j<<" location "<<solutions[i][t].first<<endl;
                }
            }
        }
        //edge conflict
        for(int i=0;i<solutions.size();i++){
            if(t>solutions[i].size()-1) continue;
            for(int j=i+1;j<solutions.size();j++){
                if(t>solutions[j].size()-1) continue;
                if(solutions[i][t].first==solutions[j][t+1].first&&solutions[i][t+1].first==solutions[j][t].first){
                    //cout<<"i "<<i<<" j "<<j<<" location "<<solutions[i][t].first<<" "<<solutions[i][t+1].first<<endl;
                    numconflict++;
                }
            }
        }
    }
    return numconflict;
}
/*
std::vector<TreeNode> MAPFPlanner::remove_node(TreeNode &p){
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
    
    for(int i=0;i<tree.size();i++){
        cout<<"for before -- tree[i].cost "<<tree[i].cost<<endl;;
        if(tree[i]==p){
            cout<<"get p tree[i].cost "<<tree[i].cost<<endl;
            //tree.erase(tree.begin()+i);
            r=i;
        }
    }
    
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
*/
void MAPFPlanner::initialize(int preprocess_time_limit)
{
    /*
    std::filesystem::path currpath=std::filesystem::current_path();
    string path_str=currpath.string();
    cout<<" path_str "<<path_str<<endl;
    path_str+="/"env->map_name
    */
    cout << "planner initialize done" << endl;

}
vector<Action> MAPFPlanner::thread_plan(){
    
    //std::cout<<"time "<<env->curr_timestep<<std::endl;
    auto actions = std::vector<Action>(env->curr_states.size(), Action::W);
    int change=0;
    for(int i=0;i<env->num_of_agents;i++){
        if(ret.empty()){
            change=1;
            break;
        }
        if(ret[i].back().first!=env->goal_locations[i].front().first){
            change=1;
            ret.clear();
            break;
        }
    }
    if(start==true&&change==1){
        TreeNode start_node(*env);
        
        start_node.cost=0;

        start_node.update_solution();
        start_node.update_cost();
        start_node.focal=focal_heuristic(start_node.solution);
        auto handle=high_open_.push(start_node);
        (*handle).handle=handle;
        high_focal_.push(handle);
        //cout<<"r.cost : "<<r.cost<<endl;
        
    }
    
    
    //std::vector<std::vector<pair<int,int>>>ret;
    if(change==1){
        int best_cost=high_open_.top().cost;
        while(lock1==true&&!high_open_.empty()&&change==1){

            cout<<"open : "<<high_open_.size()<<" focal : "<<high_focal_.size()<<" focal "<<(*high_focal_.top()).focal<<endl;
            int old_cost=best_cost;
            best_cost=high_open_.top().cost;
            if(best_cost>old_cost){
                auto it=high_open_.ordered_begin();
                auto it_end=high_open_.ordered_end();
                for(;it!=it_end;it++){
                    int val=it->cost;
                    const auto &a=*it;
                    if(std::find(high_focal_.begin(),high_focal_.end(),a.handle)!=high_focal_.end()) continue;
                    if(val<=best_cost*1.3){
                        const TreeNode&n=*it;
                        high_focal_.push(n.handle);
                    }
                    if(val>best_cost*1.3) break;
                }
            }
            auto h=high_focal_.top();
            TreeNode p=*h;
            high_focal_.pop();
            high_open_.erase(h);

            
            //p=find_best_node(tree);
            //p=t;
            //cout<<"p.cost: "<<p.cost<<endl;
            //cout<<"tree.top().cost"<<tree.top().cost<<endl;
            //remove_node(tree,p);
            //cout<<tree[0].solution[3][8].first<<endl;
            if(!hasconflict(p)&&!hasEdgeConflict(p)){
                //cout<<"no conflict----\n"; 
                ret=p.solution;
                /*
                for(int i=0;i<ret.size();i++){
                    cout<<"agentid "<<i<<endl;
                    for(int j=0;j<ret[i].size();j++){
                        cout<<" "<<ret[i][j].first<<" "<<ret[i][j].second<<"-->";
                        if(env->map[ret[i][j].first]==1) cout<<"stop------------------\n";
                    }
                    cout<<"\n";
                }
                */
                break;
            }
            else if(hasconflict(p)){
                cout<<"conflict\n";
                Conflict_my conflict =getFirstConflict(p);
                //tree.pop_back();
                
                for(int i=0;i<2;i++){
                    TreeNode A=p;
                    int idx_1=conflict.conflict_agent.first;
                    int idx_2=conflict.conflict_agent.second;
                    //cout<<"idx_1  "<<idx_1<<" idx_2  "<<idx_2<<" location "<<conflict.location1<<" time "<<conflict.time<<endl;
                    auto new_constraint=Constraint_my(conflict.location1,conflict.conflict_agent.first,conflict.time);
                    if(i==1){
                        new_constraint=Constraint_my(conflict.location2,conflict.conflict_agent.second,conflict.time);
                    }  
                    A.add_constraint(new_constraint);
                    //cout<<"A .con back agentid"<<A.constraints.back().agentid<< "location "<<A.constraints.back().location<<" time "<<A.constraints.back().time<<endl;
                    //cout<<"before a.solution : "<<A.solution[idx_1].size()<<endl;
                    
                        auto path1=A.single_agent_plan(env->curr_states[idx_1].location,env->curr_states[idx_1].orientation,env->goal_locations[idx_1].front().first,idx_1);
                        if(path1.size()==0){
                            cout<<"empty\n";
                            continue;
                        }
                        A.solution[idx_1].resize(path1.size(),{0,0});
                        A.solution[idx_1]=path1;
                    
                        auto path2=A.single_agent_plan(env->curr_states[idx_2].location,env->curr_states[idx_2].orientation,env->goal_locations[idx_2].front().first,idx_2);
                        if(path2.size()==0){
                            cout<<"empty\n";
                            continue;
                        }
                        A.solution[idx_2].resize(path2.size(),{0,0});
                        A.solution[idx_2]=path2;
                    
                        
                    //cout<<"after a.solution : "<<A.solution[idx_1].size()<<endl;
                    //cout<<"A.constraints.size()  "<<A.constraints.size()<<endl;
                    
                    //if(A.update_solution()==false) continue;
                    
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
                    A.focal=focal_heuristic(A.solution);
                    //cout<<"A.focal : "<<A.focal<<endl;
                    if(A.cost<INT_MAX){
                        auto handle=high_open_.push(A);
                        (*handle).handle=handle;
                        if(A.cost<=best_cost*1.3){
                            high_focal_.push(handle);
                        }
                    }
                    //cout<<"push after--\n";
                }
                
            }else if(hasEdgeConflict(p)){
                cout<<"EdgeConflict----\n";
                auto conflict=getFirstConflict(p);
                //cout<<conflict.location1<<" "<<conflict.location2<<" "<<conflict.time<<" "<<conflict.conflict_agent.first<<" "<<conflict.conflict_agent.second<<endl;
                for(int i=0;i<2;i++){
                    TreeNode A=p;
                    int idx_1=conflict.conflict_agent.first;
                    int idx_2=conflict.conflict_agent.second;
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
                    /*
                    cout<<"constraint\n";
                    for(int i=0;i<A.constraints.size();i++){
                        cout<<A.constraints[i].location<<" "<<A.constraints[i].agentid<<" "<<A.constraints[i].time<<endl;
                    }
                    cout<<"constraint after\n";
                    */
                        auto path1=A.single_agent_plan(env->curr_states[idx_1].location,env->curr_states[idx_1].orientation,env->goal_locations[idx_1].front().first,idx_1);
                        if(path1.size()==0){
                            cout<<"e_empty\n";
                            continue;
                        }
                        //cout<<"A.idx_1 "<<A.solution[idx_1].size()<<" - ";
                        A.solution[idx_1].resize(path1.size(),{0,0});
                        A.solution[idx_1]=path1;
                        //cout<<"A.idx_1 "<<A.solution[idx_1].size()<<endl;;
                    
                        auto path2=A.single_agent_plan(env->curr_states[idx_2].location,env->curr_states[idx_2].orientation,env->goal_locations[idx_2].front().first,idx_2);
                        if(path2.size()==0){
                            cout<<"e_empty\n";
                            continue;
                        }
                        //cout<<"A.idx_2 "<<A.solution[idx_2].size()<<" - ";
                        A.solution[idx_2].resize(path2.size(),{0,0});
                        A.solution[idx_2]=path2;
                        //cout<<"A.idx_2 "<<A.solution[idx_2].size()<<endl;
                    
                    //cout<<"e_path--\n";
                    
                    //if(A.update_solution()==false) continue;
                    
                    
                    A.update_cost();
                    A.focal=focal_heuristic(A.solution);
                    //cout<<"A.focal : "<<A.focal<<endl;
                    if(A.cost<INT_MAX){
                        auto handle=high_open_.push(A);
                        (*handle).handle=handle;
                        if(A.cost<=best_cost*1.3){
                            high_focal_.push(handle);
                        }
                    }
                    //cout<<"e_push--\n";
                }
                //remove_node(tree,p);
            }
        }
    }else{
        for(int i=0;i<ret.size();i++){
            ret[i].erase(ret[i].begin());
        }
    }
        if(lock1==true){
            high_open_.clear();
            high_focal_.clear();
        }
            
        for(int i=0;i<ret.size();i++){
            if(ret[i].size()==1){
                
                actions[i]=Action::W;
            }
                else if(ret[i][1].first!=env->curr_states[i].location){
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
        
        return actions;
    
    //cout<<"ret[0][1].first"<<ret[0][1].first<<endl;

}

// plan using simple A* that ignores the time dimension
void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{
    lock1=true;
    actions = std::vector<Action>(env->curr_states.size(), Action::W);
    std::packaged_task<std::vector<Action>()>task1(std::bind(&MAPFPlanner::thread_plan,this));
    auto future1=task1.get_future();
    auto task_td1=std::thread(std::move(task1));
    std::future_status ret=future1.wait_for(std::chrono::milliseconds(time_limit*800));
    if(ret==std::future_status::ready){
        
        task_td1.join();
        start=true;
        //cout<<"ready\n";
        actions=future1.get();
    }else if(ret==std::future_status::timeout){
        lock1=false;
        start=false;
        task_td1.join();
        //cout<<"timeout\n";
        //cout<<tree.size()<<endl;
        lock1=true;
    }
    //cout<<"plan map: "<<env->map_name<<endl;
    
    //priority_queue<TreeNode,vector<TreeNode>,cmp1>tree;
    
    
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
    //cout<<"start "<<start<<" start_direct "<<start_direct<<" end "<<end<<" agentid "<<agentid<<" time "<<node_env.curr_timestep<<"-----------------------"<<endl;
    
    //cout<<"single_agent_plan  time: "<<node_env.curr_timestep<<endl;
    //int plan_time=node_env.curr_timeste
    auto start_state=State_my(start,0,start_direct);
    std::vector<pair<int,int>> path;
    open_t open_set;
    focal_t focal_set;
    std::unordered_map<State_my,heap_handle_t,std::hash<State_my>>state_heap;
    std::unordered_set<State_my,std::hash<State_my>> closed_set;
    
    std::unordered_map<State_my,State_my,std::hash<State_my>>cameFrom;
    Node start_node(start_state,getManhattanDistance(start,end),0,0);
    
    auto handle=open_set.push(start_node);
    state_heap.insert(std::make_pair<>(start_state,handle));
    (*handle).handle=handle;
    //std::cout<<"*handle\n";
    focal_set.push(handle);
    int best_cost=(*handle).fscore;
    //AstarNode* s = new AstarNode(start, start_direct, 0, getManhattanDistance(start,end), nullptr);
    //s->t=0;
    //open_list.push(s);
    //all_nodes[start*4 + start_direct] = s;
    //int find=0;
    //cout<<"agentid : "<<agentid<<endl;
    int mx=0;
    while (!open_set.empty())
    {
        //cout<<"focal top "<<(*focal_set.top()).state.location<<endl;
        
        //cout<<"low open size "<<open_set.size()<<" focal size "<<focal_set.size()<<endl;
        //mx=std::max(mx,(int)open_set.size());
        
        int old_best=best_cost;
        best_cost=open_set.top().fscore;
        //cout<<"best cost "<<best_cost<<" location "<<open_set.top().state.location<<endl;
        if(best_cost>old_best){
            auto iter=open_set.ordered_begin();
            auto iter_end=open_set.ordered_end();
            for(;iter!=iter_end;iter++){
                const auto&s=*iter;
                //cout<<"fscore in open : "<<s.fscore<<endl;
                //if(std::find(focal_set.begin(),focal_set.end(),s.handle)!=focal_set.end()) continue;
                int val=(*iter).fscore;
                if(val>old_best*1.3&&val<=best_cost*1.3){
                    const Node&n=*iter;
                    focal_set.push(n.handle);
                    //cout<<"cost "<<val<<endl;
                    //cout<<"best_cost>old_best\n";
                }
                if(val>best_cost*1.3) break;
            }
            //cout<<"------------"<<endl;
        }
        //cout<<"best\n";
        //cout<<"after open size "<<open_set.size()<<" focal size "<<focal_set.size()<<endl;
        auto curr=focal_set.top();
        Node current=*curr;
        //cout<<"current "<<current->state.location<<" "<<current->state.orientation<<endl;
        //if(current->parent!=NULL) cout<<" current.parent "<<(current->parent)->state.location<<" "<<(current->parent)->state.orientation<<endl;
        if(current.state.location==end){
            /*
            if(agentid==1){
                cout<<"------------\n";
            for(auto c:cameFrom){
                cout<<"neighbor "<<c.first.location<<"-"<<c.first.orientation<<" time "<<c.first.timestep<<" current "<<c.second.location<<"-"<<c.second.orientation<<" time "<<c.second.timestep<<endl;
            }
            }
            */
            auto it=cameFrom.find(current.state);
            
                //cout<<"agentid "<<agentid<<endl;
            //auto it2=it->second;
            while(it!=cameFrom.end()){
                path.insert(path.begin(),std::make_pair<>(it->first.location,it->first.orientation));
                
                    //cout<<"neighbor "<<it->first.location<<"-"<<it->first.orientation<<" cur "<<it->second.location<<"-"<<it->second.orientation<<" ";
                //it2=it->second;
                it=cameFrom.find(it->second);
            }
            //if(start_state.location!=it2.location) cout<<"-----------===========--------------------\n";
            path.insert(path.begin(),{start_state.location,start_state.orientation});
            
            
            break;
        }
        focal_set.pop();
        open_set.erase(curr);
        //cout<<"open_set erase--\n";
        state_heap.erase(current.state);
        closed_set.insert(current.state);
        //cout<<"open_set after\n";
        //closed_set.insert(current.state);
        //cout<<"pop\n";
        std::vector<State_my>neighbors=getNeighbors(current.state);
        for(auto neighbor:neighbors){
            //cout<<"neighbor----------\n";
            
            if(is_constraint(agentid,neighbor.location,current.state.timestep+1,constraints)){
                //cout<<"con agentid "<<agentid<<" location "<<neighbor.location<<" time "<<current.state.timestep+1<<endl;
                continue;
            }
            /*
            int flag=0;
            if(neighbor.timestep==1){
                for(int j=0;j<node_env.curr_states.size();j++){
                    //cout<<"curr "<<curr.location<<endl;
                    if(neighbor.location==node_env.curr_states[j].location&&agentid!=j){
                        cout<<"neigbor location "<<neighbor.location<<" j "<<j<<endl;
                        flag=1;
                        break;
                    }
                }
            }
            if(flag==1) continue;
            */
            //cout<<"constratint\n";
            if(closed_set.find(neighbor)==closed_set.end()){
                int curr_g=current.gscore+1;
                auto iter=state_heap.find(neighbor);
                if(iter==state_heap.end()){
                    //cout<<"location "<<neighbor.location<<" "<<neighbor.orientation<<endl;
                    int fscore=curr_g+getManhattanDistance(neighbor.location,end);
                    int focal=current.focal+vertex_conflict(neighbor,agentid)+edge_conflict(current.state,neighbor,agentid);
                    //cout<<"conflict\n";
                    Node next_node(neighbor,fscore,curr_g,focal);
                    
                    //Node *next_node=new Node(neighbor,fscore,curr_g,focal,&current);
                    //cout<<"next node "<<(next_node)->state.location<<" current node "<<(next_node->parent)->state.location<<endl;
                    auto handle=open_set.push(next_node);
                    (*handle).handle=handle;
                    state_heap.insert(std::make_pair<>(neighbor,handle));
                    
                    if(fscore<=best_cost*1.3){
                        focal_set.push(handle);
                        //cout<<"handle.fscore 1 "<<(*handle).fscore<<endl;
                        //cout<<"focal push\n";
                    }
                    //state_heap.insert(std::make_pair<>(neighbor,handle));
                    //cout<<"iter==end()\n";
                }else{
                    auto handle=iter->second;
                    if(curr_g>=(*handle).gscore) continue;
                    int last_g=(*handle).gscore;
                    int last_f=(*handle).fscore;
                    int d=last_g-curr_g;
                    (*handle).gscore=curr_g;
                    (*handle).fscore-=d;
                    open_set.update(handle);
                    if(std::find(focal_set.begin(),focal_set.end(),handle)!=focal_set.end()) continue;
                    if((*handle).fscore<=best_cost*1.3){
                        //cout<<"handle.fscore 2 "<<(*handle).fscore<<endl;
                        focal_set.push(handle);
                        
                        //cout<<"focal push "<<(**handle).state.location<<endl;
                    }
                    //cout<<"iter!=end()\n";
                }
                cameFrom.erase(neighbor);
                cameFrom.insert(std::make_pair<>(neighbor,current.state));
                /*
                if(agentid==1){
                cout<<"neighbor "<<neighbor.location<<"-"<<neighbor.orientation<<" time "<<neighbor.timestep<<" current "<<current.state.location<<"-"<<current.state.orientation<<" time "<<current.state.timestep<<endl;
                if(cameFrom.find(State_my(492,4,1))!=cameFrom.end()) cout<<"find-----------\n";
                }
                */
                
            }
        }
        
        
        //cout<<"neigbor after--\n";

    }
    //cout<<"mx "<<mx<<endl;
        if(path.size()==0){
            path.push_back({node_env.curr_states[agentid].location,node_env.curr_states[agentid].orientation});
            int temp=node_env.curr_states[agentid].orientation+1;
            if(temp+1==4){
                temp=0;
            }
            path.push_back({node_env.curr_states[agentid].location,temp});

        } 
    //cout<<"-------------\n";
    return path;
}
int TreeNode::vertex_conflict(State_my&s,int agentid){
    int numconflict=0;
    for(int i=0;i<solution.size();i++){
        if(i!=agentid&&solution[i].size()>s.timestep){
            if(s.location==solution[i][s.timestep].first){
                numconflict++;
            }
        }
    }
    return numconflict;
}

int TreeNode::edge_conflict(State_my&s1,State_my&s2,int agentid){
    int numconflict=0;
    for(int i=0;i<solution.size();i++){
        if(agentid!=i&&solution[i].size()>s2.timestep){
            if(s2.location==solution[i][s1.timestep].first&&s1.location==solution[i][s2.timestep].first){
                numconflict++;
            }
        }
    }
    return numconflict;
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


std::vector<State_my> TreeNode::getNeighbors(State_my &current)
{
    std::vector<State_my> neighbors;
    //forward
    int candidates[4] = { current.location + 1,current.location + node_env.cols, current.location - 1, current.location - node_env.cols};
    int forward = candidates[current.orientation];
    int new_direction = current.orientation;
    //cout<<"getNeighbors  agentid: "<<agentid<<" forward : "<<forward<<" time : "<<time<<endl;
    
    if (forward>=0 && forward < node_env.map.size() && validateMove(forward,current.location))
        neighbors.emplace_back(State_my(forward,current.timestep+1,new_direction));
    //turn left
    new_direction = current.orientation-1;
    if (new_direction == -1)
        new_direction = 3;
    neighbors.emplace_back(State_my(current.location,current.timestep+1,new_direction));
    //turn right
    new_direction = current.orientation+1;
    if (new_direction == 4)
        new_direction = 0;
    neighbors.emplace_back(State_my(current.location,current.timestep+1,new_direction));
    neighbors.emplace_back(State_my(current.location,current.timestep+1,current.orientation));//wait
    return neighbors;
}
