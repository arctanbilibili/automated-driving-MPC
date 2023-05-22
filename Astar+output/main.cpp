#include <iostream>
#include <vector>
#include <queue>
#include <unistd.h>
#include <fcntl.h>
#include <unordered_map>
#include <algorithm>
using namespace std;

//在地图上打印路径
void printPaze(vector<vector<int>>& paze){
    for(int i=0;i<paze.size();++i){
        for(int j=0;j<paze[0].size();++j){
            if(paze[i][j]==9)
                std::cout<<"\033[0m\033[1;31m*\033[0m"<<" ";
            else if(paze[i][j]==8)
                std::cout<<"\033[0m\033[1;32m*\033[0m"<<" ";
            else
                std::cout<<paze[i][j]<<" ";
        }
        std::cout<<endl;
    }
}

//barrier障碍物密度输入0-1
vector<vector<int>> makePaze(int row,int col,float barrier){
    vector<vector<int>> paze(row,vector<int>(col));
    int base,upper;
    base = 1000;//精度1000
    upper = base*barrier;
    for(int i=0;i<row;++i){
        for(int j=0;j<col;++j){
            if(rand()%base <= upper){
                paze[i][j] = 1;
            }else{
                paze[i][j] = 0;
            }
        }
    }
    return paze;
}

/*
    默认8方向A*寻路，返回路径步数，存储路径到path
    \ | /
    - . -
    / | \
*/
vector<pair<int,int>> path;
int Astar(vector<vector<int>> paze,pair<int,int> s,pair<int,int> e){
    auto cmp = [&](pair<int,int>&l,pair<int,int>&r){//小的在前
        //曼哈顿距离
        return abs(l.first-e.first)+abs(l.second-e.second) > abs(r.first-e.first)+abs(r.second-e.second);
    };
    vector<pair<int,int>> dirs{{0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1}};//8方向寻路
    //vector<pair<int,int>> dirs{{0,1},{1,0},{0,-1},{-1,0}};//4方向寻路
    priority_queue<pair<int,int>,vector<pair<int,int>>,decltype(cmp)> que(cmp);
    int m=paze.size(),n=paze[0].size();
    unordered_map<int,int> cp;//哈希表记录路径 child -> parent
    paze[e.first][e.second] = 0;
    que.push(s);
    cp[s.first*n+s.second] = -1;//起始地址指向-1
    paze[s.first][s.second] = 1;
    while(!que.empty()){
        auto top = que.top();
        que.pop();
        if(top.first == e.first && top.second == e.second){
            while(cp.count(top.first*n+top.second)){
                path.push_back(top);
                int pa = cp[top.first*n+top.second];
                top = {pa/n,pa%n};
            }
            reverse(path.begin(),path.end());//反转
            break;
        }
        for(int i=0;i<dirs.size();++i){
            int x0 = top.first + dirs[i].first;
            int y0 = top.second + dirs[i].second;
            if(x0>=0 && y0>=0 && x0<m && y0<n && !paze[x0][y0]){
                paze[x0][y0] = 1;
                que.push({x0,y0});
                cp[x0*n+y0] = top.first*n+top.second;//记录路径
            }
        }
    }
    return path.size();
}

//只打印路径
void printPath(vector<pair<int,int>>& path){
    for(int i=0;i<path.size();++i){
        std::cout<<"("<<path[i].first<<","<<path[i].second<<")->";
    }
    std::cout<<endl;
}
//弗洛伊德平滑-未使用
vector<pair<int,int>> floydSmooth(vector<pair<int,int>> path){
    if(path.empty())
        return {};
    int len = path.size();
    //去掉同一条线上的点。
    if (len > 2) {
        pair<int,int> vector{path[len-1].first-path[len-2].first , path[len-1].second-path[len-2].second};//二维方向
        pair<int,int> tempvector;
        for (int i = len - 3; i>= 0; i--) {
            tempvector = {path[i+1].first-path[i].first , path[i+1].second-path[i].second};
            if (tempvector == vector){
                path.erase(path.begin()+i+1);//删掉i+1的元素
            }
            else{
                vector = tempvector;
            }
        }
    }
    return path;
}

//输出地图到map.txt 输出路径到path.txt
int output2file(vector<vector<int>>& paze,vector<pair<int,int>>& path){
    FILE* fp1 = fopen("../map.txt","w+");
    for(int i=0;i<paze.size();++i){
        for(int j=0;j<paze[0].size();++j){
            if(paze[i][j]==1){
                fprintf(fp1,"1 ");
            }else{
                fprintf(fp1,"0 ");
            }
        }
        fprintf(fp1,"\n");
    }
    fclose(fp1);

    FILE* fp2 = fopen("../path.txt","w+");
    for(int i=0;i<path.size();++i){
        fprintf(fp2,"%d %d\n",path[i].first,path[i].second);
    }
    fclose(fp2);
    return 0;
}

int main(int argc,char** arg){
    //1.随机建图,长=宽=30，障碍物密度20%,存储到paze
    srand(115);
    int row,col;
    row = col = 30;
    auto&& paze = makePaze(row,col,0.2f);

    //2.设置起点{0,0}终点{26,26},A*寻路,路径存储到path变量
    int len = Astar(paze,{0,0},{26,26});
    cout<<len<<" step\n";

    //3.存储地图paze和路径path到文件，供matlab调用
    output2file(paze,path);

    //4.命令行打印路径（仅作显示）
    for(auto p:path){
        paze[p.first][p.second] = 8;//路径标记为8
    }
    printPaze(paze);
}
