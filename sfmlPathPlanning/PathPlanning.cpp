#include <SFML/Graphics.hpp>
#include <iostream>
#include <queue>
#include <stack>
#include <cstdlib> 



#define START_WEIGHT 2
#define END_WEIGHT -2

sf::Vector2u WINDOW_SIZE{1400, 900};
constexpr unsigned TPS = 60; // ticks per seconds
const sf::Time timePerUpdate = sf::seconds(1.0f / float(TPS));
sf::Vector2f TILE_SIZE{64.f, 64.f};

struct Tile
{
        sf::RectangleShape shape;
        int value;
};
void showq(std ::queue<std::pair<int, int>> gq)
{
    std ::queue<std::pair<int, int>> g = gq;
    while (!g.empty()) {
        std::pair<int,int> t = g.front();
        std::cout << '\t' <<"x-coord="<< t.first<<"second-cord="<<t.second;
        std::cout<<std::endl;
        g.pop();
    }
    std::cout << '\n';
}




#define DOUBLEARRAY

void MouseLeftButtonPressed(sf::Vector2i &mousePos, sf::Vector2f &mouseWorldPos, std::vector<std::vector<Tile>> &doubleTileMap,
                            int value, sf::Color color)
{
        if (mouseWorldPos.x >= 0 && mouseWorldPos.y >= 0 &&
            mouseWorldPos.x < WINDOW_SIZE.x && mouseWorldPos.y < WINDOW_SIZE.y)
        {
                // std ::cout << "Hello " << std ::endl;
                auto position = mousePos;
                position.x /= TILE_SIZE.x;
                position.y /= TILE_SIZE.y;
                doubleTileMap[position.y][position.x].shape.setFillColor(color);
                doubleTileMap[position.y][position.x].value = value;
        }
}


void FillVisited(sf::Vector2i &mousePos, std::vector<std::vector<Tile>> &doubleTileMap, sf::Color color)
{

                // std ::cout << "In fillVisited" << std ::endl;
                auto position = mousePos;
                // position.x /= TILE_SIZE.x;
                // position.y /= TILE_SIZE.y;
                doubleTileMap[position.y][position.x].shape.setFillColor(color);
                // for(int i=1;i<1e6;i++){
                //         std::cout<<i;
                // }
                
        
}



void bfs(std::vector<std::vector<Tile>> &doubleTileMap, std ::queue<std ::pair<int, int>> &returnvisit,
         std::vector<std ::vector<std ::pair<int, int>>> &parent)
{       std::cout<<"BFS starting:";
        using namespace std;
        pair<int, int> start, end;
        for (int i = 0; i < doubleTileMap.size(); i++)
        {
                for (int j = 0; j < doubleTileMap[0].size(); j++)
                {
                        if (doubleTileMap[i][j].value == START_WEIGHT)
                                start = {i, j};

                        if (doubleTileMap[i][j].value == END_WEIGHT)
                                start = {i, j};
                }
        }

        queue<pair<int, int>> pq;

        pq.push({start});
        vector<vector<int>> visited(doubleTileMap.size(), vector<int>(doubleTileMap[0].size(), 0));
        // vector<vector<pair<int,int> >> parent(doubleTileMap.size(),vector<pair<int,int>>(doubleTileMap[0].size(),0));
        // stack <pair<int,int>> returnvisit;
        returnvisit.push(start);
        int counter = 0;
        visited[start.first][start.second] = 1;
        while (!pq.empty())
        {
                pair<int, int> top = pq.front();
                pq.pop();
                // visited[top.second.first][top.second.second] = 1;
                int arr[][2] = {{1, 0}, {-1, 0}, {0, -1}, {0, 1}};
                bool flag = false;
                for (int i = 0; i < 4; i++)
                {
                        int newx = top.first + arr[i][0];
                        int newy = top.second + arr[i][1];

                        // check
                        if (newx >= 0 && newx < doubleTileMap.size() && newy >= 0 && newy < doubleTileMap[0].size())
                        {
                                if (visited[newx][newy] == 0)
                                {
                                        // cout << newx << " " << newy << endl;
                                        pq.push({newx, newy});
                                        returnvisit.push({newx, newy});
                                        // sf::Vector2i tofill{newx, newy};
                                        // FillVisited(tofill, doubleTileMap, sf::Color::Blue);
                                        parent[newx][newy] = top;
                                        visited[newx][newy] = 1;
                                }
                                if (newx == end.first && newy == end.second)
                                {       
                                        cout << "In break  " << newx << " " << newy << endl;
                                        flag = true;
                                        break;
                                }
                        }
                }
                if (flag)
                {
                        break;
                }
                counter++;
        }
        // cout << counter << endl;
        showq(returnvisit);
}

void dijkstra(std::vector<std::vector<Tile>> &doubleTileMap, std ::stack<std ::pair<int, int>> &returnvisit,
              std::vector<std ::vector<std ::pair<int, int>>> &parent)
{
        using namespace std;
        pair<int, int> start, end;
        for (int i = 0; i < doubleTileMap.size(); i++)
        {
                for (int j = 0; j < doubleTileMap[0].size(); j++)
                {
                        if (doubleTileMap[i][j].value == START_WEIGHT)
                                start = {i, j};

                        if (doubleTileMap[i][j].value == END_WEIGHT)
                                start = {i, j};
                }
        }

        priority_queue<pair<int, pair<int, int>>, vector<pair<int, pair<int, int>>>, greater<pair<int, pair<int, int>>>> pq;

        pq.push({0, start});
        vector<vector<int>> visited(doubleTileMap.size(), vector<int>(doubleTileMap[0].size(), 0));
        // vector<vector<pair<int,int> >> parent(doubleTileMap.size(),vector<pair<int,int>>(doubleTileMap[0].size(),0));
        // stack <pair<int,int>> returnvisit;
        returnvisit.push(start);
        int counter = 0;
        visited[start.first][start.second] = 1;
        while (!pq.empty())
        {
                pair<int, pair<int, int>> top = pq.top();
                pq.pop();
                // visited[top.second.first][top.second.second] = 1;
                int arr[][2] = {{1, 0}, {-1, 0}, {0, -1}, {0, 1}};
                bool flag = false;
                for (int i = 0; i < 4; i++)
                {
                        int newx = top.second.first + arr[i][0];
                        int newy = top.second.second + arr[i][1];

                        // check
                        if (newx >= 0 && newx < doubleTileMap.size() && newy >= 0 && newy < doubleTileMap[0].size())
                        {
                                if (visited[newx][newy] == 0)
                                {
                                        pq.push({top.first + 1, {newx, newy}});
                                        returnvisit.push({newx, newy});
                                        parent[newx][newy] = top.second;
                                        visited[newx][newy] = 1;
                                }
                                if (newx == end.first && newy == end.second)
                                {       //returnvisit.push({newx, newy});
                                        flag = true;
                                        break;
                                }
                        }
                }
                if (flag)
                {
                        break;
                }
                counter++;
        }
        cout << counter << endl;
}

int main()
{
        sf::RenderWindow window{sf::VideoMode{WINDOW_SIZE.x, WINDOW_SIZE.y}, ""};
        //      window.setFramerateLimit(60);
        window.setPosition(sf::Vector2i{window.getPosition().x, 0});

        bool lostFocus = false;
        auto view = window.getDefaultView();

        auto speed = 100.f;

        sf::Clock clock;
        sf::Time timeSinceLastUpdate = sf::Time::Zero;
        sf::Time FrameTime = sf::seconds(1.f / 60.f);

        std::vector<std::vector<Tile>> doubleTileMap;
        for (auto y = 0; y < WINDOW_SIZE.y / TILE_SIZE.y; y++)
        {
                std ::vector<Tile> temp;
                for (auto x = 0; x < WINDOW_SIZE.x / TILE_SIZE.x; x++)
                {
                        auto tile = new Tile();
                        tile->shape.setSize({TILE_SIZE.x, TILE_SIZE.y});
                        tile->shape.setFillColor({0, 125, 0, 45});
                        tile->shape.setOutlineThickness(0.5f);
                        tile->shape.setOutlineColor({sf::Color::White});
                        tile->shape.setPosition(x * TILE_SIZE.x, y * TILE_SIZE.y);
                        tile->value = 0;
                        // doubleTileMap[y].push_back(*tile);
                        temp.push_back(*tile);
                }
                // std ::cout << y << std ::endl;
                doubleTileMap.push_back(temp);
        }

        // std ::cout << "This is the size : " << doubleTileMap.size() << " " << doubleTileMap[0].size() << std ::endl;
        std::cout<<"Click anywhere in the grid to give Start Position"<<std::endl;
        int counter = 2;
        std::vector<std ::vector<std::pair<int, int>>> parent(doubleTileMap.size(), std ::vector<std ::pair<int, int>>(doubleTileMap[0].size(), {0, 0}));
        std ::queue<std::pair<int, int>> returnvisit;
        int flag = false;
        while (window.isOpen())
        {

                sf::Time dt = clock.restart();
                timeSinceLastUpdate += dt;

                sf::Event event{};
                while (window.pollEvent(event))
                {
                        if (event.type == sf::Event::Closed)
                                window.close();
                        if (event.type == sf::Event::KeyPressed)
                        {
                                switch (event.key.code)
                                {
                                case sf::Keyboard::Enter:
                                        std::cout << "Enter Pressed\n";
                                        break;
                                case sf::Keyboard::Space:
                                        std::cout << "Space Pressed\n";
                                        // std ::cout << "Before calling " << returnvisit.size() << std ::endl;
                                        if (counter == 0 && flag == false)
                                                bfs(doubleTileMap, returnvisit, parent);
                                        flag = true;
                                        // cout<<
                                        // std ::cout << std ::endl;
                                        // std ::cout << "This is the size : " << doubleTileMap.size() << " " << doubleTileMap[0].size() << std ::endl;
                                        // std ::cout << returnvisit.size() << std ::endl;
                                default:
                                        break;
                                }
                        }
                        if (event.type == sf::Event::Resized)
                        {
                                // update the view to the new size of the window
                                view.setSize(static_cast<float>(event.size.width), static_cast<float>(event.size.height));
                        }
                        if (event.type == sf::Event::LostFocus)
                        {
                                lostFocus = true;
                        }
                        if (event.type == sf::Event::GainedFocus)
                        {
                                lostFocus = false;
                        }
                        if (event.type == sf::Event::MouseButtonReleased && !lostFocus)
                        {
                                auto mousePos = sf::Mouse::getPosition(window);
                                auto mouseWorldPos = window.mapPixelToCoords(mousePos, view);
                                if (counter == 2)
                                {       
                                        // std::cout << counter << std::endl;
                                        MouseLeftButtonPressed(mousePos, mouseWorldPos, doubleTileMap, START_WEIGHT, sf::Color::Green);
                                        std::cout<<"Click anywhere in the grid to give Goal Position"<<std::endl;
                                        counter--;
                                }
                                else if (counter == 1)
                                {
                                        // std ::cout << counter << std ::endl;
                                        MouseLeftButtonPressed(mousePos, mouseWorldPos, doubleTileMap, END_WEIGHT, sf::Color::White);
                                        counter--;
                                        std::cout<<"Now Press Space to start Path-Planning using BFS";
                                }
                        }
                }
                if(flag==true){

                        if(returnvisit.size()!=0){
                                // showq(returnvisit);
                                std::pair<int, int> p=returnvisit.front();          
                                sf::Vector2i to_color{p.first,p.second};
                                returnvisit.pop();
                                // sf::Color color(255, 165, 0,1.0);
                                sf::Color color(255, 165, 0,255/4);
                                FillVisited(to_color, doubleTileMap,  color);

                        }
                        else{
                        flag ==false;
                        std ::cout << "Success" << std ::endl;
                        break;}
                }
                if (!lostFocus)
                {
                        auto mousePos = sf::Mouse::getPosition(window);
                        auto mouseWorldPos = window.mapPixelToCoords(mousePos, view);

                        window.setTitle("Mouse Position: (" + std::to_string(int(mouseWorldPos.x / 64.f)) + ", " +
                                        std::to_string(int(mouseWorldPos.y / 64.f)) + ")");

                        while (timeSinceLastUpdate > FrameTime)
                        {
                                timeSinceLastUpdate -= FrameTime; 
                                while (timeSinceLastUpdate > FrameTime) {
                                                timeSinceLastUpdate -= FrameTime;}
                        }
                }
                // view.setCenter(shape.getPosition());
                window.setView(view);

                //! ** DRAW SECTION **
                window.clear();

                for (auto &index : doubleTileMap)
                {
                        for (auto &it : index)
                        {
                                window.draw(it.shape);
                        }
                }

                window.setView(window.getDefaultView());

                window.display();
        }

        return EXIT_SUCCESS;
}