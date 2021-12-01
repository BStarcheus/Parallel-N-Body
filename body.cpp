#include <string>

class Body{
    public:
        std::string name;
        std::string color;
        float mass;
        float radius;
        float pos_x, pos_y;
        float vel_x, vel_y;
        bool hasCollided; // boolean to keep track of which two bodies have collided
        std::vector<float> plot_x;
        std::vector<float> plot_y;
        std::vector<std::string> colors;
        
        Body(std::string n, std::string c, float m, float r, float px, float py, float vx, float vy, bool hasCollided){
            name = n;
            color = c;
            mass = m;
            radius = r;
            pos_x = px;
            pos_y = py;
            vel_x = vx;
            vel_y = vy;
            hasCollided = hasCollided;
        }
    
    private:
};
